#!/usr/bin/env python3
# Republish gz's ground-truth pose so RViz and evo can both consume it.
#
# Inputs:
#   /ground_truth/pose  (tf2_msgs/TFMessage from gz scene_broadcaster)
#
# Outputs:
#   /ground_truth/path  (nav_msgs/Path)      — trail for RViz overlay + evo
#   /ground_truth/odom  (nav_msgs/Odometry)  — current-pose arrow for RViz
#
# Both outputs are expressed in fixed_frame (default `explorer_r2/odom`).
# gz publishes the pose in its own world frame (e.g. "tunnel"), so we
# capture the first transform as origin and subtract it — the GT trail
# then starts at (0, 0, 0), directly comparable to wheel-odom, VIO, and
# LIO outputs in the same RViz view.
#
# Usage:
#   ros2 run explorer_r2_sim gt_to_path.py
#   ros2 run explorer_r2_sim gt_to_path.py --ros-args -p target_frame:=explorer_r2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped


class GroundTruthToPath(Node):
    def __init__(self):
        super().__init__("gt_to_path")
        self.declare_parameter("target_frame", "explorer_r2")
        self.declare_parameter("fixed_frame",  "explorer_r2/odom")
        self.target_frame = self.get_parameter("target_frame").value
        self.fixed_frame = self.get_parameter("fixed_frame").value

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.path = Path()
        self.path.header.frame_id = self.fixed_frame
        self.origin = None  # captured from the first transform

        self.create_subscription(
            TFMessage, "/ground_truth/pose", self._on_tf, qos)
        self.path_pub = self.create_publisher(
            Path, "/ground_truth/path", qos)
        self.odom_pub = self.create_publisher(
            Odometry, "/ground_truth/odom", qos)

        self.get_logger().info(
            f"Tracking '{self.target_frame}'; publishing "
            "/ground_truth/path + /ground_truth/odom "
            f"in frame '{self.fixed_frame}' (origin captured on first sample)")

    def _on_tf(self, msg: TFMessage) -> None:
        # ros_gz bridge maps gz Pose_V → tf2_msgs/TFMessage but leaves the
        # transform names blank — frame_id and child_frame_id are both
        # empty strings. So we can't filter by target_frame the obvious
        # way. Fall back: take the transform whose name matches if any do,
        # otherwise take the FIRST transform in the list (gz publishes
        # the robot first in /world/<w>/dynamic_pose/info because it's
        # the only dynamic entity in our worlds).
        chosen = next(
            (tf for tf in msg.transforms if tf.child_frame_id == self.target_frame),
            msg.transforms[0] if msg.transforms else None,
        )
        if chosen is None:
            return
        for tf in [chosen]:
            t = tf.transform.translation
            if self.origin is None:
                self.origin = (t.x, t.y, t.z)
                self.get_logger().info(
                    f"GT origin captured: ({t.x:.3f}, {t.y:.3f}, {t.z:.3f})")

            ps = PoseStamped()
            # The ros_gz bridge leaves tf.header.stamp at (0, 0) for
            # /world/<w>/dynamic_pose/info — that breaks evo_ape because
            # it can't time-match against estimator outputs that DO have
            # stamps. Fall back to the node's clock (which uses sim_time
            # when launched with use_sim_time:=true).
            stamp = tf.header.stamp
            if stamp.sec == 0 and stamp.nanosec == 0:
                stamp = self.get_clock().now().to_msg()
            ps.header.stamp = stamp
            ps.header.frame_id = self.fixed_frame
            ps.pose.position.x = t.x - self.origin[0]
            ps.pose.position.y = t.y - self.origin[1]
            ps.pose.position.z = t.z - self.origin[2]
            ps.pose.orientation = tf.transform.rotation

            self.path.header.stamp = tf.header.stamp
            self.path.poses.append(ps)
            # Keep memory bounded for long runs.
            if len(self.path.poses) > 20000:
                self.path.poses = self.path.poses[-20000:]
            self.path_pub.publish(self.path)

            odom = Odometry()
            odom.header = ps.header
            odom.child_frame_id = "explorer_r2/base_link"
            odom.pose.pose = ps.pose
            self.odom_pub.publish(odom)


def main():
    rclpy.init()
    try:
        rclpy.spin(GroundTruthToPath())
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
