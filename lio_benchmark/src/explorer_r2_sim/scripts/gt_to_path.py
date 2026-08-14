#!/usr/bin/env python3
# Two input modes — picked by the `tum_file` parameter:
#
#   tum_file == ""  (default, sim mode):
#       subscribe to /ground_truth/pose (tf2_msgs/TFMessage from gz
#       scene_broadcaster); republish each pose.
#
#   tum_file == "<path>"  (bag-replay mode, e.g. NCD):
#       read the TUM-format CSV once at startup (timestamp tx ty tz
#       qx qy qz qw), publish the full trajectory as a single static
#       Path on /ground_truth/path. No tf2 stream needed — the public
#       datasets ship GT as a CSV, not a live topic.
#
# Outputs (both modes):
#   /ground_truth/path  (nav_msgs/Path)      — trail for RViz overlay + evo
#   /ground_truth/odom  (nav_msgs/Odometry)  — current-pose arrow for RViz
#                                             (sim mode only; TUM mode
#                                             publishes the last pose once)
#   /tf                 (tf2_msgs/TFMessage) — fixed_frame → base_link
#                                             (sim mode only)
#
# All outputs are expressed in fixed_frame (default `explorer_r2/odom`).
# Each mode captures the first sample as origin and subtracts it, so
# the GT trail starts at (0, 0, 0) — directly comparable to LIO outputs
# which start from their own initialization pose.
#
# Usage:
#   ros2 run explorer_r2_sim gt_to_path.py
#   ros2 run explorer_r2_sim gt_to_path.py --ros-args \
#       -p tum_file:=/data/.../gt-nc-stairs.csv
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped


class GroundTruthToPath(Node):
    def __init__(self):
        super().__init__("gt_to_path")
        self.declare_parameter("target_frame", "explorer_r2")
        self.declare_parameter("fixed_frame",  "explorer_r2/odom")
        self.target_frame = self.get_parameter("target_frame").value
        self.fixed_frame = self.get_parameter("fixed_frame").value

        self.declare_parameter("baselink_frame", "explorer_r2/base_link")
        self.baselink_frame = self.get_parameter("baselink_frame").value
        # If false, only the /ground_truth/path and /ground_truth/odom
        # topics are published — the explorer_r2/odom→base_link TF is
        # left to some other source (typically DiffDrive's wheel odom,
        # bridged separately when launched with rover_tf:=wheel).
        self.declare_parameter("publish_tf", True)
        self.publish_tf = self.get_parameter("publish_tf").value

        # When set, replaces the /ground_truth/pose subscription with a
        # one-shot read of a TUM-format CSV (the form most public LIO
        # datasets ship their GT in). Used by bag.launch.py for NCD,
        # HILTI, etc.
        self.declare_parameter("tum_file", "")
        self.tum_file = self.get_parameter("tum_file").value

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        # Bag-replay GT is large and static — we publish it once and
        # let subscribers (RViz, evo) pick it up from the latched
        # cache. Use TRANSIENT_LOCAL so late-joining RViz still sees
        # the path without us republishing.
        from rclpy.qos import QoSDurabilityPolicy
        latched_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path = Path()
        self.path.header.frame_id = self.fixed_frame
        self.origin = None  # captured from the first transform

        self.path_pub = self.create_publisher(
            Path, "/ground_truth/path",
            latched_qos if self.tum_file else qos)
        self.odom_pub = self.create_publisher(
            Odometry, "/ground_truth/odom", qos)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        if self.tum_file:
            self._load_tum_and_publish()
        else:
            self.create_subscription(
                TFMessage, "/ground_truth/pose", self._on_tf, qos)
            tf_suffix = (f" and broadcasting '{self.fixed_frame}' → '{self.baselink_frame}' on /tf"
                         if self.publish_tf else " (TF broadcast disabled — rover_tf:=wheel)")
            self.get_logger().info(
                f"Tracking '{self.target_frame}'; publishing "
                "/ground_truth/path + /ground_truth/odom"
                f"{tf_suffix} (origin captured on first sample)")

    def _load_tum_and_publish(self) -> None:
        # TUM format: one whitespace-separated row per pose:
        #   timestamp tx ty tz qx qy qz qw
        # Lines starting with '#' are comments. The first row is
        # captured as origin (so the trail starts at (0,0,0) in RViz,
        # comparable with each estimator's path).
        import os
        if not os.path.exists(self.tum_file):
            self.get_logger().error(
                f"tum_file does not exist: {self.tum_file}")
            return
        n = 0
        first = None
        with open(self.tum_file) as f:
            for line in f:
                if not line.strip() or line.startswith("#"):
                    continue
                parts = line.split()
                if len(parts) < 8:
                    continue
                _, tx, ty, tz, qx, qy, qz, qw = parts[:8]
                tx, ty, tz = float(tx), float(ty), float(tz)
                qx, qy, qz, qw = float(qx), float(qy), float(qz), float(qw)
                if first is None:
                    first = (tx, ty, tz)
                ps = PoseStamped()
                ps.header.frame_id = self.fixed_frame
                ps.header.stamp = self.get_clock().now().to_msg()
                ps.pose.position.x = tx - first[0]
                ps.pose.position.y = ty - first[1]
                ps.pose.position.z = tz - first[2]
                ps.pose.orientation.x = qx
                ps.pose.orientation.y = qy
                ps.pose.orientation.z = qz
                ps.pose.orientation.w = qw
                self.path.poses.append(ps)
                n += 1
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path)
        # Also publish the last pose as the GT "current" odometry, so
        # the RViz GT Odom arrow has something to display.
        if self.path.poses:
            odom = Odometry()
            odom.header = self.path.poses[-1].header
            odom.child_frame_id = self.baselink_frame
            odom.pose.pose = self.path.poses[-1].pose
            self.odom_pub.publish(odom)
        self.get_logger().info(
            f"Loaded {n} poses from TUM file → /ground_truth/path "
            f"(latched, fixed_frame={self.fixed_frame})")

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

            self.path.header.stamp = stamp
            self.path.poses.append(ps)
            # Keep memory bounded for long runs.
            if len(self.path.poses) > 20000:
                self.path.poses = self.path.poses[-20000:]
            self.path_pub.publish(self.path)

            odom = Odometry()
            odom.header = ps.header
            odom.child_frame_id = self.baselink_frame
            odom.pose.pose = ps.pose
            self.odom_pub.publish(odom)

            # Broadcast TF: fixed_frame → base_link. This positions the
            # rover mesh in RViz at ground truth (replacing DiffDrive's
            # wheel-odo TF, which we no longer bridge by default — see
            # bridge.yaml). Skipped when publish_tf:=false, in which case
            # DiffDrive's wheel-odo TF is bridged instead (rover_tf:=wheel).
            if self.tf_broadcaster is not None:
                tfs = TransformStamped()
                tfs.header.stamp = stamp
                tfs.header.frame_id = self.fixed_frame
                tfs.child_frame_id = self.baselink_frame
                tfs.transform.translation.x = ps.pose.position.x
                tfs.transform.translation.y = ps.pose.position.y
                tfs.transform.translation.z = ps.pose.position.z
                tfs.transform.rotation = ps.pose.orientation
                self.tf_broadcaster.sendTransform(tfs)


def main():
    rclpy.init()
    try:
        rclpy.spin(GroundTruthToPath())
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
