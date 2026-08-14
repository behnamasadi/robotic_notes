#!/usr/bin/env python3
# Accumulate nav_msgs/Odometry samples into a nav_msgs/Path so RViz can
# render a proper trajectory line. Used for KISS-ICP, which publishes
# only /kiss/odometry — no Path topic of its own (FAST-LIO and DLIO both
# publish a Path natively). Generic enough to reuse for any odometry
# source that lacks a Path companion.
#
# Parameters:
#   input_topic   nav_msgs/Odometry topic to subscribe to (default /odom)
#   output_topic  nav_msgs/Path topic to publish on       (default /path)
#   fixed_frame   if set, override the path/header frame  (default "" = keep odom's)
#   max_poses     ring-buffer cap to keep memory bounded  (default 20000)
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OdomToPath(Node):
    def __init__(self):
        super().__init__("odom_to_path")
        self.declare_parameter("input_topic",  "/odom")
        self.declare_parameter("output_topic", "/path")
        self.declare_parameter("fixed_frame",  "")
        self.declare_parameter("max_poses",    20000)

        self.input_topic  = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.fixed_frame  = self.get_parameter("fixed_frame").value
        self.max_poses    = self.get_parameter("max_poses").value

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.path = Path()
        self.create_subscription(Odometry, self.input_topic, self._on_odom, qos)
        self.path_pub = self.create_publisher(Path, self.output_topic, qos)

        self.get_logger().info(
            f"Accumulating '{self.input_topic}' (Odometry) → "
            f"'{self.output_topic}' (Path)"
            f"{', overriding frame to ' + self.fixed_frame if self.fixed_frame else ''}")

    def _on_odom(self, msg: Odometry) -> None:
        frame = self.fixed_frame if self.fixed_frame else msg.header.frame_id

        ps = PoseStamped()
        ps.header.stamp    = msg.header.stamp
        ps.header.frame_id = frame
        ps.pose            = msg.pose.pose

        self.path.header.stamp    = msg.header.stamp
        self.path.header.frame_id = frame
        self.path.poses.append(ps)
        if len(self.path.poses) > self.max_poses:
            self.path.poses = self.path.poses[-self.max_poses:]
        self.path_pub.publish(self.path)


def main():
    rclpy.init()
    try:
        rclpy.spin(OdomToPath())
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
