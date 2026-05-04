"""Slow consumer (50 ms/frame) on a 60 Hz publisher.

With qos_profile_sensor_data the subscriber sees only the freshest frame each
callback — intermediate frames are dropped at the queue, not buffered.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image


class LatestFrameSubscriber(Node):
    def __init__(self) -> None:
        super().__init__('latest_frame_subscriber_py')
        self.sub = self.create_subscription(
            Image, 'camera/image', self._on_frame, qos_profile_sensor_data)

    def _on_frame(self, msg: Image) -> None:
        time.sleep(0.05)  # simulate slow processing
        marker = msg.data[0] if len(msg.data) else 0
        now = self.get_clock().now()
        stamp = Time.from_msg(msg.header.stamp)
        age_ms = (now - stamp).nanoseconds / 1e6
        self.get_logger().info(
            f'processed frame marker={marker}, age={age_ms:.1f} ms')


def main() -> None:
    rclpy.init()
    rclpy.spin(LatestFrameSubscriber())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
