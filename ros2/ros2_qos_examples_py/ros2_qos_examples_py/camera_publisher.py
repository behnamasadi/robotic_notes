"""Publishes a synthetic camera Image at 60 Hz with qos_profile_sensor_data."""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CameraPublisher(Node):
    def __init__(self) -> None:
        super().__init__('camera_publisher_py')
        self.pub = self.create_publisher(
            Image, 'camera/image', qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / 60.0, self._publish_frame)
        self.frame_id = 0

    def _publish_frame(self) -> None:
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.height = 8
        msg.width = 8
        msg.encoding = 'mono8'
        msg.step = 8
        msg.data = bytes([self.frame_id & 0xFF] * 64)
        self.pub.publish(msg)

        if self.frame_id % 60 == 0:
            self.get_logger().info(f'published frame {self.frame_id}')
        self.frame_id += 1


def main() -> None:
    rclpy.init()
    rclpy.spin(CameraPublisher())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
