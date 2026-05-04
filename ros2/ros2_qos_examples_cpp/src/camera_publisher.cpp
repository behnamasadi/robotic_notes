// Publishes a synthetic "camera" Image at 60 Hz using SensorDataQoS:
//   KEEP_LAST(5), BEST_EFFORT, VOLATILE.
// Pair with latest_frame_subscriber to observe drop behavior under a slow consumer.

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class CameraPublisher : public rclcpp::Node
{
public:
  CameraPublisher() : Node("camera_publisher")
  {
    pub_ = create_publisher<sensor_msgs::msg::Image>(
      "camera/image", rclcpp::SensorDataQoS());

    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / 60),
      [this]() { publish_frame(); });
  }

private:
  void publish_frame()
  {
    sensor_msgs::msg::Image msg;
    msg.header.stamp = now();
    msg.header.frame_id = "camera";
    msg.height = 8;
    msg.width = 8;
    msg.encoding = "mono8";
    msg.step = 8;
    msg.data.assign(64, static_cast<uint8_t>(frame_id_ & 0xFF));
    pub_->publish(msg);

    if ((frame_id_ % 60) == 0) {
      RCLCPP_INFO(get_logger(), "published frame %lu", frame_id_);
    }
    ++frame_id_;
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  uint64_t frame_id_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraPublisher>());
  rclcpp::shutdown();
  return 0;
}
