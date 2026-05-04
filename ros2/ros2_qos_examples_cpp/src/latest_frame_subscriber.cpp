// Slow consumer (50 ms/frame) on a 60 Hz publisher. With KEEP_LAST(5) +
// BEST_EFFORT, the subscriber sees only the freshest frame each callback —
// intermediate frames are dropped at the queue, not buffered.

#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class LatestFrameSubscriber : public rclcpp::Node
{
public:
  LatestFrameSubscriber() : Node("latest_frame_subscriber")
  {
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      "camera/image",
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { on_frame(msg); });
  }

private:
  void on_frame(sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    std::this_thread::sleep_for(50ms);

    const uint8_t marker = msg->data.empty() ? 0 : msg->data.front();
    const double age_ms = (now() - msg->header.stamp).seconds() * 1000.0;
    RCLCPP_INFO(get_logger(),
      "processed frame marker=%u, age=%.1f ms", marker, age_ms);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LatestFrameSubscriber>());
  rclcpp::shutdown();
  return 0;
}
