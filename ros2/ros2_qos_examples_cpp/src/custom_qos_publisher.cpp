// Explicit QoS construction: KEEP_LAST(10) history, RELIABLE delivery,
// TRANSIENT_LOCAL durability so late-joining subscribers receive the most
// recent samples. This is the wrong profile for cameras (causes backpressure)
// but the right starting point for one-shot config or latched state like maps.

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class CustomQoSPublisher : public rclcpp::Node
{
public:
  CustomQoSPublisher() : Node("custom_qos_publisher")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                 .reliable()
                 .transient_local();

    pub_ = create_publisher<sensor_msgs::msg::Image>("config/snapshot", qos);
    timer_ = create_wall_timer(1s, [this]() { publish_once(); });
  }

private:
  void publish_once()
  {
    sensor_msgs::msg::Image msg;
    msg.header.stamp = now();
    msg.header.frame_id = "snapshot";
    msg.height = 1;
    msg.width = 1;
    msg.encoding = "mono8";
    msg.step = 1;
    msg.data = {static_cast<uint8_t>(seq_ & 0xFF)};
    pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "published snapshot seq=%lu", seq_);
    ++seq_;
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  uint64_t seq_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CustomQoSPublisher>());
  rclcpp::shutdown();
  return 0;
}
