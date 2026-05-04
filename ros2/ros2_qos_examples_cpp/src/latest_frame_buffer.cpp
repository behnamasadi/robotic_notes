// Production pattern: one ROS subscription, an internal latest-frame slot
// guarded by a mutex, and a worker thread that processes frames off the
// executor thread. The producer (callback) only swaps a shared_ptr — never
// blocked by the consumer's processing time.

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class LatestFrameBuffer : public rclcpp::Node
{
public:
  LatestFrameBuffer() : Node("latest_frame_buffer")
  {
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      "camera/image",
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { on_frame(msg); });

    worker_ = std::thread(&LatestFrameBuffer::worker_loop, this);
  }

  ~LatestFrameBuffer() override
  {
    {
      std::scoped_lock lock(mtx_);
      running_ = false;
    }
    cv_.notify_all();
    if (worker_.joinable()) worker_.join();
  }

private:
  // Producer: cheap. Atomic-ish swap of the latest pointer.
  void on_frame(sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    {
      std::scoped_lock lock(mtx_);
      latest_ = std::move(msg);
    }
    cv_.notify_one();
  }

  // Consumer: heavy work runs here, off the executor thread.
  void worker_loop()
  {
    while (true) {
      sensor_msgs::msg::Image::ConstSharedPtr frame;
      {
        std::unique_lock lock(mtx_);
        cv_.wait(lock, [this]() { return !running_ || latest_ != nullptr; });
        if (!running_) return;
        frame = std::move(latest_);
        latest_.reset();
      }

      // Heavy processing happens outside the lock: producer is never blocked.
      std::this_thread::sleep_for(50ms);

      const uint8_t marker = frame->data.empty() ? 0 : frame->data.front();
      const double age_ms = (now() - frame->header.stamp).seconds() * 1000.0;
      RCLCPP_INFO(get_logger(),
        "worker processed frame marker=%u, age=%.1f ms", marker, age_ms);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  std::thread worker_;
  std::mutex mtx_;
  std::condition_variable cv_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_;
  bool running_{true};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LatestFrameBuffer>());
  rclcpp::shutdown();
  return 0;
}
