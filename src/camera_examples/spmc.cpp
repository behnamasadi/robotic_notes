// Minimal single-producer multi-consumer (SPMC) frame fan-out.
//
// One camera thread produces frames. Several worker threads each want every
// frame (logger, processor, visualizer, ...). Each consumer has its OWN
// bounded queue, so a slow consumer cannot stall a fast one — its queue
// just fills up and the producer drops frames into it.
//
// Compared to the Broker version: there is no Broker class. The producer
// is given a vector of queues directly and fans out by iterating it.
//
// Build: g++ -std=c++20 -O2 -pthread spmc.cpp -o spmc

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

struct Frame {
    std::uint64_t             seq;
    std::vector<std::uint8_t> data;   // pretend pixels
};

// Per-consumer bounded queue. The whole synchronization story lives here.
class FrameQueue {
public:
    explicit FrameQueue(std::size_t capacity) : cap_(capacity) {}

    // Non-blocking push. Returns false if full → producer drops this frame
    // for this consumer (other consumers may still get it).
    bool try_push(std::shared_ptr<const Frame> f) {
        { std::lock_guard lk(m_);
          if (q_.size() >= cap_) return false;
          q_.push_back(std::move(f)); }
        cv_.notify_one();
        return true;
    }

    // Blocking pop. Returns nullopt once stop() has been called AND the
    // queue is empty — signals the consumer to exit.
    std::optional<std::shared_ptr<const Frame>> pop() {
        std::unique_lock lk(m_);
        cv_.wait(lk, [&]{ return !q_.empty() || stop_; });
        if (q_.empty()) return std::nullopt;
        auto f = std::move(q_.front());
        q_.pop_front();
        return f;
    }

    void stop() {
        { std::lock_guard lk(m_); stop_ = true; }
        cv_.notify_all();
    }

private:
    std::size_t                              cap_;
    std::deque<std::shared_ptr<const Frame>> q_;
    std::mutex                               m_;
    std::condition_variable                  cv_;
    bool                                     stop_ = false;
};

// Producer. Takes the list of consumer queues directly — no Broker.
void camera(std::vector<std::shared_ptr<FrameQueue>>& queues,
            double fps, int n_frames)
{
    using namespace std::chrono;
    const auto period = duration_cast<nanoseconds>(duration<double>(1.0 / fps));

    for (int i = 0; i < n_frames; ++i) {
        // Build the frame ONCE, wrapped in shared_ptr.
        // Every consumer holds the same shared_ptr — fan-out copies a
        // 16-byte control block, not the pixels.
        auto f  = std::make_shared<Frame>();
        f->seq  = static_cast<std::uint64_t>(i);
        f->data.assign(16, 0);

        for (auto& q : queues) q->try_push(f);   // drop-on-full per consumer
        std::this_thread::sleep_for(period);
    }
    for (auto& q : queues) q->stop();
}

// Consumer. Reads from its own queue at its own pace.
void consumer(std::string name,
              std::shared_ptr<FrameQueue> q,
              std::chrono::milliseconds work_time)
{
    for (;;) {
        auto maybe = q->pop();
        if (!maybe) { std::cout << "[" << name << "] shutting down\n"; return; }
        std::this_thread::sleep_for(work_time);   // simulate processing
        std::cout << "[" << name << "] processed frame " << (*maybe)->seq << "\n";
    }
}

int main() {
    // One queue per consumer. Capacity 4 is enough to absorb small jitter
    // without letting a slow consumer hoard memory.
    std::vector<std::shared_ptr<FrameQueue>> queues = {
        std::make_shared<FrameQueue>(4),   // logger
        std::make_shared<FrameQueue>(4),   // processor (slow → will drop)
        std::make_shared<FrameQueue>(4),   // visualizer
    };

    std::thread t_logger    (consumer, "logger",     queues[0], std::chrono::milliseconds(5));
    std::thread t_processor (consumer, "processor",  queues[1], std::chrono::milliseconds(50));
    std::thread t_visualizer(consumer, "visualizer", queues[2], std::chrono::milliseconds(20));

    std::thread t_camera(camera, std::ref(queues), /*fps=*/30.0, /*n_frames=*/60);

    t_camera.join();
    t_logger.join();
    t_processor.join();
    t_visualizer.join();
    return 0;
}
