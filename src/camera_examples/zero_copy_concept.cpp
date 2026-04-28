// Conceptual zero-copy demo (no kernel, no V4L2, no platform deps).
//
// Same pattern as v4l2_zero_copy_capture.cpp, stripped to plain C++ so the
// idea stands on its own:
//
//   1. Allocate a small pool of buffers ONCE, up front.
//   2. The producer fills a buffer IN PLACE and hands its INDEX to the
//      consumer (not the bytes).
//   3. The consumer reads from that same memory and hands the index back.
//
// The pixel bytes never move between threads — only the small integer
// index does. That is what "zero-copy" means.
//
// Compare:
//   - copy:      producer.write(data) -> queue<vector<uint8_t>> -> consumer
//                each frame allocates + memcpys.
//   - zero-copy: producer fills pool[i] -> queue<int>(i) -> consumer reads pool[i]
//                no allocation, no memcpy, just an index handoff.
//
// Build:  g++ -std=c++20 -O2 -pthread zero_copy_concept.cpp -o zero_copy_concept

#include <array>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

// A trivially-correct bounded queue of buffer indices.
// This is the moral equivalent of V4L2's QBUF/DQBUF queues.
class IndexQueue {
public:
    void push(int idx) {
        { std::lock_guard lk(m_); q_.push(idx); }
        cv_.notify_one();
    }
    int pop() {
        std::unique_lock lk(m_);
        cv_.wait(lk, [&]{ return !q_.empty(); });
        int idx = q_.front();
        q_.pop();
        return idx;
    }
private:
    std::queue<int>         q_;
    std::mutex              m_;
    std::condition_variable cv_;
};

// One "frame buffer". In V4L2 this would be an mmap'd region the kernel
// writes into; here it's just a vector we allocate once and reuse forever.
struct Buffer {
    std::vector<std::uint8_t> pixels;   // allocated once
    std::uint64_t             frame_id = 0;
};

int main() {
    constexpr int kPoolSize  = 4;       // small ring, like V4L2's REQBUFS
    constexpr int kFrameSize = 640 * 480;
    constexpr int kNumFrames = 20;

    // Pre-allocate the pool. After this point, no pixel-data allocations
    // happen anywhere in the program.
    std::array<Buffer, kPoolSize> pool;
    for (auto& b : pool) b.pixels.resize(kFrameSize);

    // Two queues of indices — same shape as V4L2's "queued for driver" /
    // "ready for userspace" split.
    IndexQueue empty;   // buffers the producer may fill
    IndexQueue filled;  // buffers the consumer may read

    // Initially every buffer is empty and available to the producer.
    for (int i = 0; i < kPoolSize; ++i) empty.push(i);

    // Producer: pulls an empty buffer, writes pixels straight into it,
    // hands the INDEX (not the bytes) to the consumer.
    std::thread producer([&]{
        for (int f = 0; f < kNumFrames; ++f) {
            int i = empty.pop();
            auto& buf = pool[i];

            // "Capture" — fill in place. This is the only place pixels are written.
            buf.frame_id = f;
            for (std::size_t k = 0; k < buf.pixels.size(); ++k)
                buf.pixels[k] = static_cast<std::uint8_t>((f + k) & 0xFF);

            std::printf("producer: frame %2d -> pool[%d] @ %p\n",
                        f, i, static_cast<void*>(buf.pixels.data()));
            filled.push(i);
        }
    });

    // Consumer: pulls a filled index, reads from the SAME memory the
    // producer wrote, returns the buffer to the empty queue.
    std::thread consumer([&]{
        for (int f = 0; f < kNumFrames; ++f) {
            int i = filled.pop();
            auto& buf = pool[i];

            // "Process" — read in place. No copy, no allocation.
            std::uint64_t checksum = 0;
            for (auto px : buf.pixels) checksum += px;

            std::printf("consumer: frame %2lu <- pool[%d] @ %p  sum=%lu\n",
                        buf.frame_id, i, static_cast<void*>(buf.pixels.data()), checksum);
            empty.push(i);
        }
    });

    producer.join();
    consumer.join();

    std::printf("\ndone. %d frames moved through %d buffers, zero pixel copies.\n",
                kNumFrames, kPoolSize);
    return 0;
}
