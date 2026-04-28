// Minimal SPMC ring buffer (LMAX Disruptor style).
//
// One producer, many consumers, one fixed-size array of slots.
//
//   - Producer writes to slot[seq % N] and publishes `seq` atomically.
//   - Each consumer holds its OWN cursor (last-read seq).
//   - Consumers never block each other and never block the producer.
//   - If a consumer lags more than N slots behind, the producer has
//     overwritten old data — the consumer detects this and skips
//     forward to the latest still-available slot. THAT IS THE DESIGN,
//     not a bug. Sensor fan-out wants the freshest frame, not all of
//     them, so dropping old frames silently is the correct behavior.
//
// Compare with spmc.cpp (per-consumer queue): there, every consumer has
// its own backlog and a slow consumer drops frames inside *its own*
// queue. Here, a single shared ring overwrites the oldest slot — slow
// consumers get the latest, never an old backlog.
//
// Build: g++ -std=c++20 -O2 -pthread spmc_ring.cpp -o spmc_ring

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <thread>

template <class T, std::size_t N>
class SpmcRing {
    static_assert((N & (N - 1)) == 0, "N must be a power of two");

    std::array<T, N>           slots_{};
    std::atomic<std::uint64_t> write_{0};   // next slot to write

public:
    void publish(T v) {
        const auto w = write_.load(std::memory_order_relaxed);
        slots_[w & (N - 1)] = std::move(v);
        // release: makes the slot store visible to any consumer that
        // performs an acquire-load of write_.
        write_.store(w + 1, std::memory_order_release);
    }

    // Each consumer keeps its own `cursor` (their last-read seq).
    // Returns false if there's nothing new; rewinds cursor if it lapped.
    bool try_consume(std::uint64_t& cursor, T& out) {
        const auto w = write_.load(std::memory_order_acquire);
        if (cursor >= w) return false;
        if (w - cursor > N) cursor = w - N;          // we got lapped: skip
        out = slots_[cursor & (N - 1)];
        ++cursor;
        return true;
    }

    std::uint64_t latest() const { return write_.load(std::memory_order_acquire); }
};

int main() {
    constexpr std::size_t kRingSize  = 8;       // power of two; small so laps are visible
    constexpr int         kNumFrames = 60;
    constexpr auto        kPeriod    = std::chrono::milliseconds(33);  // ~30 fps

    SpmcRing<std::uint64_t, kRingSize> ring;
    std::atomic<bool> producer_done{false};

    // Producer: publishes frame numbers 0..N-1 at 30 fps.
    std::thread producer([&]{
        for (int i = 0; i < kNumFrames; ++i) {
            ring.publish(static_cast<std::uint64_t>(i));
            std::printf("producer: published %d\n", i);
            std::this_thread::sleep_for(kPeriod);
        }
        producer_done.store(true);
    });

    // Fast consumer: keeps up easily, never gets lapped.
    std::thread fast([&]{
        std::uint64_t cursor = 0, v = 0, seen = 0;
        while (!producer_done.load() || cursor < ring.latest()) {
            if (ring.try_consume(cursor, v)) {
                ++seen;
            } else {
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
        }
        std::printf("fast    : saw %lu values out of %d\n", seen, kNumFrames);
    });

    // Slow consumer: 6× slower than the producer → guaranteed to lap.
    // We detect lap by checking whether the cursor jumped forward by
    // more than 1.
    std::thread slow([&]{
        std::uint64_t cursor = 0, v = 0, seen = 0, skips = 0;
        while (!producer_done.load() || cursor < ring.latest()) {
            std::uint64_t before = cursor;
            if (ring.try_consume(cursor, v)) {
                const bool lapped = (cursor - before) > 1;
                if (lapped) ++skips;
                ++seen;
                std::printf("slow    : got %lu%s\n",
                            v, lapped ? "  (LAPPED, jumped forward)" : "");
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        std::printf("slow    : saw %lu values, lapped %lu times\n", seen, skips);
    });

    producer.join();
    fast.join();
    slow.join();
    return 0;
}
