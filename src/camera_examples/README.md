# Zero-Copy Camera Capture

Two examples sit side by side here:

- [zero_copy_concept.cpp](zero_copy_concept.cpp) — portable, plain C++. Producer/consumer threads sharing a pool of buffers. No kernel, no platform deps. Read this first.
- [v4l2_zero_copy_capture.cpp](v4l2_zero_copy_capture.cpp) — the same idea applied to a real Linux camera via V4L2 + `mmap`.

## What "zero-copy" means

Naive capture pipeline:

```
camera → kernel buffer → memcpy → user buffer → memcpy → processing buffer
```

Each `memcpy` is per-frame work that scales with image size. At 1080p / 60 fps a single copy is ~370 MB/s of memory bandwidth doing nothing useful.

Zero-copy pipeline:

```
camera → buffer (mmap'd / pre-allocated) → processing reads it in place
```

The pixel bytes are written **once**, into memory the consumer can already see. What gets passed from producer to consumer is a small handle (an index, an offset, a pointer) — **not the bytes**.

## The pattern

Every zero-copy capture API (V4L2, GStreamer's `dmabuf`, NVIDIA's CUDA-interop surfaces, Vulkan external memory, …) is a variation on the same three rules:

1. **Allocate a small pool of buffers up front.** No per-frame allocation.
2. **Producer fills a buffer in place.** It receives an empty buffer, writes pixels into it, then enqueues *the index*.
3. **Consumer reads from that same memory**, then returns the buffer to the empty pool.

There are typically two queues:

| Queue   | Owner    | Meaning                                      |
| ------- | -------- | -------------------------------------------- |
| empty   | producer | "you may fill these"                         |
| filled  | consumer | "these are ready to read"                    |

A buffer is always in exactly one queue. Ownership transfers by moving the *index* between queues — the bytes never move.

## How [zero_copy_concept.cpp](zero_copy_concept.cpp) maps to the pattern

| Concept                  | In the code                                    |
| ------------------------ | ---------------------------------------------- |
| Pre-allocated pool       | `std::array<Buffer, kPoolSize> pool`           |
| Empty queue              | `IndexQueue empty`                             |
| Filled queue             | `IndexQueue filled`                            |
| "Hand buffer to producer"| `empty.push(i)`                                |
| "Hand buffer to consumer"| `filled.push(i)`                               |
| Filling in place         | `pool[i].pixels[k] = …`                        |
| Reading in place         | `for (auto px : pool[i].pixels) …`             |

Run it and notice: only `kPoolSize` distinct pointers appear in the log, recycled across all frames. That recycling **is** zero-copy.

## How V4L2 implements the same pattern

V4L2 ([v4l2_zero_copy_capture.cpp](v4l2_zero_copy_capture.cpp)) uses kernel ioctls instead of C++ threads, but the structure lines up one-to-one:

| Concept                  | V4L2 call                          |
| ------------------------ | ---------------------------------- |
| Pre-allocate pool        | `VIDIOC_REQBUFS` + `mmap` per buf  |
| "Hand buffer to producer"| `VIDIOC_QBUF` (give to driver)     |
| "Hand buffer to consumer"| `VIDIOC_DQBUF` (driver gives back) |
| Start streaming          | `VIDIOC_STREAMON`                  |

The "producer" is the kernel driver writing DMA from the camera straight into the `mmap`'d memory. The "consumer" is your program. Because the buffer is `mmap`'d (`MAP_SHARED`), kernel and userspace see the same physical pages — that's where the kernel→user copy disappears.

## When zero-copy stops being free

- **Cache effects.** If the producer wrote on one CPU and the consumer reads on another, the cache lines still have to migrate. Cheaper than a copy, not free.
- **Format conversion.** If you need RGB but the camera gives YUYV, *something* has to read every pixel. Zero-copy gets the bytes in front of you cheaply; it doesn't eliminate the conversion itself.
- **Lifetime bugs.** The consumer must finish with `pool[i]` *before* it returns to the empty queue. The borrow model is implicit. In V4L2 this becomes "don't read the pointer after you `QBUF` it back" — a common bug.
- **Backpressure.** If the consumer is slower than the producer, the producer eventually blocks waiting for an empty buffer. That is correct behavior — the alternative is dropping frames or unbounded memory growth — but it means the slowest stage sets the rate.

## Build & run

Both targets are wired into the top-level `CMakeLists.txt`. After a normal build:

```
./build/zero_copy_concept
./build/v4l2_zero_copy_capture          # /dev/video0, 30 frames
./build/v4l2_zero_copy_capture /dev/video1 60
```

The V4L2 binary is Linux-only; the conceptual demo is portable.
