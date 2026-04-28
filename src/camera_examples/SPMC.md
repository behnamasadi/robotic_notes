# Single-Producer Multi-Consumer (SPMC) Frame Fan-Out

Source: [spmc.cpp](spmc.cpp)

## The problem

One camera produces frames at a steady rate (say 30 fps). Several independent
parts of your program want **every** frame:

- a logger that records frames to disk,
- a processor that runs detection on them,
- a visualizer that renders them on screen,
- …

These consumers run at very different speeds. The detector might take 50 ms per
frame, the logger 1 ms, the visualizer 20 ms. You want three things at once:

1. **Every consumer sees every frame** (broadcast, not work-stealing).
2. **A slow consumer does not stall the others** — and does not stall the camera.
3. **No unbounded memory growth** if a consumer falls behind.

A naive design — one shared queue with multiple readers — fails (1): a frame
popped by the logger never reaches the processor.

## The pattern

**Give each consumer its own bounded queue. The producer fans out by pushing
the same frame onto every queue.**

```
              ┌──── logger queue ────► logger thread
camera ──────►├──── processor queue ─► processor thread
              └──── visualizer queue ─► visualizer thread
```

That is the entire idea. Three small details make it work in practice:

### 1. `shared_ptr<const Frame>` for cheap fan-out

The frame is built **once** in a `shared_ptr` and the same `shared_ptr` is
pushed onto every consumer's queue. What gets duplicated is the 16-byte control
block — not the pixels. The frame is freed automatically when the last consumer
is done with it.

`const Frame` because no consumer should mutate a frame other consumers can see.

### 2. Bounded queues + drop-on-full

Each queue has a fixed capacity (4 in the demo). `try_push` returns `false`
when full, and the producer just moves on. That gives each consumer
**independent backpressure**:

| Consumer            | Falls behind → |
| ------------------- | -------------- |
| Slow processor      | Its queue fills, frames dropped *for it only*. |
| Fast logger         | Its queue stays near empty, sees every frame. |
| Camera (producer)   | Never blocks — `try_push` is non-blocking. |

The producer's only timing source is the camera clock, never a downstream
consumer.

### 3. Clean shutdown

Each queue carries a `stop_` flag set by `stop()`. `pop()` waits until either
a frame arrives **or** the queue is being shut down; once stopped and drained
it returns `nullopt`, the consumer's signal to exit. The producer calls
`stop()` on every queue after sending the last frame.

## Mapping to the code

| Concept                          | In [spmc.cpp](spmc.cpp)                           |
| -------------------------------- | ------------------------------------------------- |
| Per-consumer bounded queue       | `class FrameQueue` (`deque` + mutex + cv)         |
| Drop-on-full                     | `FrameQueue::try_push` returns `false`            |
| Cheap broadcast                  | `std::shared_ptr<const Frame>`                    |
| Fan-out                          | `for (auto& q : queues) q->try_push(f);`          |
| Shutdown signal                  | `FrameQueue::stop()` + `pop()` returning nullopt  |

There is no Broker class. The list of queues is just a `std::vector` that
`main` builds and hands to the producer. Adding a consumer is two lines:
push another queue, spawn another thread.

## Why no Broker

The Broker version wraps the vector of queues in a class with `subscribe()` /
`publish()` / `shutdown()`. That's nicer if subscribers come and go at runtime
or if you want to hide the queue list from the producer. For a fixed,
known-at-startup set of consumers it's pure overhead — the producer already
knows the queues, and the indirection adds nothing. Dropping it makes the data
flow visible at a glance.

If later you need dynamic subscribe/unsubscribe, that's the moment to put a
Broker back.

## When SPMC is the wrong shape

- **Work-stealing** — if consumers are interchangeable workers and only **one**
  needs to handle each frame, use a single shared queue (MPMC), not SPMC.
- **Strict no-drop guarantees** — if every consumer must see every frame even
  under sustained slowness, switch from drop-on-full to blocking push (which
  re-couples producer rate to the slowest consumer) or grow the queue. Pick
  one of those costs deliberately; you can't avoid both.
- **Per-frame mutation** — `shared_ptr<const Frame>` only works if frames are
  read-only post-publication. If a consumer needs to write, hand it a copy or
  a separate buffer.

## Build & run

Wired into the top-level `CMakeLists.txt`:

```
./build/spmc
```

Watch the output: `logger` and `visualizer` see every frame; `processor`
prints a sparse subset because its queue keeps filling up — exactly what you
want a slow consumer to do.
