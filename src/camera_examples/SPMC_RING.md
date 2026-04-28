# SPMC Ring Buffer (Disruptor Style)

Source: [spmc_ring.cpp](spmc_ring.cpp)

## Why a ring buffer when we already have [spmc.cpp](spmc.cpp)?

[spmc.cpp](spmc.cpp) gives every consumer its own queue. A slow consumer
fills its queue, the producer drops new frames into it, and other consumers
are unaffected. That's the right pattern when each consumer has its own
backlog policy.

For sensor fan-out — perception, visualizer, logger reading the same camera
stream — you usually want something different:

- **No backlog.** A consumer that wakes up after a stall should see the
  *latest* frame, not a queue of stale ones.
- **No locks.** At 60 fps the producer should not contend with consumers
  on a mutex; tens of microseconds per `futex` wakeup is wasted budget.
- **Single buffer.** N consumers reading the same frame should not cost
  N copies in N queues.

The Disruptor-style SPMC ring solves all three by using a **single fixed
array** and **per-consumer cursors**.

## The pattern

```
   ┌────┬────┬────┬────┬────┬────┬────┬────┐
   │  0 │  1 │  2 │  3 │  4 │  5 │  6 │  7 │   N=8 slots, indexed by seq % N
   └────┴────┴────┴────┴────┴────┴────┴────┘
            ▲                    ▲
            │                    │
       cursor=2              write_=5
       (slow consumer)       (producer's next slot)
```

- Producer writes to `slots[write_ % N]`, then atomically bumps `write_`.
- Each consumer keeps its own `cursor`. To read it does:
    - load `write_` (acquire),
    - if `write_ - cursor > N` it's been lapped → set `cursor = write_ - N`,
    - read `slots[cursor % N]`, increment `cursor`.

Consumers are **completely decoupled**. Their cursors live in their own
threads. They don't write any shared state, so they don't synchronize with
each other at all.

## The release/acquire handshake

```cpp
// producer
slots_[w & (N - 1)] = std::move(v);
write_.store(w + 1, std::memory_order_release);   // ① publish

// consumer
const auto w = write_.load(std::memory_order_acquire);   // ② observe
if (cursor >= w) return false;
out = slots_[cursor & (N - 1)];
```

The release at ① pairs with the acquire at ②. Any consumer that observes
`write_ ≥ cursor + 1` is guaranteed to see the slot store that the producer
made *before* publishing — that's the only memory-ordering rule the example
needs.

## Lap detection

```cpp
if (w - cursor > N) cursor = w - N;
```

If the producer is more than N slots ahead of the consumer, the consumer's
intended slot has already been overwritten. It snaps `cursor` forward to
`w - N` — the oldest slot still in the ring. The example flags this as
`LAPPED` when the cursor jumps by more than one.

For sensors this is exactly right: "you fell behind, here's the freshest
data we still have."

## What the simple version trades away

The educational ring above is not a production Disruptor. The differences:

- **Slot tearing window.** When a consumer lags by *almost* N, the
  producer can begin overwriting the slot the consumer is mid-read. A
  real Disruptor stores a sequence number alongside the value and
  re-checks it after the read. For trivially-copyable T (`uint64_t`)
  on x86, the read is one atomic word so tearing is effectively
  invisible — but don't put a 256-byte struct in here without fixing
  this.
- **Single producer only.** With multiple producers you'd need a CAS
  loop on `write_` plus a per-slot "ready" sequence. The whole point
  of *S*-PMC is that we get to skip both.
- **No back-pressure for must-not-drop consumers.** A logger that must
  see every frame doesn't belong on this ring — give it a separate
  bounded queue (back to the [spmc.cpp](spmc.cpp) pattern).

## Mapping to the code

| Concept                          | In [spmc_ring.cpp](spmc_ring.cpp)                |
| -------------------------------- | ------------------------------------------------ |
| Fixed slot array                 | `std::array<T, N> slots_`                        |
| Producer's write seq             | `std::atomic<uint64_t> write_`                   |
| Power-of-two for cheap modulo    | `static_assert((N & (N - 1)) == 0)` + `& (N-1)`  |
| Publish                          | `slots_[w & (N-1)] = v; write_.store(w+1, release)` |
| Per-consumer cursor              | local `uint64_t cursor` in each consumer thread  |
| Lap detection                    | `if (w - cursor > N) cursor = w - N;`            |
| Lap *visibility* in the demo     | `(cursor - before) > 1` after `try_consume`      |

## When to pick which fan-out

| Situation                                                          | Use                              |
| ------------------------------------------------------------------ | -------------------------------- |
| Independent, possibly different queue depths per consumer           | [spmc.cpp](spmc.cpp) (per-consumer queue) |
| Many consumers want freshest sensor frame, drops are acceptable     | [spmc_ring.cpp](spmc_ring.cpp) (this) |
| One must-not-drop consumer (logger) alongside drop-OK consumers     | Both — ring for the realtime ones, separate bounded queue for the logger |

## Build & run

Wired into the top-level `CMakeLists.txt`:

```
./build/spmc_ring
```

In the output you'll see the fast consumer reach 60/60 while the slow one
reports several `LAPPED` events and a final count well below 60 — exactly
the overwrite-oldest behavior the design wants.
