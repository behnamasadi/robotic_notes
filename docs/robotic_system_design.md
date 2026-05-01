# Robotic System Design & Design Patterns

This note collects the architectural patterns, concurrency idioms, and runtime
trade-offs that come up over and over when building a real robot stack
(humanoid, mobile manipulator, drone, or AV). It is written from a systems
engineering point of view — i.e. *how* the parts fit together and *why*, not
*which* algorithm runs in each box.

- [1. High-Level Architecture of a Humanoid Robot](#1-high-level-architecture-of-a-humanoid-robot)
- [2. Perception Pipeline: Consistency, Robustness, Synchronization](#2-perception-pipeline-consistency-robustness-synchronization)
- [3. High-Frequency Sensor Pipelines in C++](#3-high-frequency-sensor-pipelines-in-c)
- [4. Concurrency & Real-Time: SPMC Sensor Distribution](#4-concurrency--real-time-spmc-sensor-distribution)
- [5. Task Execution: State Machine vs Behavior Tree](#5-task-execution-state-machine-vs-behavior-tree)
- [6. Debugging a Regressing Perception Model in Production](#6-debugging-a-regressing-perception-model-in-production)
- [7. Time Synchronization](#7-time-synchronization)
- [8. Safety & Fault Tolerance](#8-safety--fault-tolerance)
- [9. Logging, Observability & Replay](#9-logging-observability--replay)

---

## 1. High-Level Architecture of a Humanoid Robot

A robust humanoid stack is layered. Each layer runs at its own rate, exposes
a narrow interface, and communicates with the others through well-defined
data contracts (typed messages + timestamps + frame ids).

```
                           ┌────────────────────────────────────┐
                           │      Mission / Task Layer          │   0.1–10 Hz
                           │  (BT/HTN, skills, recovery, HMI)   │
                           └───────────────▲────────────────────┘
                                           │ goals / skills
                           ┌───────────────┴────────────────────┐
                           │           Planning Layer           │   1–50 Hz
                           │  global path • footstep • grasp    │
                           │  motion plan • trajectory opt      │
                           └───────────────▲────────────────────┘
                                           │ trajectories / setpoints
                           ┌───────────────┴────────────────────┐
                           │            World Model             │   10–200 Hz
                           │  TF tree • semantic map • objects  │
                           │  occupancy • robot state estimate  │
                           └───────▲────────────────────▲───────┘
                                   │ percepts            │ state
                ┌──────────────────┴─────┐    ┌──────────┴────────────┐
                │     Perception          │   │  State Estimation      │  100–500 Hz
                │ detect • track • seg    │   │  IMU + kin + VIO/LIO   │
                │ pose • affordance       │   │  contact, COM, ZMP     │
                └──────────────────▲──────┘   └──────────▲─────────────┘
                                   │                     │
                                   │                     │ joint state, IMU
                           ┌───────┴─────────────────────┴──────┐
                           │            Control Layer           │   500 Hz – 2 kHz
                           │  whole-body QP • impedance • grav  │
                           │  comp • balance • joint torque     │
                           └───────────────▲────────────────────┘
                                           │ torques / currents
                           ┌───────────────┴────────────────────┐
                           │     Drivers / HAL / EtherCAT       │   1–4 kHz
                           └────────────────────────────────────┘
```

**Design principles**

- **Rate decoupling.** Slower layers must never block faster ones. Control
  reads the *latest* world-model snapshot; it does not wait for perception.
- **Immutable snapshots.** The world model publishes versioned snapshots
  (`shared_ptr<const WorldState>`), so a planner can reason about a
  consistent view while perception keeps writing new ones.
- **One source of truth per quantity.** Robot pose lives in state estimation,
  object poses live in the world model — never duplicated, only referenced.
- **Time everywhere.** Every message carries `(stamp, frame_id, seq)`.
  No layer is allowed to use wall-clock or "now"; time is a first-class input
  so the stack is replayable from a log.
- **Graceful degradation.** Each layer has a *safe fallback* if its input
  goes stale: control freezes the joint setpoint, planner aborts to a hold,
  mission goes to "safe stop". This is what keeps a humanoid standing when
  perception crashes.
- **Hard real-time only where it has to be.** Control + drivers are RT
  (Xenomai / PREEMPT_RT, lock-free, no allocation). Perception, planning
  and mission are soft RT and run on standard Linux.

---

## 2. Perception Pipeline: Consistency, Robustness, Synchronization

Per-frame detections are not enough. A downstream planner needs *the same
object* across frames, with a *believable pose*, and timestamps it can
relate to the control loop. Three orthogonal problems:

### 2.1 Consistency over time — tracking

- **Detection → Track association.** Per-frame detections (`{class, bbox,
  pose, confidence}`) are matched to existing tracks via the **Hungarian
  algorithm** on a cost matrix. Costs combine geometric distance
  (Mahalanobis on the predicted state), IoU (boxes), and appearance
  (re-id embedding cosine).
- **Per-track filter.** Each track owns a small **EKF / UKF** in SE(3)
  that predicts pose between detections and updates on association. This
  gives sub-frame interpolation and a covariance the planner can trust.
- **Track lifecycle.** `tentative → confirmed → coasting → deleted`. A
  track only goes to `confirmed` after N consecutive hits, and is deleted
  after M consecutive misses. This kills detector flicker.
- **Stable IDs.** Downstream code (planner, BT) keys on `track_id`, never
  on `detection_index`. ID switches must be measurable and rare.

### 2.2 Robustness to noise

- **Gating before association.** Reject any detection-track pair whose
  Mahalanobis distance exceeds χ² threshold — avoids associating a noisy
  blip to a real track.
- **Multi-hypothesis on hard scenes.** When the gate passes >1 candidate,
  keep both hypotheses (MHT / JPDA) instead of committing greedily.
- **Sensor fusion.** Fuse RGB-D depth with stereo, or camera with LiDAR,
  at the *measurement* level (not the *decision* level) — the EKF update
  becomes the natural fusion point.
- **Confidence-weighted updates.** `R = R0 / detector_confidence` makes
  the filter quietly distrust low-confidence detections without hard-
  thresholding (which causes drop-outs).
- **Outlier rejection.** RANSAC for pose estimation; for trackers,
  gating + chi-square innovation tests.
- **Temporal smoothing.** Always smooth the *state* (filter), never the
  *output* (low-pass on poses) — low-pass adds lag with no covariance.

### 2.3 Synchronization with the control loop

- **Stamp at capture, not at receive.** The driver stamps the message at
  the hardware trigger / shutter-mid time. Anything else accumulates jitter.
- **TF tree with time queries.** `tf2::Buffer::lookupTransform(target, source,
  stamp)` so the planner asks "where was the gripper *when this image was
  taken*", not "now". Required for any moving-base manipulation.
- **Approximate-time sync** (e.g. ROS 2 `message_filters::ApproximateTime`)
  for multi-camera or camera+depth.
- **Latency compensation.** Perception is slow (10–100 ms). The control
  loop should consume `(pose, stamp)`, predict forward to *its* current
  time using the EKF, and act on the predicted pose.
- **Bounded staleness.** The control loop refuses to act on percepts older
  than `T_max` (typically 2–3× the expected perception period) and falls
  back to the last safe behavior. This is how you stop a humanoid from
  reaching toward an object that disappeared 500 ms ago.
- **Double-buffer the world model.** Perception writes to a back buffer;
  on commit, an atomic pointer swap exposes it to readers. Readers never
  block writers and vice versa.

---

## 3. High-Frequency Sensor Pipelines in C++

For a 60 FPS camera (16.6 ms/frame, ~6 MB at 1080p RGB) the dominant cost
is *memory traffic*, not CPU. The patterns below are what make the
difference between "works on a laptop" and "works on the robot".

### 3.1 Avoid memory copies

- **One allocation, many readers.** Wrap the frame in a
  `std::shared_ptr<const Frame>` the moment the driver hands it over.
  Every consumer holds a `shared_ptr`; the buffer is freed (or returned
  to a pool) when the last reader drops it.

  ```cpp
  struct Frame {
      uint64_t            stamp_ns;
      uint64_t            seq;
      int                 width, height, stride;
      cv::Mat             image;          // header only, refcounts pixels
      // pixel buffer lives in a pool, not on the heap per-frame
  };
  using FramePtr = std::shared_ptr<const Frame>;
  ```

- **Memory pool, not `new` on the hot path.** Pre-allocate N frame buffers
  at startup (`boost::lockfree::stack<Buffer*>` or a custom slab). The
  driver pops a buffer, fills it, wraps it in a `shared_ptr` with a
  custom deleter that returns it to the pool.

- **Zero-copy at boundaries.** Use ROS 2 **intra-process comms**
  (`rclcpp::IntraProcessSetting::Enable`) so messages between nodes in the
  same process pass as `unique_ptr` / `shared_ptr` — no serialize/deserialize.
  Across processes use **shared memory** transports (Iceoryx, Cyclone DDS
  loan API, FastDDS shared memory) or `mmap`'d ring buffers.

- **DMA / GPU-mapped buffers.** With V4L2 use `MMAP` / `DMABUF` so the
  kernel hands you a pointer to the ISP buffer directly. Pass the same
  buffer to CUDA via `cudaHostRegister` or `cuMemImportFromShareableHandle`
  to avoid host→device copies.

- **Views, not slices.** Pass `cv::Mat` ROIs, `std::span<const uint8_t>`,
  or `Eigen::Map`. Anything that allocates is suspicious in the hot path.

### 3.2 Reduce latency

- **Pre-allocate everything.** No `new`/`delete`, no `std::vector::resize`,
  no `std::string` operations on the hot path. Reserve at startup.
- **Single-producer single-consumer (SPSC) ring buffers** between stages —
  lock-free, wait-free, no syscalls. `boost::lockfree::spsc_queue` or
  `folly::ProducerConsumerQueue`.
- **Pin threads & set RT priority.** `pthread_setaffinity_np` to isolate
  the capture thread on a dedicated core; `SCHED_FIFO` priority for the
  control thread; isolate those cores via `isolcpus` / `cset`.
- **Drop, don't queue.** If the consumer falls behind, *drop the oldest
  frame* (overwrite the slot). A growing queue is just stored latency.
- **Pipeline, don't batch.** A 4-stage pipeline running at 60 FPS each
  delivers a result every 16 ms with ~64 ms end-to-end; a batched
  60×4-frame pass gives the same throughput at 4× the latency.
- **Avoid syscalls.** No `printf`, no `std::cout`, no `malloc`, no
  un-pinned mutexes, no `std::chrono::system_clock` in the hot loop.
  Use a non-blocking ring-buffer logger (e.g. spdlog async, or a custom
  SPSC trace ring).
- **NUMA awareness.** Allocate the frame buffer on the same NUMA node as
  the consumer thread (`numactl`, `mbind`).

### 3.3 Thread safety

- **Immutability is the cheapest synchronization.** Once the producer
  publishes a `shared_ptr<const Frame>`, the data is immutable. Readers
  need no lock to read pixels.
- **Atomic publication.** Publish with `std::atomic<std::shared_ptr<T>>::
  store(..., memory_order_release)` (C++20) or with a hand-rolled hazard
  pointer / RCU. Readers `load(memory_order_acquire)`.
- **Double-buffer when sharing a mutable buffer is unavoidable** (e.g.
  per-frame depth map): producer writes to B, atomic-swaps the pointer,
  consumers read from A.
- **Never hold a lock across an I/O or wait.** That's the canonical
  source of priority inversion and missed control deadlines. Use
  `std::scoped_lock` on the *smallest possible* critical section.

A runnable end-to-end zero-copy capture pipeline — plain-C++
pool/producer/consumer first, then the same pattern over V4L2 `mmap` —
is documented in [Zero-Copy Camera Capture](../src/camera_examples/README.md).

---

## 4. Concurrency & Real-Time: SPMC Sensor Distribution

Classic single-producer, multiple-consumer (SPMC): the camera/IMU thread
produces, while perception, logger, and visualizer consume. The right
primitive depends on what each consumer is allowed to lose and how late
it can be.

| Primitive                       | Latency | Throughput | Backpressure | Loss policy            | When to use                                    |
|---------------------------------|---------|------------|--------------|------------------------|------------------------------------------------|
| `mutex` + `condition_variable`  | µs–ms   | low        | yes          | none (blocks producer) | low-rate config / control-plane messages       |
| Lock-free MPMC queue (`moodycamel`) | sub-µs | high      | yes (bounded)| caller decides         | many producers, many consumers, soft RT        |
| **SPMC ring buffer (Disruptor)**| ns–µs   | very high  | optional     | overwrite oldest       | **the right default for sensor fan-out**       |
| Per-consumer SPSC ring          | ns      | very high  | per consumer | independent per reader | when each consumer has different pacing needs  |

### Recommended pattern

Use a **bounded SPMC ring buffer** (LMAX Disruptor style) for the sensor
stream. The producer writes into slot `seq % N` and publishes `seq` via
`std::atomic<uint64_t>::store(release)`. Each consumer holds its own
read cursor; readers don't block each other and never block the producer.
If a consumer falls more than N slots behind, it sees a sequence gap and
*skips forward* to the latest — this is a feature, not a bug.

```cpp
template <class T, size_t N>   // N must be a power of two
class SpmcRing {
    static_assert((N & (N - 1)) == 0);
    std::array<T, N>          slots_;
    std::atomic<uint64_t>     write_{0};   // next slot to write

public:
    void publish(T v) {
        const auto w = write_.load(std::memory_order_relaxed);
        slots_[w & (N - 1)] = std::move(v);
        write_.store(w + 1, std::memory_order_release);   // visible to readers
    }

    // Each consumer keeps its own `cursor` (their last-read seq).
    // Returns false if there's nothing new; rewinds cursor if it lapped.
    bool try_consume(uint64_t& cursor, T& out) {
        const auto w = write_.load(std::memory_order_acquire);
        if (cursor >= w) return false;
        if (w - cursor > N) cursor = w - N;          // we got lapped: skip
        out = slots_[cursor & (N - 1)];
        ++cursor;
        return true;
    }
};
```

**Per-consumer pacing.**

- **Perception** (must be fresh): sets `cursor = write_ - 1` on each tick
  → "give me the latest", drops old frames silently.
- **Logger** (must be lossless): subscribes via a *separate* unbounded
  MPSC queue (or writes to disk through a bounded queue + back-pressure
  logging "X frames dropped"). Don't put the logger on the realtime ring.
- **Visualizer** (best-effort): same overwrite-oldest semantics as
  perception. It's UX, not safety.

**Why a mutex+condvar is wrong here.** A 60 FPS producer would wake all
consumers every 16 ms; the slowest consumer (visualizer doing GPU
upload) would block the others through priority inversion. Even with
`SCHED_FIFO`, the futex syscall is in the tens of microseconds — fine
for control-plane messages, wasteful per frame.

Two implementations of the SPMC fan-out pattern are documented and
runnable:

- [Per-consumer queue SPMC](../src/camera_examples/SPMC.md) — simpler
  baseline with one bounded queue per consumer; each consumer paces
  itself independently.
- [Disruptor-style SPMC ring buffer](../src/camera_examples/SPMC_RING.md)
  — single shared ring, lock-free, overwrite-oldest semantics. The right
  default for high-rate sensor streams.

---

## 5. Task Execution: State Machine vs Behavior Tree

For "*Pick up object A and place it on table B*":

### Option A: Hierarchical Finite State Machine (HFSM)

```
IDLE → PERCEIVE_A → APPROACH_A → GRASP_A → LIFT
     → APPROACH_B → ALIGN → PLACE → RETREAT → DONE
                                                 ↘ (any state) → ERROR → RECOVER
```

- ✅ Simple, deterministic, easy to draw on a whiteboard.
- ✅ Clear "current state" — easy to log, replay, debug.
- ❌ Recovery edges multiply combinatorially; every state needs a
  transition for every failure mode.
- ❌ Reactivity is bolted on (you have to add interrupt edges).
- ❌ Hard to reuse skills across tasks — each FSM is bespoke.

### Option B: Behavior Tree (BT)

```
Sequence(
  Fallback(IsObjectKnown(A), Action(PerceiveScene)),
  Sequence(
    Action(MoveTo, near(A)),
    Fallback(Action(Grasp, A), Sequence(Action(Replan), Action(Grasp, A))),
    Action(Lift, A),
    Action(MoveTo, near(B)),
    Action(Place, on(B))
  )
)
```

- ✅ **Reactive by construction** — the tree is re-ticked at e.g. 20 Hz;
  if a precondition (e.g. "object visible") becomes false, the current
  node returns FAILURE and the tree picks a recovery branch automatically.
- ✅ **Composable.** `Grasp`, `MoveTo`, `Place` are leaves you reuse for
  any task ("stack three blocks" is a different tree over the same leaves).
- ✅ **Recovery as a first-class pattern** — `Fallback(do, recover)`.
- ❌ Reasoning about "where am I in the task" is harder than in a FSM —
  the tree's runtime state is implicit in node statuses.
- ❌ Trees grow deep and unreadable without a good editor (Groot, etc.).

### Option C: HTN / Skill graph

Plan symbolically (PDDL / HTN) over typed skills, expand to a BT or
sequence at runtime. The right call for *long-horizon, varied* tasks
(household robot). Overkill for one pick-and-place.

### Recommendation

For a humanoid pick-and-place: **BT at the top, FSM inside leaves.**

- The BT handles task-level structure, preconditions, and recovery
  ("if grasp fails, replan; if object is gone, abort") — that's where
  reactivity matters.
- Each leaf (e.g. `Grasp`) is a small FSM with deterministic phases
  (`PRE_GRASP → CLOSE → CONTACT → CONFIRM`) — that's where you want
  a clear, debuggable state.

Avoid putting motion-level state in the BT (too much ticking) or
task-level recovery in an FSM (too many edges). The split scales.

---

## 6. Debugging a Regressing Perception Model in Production

Symptom after deploying a new perception model: CPU spikes, latency rises,
robot becomes unstable. Work the problem in this order — **bisect first,
optimize last**.

### Step 0 — Make the robot safe

1. Switch to a **safe behavior** (hold position, lower the arms, exit
   autonomy). Don't debug on a robot that's about to fall.
2. Flip the rollback flag — revert the perception node to the previous
   model image. Confirm the symptom disappears with the old model.
   This isolates the regression to the model change vs an environment
   change.

### Step 1 — Reproduce off-robot

3. Replay the **rosbag** that triggered the instability through the new
   model on a workstation. If you can't reproduce, the issue is
   environmental (thermals, contention from another process, GPU shared
   with display, etc.) — go check `nvidia-smi`, `htop`, `dmesg`.

### Step 2 — Measure, don't guess

4. **End-to-end latency**: instrument `t_capture → t_detection →
   t_world_model → t_control_consumed`. Plot histograms, not means;
   the tail is what causes instability.
5. **CPU/GPU**: `perf top`, `perf record -g`, `nsys profile`,
   `nvprof` / Nsight Systems. Look for: a kernel that grew, NMS on CPU,
   a copy you didn't expect, a thread that's now spinning.
6. **Memory**: `/proc/<pid>/status` RSS, `pmap`, `valgrind --tool=massif`.
   A new model often allocates per-frame (e.g. NumPy arrays in Python pre-
   /post-processing). Allocations on the hot path cause both latency and
   jitter.
7. **Thread starvation**: `pidstat -t`, `htop` per-thread CPU. If the
   capture thread or control thread isn't getting its core, the new model
   is starving them — fix priorities/affinity, not the model.

### Step 3 — Identify the class of regression

Likely one of:

| Symptom                                  | Likely cause                                    | Fix                                                                   |
|------------------------------------------|-------------------------------------------------|-----------------------------------------------------------------------|
| CPU saturated, GPU idle                  | pre/post-processing on CPU                      | move to GPU, fuse with model, use TensorRT plugins                    |
| GPU saturated                            | bigger model / no quantization                  | INT8/FP16, prune, distill, or downscale input                         |
| Latency tail grows, mean fine            | per-frame allocations / GC pauses               | pre-allocate, pool tensors, switch to C++ inference                   |
| Latency *and* mean grow                  | model genuinely heavier                         | smaller backbone or run at lower rate + temporal interpolation        |
| CPU fine, control still unstable         | perception thread starves control via priority  | pin & raise control priority; isolate perception on other cores       |
| Unstable only sometimes                  | thermal throttling / power limit                | check `thermal_zone*`, `nvidia-smi -q -d POWER`                       |

### Step 4 — Decouple, then optimize

8. **Bound the blast radius.** Put the model behind a bounded SPMC ring
   (see §4) so it cannot back-pressure capture or control. Drop frames
   when it falls behind. A robot that uses 30 FPS perception stably is
   better than one that uses 60 FPS perception and falls over.
9. **Latency compensation.** If the model now takes 80 ms instead of
   30 ms, push the predicted-pose forward by 50 ms more in the EKF (§2.3).
   Often this alone restores stability without touching the model.
10. **Then** optimize the model: TensorRT / ONNX Runtime, FP16/INT8,
    fused pre-processing, batch=1, CUDA graphs.

### Step 5 — Land it safely

11. **Canary on one robot** with a watchdog: if end-to-end latency
    p99 > threshold for N seconds, auto-rollback to previous model.
12. **Add the regression to CI.** Replay the offending rosbag in CI
    and assert latency p50/p99 + CPU + memory stays under budget.
    The next model change will trip the same wire.

The meta-lesson: **a perception regression on a robot is almost never a
pure ML problem.** It is a systems problem (latency, scheduling,
allocation, decoupling) wearing an ML hat. Fix the system first, then
the model.

---

## 7. Time Synchronization

Time is the silent killer of robot stacks. A control loop fed with a pose
stamped 80 ms in the future from a sensor whose clock has drifted 30 ms
relative to the host will misbehave in ways no log can explain. Get the
clocks right *before* you debug anything else.

### 7.1 Three clocks, three jobs

| Clock                           | Source                          | Use it for                                          | Don't use it for                          |
|---------------------------------|---------------------------------|-----------------------------------------------------|-------------------------------------------|
| **Monotonic / steady**          | `CLOCK_MONOTONIC`, `steady_clock` | measuring durations, timeouts, watchdogs, profiling | tagging messages, comparing across hosts  |
| **Wall / system**               | `CLOCK_REALTIME`, NTP/PTP-disciplined | message stamps, log timestamps, cross-host events   | measuring durations (it can jump backward!) |
| **Sim / replay (ROS time)**     | `/clock` topic in ROS, `--clock` from rosbag2 | anything inside the ROS graph during replay         | wall-clock-only operations (`sleep_for`)  |

The monotonic clock never goes backward and is unaffected by NTP/PTP
adjustments. **Always** use it for `auto t0 = steady_clock::now(); ...; auto
dt = steady_clock::now() - t0;` patterns. Using `system_clock` for that has
caused real outages when NTP stepped the clock during a measurement.

### 7.2 Within a single machine: hardware sync

- **Capture-time stamping.** The driver must stamp the message at the
  hardware trigger (or shutter midpoint), not at the moment the kernel
  hands the frame to userspace. The latter accumulates 5–30 ms of
  jitter from USB/MIPI/PCIe queuing and scheduler delay.
- **Hardware-trigger camera + IMU.** If the camera and IMU share a
  trigger line (or the IMU is sampled on the camera's strobe), the
  IMU stamp can be aligned to the image stamp with sub-microsecond
  accuracy. This is what VIO needs to converge cleanly.
- **GPU/PTP-aware drivers.** V4L2 with `V4L2_BUF_FLAG_TIMESTAMP_COPY`
  or `_MONOTONIC` propagates the kernel timestamp; for industrial
  cameras (GenICam) use the on-camera PTP timestamp.

```cpp
// Hot-path duration measurement — always monotonic.
const auto t0 = std::chrono::steady_clock::now();
do_work();
const auto dt_us = std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::steady_clock::now() - t0).count();

// Message stamp — wall clock, set at capture not at receive.
msg.header.stamp = driver.last_trigger_time();   // NOT now()
```

### 7.3 Across machines: PTP

NTP gives you ~1–10 ms across a LAN — fine for logging, **not** fine for
sensor fusion. Use **PTP (IEEE 1588)** with hardware timestamping
(`SO_TIMESTAMPING`) and a boundary clock on the switch:

- Sub-microsecond synchronization across NICs that support it.
- `linuxptp` (`ptp4l` + `phc2sys`) is the standard stack on Linux.
- For automotive / AVB networks: **gPTP (802.1AS)**.
- Verify with `pmc -u -b 0 'GET CURRENT_DATA_SET'` — `offsetFromMaster`
  should be in the hundreds of nanoseconds, not milliseconds.

If you can't deploy PTP (e.g. WiFi link), fall back to **chrony**
(better than `ntpd` for jittery links) and *widen your sync tolerance
in the EKF* — don't pretend the clocks are tight when they aren't.

### 7.4 Common time pitfalls

- **`ros::Time::now()` in a callback** when the message has a perfectly
  good `header.stamp`. You just threw away your sensor's timing.
- **Mixing clock sources in an EKF** — IMU on monotonic, camera on
  PTP, controller on wall clock. The estimator sees apparent jumps
  that look like sensor faults. Pick one timeline (usually wall/PTP)
  and convert at the boundary.
- **Forgetting `--clock` on rosbag2 replay.** Nodes use wall time,
  see "stale" data, and start dropping it. Always run replay with the
  `/clock` publisher and `use_sim_time:=true`.
- **Daylight saving / leap seconds in logs.** Log in UTC, render in
  local time. A 1-hour discontinuity at 02:00 is not a bug in your
  code.
- **`sleep_for` after a long callback** — the deadline has already
  passed. Use absolute deadlines (`sleep_until(next_tick)`), not
  relative sleeps, for periodic work.

---

## 8. Safety & Fault Tolerance

A humanoid is a 50 kg machine standing next to humans. The interesting
question is not "how do I make it work?" but "how does it fail?" — and
specifically, **does it fail in a way that doesn't hurt anyone or damage
itself?** Design for failure first.

### 8.1 Defense in depth

Safety is layered. Each layer assumes the one above it can fail:

```
┌────────────────────────────────────────────────────────────────┐
│ L5  Operator E-stop (button, wireless deadman)                 │ <- human
├────────────────────────────────────────────────────────────────┤
│ L4  Mission/BT safety policies (geofence, force limits)        │ <- soft RT
├────────────────────────────────────────────────────────────────┤
│ L3  Application watchdogs (perception alive, planner alive)    │ <- soft RT
├────────────────────────────────────────────────────────────────┤
│ L2  Control-layer limits (joint, torque, velocity, jerk)       │ <- hard RT
├────────────────────────────────────────────────────────────────┤
│ L1  Driver / firmware safety (current limits, thermals)        │ <- µC
├────────────────────────────────────────────────────────────────┤
│ L0  Hardware (mechanical brakes, hardware E-stop loop, fuses)  │ <- physics
└────────────────────────────────────────────────────────────────┘
```

The crucial property: **a failure at level N is contained by level N-1.**
If the planner publishes an absurd setpoint, the controller clips it. If
the controller goes silent, the firmware decays current. If the firmware
hangs, the hardware E-stop cuts power and the brakes engage.

### 8.2 Fail-operational vs fail-safe

| System          | "Safe" means                          | Why                                           |
|-----------------|---------------------------------------|-----------------------------------------------|
| Wheeled AMR     | brake to stop                         | stopping is genuinely safe                    |
| Drone           | controlled descent / RTL              | falling out of the sky is *not* safe — must keep flying |
| Manipulator arm | freeze in place, gravity-comp on      | dropping the payload may be safe, free-falling joints isn't |
| **Humanoid**    | **controlled fall** (tuck, protect)   | freezing mid-step also falls — but uncontrolled |

The interesting case is the humanoid: there is no "off" that's safe.
"Safe" is a *behavior*, not a state. The safety controller is a real
controller, not just `set_torque(0)`. Budget for it from day one.

### 8.3 Heartbeats and watchdogs

Every subsystem publishes a heartbeat (alive token + sequence + stamp +
self-reported health) at a known rate. A central **safety supervisor**
subscribes to all heartbeats and triggers a state transition if any goes
stale.

```cpp
// In each subsystem (perception, planner, control, ...):
void HeartbeatPublisher::tick() {
    Heartbeat hb{
        .node     = "perception",
        .seq      = ++seq_,
        .stamp_ns = wall_clock_ns(),
        .state    = self_check(),     // OK / DEGRADED / FAULT
        .latency_p99_ms = latency_window_.p99(),
    };
    pub_.publish(hb);
}

// In the supervisor (runs at e.g. 100 Hz, hard RT):
void Supervisor::tick() {
    const auto now = steady_clock::now();
    for (auto& [name, last] : heartbeats_) {
        if (now - last.recv_steady > kDeadline[name]) {
            transition_to(SafetyState::SAFE_HOLD,
                          /*reason=*/name + " heartbeat lost");
            return;
        }
        if (last.msg.state == NodeState::FAULT) {
            transition_to(SafetyState::SAFE_HOLD, name + " self-reported FAULT");
            return;
        }
    }
}
```

**Sizing the deadline.** Pick `kDeadline = 3 × expected_period`. Tighter
than that and you'll trip on a single GC pause; looser and the robot
keeps acting on a dead subsystem for too long. The supervisor itself
runs on a *separate process and CPU* so it can outlive a userspace
crash.

### 8.4 The safety state machine

```
            ┌──────────┐  any subsystem DEGRADED   ┌──────────┐
            │  NORMAL  │ ────────────────────────▶ │ DEGRADED │
            │ autonomy │                           │  reduced │
            │   on     │ ◀──── all OK for T_hys ── │  speed   │
            └────┬─────┘                           └────┬─────┘
                 │                                      │
   heartbeat     │                                      │ critical fault
   lost / op     ▼                                      ▼
            ┌──────────┐    operator confirm     ┌──────────┐
            │SAFE_HOLD │ ◀────────────────────── │  E_STOP  │
            │ stand /  │                         │ brakes,  │
            │ hover /  │                         │ power    │
            │ park     │                         │ removed  │
            └────┬─────┘                         └──────────┘
                 │ operator clear
                 ▼
              NORMAL
```

Important rules:

- **Transitions are one-way without operator action.** Going from
  `SAFE_HOLD` back to `NORMAL` requires a human (and a reason logged).
  Never auto-recover from a safety event silently.
- **Hysteresis on the way out.** Toggling `NORMAL ↔ DEGRADED` every few
  ticks because a metric oscillates is worse than staying degraded.
- **Each transition logs (state, reason, all_heartbeats_snapshot)**.
  Post-mortems are won or lost on this single log line.

### 8.5 Where redundancy actually helps

Redundancy is expensive. Apply it where the failure mode is "silent
wrong answer", not where it's "loud crash":

- **Sensors (silent drift).** Two IMUs, cross-check against kinematics;
  reject the disagreeing one.
- **Estimation (silent divergence).** Run a backup attitude filter
  (complementary) alongside the primary EKF; switch on innovation
  blow-up.
- **Compute (loud crash).** Don't bother with hot-standby userspace —
  rely on the watchdog + safety supervisor on a separate core/MCU.
- **Power & brakes.** Always redundant — single point of failure here
  is non-negotiable.

---

## 9. Logging, Observability & Replay

You will only ever debug your robot from logs. The robot will not
reproduce the bug at your desk. Invest in observability up front; the
ROI is enormous.

### 9.1 Three pillars

| Pillar      | Question it answers                          | Tool                         | Sampling                  |
|-------------|----------------------------------------------|------------------------------|---------------------------|
| **Logs**    | "What happened, in order?"                   | spdlog, glog, journald, MCAP | every event, structured   |
| **Metrics** | "How is the system behaving over time?"      | Prometheus, statsd           | aggregated, low-cardinality |
| **Traces**  | "Where did this single request spend time?"  | OpenTelemetry, custom spans  | sampled, correlation-id'd |

A robotics-specific fourth pillar: **time-series sensor data**
(rosbag2/MCAP). Different from logs because it's *rebroadcastable* —
you can replay it back into the live stack.

### 9.2 What to always record

The "always-on" recording set. Cheap to capture, priceless when something
goes wrong:

- **TF tree** (full, at native rate) — without this, nothing else is
  interpretable spatially.
- **Joint state, IMU, odometry** — full rate.
- **Control setpoints and actual feedback** — both, paired by stamp.
- **Perception outputs** (detections, tracks, world model deltas) —
  full rate. Often a few KB/s.
- **Heartbeats and safety state transitions** — full rate.
- **Camera / LiDAR** — *throttled* (e.g. 5 Hz, downscaled) by default;
  full rate enabled by a topic on demand or auto-triggered by a fault.
- **Build / config snapshot** — git SHA, model version, calibration file
  hash, parameter dump. One message at startup. The single most useful
  log entry when chasing "this used to work".

Use **MCAP** (rosbag2's modern format) — chunked, indexed, compressed,
language-agnostic, supports out-of-order writes from multiple processes.

### 9.3 Structured logs, not printf

```cpp
// Bad — unparseable, untimed, untagged.
std::cerr << "grasp failed for object " << id << "\n";

// Good — structured, timestamped, queryable.
LOG_EVENT("grasp_failed",
          "object_id", id,
          "reason",    classify_failure(),
          "ee_pose",   T_ee_world,
          "force",     ft_reading_);
```

Structured logs let you `grep` by key, ship to ELK/Loki, and join across
subsystems by `correlation_id`. They cost nothing extra to write and
save days of post-mortem work.

### 9.4 The black-box recorder pattern

The robot can't record everything at full rate forever. But you almost
always want the *last few seconds before a fault* at full rate. Pattern:

- A **ring buffer** holds the last N seconds (e.g. 30 s) of all
  high-rate topics in memory.
- On any safety transition (`NORMAL → SAFE_HOLD`), the supervisor sends
  `dump_blackbox(reason)`.
- The recorder atomically flushes the ring + the next M seconds (e.g.
  10 s of post-fault) to a timestamped MCAP file on disk.
- The file is uploaded on next connection, or pulled at end of shift.

This is the same pattern as an aircraft FDR, and it works for the same
reason: the events you want are surrounded by events you can't predict
in advance.

### 9.5 Replay-driven CI

Once you have rosbags, you have regression tests:

1. Curate a small set of **golden bags** covering nominal + each known
   failure mode (the one that bit you last quarter; the one with the
   tricky lighting; the one from the customer demo).
2. CI replays each bag through the current build of the stack, with
   `use_sim_time:=true`, in a container.
3. Assertions: end-to-end latency p50/p99 under budget, no safety
   transitions, perception track ID switches < threshold, control
   error within bounds, no allocations on the RT thread (LTTng /
   `mlock` checks).
4. Failure → block the merge.

This is how you avoid "the new model is faster on the benchmark but
breaks tracking on real data" — the bag *is* the real data.

### 9.6 Shadow mode

Before a new perception model touches the robot's behavior, run it in
**shadow mode**: it consumes the live sensor stream and publishes its
output to a `/shadow/...` namespace, but the planner ignores it. You
log both old and new outputs and diff offline. Promote to active only
when shadow has been clean for a defined soak window (hours of
operation, or N tracked objects, etc.).

Shadow mode is the single highest-leverage tool for shipping changes to
a perception stack on a real robot. Every team I've seen reinvent it
wishes they'd had it from day one.
