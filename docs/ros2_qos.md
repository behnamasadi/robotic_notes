# ROS 2 QoS and Typed Messages

How ROS 2's DDS-based middleware actually behaves under producer/consumer rate mismatches. The recurring example is a **60 Hz camera** feeding a slower consumer, but the same rules apply to LiDAR, IMU, and any other high-rate sensor stream.

**Short answer up front:** QoS does *not* throttle the publisher. It controls *what gets buffered, what gets dropped, and what guarantees exist between endpoints* — but the camera will keep grabbing at 60 Hz regardless of what the subscriber does.

> **Runnable examples:** all snippets below have full executable counterparts in `~/ros2_ws/src/ros2_qos_examples_cpp` and `ros2_qos_examples_py`. Build instructions and run pairs are in that package's [README](../../ros2_ws/src/ros2_qos_examples_cpp/README.md).

---

## 1. Typed messages

Every ROS 2 topic carries a strongly-typed message defined in a `.msg` file (or generated from IDL). For a camera, that's typically `sensor_msgs/msg/Image` or `sensor_msgs/msg/CompressedImage`, plus a paired `sensor_msgs/msg/CameraInfo` on a separate topic.

The type is enforced at compile time (in C++) and at discovery time — a publisher and subscriber on the same topic *must* agree on the type, or the middleware refuses to connect them.

```cpp
auto pub = node->create_publisher<sensor_msgs::msg::Image>("camera/image", qos);
auto sub = node->create_subscription<sensor_msgs::msg::Image>(
    "camera/image", qos, callback);
```

The `qos` argument is where the interesting behavior lives.

---

## 2. The QoS policies that matter

ROS 2 exposes a handful of QoS policies. The four that govern rate-mismatch behavior:

### History
`KEEP_LAST(N)` or `KEEP_ALL`. With `KEEP_LAST(N)`, only the most recent N samples are retained in the queue. With `KEEP_ALL`, everything is kept (bounded only by `RESOURCE_LIMITS`, which most RMW implementations expose indirectly).

### Depth
The `N` in `KEEP_LAST(N)`. This is the queue size on both publisher and subscriber side.

### Reliability
- `RELIABLE` — middleware retransmits lost samples and blocks/buffers to guarantee delivery.
- `BEST_EFFORT` — fire and forget; if it doesn't arrive, oh well.

### Durability
- `VOLATILE` — only currently-connected subscribers receive samples.
- `TRANSIENT_LOCAL` — late-joining subscribers receive previously-published samples (latched-style topics like maps).

There's also **Deadline**, **Lifespan**, and **Liveliness** — less central for the rate-mismatch scenario.

---

## 3. What actually happens at 60 Hz with a slow consumer

Mental model: the publisher pushes at its own rate. The middleware has a queue on each side. If the subscriber's callback can't keep up, samples accumulate — and what happens next depends entirely on QoS.

### Case 1 — `KEEP_LAST(1)` + `BEST_EFFORT`

The canonical "I only want the latest frame" pattern for cameras. The subscriber's queue holds at most one image. When a new frame arrives and the old one hasn't been processed yet, the old one is overwritten. Best-effort means the publisher never blocks waiting for ACKs.

This is what you almost always want for real-time perception: process the freshest frame, drop stale ones, never build up latency.

```cpp
auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
```

### Case 2 — `KEEP_LAST(10)` + `RELIABLE`

The publisher will try to deliver every frame. If the subscriber falls behind, the middleware buffers up to 10. Once full, the publisher's behavior depends on the RMW: it may block on the next `publish()` call, or start dropping the *oldest* queued samples (depending on implementation and `RESOURCE_LIMITS`).

Either way: backpressure into the publisher (camera node stalls, frames pile up in the driver) or unbounded latency. Bad for cameras.

```cpp
auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
```

### Case 3 — `KEEP_ALL` + `RELIABLE`

Really bad for a 60 Hz stream with a slow consumer. Memory grows, or the publisher blocks indefinitely.

```cpp
auto qos = rclcpp::QoS(rclcpp::KeepAll()).reliable();
```

### So, to be precise

**QoS makes the *receiver* effectively process only the latest frame by dropping intermediate ones in its queue. It does not slow the camera down.** The camera node keeps capturing at 60 Hz, the network keeps shipping (or not, with best-effort), and the slow consumer just sees whatever happened to be in its 1-deep queue when its callback ran.

---

## 4. The recommended sensor profile

ROS 2 ships a predefined profile for exactly this case:

```cpp
rclcpp::QoS qos = rclcpp::SensorDataQoS();
// equivalent to: KEEP_LAST(5), BEST_EFFORT, VOLATILE
```

Five is a small buffer — enough to absorb scheduling jitter, small enough that stale data doesn't accumulate. Best-effort means no retransmission overhead. **This is the right starting point for any high-rate sensor stream.**

Publisher and subscriber, end to end:

```cpp
auto pub = create_publisher<sensor_msgs::msg::Image>(
  "camera/image", rclcpp::SensorDataQoS());

auto sub = create_subscription<sensor_msgs::msg::Image>(
  "camera/image", rclcpp::SensorDataQoS(),
  [](sensor_msgs::msg::Image::ConstSharedPtr msg) { /* ... */ });
```

A custom profile uses the chained API:

```cpp
auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
             .reliable()
             .transient_local();   // for latched-style topics like maps
```

Python equivalent:

```python
from rclpy.qos import qos_profile_sensor_data
sub = node.create_subscription(Image, 'camera/image', cb, qos_profile_sensor_data)
```

---

## 5. If you actually want to throttle the publisher

QoS won't do this. Three options, in order of preference:

1. **Configure the camera driver** — most expose a `frame_rate` parameter. Cleanest if you don't need 60 Hz to begin with.
2. **Insert a throttling node** — `topic_tools throttle` republishes a topic at a capped rate.
   ```bash
   ros2 run topic_tools throttle messages /camera/image 15.0 /camera/image_throttled
   ```
3. **Rate-control inside your callback** — process every Nth frame, discard the rest. Wasteful: you've already paid the deserialization cost. Use only if the first two aren't viable.

---

## 6. QoS compatibility — the silent-failure gotcha

Publisher and subscriber QoS must be *compatible*, not identical. The rules are **asymmetric**: the subscriber demands a level of guarantee, and the publisher must meet or exceed it.

| Publisher | Subscriber | Connect? |
|---|---|---|
| `RELIABLE` | `RELIABLE` | yes |
| `RELIABLE` | `BEST_EFFORT` | yes |
| `BEST_EFFORT` | `RELIABLE` | **no** |
| `BEST_EFFORT` | `BEST_EFFORT` | yes |
| `TRANSIENT_LOCAL` | `TRANSIENT_LOCAL` | yes |
| `TRANSIENT_LOCAL` | `VOLATILE` | yes |
| `VOLATILE` | `TRANSIENT_LOCAL` | **no** |
| `VOLATILE` | `VOLATILE` | yes |

If your camera node uses `SensorDataQoS()` (best-effort) and you write a subscriber with the **default** QoS (which is reliable), they silently won't communicate — you'll see the topic in `ros2 topic list`, see the publisher in `ros2 topic info`, but your callback never fires. This catches everyone at least once.

The silent-failure pair:

```cpp
// Publisher — best-effort
create_publisher<Image>("camera/image", rclcpp::SensorDataQoS());

// Subscriber — default reliable. Discovery succeeds, callback never fires.
create_subscription<Image>("camera/image", 10, callback);
```

Diagnose with:

```bash
ros2 topic info -v /camera/image
```

It prints the QoS of each endpoint and flags incompatibilities explicitly.

---

## 7. Multi-subscriber patterns

A real perception pipeline often wants the same topic at different "freshness" levels — e.g., the tracking thread wants the freshest frame; a mapping or loop-closure thread might want a longer history.

Two ways to handle this:

1. **Two subscriptions on the same topic with different QoS** — simple but pays the deserialization cost twice.
2. **One subscription that hands frames to an internal bounded queue** managed by your own threading logic (typically `std::scoped_lock` / `std::unique_lock` around the queue). One deserialize, multiple consumers, full control over the drop policy.

Pattern 2 is more common in production SLAM/perception stacks because it cleanly separates ROS-level QoS (which only governs the wire) from internal backpressure (which is your problem to solve).

Sketch — the producer (ROS callback) only swaps a pointer; the worker thread does the heavy work outside the lock:

```cpp
// Producer side — runs on the executor thread.
void on_frame(Image::ConstSharedPtr msg) {
  {
    std::scoped_lock lock(mtx_);
    latest_ = std::move(msg);
  }
  cv_.notify_one();
}

// Consumer side — runs on its own std::thread.
void worker_loop() {
  while (rclcpp::ok()) {
    Image::ConstSharedPtr frame;
    {
      std::unique_lock lock(mtx_);
      cv_.wait(lock, [&]{ return latest_ != nullptr; });
      frame = std::move(latest_);  // hand off, then drop the lock
    }
    process(frame);   // heavy work, lock NOT held
  }
}
```

Full version (with shutdown handling) at `~/ros2_ws/src/ros2_qos_examples_cpp/src/latest_frame_buffer.cpp`.

---

## 8. Changing QoS — three options

When you want to try a different reliability/depth/durability without re-deriving the right answer from first principles, pick the option that matches the tightness of your iteration loop.

### Option 1 — Edit + rebuild (simplest)

QoS is a single line in each demo. Edit, rebuild, re-run:

```cpp
// before
auto qos = rclcpp::SensorDataQoS();

// after — try any of:
auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();                 // freshest only
auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();                   // backpressure case
auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local(); // latched
```

```bash
colcon build --packages-select ros2_qos_examples_cpp --symlink-install
source install/setup.bash
```

Rebuild loop is ~5 s for one package — fine for occasional tweaks, painful if you're sweeping settings.

### Option 2 — QoS as a runtime parameter

Read `reliability` and `depth` from ROS parameters; tune from the CLI without rebuilding:

```cpp
declare_parameter<std::string>("reliability", "best_effort");
declare_parameter<int>("depth", 5);

rclcpp::QoS qos(rclcpp::KeepLast(get_parameter("depth").as_int()));
if (get_parameter("reliability").as_string() == "reliable") qos.reliable();
else                                                        qos.best_effort();
```

```bash
ros2 run ros2_qos_examples_cpp camera_publisher \
  --ros-args -p reliability:=reliable -p depth:=10
```

Best when *you* are the user of the node and want to sweep settings during development.

### Option 3 — ROS 2 QoS overrides (no rebuild ever)

Since Humble, any node can opt in to letting *its users* override QoS via well-known parameters. You declare which policies are overridable; users tune them with no source change:

```cpp
rclcpp::PublisherOptions opts;
opts.qos_overriding_options =
  rclcpp::QosOverridingOptions::with_default_policies();

pub_ = create_publisher<sensor_msgs::msg::Image>(
  "camera/image", rclcpp::SensorDataQoS(), opts);
```

```bash
ros2 run ros2_qos_examples_cpp camera_publisher --ros-args \
  -p qos_overrides./camera/image.publisher.reliability:=reliable \
  -p qos_overrides./camera/image.publisher.depth:=20
```

The "production" pattern — ship a sensible default, let downstream users tune without forking your node. Slightly more boilerplate, but no rebuild for any future tweak.

### Inspecting and overriding QoS from the CLI

```bash
# What QoS are the live endpoints actually using?
ros2 topic info -v /camera/image

# Subscribe with explicit QoS (handy for compatibility tests)
ros2 topic echo --qos-reliability best_effort \
                --qos-history     keep_last \
                --qos-depth       1 \
                /camera/image

# Publish a one-off with explicit QoS
ros2 topic pub --qos-reliability reliable \
               --qos-durability  transient_local \
               /test std_msgs/msg/String '{data: hi}'
```

`ros2 topic info -v` is the first thing to run when a subscriber sees no data — it prints both endpoints' QoS side-by-side and flags incompatibilities explicitly.

---

## References

- [QoS settings — ROS 2 design doc](https://design.ros2.org/articles/qos.html)
- [`rclcpp::QoS` API](https://docs.ros.org/en/jazzy/p/rclcpp/generated/classrclcpp_1_1QoS.html)
- [About QoS — ROS 2 Jazzy concepts](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
