# ros2_qos_examples (C++ + Python)

Runnable demos for [`docs/ros2_qos.md`](../../docs/ros2_qos.md). Source lives in the notes repo (`robotic_notes/ros2/`); the colcon workspace at `~/ros2_ws/src/` holds **relative symlinks** to it.

## Layout

```
~/workspace/robotic_notes/ros2/ros2_qos_examples_cpp/   # source (this dir)
~/workspace/robotic_notes/ros2/ros2_qos_examples_py/
~/ros2_ws/src/ros2_qos_examples_cpp -> ../../workspace/robotic_notes/ros2/ros2_qos_examples_cpp
~/ros2_ws/src/ros2_qos_examples_py  -> ../../workspace/robotic_notes/ros2/ros2_qos_examples_py
```

The relative symlinks resolve to the same place inside the container *as long as the notes repo is mounted at `/root/workspace/robotic_notes`*.

## Setup — one-time

Recreate the persistent dev container with the additional mount:

```bash
docker rm -f ros2 2>/dev/null

docker run -d --name ros2 \
  --network host --ipc host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $HOME/ros2_ws:/root/ros2_ws \
  -v $HOME/workspace/robotic_notes:/root/workspace/robotic_notes \
  -v $HOME/.ros:/root/.ros \
  --device /dev/dri:/dev/dri \
  osrf/ros:humble-desktop-full \
  sleep infinity
```

Build:

```bash
docker exec -it ros2 bash
# inside the container:
source /opt/ros/humble/setup.bash
cd /root/ros2_ws
rosdep install --from-paths src --ignore-src -y    # first time only
colcon build --packages-select ros2_qos_examples_cpp ros2_qos_examples_py --symlink-install
source install/setup.bash
```

For new shells, source both:

```bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
```

(Bake those two lines into `~/.bashrc` inside the container — see `docs/ros2.md` §6 for the Dockerfile pattern.)

## Run

Each demo wants two shells. The shorthand below assumes you've sourced both setup files in each new `docker exec -it ros2 bash`.

### 1. Slow consumer drops intermediate frames (the canonical pattern)

```bash
# Shell A — 60 Hz publisher with SensorDataQoS
ros2 run ros2_qos_examples_cpp camera_publisher

# Shell B — 50 ms/frame consumer; observe non-monotonic markers
ros2 run ros2_qos_examples_cpp latest_frame_subscriber
```

The `marker=` value (frame counter mod 256) jumps as intermediate frames are dropped at the queue. `age=` stays bounded — that's QoS doing its job.

### 2. Threaded latest-frame buffer (production pattern)

```bash
# Shell A
ros2 run ros2_qos_examples_cpp camera_publisher

# Shell B — ROS callback only swaps a pointer; worker thread does the work
ros2 run ros2_qos_examples_cpp latest_frame_buffer
```

Same drop behavior, but the executor thread is never blocked by processing time. Scales when you have multiple consumers or expensive per-frame work.

### 3. Custom QoS (RELIABLE + TRANSIENT_LOCAL)

```bash
# Shell A — publish at 1 Hz, history of 10 with transient-local durability
ros2 run ros2_qos_examples_cpp custom_qos_publisher

# Shell B — late join; receives the last 10 snapshots immediately
ros2 topic echo --qos-reliability reliable --qos-durability transient_local /config/snapshot
```

### 4. Python equivalents

```bash
ros2 run ros2_qos_examples_py camera_publisher
ros2 run ros2_qos_examples_py latest_frame_subscriber
```

## Changing QoS

See [`docs/ros2_qos.md` §8](../../../robotic_notes/docs/ros2_qos.md#8-changing-qos--three-options) for the three approaches (edit + rebuild, runtime parameters, QoS overrides) and the runtime inspection commands.

The QoS line in each demo is a single statement near the top of the constructor — easy to edit:

```cpp
// camera_publisher.cpp
pub_ = create_publisher<sensor_msgs::msg::Image>(
  "camera/image", rclcpp::SensorDataQoS());   // ← change me
```

After editing:

```bash
colcon build --packages-select ros2_qos_examples_cpp --symlink-install
source install/setup.bash
```
