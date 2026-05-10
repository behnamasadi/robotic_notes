# ROS 2 ↔ Gazebo Integration

**Stack:** ROS 2 Jazzy + `ros_gz` (jazzy branch) + Gazebo **Harmonic**.

---

## Table of contents

1. [Distro / Gazebo mapping](#1-distro--gazebo-mapping)
2. [Dockerfile (Jazzy + Harmonic + ros_gz)](#2-dockerfile-jazzy--harmonic--ros_gz)
3. [Running the container](#3-running-the-container)
4. [GPU access (NVIDIA)](#4-gpu-access-nvidia)
5. [Launching Gazebo](#5-launching-gazebo)
6. [`ros_gz_bridge` essentials](#6-ros_gz_bridge-essentials)
7. [Demo recipes](#7-demo-recipes)
8. [Troubleshooting](#8-troubleshooting)

---

## 1. Distro / Gazebo mapping

| ROS 2 distro | `ros_gz` branch | `GZ_VERSION` | Gazebo release |
|---|---|---|---|
| **Jazzy** (LTS, 2024–2029) | `jazzy` | `harmonic` | Gazebo Harmonic |
| Iron *(EOL Nov 2024)* | `iron` | `garden` | Gazebo Garden |
| Humble (LTS, 2022–2027) | `humble` | `fortress` | Gazebo Fortress |

For Jazzy:

```bash
export GZ_VERSION=harmonic
```

Stick to one pairing per image — mixing distros inside the same container is what produces the worst class of `ros_gz` bugs.

---

## 2. Dockerfile (Jazzy + Harmonic + ros_gz)

Project layout:

```
ros2_ws/
├── docker/
│   ├── Dockerfile.gz
│   └── entrypoint.sh
├── docker-compose.yml
└── src/
```

**`docker/Dockerfile.gz`**

```dockerfile
FROM osrf/ros:jazzy-desktop-full

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000

# Gazebo Harmonic + ros_gz bridge + sim demos + common deps
RUN apt-get update && apt-get install -y --no-install-recommends \
      curl gnupg lsb-release ca-certificates && \
    curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      gz-harmonic \
      ros-jazzy-ros-gz \
      ros-jazzy-ros-gz-bridge \
      ros-jazzy-ros-gz-sim \
      ros-jazzy-ros-gz-sim-demos \
      ros-jazzy-rmw-cyclonedds-cpp \
      ros-jazzy-teleop-twist-keyboard \
      ros-jazzy-rqt* \
      ros-jazzy-xacro \
      ros-jazzy-urdf-tutorial \
      python3-colcon-common-extensions python3-rosdep python3-vcstool \
      git vim x11-apps && \
    rm -rf /var/lib/apt/lists/*

# Match host UID/GID so files written into the mounted workspace are owned by you
RUN groupadd --gid $USER_GID $USERNAME 2>/dev/null || true && \
    useradd  --uid $USER_UID --gid $USER_GID -m $USERNAME 2>/dev/null || true && \
    apt-get update && apt-get install -y sudo && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    rm -rf /var/lib/apt/lists/*

ENV GZ_VERSION=harmonic \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_DOMAIN_ID=42

USER $USERNAME
WORKDIR /home/$USERNAME/ros2_ws

RUN echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc && \
    echo '[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash' >> ~/.bashrc && \
    echo 'export PS1="\[\e[33m\][ros2-jazzy|gz-harmonic]\[\e[0m\] \w\$ "' >> ~/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
```

**`docker/entrypoint.sh`**

```bash
#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
exec "$@"
```

Build it (UID/GID match avoids root-owned files in your mounted workspace):

```bash
docker build -t ros2-jazzy-gz-harmonic \
  --build-arg USERNAME=$USER \
  --build-arg USER_UID=$(id -u) \
  --build-arg USER_GID=$(id -g) \
  -f docker/Dockerfile.gz .
```

---

## 3. Running the container

One-time per session, allow X11 access from containers:

```bash
xhost +local:docker
```

### Persistent dev container

```bash
docker run -d \
  --name ros2_gz \
  --restart unless-stopped \
  --network host \
  --ipc host \
  --pid host \
  --ulimit nofile=524288 \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e XDG_RUNTIME_DIR=/tmp/runtime-$USER \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $HOME/ros2_ws:/home/$USER/ros2_ws:rw \
  -v $HOME/.ros:/home/$USER/.ros:rw \
  --device /dev/dri:/dev/dri \
  ros2-jazzy-gz-harmonic \
  sleep infinity

docker exec -it ros2_gz bash
```

### Important flag rationale

| Flag | Why it matters for ROS 2 + Gazebo |
|---|---|
| `--network host` | DDS multicast discovery; also lets host `gz` tools see the sim |
| `--ipc host` | Gazebo + RViz heavily use `/dev/shm`; default 64 MB shm crashes them |
| `-v /tmp/.X11-unix:rw` | **Must be `:rw`** — Ubuntu 24.04, Fedora, Arch fail silently with `:ro` |
| `--device /dev/dri` | DRM render node for hardware OpenGL (RViz, Gazebo viewport) |
| `--ulimit nofile=524288` | Prevents "too many open files" with large topic counts |
| `-v $HOME/ros2_ws:...` | Workspace persists; rebuilds aren't lost |

### Quick verification (inside container)

```bash
echo $ROS_DISTRO          # → jazzy
gz sim --version          # → Gazebo Sim, version 8.x (Harmonic)
xclock                    # X11 sanity check
ros2 doctor               # ROS 2 environment sanity check
```

---

## 4. GPU access (NVIDIA)

Install [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) on the host once:

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

Add to `docker run`:

```bash
  --gpus all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e NVIDIA_VISIBLE_DEVICES=all \
```

Verify inside the container:

```bash
nvidia-smi                  # should show your GPU
glxinfo | grep "OpenGL renderer"   # should show NVIDIA, not llvmpipe
```

**What to install where:** the NVIDIA driver lives on the host only; never inside the container. The CUDA toolkit (`nvcc`) is not needed for Gazebo — pip wheels for PyTorch/TF bundle their own CUDA runtime. If `nvcc --version` reports "command not found" inside the container, that is correct.

---

## 5. Launching Gazebo

Inside the container:

```bash
gz sim                              # GUI only
gz sim -r empty.sdf                 # run an empty world
gz sim -v 4 -r sensors_demo.sdf     # verbose, with sensors
```

Flags:
- `-v 4` — verbosity 0–4 (4 = debug)
- `-r` — start the simulation immediately (otherwise it loads paused)

Bundled worlds live in `/usr/share/gz/gz-sim8/worlds/`.

Launch from a ROS 2 launch file:

```bash
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="-r empty.sdf"
```

Refs: [Launch Gazebo from ROS 2](https://gazebosim.org/docs/harmonic/ros2_launch_gazebo/)

---

## 6. `ros_gz_bridge` essentials

The bridge **only relays messages** — it never produces them. A working data flow needs three things:

1. The bridge is running (creates the topic on both sides).
2. Something is **publishing** (ROS 2 node, Gazebo plugin, or CLI).
3. Something is **subscribing** on the opposite side.

### `parameter_bridge` syntax

```
ros2 run ros_gz_bridge parameter_bridge <topic>@<ROS_type><dir><GZ_type>
```

Direction symbol between the ROS type and the Gazebo type:

| Symbol | Direction |
|---|---|
| `@` | Bidirectional |
| `[` | Gazebo → ROS only |
| `]` | ROS → Gazebo only |

Examples:

```bash
# bidirectional
ros2 run ros_gz_bridge parameter_bridge \
  /chatter@std_msgs/msg/String@gz.msgs.StringMsg

# gazebo → ros
ros2 run ros_gz_bridge parameter_bridge \
  /lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan

# ros → gazebo
ros2 run ros_gz_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist
```

Smoke test:

```bash
# T1
ros2 run ros_gz_bridge parameter_bridge /chatter@std_msgs/msg/String@gz.msgs.StringMsg
# T2
ros2 topic echo /chatter
# T3
ros2 topic pub /chatter std_msgs/msg/String '{data: "hello"}'
```

### Common bridged types

| ROS type | Gazebo type |
|---|---|
| `geometry_msgs/msg/Twist` | `gz.msgs.Twist` |
| `sensor_msgs/msg/Imu` | `gz.msgs.IMU` |
| `sensor_msgs/msg/Image` | `gz.msgs.Image` |
| `sensor_msgs/msg/CameraInfo` | `gz.msgs.CameraInfo` |
| `sensor_msgs/msg/LaserScan` | `gz.msgs.LaserScan` |
| `sensor_msgs/msg/PointCloud2` | `gz.msgs.PointCloudPacked` |
| `sensor_msgs/msg/MagneticField` | `gz.msgs.Magnetometer` |
| `sensor_msgs/msg/NavSatFix` | `gz.msgs.NavSat` |
| `nav_msgs/msg/Odometry` | `gz.msgs.Odometry` |
| `tf2_msgs/msg/TFMessage` | `gz.msgs.Pose_V` |

Full list: [`ros_gz_bridge` README](https://github.com/gazebosim/ros_gz/tree/jazzy/ros_gz_bridge).

YAML config form (preferred for >1 topic):

```yaml
# bridge.yaml
- ros_topic_name: "/cmd_vel"
  gz_topic_name: "/model/vehicle/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ
- ros_topic_name: "/scan"
  gz_topic_name: "/lidar"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS
```

```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=bridge.yaml
```

---

## 7. Demo recipes

All commands run inside the container. Many of these have an equivalent `ros2 launch ros_gz_sim_demos ...` shortcut — both forms are shown.

### 7.1 Camera

```bash
gz sim -v 4 -r sensors_demo.sdf
ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image
rviz2          # add Image display, set topic /camera
```

Refs: [`ros_gz_sim_demos` Camera](https://github.com/gazebosim/ros_gz/tree/jazzy/ros_gz_sim_demos#camera)

### 7.2 Differential drive

```bash
gz sim -v 4 -r visualize_lidar.sdf
ros2 topic list

# Drive
ros2 run ros_gz_bridge parameter_bridge \
  /model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist

ros2 topic pub /model/vehicle_blue/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 5.0}, angular: {z: 0.5}}"

# Or with the keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/model/vehicle_blue/cmd_vel

# LiDAR
ros2 run ros_gz_bridge parameter_bridge \
  /lidar2/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked

# In rviz2 you must set Fixed Frame to the lidar's frame_id:
ros2 topic echo /lidar2/points --once | grep frame_id
# → frame_id: vehicle_blue/lidar_link/gpu_lidar
```

One-shot equivalent:

```bash
ros2 launch ros_gz_sim_demos diff_drive.launch.py
```

> **rqt tip:** when steering with the rqt Message Publisher, click the broom icon to clear stale Twist publishers between runs — otherwise old commands keep firing.

### 7.3 GPU LiDAR

```bash
gz sim -r gpu_lidar_sensor.sdf
ros2 run ros_gz_bridge parameter_bridge /lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan
ros2 run ros_gz_bridge parameter_bridge /lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked
ros2 topic echo /lidar/points --once | grep frame_id
# → frame_id: model_with_lidar/link/gpu_lidar
rviz2
```

Or:
```bash
ros2 launch ros_gz_sim_demos gpu_lidar_bridge.launch.py
```

Refs: [GPU lidar demo](https://github.com/gazebosim/ros_gz/tree/jazzy/ros_gz_sim_demos#gpu-lidar)

### 7.4 IMU + Magnetometer

```bash
gz sim -r sensors.sdf
ros2 run ros_gz_bridge parameter_bridge /imu@sensor_msgs/msg/Imu@gz.msgs.IMU
ros2 run ros_gz_bridge parameter_bridge /magnetometer@sensor_msgs/msg/MagneticField@gz.msgs.Magnetometer
```

Or:
```bash
ros2 launch ros_gz_sim_demos imu.launch.py
ros2 launch ros_gz_sim_demos magnetometer.launch.py
```

### 7.5 GNSS

```bash
gz sim -r spherical_coordinates.sdf
ros2 topic list   # find /navsat
ros2 run ros_gz_bridge parameter_bridge /navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat
```

### 7.6 Spawn a URDF model

Convert + sanity-check the URDF (xacro source available e.g. at [`urdf_tutorial`](https://github.com/ros/urdf_tutorial)):

```bash
xacro 08-macroed.urdf.xacro > 08-macroed.urdf
check_urdf 08-macroed.urdf
urdf_to_graphviz 08-macroed.urdf
dot -Tsvg 08-macroed.gv -o 08-macroed.svg
```

Spawn into a running sim via the world's create service:

```bash
gz sim empty.sdf

# In another shell:
gz service -l                                     # list services
gz service -is /world/empty/create                # → gz.msgs.EntityFactory, gz.msgs.Boolean
gz service -s /world/empty/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 1000 \
  --req 'sdf_filename: "08-macroed.urdf", name: "macroed"'
```

> The path in `sdf_filename` is resolved relative to the **`gz sim` process's working directory**, not yours — `cd` to the URDF's directory before launching `gz sim`.

### 7.7 Joint states / TF

```bash
ros2 launch ros_gz_sim_demos tf_bridge.launch.py
```

If you are running a Fortress-era demo file that still calls `ign gazebo`, replace it with `gz sim` in the launch file:

```python
cmd=[
    'gz', 'sim', '-r',
    os.path.join(pkg_ros_gz_sim_demos, 'models', 'double_pendulum_model.sdf'),
]
```

The Jazzy branch of `ros_gz_sim_demos` ships this fix already — only relevant if you pulled an older branch.

---

## 8. Troubleshooting

### GUI windows don't appear / "cannot open display"

1. Run `xhost +local:docker` on the **host** before `docker run`.
2. Confirm `-e DISPLAY=$DISPLAY` and `-v /tmp/.X11-unix:/tmp/.X11-unix:rw` are present (note `:rw`, not `:ro`).
3. Inside the container: `echo $DISPLAY` (commonly `:0` or `:1`); if empty, `export DISPLAY=:0`.
4. Test: `xclock` should pop up a clock.

### Gazebo viewport is sluggish / black

- Confirm hardware OpenGL: `glxinfo | grep "OpenGL renderer"` should not say `llvmpipe`.
- Pass `--device /dev/dri` (Intel/AMD) or `--gpus all` (NVIDIA).
- Ensure `--ipc host` is set (Gazebo + RViz need real shared memory).
- Force a render engine if Ogre 2 misbehaves on your driver: `gz sim --render-engine ogre`.

### `ros2 topic echo /foo` shows nothing after starting the bridge

- The bridge does not generate data — start a publisher (ROS 2 or Gazebo plugin).
- Subscribe **before** publishing one-shot messages, or use `--once`/`-r` flags.
- If `ros2 topic list` shows `/foo` but no data: the bridge is up; the missing piece is the publisher.

### `gz topic` CLI behaves oddly inside Docker

`gz topic echo` sometimes fails to attach to the simulator from a *different* container than the one running `gz sim` due to shared-memory transport quirks even with `--ipc host`. Workarounds:

- Run `gz topic` from another shell **inside the same container** (`docker exec -it ros2_gz bash`) — they share `/tmp` and shared memory.
- Or bridge the topic and use `ros2 topic echo` over DDS — that path is reliable across containers.

### "ROS 2 nodes in two containers don't see each other"

- Both containers must use the same `ROS_DOMAIN_ID` to communicate.
- Use `--network host --ipc host` in both.
- Same `RMW_IMPLEMENTATION` on both ends.

### `libnvidia-ml.so.1: cannot open shared object file`

```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

If the issue persists, drop `--gpus all` and try `--runtime=nvidia` instead (or vice versa). Only one of the two should be needed depending on your toolkit version.

### Adding a missing ROS package

Preferred — bake into the image:

```bash
# Edit docker/Dockerfile.gz, add `ros-jazzy-<pkg>` to the apt install list
docker build -t ros2-jazzy-gz-harmonic -f docker/Dockerfile.gz \
  --build-arg USERNAME=$USER --build-arg USER_UID=$(id -u) --build-arg USER_GID=$(id -g) .
```

Quick experiment — install in the running persistent container:

```bash
docker exec -it ros2_gz bash -c 'sudo apt update && sudo apt install -y ros-jazzy-<pkg>'
```

Persistent containers retain apt installs across restart; `--rm` containers don't.

---

## References

- [ROS 2 Jazzy docs](https://docs.ros.org/en/jazzy/)
- [Gazebo Harmonic docs](https://gazebosim.org/docs/harmonic)
- [`ros_gz` (Jazzy branch)](https://github.com/gazebosim/ros_gz/tree/jazzy)
- [`ros_gz_sim_demos`](https://github.com/gazebosim/ros_gz/tree/jazzy/ros_gz_sim_demos)
