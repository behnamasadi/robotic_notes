# ROS 2 — Docker-based workflow

This guide assumes you **never install ROS 2 on the host**. Everything (CLI, build, GUI tools like RViz/Gazebo, packages) runs inside a container. Your ROS 2 workspace lives on the host and is bind-mounted into the container so edits and `colcon build` artefacts persist.

- Distribution: **Jazzy Jalisco** (May 2024 → May 2029, Ubuntu 24.04 Noble Numbat base)
- Image: `osrf/ros:jazzy-desktop-full` (includes RViz2, demos, Gazebo bindings)

---

## 0. Which distro should you actually use?

Honest answer: **Humble is still the most mainstream** in mid-2026. Jazzy is newer, has a longer support window, and is what this guide targets — but if you're following tutorials or integrating off-the-shelf robot stacks, Humble currently has the larger ecosystem.

| Distro | Released | EOL | Mainstream status |
|---|---|---|---|
| **Humble** | May 2022 | May 2027 | **Still the default.** Most tutorials, most third-party drivers (TurtleBot, Clearpath, Husarion, UR, Franka), most Nav2 / MoveIt2 examples, most Stack Overflow answers |
| Iron | May 2023 | Nov 2024 | EOL — don't use |
| **Jazzy** | May 2024 | May 2029 | **Newest LTS, growing adoption,** but ecosystem is still catching up. A few niche drivers haven't been ported yet |
| Rolling | continuous | — | Dev branch, not for production |

### When each makes sense

- **Pick Humble** if you'll be following tutorials, integrating off-the-shelf robot stacks, or want the broadest "it just works" package coverage. Safe default.
- **Pick Jazzy** if you're starting fresh, want the longest support window (2029), and have verified the specific drivers you need exist on Jazzy.

### Switching this guide to Humble

It's mostly a search-and-replace:

| Jazzy | Humble |
|---|---|
| `osrf/ros:jazzy-desktop-full` | `osrf/ros:humble-desktop-full` |
| `ros-jazzy-<pkg>` | `ros-humble-<pkg>` |
| `/opt/ros/jazzy/setup.bash` | `/opt/ros/humble/setup.bash` |
| Gazebo Harmonic, `gz sim` | Gazebo Fortress, `ign gazebo` |
| Ubuntu 24.04 base | Ubuntu 22.04 base |

Everything else (Dockerfile structure, colcon, launch files, bridges) is identical.

---

## 1. Pull the image

```bash
docker pull osrf/ros:jazzy-desktop-full
```

Variants:
- `ros:jazzy-ros-core` — minimum runtime, no tools
- `ros:jazzy-ros-base` — runtime + common ROS packages
- `osrf/ros:jazzy-desktop` — adds RViz, demos, tutorials
- `osrf/ros:jazzy-desktop-full` — adds simulation, perception, extra tools

## 2. Allow GUI access from the container

Run on the **host** once per login session (X11):

```bash
xhost +local:docker
```

Then `xhost -local:docker` when done if you want to revoke.

For Wayland, either run an XWayland-backed terminal or use `xhost +SI:localuser:root`.

## 3. Run a persistent container

Create the workspace on the host first:

```bash
mkdir -p ~/ros2_ws/src
```

Start a long-lived container named `ros2`:

```bash
docker run -it --privileged \
  --name ros2 \
  --network host \
  --ipc host \
  --pid host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e XDG_RUNTIME_DIR=/tmp/runtime-root \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v $HOME/ros2_ws:/root/ros2_ws:rw \
  -v $HOME/.ros:/root/.ros:rw \
  --device /dev/dri:/dev/dri \
  osrf/ros:jazzy-desktop-full \
  bash
```

Flag rationale:

| Flag | Why |
|---|---|
| `--network host` | DDS discovery uses multicast; `host` mode avoids NAT issues |
| `--ipc host` | Lets DDS use shared-memory transport with host-side processes |
| `--privileged` + `--device /dev/dri` | GPU/DRM access for RViz, Gazebo |
| `-v ~/ros2_ws:/root/ros2_ws` | Workspace persists outside the container |
| `-v ~/.ros:/root/.ros` | Logs and bag files persist |
| `-e DISPLAY` + X11 socket | GUI apps (RViz2, rqt) render on host |

Re-attach later (without recreating):

```bash
docker start -ai ros2          # start + attach
docker exec -it ros2 bash      # extra shell while it runs
```

## 4. NVIDIA GPU (optional)

Install [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/) on the host, then add to `docker run`:

```bash
  --gpus all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
```

## 5. USB / serial / camera / CAN devices

Pass the specific device, not `--privileged`, in production:

```bash
  --device=/dev/ttyUSB0 \
  --device=/dev/video0 \
  --device=/dev/can0 \
  --group-add dialout \
```

---

## 6. Extending the image with a Dockerfile

Don't `apt install` inside a running container — bake it in. A typical project layout:

```
ros2_ws/
├── docker/
│   ├── Dockerfile
│   └── entrypoint.sh
├── docker-compose.yml
└── src/
    └── <your_packages>/
```

**`docker/Dockerfile`**

```dockerfile
FROM osrf/ros:jazzy-desktop-full

ARG DEBIAN_FRONTEND=noninteractive

# System deps your nodes need
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-vcstool \
      ros-jazzy-nav2-bringup \
      ros-jazzy-slam-toolbox \
      ros-jazzy-rmw-cyclonedds-cpp \
      git wget vim \
    && rm -rf /var/lib/apt/lists/*

# Use Cyclone DDS (more reliable on Linux than the default rmw)
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Workspace lives where the host mounts it
WORKDIR /root/ros2_ws

# Auto-source ROS in every interactive shell
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc \
 && echo '[ -f /root/ros2_ws/install/setup.bash ] && source /root/ros2_ws/install/setup.bash' >> /root/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
```

**`docker/entrypoint.sh`**

```bash
#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
if [ -f /root/ros2_ws/install/setup.bash ]; then
  source /root/ros2_ws/install/setup.bash
fi
exec "$@"
```

Build and run:

```bash
docker build -t ros2-dev -f docker/Dockerfile .
docker run -it --rm --net host --ipc host \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v $PWD:/root/ros2_ws ros2-dev
```

---

## 7. Docker Compose for multi-container setups

`docker-compose.yml` — separates simulator, navigation, and developer shell:

```yaml
services:
  dev:
    build: { context: ., dockerfile: docker/Dockerfile }
    image: ros2-dev
    network_mode: host
    ipc: host
    privileged: true
    environment:
      - DISPLAY=${DISPLAY}
      - ROS_DOMAIN_ID=42
      - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:ro
      - ./:/root/ros2_ws
      - ~/.ros:/root/.ros
    stdin_open: true
    tty: true
    command: bash

  nav2:
    image: ros2-dev
    network_mode: host
    ipc: host
    environment:
      - ROS_DOMAIN_ID=42
      - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    command: ros2 launch nav2_bringup tb3_simulation_launch.py
```

```bash
docker compose up -d nav2     # start navigation in background
docker compose run --rm dev   # interactive dev shell
```

---

## 8. ROS 2 ↔ ROS 1 bridge (also Dockerized)

Run a noetic container alongside the jazzy one, bridged with `ros1_bridge`. Both must share `--network host` and use a compatible DDS config.

```bash
# ROS 1 side
docker run -it --rm --network host --ipc host \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  --name ros1 ros:noetic-perception-focal bash

# Bridge container (has both noetic + foxy/galactic — use the matching tag)
docker run -it --rm --network host --ipc host \
  --name bridge ros:galactic-ros1-bridge \
  ros2 run ros1_bridge dynamic_bridge
```

> Note: there is no official Jazzy↔Noetic bridge image; the bridge stops at Galactic/Humble on the ROS 2 side. For Jazzy, run a Humble bridge container in between, or migrate the ROS 1 nodes.

---

## 9. ROS 2 inside the container

Everything below runs **inside** `docker exec -it ros2 bash`.

### Verify

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
printenv | grep -i ROS
ros2 doctor
```

### Domain ID

DDS partitions logical networks via `Domain ID`. Nodes on the same ID discover each other; default is 0.

```bash
export ROS_DOMAIN_ID=<your_domain_id>
```

Set it in the Dockerfile or compose file so every container in the project agrees.

### `ROS_AUTOMATIC_DISCOVERY_RANGE`

Limit DDS discovery scope. Useful when many containers share `--network host` on the same LAN:

```bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST   # or SUBNET, SYSTEM_DEFAULT
```

### QoS (Quality of Service)

DDS QoS controls *what gets buffered, what gets dropped, and what guarantees exist between endpoints*. It does **not** throttle the publisher. Critical for high-rate sensor topics (cameras, LiDAR, IMU) where a slow consumer would otherwise build up unbounded latency. See [`ros2_qos.md`](./ros2_qos.md) for the full breakdown — sensor profile, publisher/subscriber compatibility rules, and the silent-failure gotcha when a default-QoS subscriber tries to connect to a `BEST_EFFORT` publisher.

### Remapping

```bash
ros2 run turtlesim turtle_teleop_key
# publishes /turtle1/cmd_vel

ros2 run turtlesim turtle_teleop_key --ros-args \
  --remap turtle1/cmd_vel:=turtle2/cmd_vel

ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
```

### Inspect a message type

```bash
ros2 interface show geometry_msgs/msg/Twist
```

```
Vector3  linear
        float64 x
        float64 y
        float64 z
Vector3  angular
        float64 x
        float64 y
        float64 z
```

### Load a parameter file

```bash
ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
```

---

## 10. Colcon (inside the container)

Already installed via the Dockerfile above. Build the mounted workspace:

```bash
cd /root/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` symlinks Python/launch/config files so edits on the host take effect without rebuilding.

Place `COLCON_IGNORE` in any package directory to skip it.

### Build a single package

```bash
colcon build --packages-select <my_package>
```

### Pass CMake args

```bash
colcon build --cmake-args \
  -DCeres_DIR=/opt/ceres/lib/cmake/Ceres/ \
  -Dabsl_DIR=/opt/absl/lib/cmake/absl/ \
  --packages-select slam_toolbox
```

### Underlay vs. overlay

- **Underlay** — pre-built workspace already sourced (the ROS distro itself):
  ```bash
  source /opt/ros/$ROS_DISTRO/setup.bash
  ```
- **Overlay** — your workspace built on top:
  ```bash
  source /root/ros2_ws/install/setup.bash
  ```

The Dockerfile entrypoint sources both automatically.

### Resolve dependencies

From the workspace root:

```bash
rosdep update
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
```

Flags:
1. `--from-paths src` — scan `package.xml` files for keys
2. `-y` — non-interactive
3. `--ignore-src` — skip keys already provided by packages in this workspace

`rosdep` is not a package manager; it maps rosdep keys to the host's package manager (apt on Ubuntu). Keys are documented in [REP-149](https://ros.org/reps/rep-0149.html).

#### `<depend>`
Build- and run-time. Use this for C++ deps when in doubt.

#### `<build_depend>`
Build only.
```xml
<build_depend>gtest</build_depend>
<build_depend>ament_cmake</build_depend>
```

#### `<build_export_depend>`
Needed at build time *and* exposed to downstream packages (e.g. through public headers).
```xml
<build_export_depend>Eigen3</build_export_depend>
```

#### `<exec_depend>`
Runtime only — shared libs, Python modules, launch scripts.
```xml
<exec_depend>ros2launch</exec_depend>
```

#### Where keys come from
1. ROS packages — [`rosdistro/<distro>/distribution.yaml`](https://github.com/ros/rosdistro)
2. apt deps — [`rosdep/base.yaml`](https://github.com/ros/rosdistro/blob/master/rosdep/base.yaml)
3. Python deps — [`rosdep/python.yaml`](https://github.com/ros/rosdistro/blob/master/rosdep/python.yaml)

---

## 11. Packages

Build system: `ament`. Build tool: `colcon`.

**CMake package layout (minimum):**
- `CMakeLists.txt`
- `include/<package_name>/` — public headers
- `package.xml`
- `src/`

**Python package layout (minimum):**
- `package.xml`
- `resource/<package_name>` — marker file
- `setup.cfg` — required for `ros2 run` to find executables
- `setup.py`
- `<package_name>/__init__.py`

Create:

```bash
ros2 pkg create --build-type ament_cmake   --license Apache-2.0 <package_name>
ros2 pkg create --build-type ament_python  --license Apache-2.0 <package_name>
```

```
ros2 pkg
  create       Create a new ROS 2 package
  executables  Output a list of package specific executables
  list         Output a list of available packages
  prefix       Output the prefix path of a package
  xml          Output the XML of the package manifest or a specific tag
```

### Workspaces
A workspace can contain any number of packages. **Packages cannot be nested.**

Refs: [Colcon tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)

---

## 12. Xacro
Refs: [Using Xacro](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html)

---

## 13. Launch files

Written in Python, XML, or YAML. Launched with `ros2 launch <package> <file>`.

### Python launch file

Filename must end in `launch.py`. Define `generate_launch_description()` returning a `LaunchDescription`.

```python
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim',
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim',
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ],
        ),
    ])
```

Unique namespaces let two `turtlesim_node`s coexist:

```
$ ros2 node list
/mimic
/turtlesim1/sim
/turtlesim2/sim

$ ros2 topic list
/parameter_events
/rosout
/turtlesim1/turtle1/cmd_vel
/turtlesim1/turtle1/color_sensor
/turtlesim1/turtle1/pose
/turtlesim2/turtle1/cmd_vel
/turtlesim2/turtle1/color_sensor
/turtlesim2/turtle1/pose
```

Install the launch file from `setup.py`:

```python
('share/' + package_name + '/launch', ['launch/turtlesim_mimic_launch.py']),
```

i.e.
```python
data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', ['launch/turtlesim_mimic_launch.py']),
]
```

Or glob all launch files:

```python
import os
from glob import glob

data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'),
     glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
]
```

### C++ launch file

In `CMakeLists.txt`:

```cmake
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
```

`launch/my_script_launch.py`:

```python
import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='cpp_pubsub', executable='talker',   name='talker'),
        launch_ros.actions.Node(
            package='cpp_pubsub', executable='listener', name='listener'),
    ])
```

### Substitutions
Argument values resolved at launch time — useful for reusable launch files.

### Event handlers
The launch system tracks process state and reacts to events; register handlers to act on start/exit/output.

Refs: [Creating launch files](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html), [Large projects](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html), [Architecture](https://github.com/ros2/launch/blob/humble/launch/doc/source/architecture.rst)

---

## 14. Nav2

Run via the bundled launch (image must include `ros-jazzy-nav2-bringup`):

```bash
ros2 launch nav2_bringup tb3_simulation_launch.py
```

Refs: [Nav2 docs](https://docs.nav2.org/), [navigation2 repo](https://github.com/ros-navigation/navigation2)

---

## 15. teleop_twist_keyboard

Install (already in extended image if added):

```bash
apt install -y ros-jazzy-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Publishes `geometry_msgs/Twist` on `/cmd_vel` (`linear` + `angular` vectors).

```
   u    i    o
   j    k    l
   m    ,    .
```

- `i` — forward (linear.x +)
- `,` — backward (linear.x −)
- `j` — turn left (angular.z +)
- `l` — turn right (angular.z −)
- `u` — forward + left
- `o` — forward + right
- `m` — backward + left
- `.` — backward + right
- `k` — stop (zero everything)

Combinations like `u`/`o` set both linear and angular components for arc motion.

---

## 16. Useful container shortcuts

```bash
# Stop / start the persistent dev container
docker stop ros2 && docker start -ai ros2

# Open another shell into a running container
docker exec -it ros2 bash

# Tail logs (e.g. for a launched container)
docker logs -f nav2

# Wipe and rebuild a workspace from scratch
docker exec -it ros2 bash -c \
  'cd /root/ros2_ws && rm -rf build install log && colcon build --symlink-install'

# Record a bag from the host filesystem (mounted volume)
docker exec -it ros2 bash -c \
  'cd /root/.ros && ros2 bag record -a -o run_$(date +%s)'
```
