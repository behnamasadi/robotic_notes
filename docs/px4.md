# PX4 Autopilot — Building, SITL, and ROS 2

Tested stack: **Ubuntu 22.04 + ROS 2 Humble + Gazebo Harmonic** (`gz sim --version` reports 8.x).

Notes:
- PX4 deprecated *Gazebo Classic* in favor of *modern Gazebo* (Harmonic/Garden) — use `gz_*` build targets, not `gazebo-classic`.
- The matching ROS 2 binding for Harmonic is `ros-humble-ros-gzharmonic` (not the default `ros-humble-ros-gz`, which is built against Fortress on Humble).

## 1. Clone PX4

```bash
git clone --recursive https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh    # installs build deps
```

If you skip `--recursive`, run `git submodule update --init --recursive` afterwards — the simulation worlds and airframes live in submodules.

## 2. Build for SITL

Quick start with the X500 quadcopter and Gazebo Harmonic:

```bash
make px4_sitl gz_x500
```

This compiles PX4 for the SITL target, then launches `gz sim` with the X500 world.

### Build target syntax

```
make [VENDOR_]MODEL[_VARIANT] [VIEWER_MODEL_DEBUGGER_WORLD]
```

**`CONFIGURATION_TARGET`** (`VENDOR_MODEL_VARIANT`):
- `VENDOR`: `px4`, `aerotenna`, `airmind`, `holybro`, …
- `MODEL`: board model — `sitl`, `fmu-v5`, `fmu-v6x`, `navio2`, …
- `VARIANT`: optional, usually `default` (omitted).

List every available target:

```bash
make list_config_targets
```

**`VIEWER_MODEL_DEBUGGER_WORLD`** — joined by underscores; missing fields are skipped:
- `VIEWER`: `gz` (modern Gazebo), `jmavsim`, `none`
- `MODEL`: vehicle model — `x500`, `rover`, `standard_vtol`, … Sets `PX4_SIM_MODEL`.
- `DEBUGGER`: `none`, `ide`, `gdb`, `lldb`, `valgrind`, `callgrind`
- `WORLD`: world name (e.g. `default`, `baylands`, `windy`)

Example:

```bash
make px4_sitl none_iris      # PX4 only, no simulator (jmavsim airframe loaded)
```

Refs: [PX4 build targets](https://docs.px4.io/main/en/dev_setup/building_px4.html#px4-make-build-targets).

### Selecting a world

Either via the build target string or the `PX4_GZ_WORLD` env var:

```bash
PX4_GZ_WORLD=windy make px4_sitl gz_x500
```

Available worlds live in `Tools/simulation/gz/worlds/`. To pull in extra worlds at runtime:

```bash
export GZ_SIM_RESOURCE_PATH=$PWD/Tools/simulation/gz/worlds:$GZ_SIM_RESOURCE_PATH
```

### Common vehicle targets

```bash
make px4_sitl gz_x500
make px4_sitl gz_x500_depth         # X500 with depth camera
make px4_sitl gz_x500_vision        # X500 with monocular camera
make px4_sitl gz_x500_lidar         # X500 with 2D lidar
make px4_sitl gz_standard_vtol
make px4_sitl gz_rc_cessna
make px4_sitl gz_advanced_plane
make px4_sitl gz_r1_rover
make px4_sitl gz_rover_ackermann
```

All vehicle and world models are pulled in as a submodule from [PX4-gazebo-models](https://github.com/PX4/PX4-gazebo-models).

Refs: [PX4 simulation with Gazebo](https://docs.px4.io/main/en/sim_gazebo_gz/#running-the-simulation).

### Standalone mode (PX4 and Gazebo in separate terminals)

```bash
# Terminal 1 — start Gazebo
cd ~/workspace/PX4-Autopilot/Tools/simulation/gz
python3 simulation-gazebo                 # default world
python3 simulation-gazebo --world walls   # specific world
```

```bash
# Terminal 2 — start PX4 attached to the running Gazebo
PX4_GZ_STANDALONE=1 make px4_sitl gz_x500
```

Refs: [Standalone mode](https://docs.px4.io/main/en/sim_gazebo_gz/gazebo_models.html#basic-usage).

---

## 3. ROS 2 integration

PX4 talks to ROS 2 over uXRCE-DDS, which needs an agent running on the ROS 2 side and the client baked into the PX4 firmware (default since v1.14).

### 3.1 Build and run the uXRCE-DDS Agent

```bash
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

Run the agent (matches the default PX4 SITL UDP port 8888):

```bash
MicroXRCEAgent udp4 -p 8888
```

### 3.2 Build the PX4 ROS 2 packages

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`px4_msgs` defines the uORB-flavored message types; `px4_ros_com` provides example nodes (`sensor_combined_listener`, `offboard_control`, etc.).

### 3.3 Verify the link

With PX4 SITL + the agent running, topics should appear:

```bash
ros2 topic list | grep fmu      # /fmu/in/... and /fmu/out/...
ros2 topic echo /fmu/out/vehicle_status
```

Refs: [PX4 ROS 2 user guide (Humble)](https://docs.px4.io/main/en/ros2/user_guide.html#humble).

---

## 4. Bridging Gazebo sensor topics to ROS 2

Sensor data published *inside Gazebo* (camera, lidar, IMU on the simulated airframe) is **not** automatically forwarded to ROS 2 — the uXRCE-DDS link only carries PX4-internal uORB topics. Use `ros_gz_bridge` for raw simulator sensors.

Install the Harmonic-matched bridge:

```bash
# If you previously installed the Fortress version, remove it first
sudo apt remove ros-humble-ros-gz
sudo apt install ros-humble-ros-gzharmonic
```

Inspect what Gazebo publishes:

```bash
gz topic -l
gz topic -i -t /world/default/model/x500_lidar_0/link/lidar_sensor/sensor/lidar/scan
```

Bridge an example camera topic:

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /world/default/model/x500_vision_0/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image
```

(Topic paths depend on the airframe — list with `gz topic -l` first; the structure is `/world/<world>/model/<vehicle>/link/<link>/sensor/<sensor>/<channel>`.)

---

## 5. Docker (optional)

Useful when host deps drift. The official PX4 dev images live under `px4io/`.

```bash
# Allow GUI from container
xhost +local:docker

docker run -it --privileged \
  --env=LOCAL_USER_ID="$(id -u)" \
  -v ~/workspace/PX4-Autopilot:/src/PX4-Autopilot:rw \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e DISPLAY=$DISPLAY \
  --network host \
  --name=px4-dev \
  px4io/px4-dev-simulation-jammy:latest \
  bash

# Inside the container
cd /src/PX4-Autopilot
make px4_sitl gz_x500
```

Re-attach later:

```bash
docker start -i px4-dev
docker exec -it px4-dev bash
```

Refs: [PX4 Docker guide](https://docs.px4.io/main/en/test_and_ci/docker.html).

---

## 6. QGroundControl

Ground-control station that auto-discovers SITL on `udp://:14550`:

- Download: <https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/quick_start.html>

---

## References

- [Building PX4 Software](https://docs.px4.io/main/en/dev_setup/building_px4.html)
- [Gazebo simulation](https://docs.px4.io/main/en/sim_gazebo_gz/)
- [ROS 2 user guide](https://docs.px4.io/main/en/ros2/user_guide.html)
- [uXRCE-DDS](https://docs.px4.io/main/en/middleware/uxrce_dds.html)
- [PX4-Autopilot repo](https://github.com/PX4/PX4-Autopilot)
- [Micro-XRCE-DDS-Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent)
