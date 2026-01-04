# ROS 2 & Gazebo Integration Guide
**(ROS 2 Humble + ros_gz + Gazebo Fortress)**

---

## Table of Contents

1. [Official ROS 2 / Gazebo Mapping](#official-ros-2--gazebo-mapping-2024-2025) - Version compatibility matrix
2. [Docker Installation](#docker-installation) - Containerized setup with GUI support
3. [Usage Guide](#usage-guide) - Launch Gazebo from ROS2
4. [ROS2-Gazebo Interaction](#ros2-interaction-with-gazebo) - Bridge examples and demos
5. [FAQ & Troubleshooting](#faq--troubleshooting) - Common issues and solutions

---

## Official ROS 2 / Gazebo Mapping (2024–2025)

| ROS 2 distro | ros_gz git branch | Required `GZ_VERSION` value | Gazebo release name |
|--------------|-------------------|-----------------------------|---------------------|
| **ROS Jazzy** | `jazzy`           | `harmonic`                  | Gazebo Harmonic     |
| **ROS Iron**  | `iron`            | `garden`                    | Gazebo Garden       |
| **ROS Humble** | `humble`          | `fortress`                  | Gazebo Fortress     |

**For ROS 2 Humble (this guide):**

Always export:

```bash
export GZ_VERSION=fortress
```

---

## Docker Installation

**2. Setup GUI Support (X11 Forwarding)**

For GUI applications (rviz2, Gazebo, rqt) to work, you need to allow Docker to access your X server:

```bash
# Allow Docker containers to connect to X server
xhost +local:docker
```

**Note:** This command allows any local Docker container to access your X server. For better security, you can be more specific:

```bash
# More secure: allow only specific container
xhost +local:`docker inspect --format='{{ .Config.Hostname }}' <container_id>`
```

**3. NVIDIA GPU Support (Optional but Recommended)**

If you have an NVIDIA GPU and want GPU acceleration for Gazebo:

**One-Time Setup (Host Only):**

Install NVIDIA Container Toolkit:

```bash
# Configure the production repository (modern method, not deprecated)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Update and install
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

# Configure Docker to use nvidia as default runtime
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

**Verify GPU access:**

```bash
nvidia-smi
```

If this fails, install NVIDIA drivers first:

```bash
# Check available drivers
ubuntu-drivers devices

# Install recommended driver
sudo ubuntu-drivers autoinstall

# Reboot
sudo reboot
```

---

### Custom Dockerfile for ROS2 Humble with Gazebo Fortress

#### Dockerfile Overview

#### Complete Dockerfile

The Dockerfile is located at the project root: `src/Dockerfile`

Key features:
- Non-root user setup (default: `behnam`, UID/GID 1000)
- Proper permissions for workspace
- All packages pre-installed for immediate use
- Custom orange prompt to identify container environment

#### Build the Image

```bash
cd /path/to/project/src
docker build -t ros2-humble-gazebo-fortress \
  --build-arg USER_UID=$(id -u) \
  --build-arg USER_GID=$(id -g) \
  --build-arg USERNAME=$USER .
```

---

### Creating and Running Containers with GUI Support

**IMPORTANT: Before running any container, enable GUI support:**

```bash
xhost +local:docker
```

#### Option A: Temporary Container (Deleted on Exit)

Good for quick tests and experiments:

```bash
docker run -it --rm \
  --name humble-fortress \
  --privileged \
  --network host \
  --ipc=host \
  --ulimit nofile=524288 \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$HOME":"$HOME":rw \
  --workdir "$PWD" \
  ros2-humble-gazebo-fortress \
  bash
```

**With GPU support (NVIDIA):**

```bash
docker run -it --rm \
  --name humble-fortress \
  --privileged \
  --network host \
  --ipc=host \
  --ulimit nofile=524288 \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$HOME":"$HOME":rw \
  --workdir "$PWD" \
  --gpus all \
  ros2-humble-gazebo-fortress \
  bash
```

#### Option B: Persistent Container (Survives Exit)

Best for development — install packages once and they stay:

```bash
docker run -d \
  --name humble-fortress-permanent \
  --restart unless-stopped \
  --privileged \
  --network host \
  --ipc=host \
  --ulimit nofile=524288 \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$HOME":"$HOME":rw \
  --workdir "$PWD" \
  ros2-humble-gazebo-fortress \
  sleep infinity
```

**With GPU support (NVIDIA):**

```bash
docker run -d \
  --name humble-fortress-permanent \
  --restart unless-stopped \
  --privileged \
  --network host \
  --ipc=host \
  --ulimit nofile=524288 \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$HOME":"$HOME":rw \
  --workdir "$PWD" \
  --gpus all \
  ros2-humble-gazebo-fortress \
  sleep infinity
```

Then connect to it anytime:

```bash
docker exec -it humble-fortress-permanent bash
```

Or start/stop it:

```bash
docker start humble-fortress-permanent
docker stop humble-fortress-permanent
```

#### Important Docker Flags Explained

| Flag | Why it matters |
|------|----------------|
| `-e DISPLAY=$DISPLAY` | Passes your X11 display to the container for GUI applications |
| `-v /tmp/.X11-unix:/tmp/.X11-unix:rw` | Mounts X11 socket for GUI support. **Must be `:rw` (read-write)**, not `:ro`. Some distributions (Ubuntu 24.04, Fedora, Arch) require write access. Using `:ro` silently fails → black windows or "Cannot open display". |
| `--ipc=host` | ROS 2 tools (especially rviz2, Gazebo, Foxglove) heavily use shared memory (`/dev/shm`). Without this flag you get crashes or extremely slow rendering. |
| `--ulimit nofile=524288` | Prevents "Too many open files" errors when running large ROS 2 nodes or rviz2 with many topics. |
| `--network host` | Allows ROS 2 DDS discovery to work seamlessly between host and container. |
| `--privileged` | Gives access to hardware devices (cameras, IMUs, serial ports, etc.). |
| `--gpus all` | Enables GPU access for NVIDIA GPUs (requires NVIDIA Container Toolkit setup) |

---

### Verify Container Setup

Once inside the container:

```bash
# Check ROS2 is sourced (you should see [ROS2-Humble ...] in your prompt)
echo $ROS_DISTRO
# Should output: humble

# Check Gazebo version
gz sim --version

# Check GPU access (if using --gpus all)
nvidia-smi

# Test GUI with a simple X11 app
xclock
# If this opens a clock window, GUI is working!
```

---

### GPU Support (NVIDIA)

#### What to Install Where

| Component | Install on Host? | Install in Container? | Notes |
|-----------|------------------|----------------------|-------|
| **NVIDIA driver** | ✅ Yes (already done) | ❌ Never | Host only |
| **CUDA toolkit** | ❌ No | ❌ No (manual install) | Bundled with PyTorch/TF wheels |
| **cuDNN** | ❌ No | ❌ Usually no | Bundled with ML frameworks |
| **NVIDIA Container Toolkit** | ✅ Yes (once) | — | Enables `--gpus all` |
| **PyTorch/TensorFlow/JAX** | — | ✅ Yes (pip/conda) | Automatically GPU-enabled |

#### Key Points

**CUDA Toolkit:** Don't install the CUDA toolkit manually — let pip/conda packages bring their own CUDA runtime. Modern ML frameworks like PyTorch and TensorFlow ship with their own CUDA runtime libraries (e.g., `libcudart.so`) bundled in the wheel.

**cuDNN:** Usually not needed (bundled with ML frameworks). But if you're using bare CUDA code or TensorRT, you might need `libcudnn8` via `apt` or conda.

**`--runtime nvidia` is redundant:** After you run `nvidia-ctk runtime configure --runtime=docker`, that command sets `nvidia` as the *default* runtime, so you don't need to specify `--runtime nvidia` again.

#### Verify GPU Access Inside Container

```bash
# Enter the container
docker exec -it humble-fortress-permanent bash

# Check if GPU is visible
nvidia-smi

# Verify CUDA toolkit is NOT installed (correct behavior)
nvcc --version  # Should output: command not found

# Install PyTorch to test GPU (example)
pip install torch torchvision torchaudio

# Test in Python
python3 -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}'); print(f'GPU count: {torch.cuda.device_count()}')"
```

If everything is set up correctly:
- `nvidia-smi` should show your GPU
- `nvcc --version` should NOT exist (correct — no toolkit installed)
- PyTorch (or TensorFlow) should detect the GPU automatically

#### Troubleshooting GPU Issues

**If you get "libnvidia-ml.so.1: cannot open shared object file" error:**

1. Reconfigure the runtime:

```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

2. If still failing, try without default runtime (remove `"default-runtime": "nvidia"` from `/etc/docker/daemon.json`):

```bash
sudo systemctl restart docker

docker run -d \
  --name humble-fortress-permanent \
  --restart unless-stopped \
  --privileged \
  --network host \
  --ipc=host \
  --ulimit nofile=524288 \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$HOME":"$HOME":rw \
  --runtime=nvidia \
  --gpus all \
  ros2-humble-gazebo-fortress \
  sleep infinity
```

**To run without GPU (if troubleshooting):**

Simply omit the `--gpus all` flag:

```bash
docker run -d \
  --name humble-fortress-permanent \
  --restart unless-stopped \
  --privileged \
  --network host \
  --ipc=host \
  --ulimit nofile=524288 \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$HOME":"$HOME":rw \
  ros2-humble-gazebo-fortress \
  sleep infinity
```

---

### Managing Packages

#### Strategy 1: Rebuild Image (Recommended)

Best for reproducibility. Add packages to Dockerfile, rebuild (30–60 seconds):

```bash
cd /path/to/project/src
docker build -t ros2-humble-gazebo-fortress \
  --build-arg USERNAME=$USER \
  --build-arg USER_UID=$(id -u) \
  --build-arg USER_GID=$(id -g) .
```

#### Strategy 2: Install in Persistent Container

Use persistent container (from Option B above), then install packages:

```bash
docker exec -it humble-fortress-permanent bash
sudo apt install ros-humble-some-package
pip install some-python-package
```

Everything persists across restarts.

---


---

## Usage Guide

### Launch Gazebo

Inside the container:

```bash
gz sim
```

**Launch with ROS2:**

```bash
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=empty.sdf
```

**Note:** The `ros2 launch` method may not work reliably in Docker due to discovery issues. Use `gz sim` directly instead.

Refs: [1](https://gazebosim.org/docs/fortress/ros2_launch_gazebo/)

### Quick Test

```bash
# Terminal 1 (inside container): Start Gazebo
gz sim

# Terminal 2 (inside container): Start a simple bridge
ros2 run ros_gz_bridge parameter_bridge /chatter@std_msgs/msg/String@gz.msgs.StringMsg
```

---

## ROS2 Interaction With Gazebo

### Bridge Communication between ROS and Gazebo

The following message types can be bridged for topics:

| ROS type                                    | Gazebo type                                 |
|---------------------------------------------|:-------------------------------------------:|
| geometry_msgs/msg/Twist                     | gz.msgs.Twist                               |
| sensor_msgs/msg/Imu                         | gz.msgs.IMU                                 |
| sensor_msgs/msg/Image                       | gz.msgs.Image                               |
| sensor_msgs/msg/LaserScan                   | gz.msgs.LaserScan                           |
| sensor_msgs/msg/MagneticField               | gz.msgs.Magnetometer                        |
| sensor_msgs/msg/PointCloud2                 | gz.msgs.PointCloudPacked                    |
| tf2_msgs/msg/TFMessage                      | gz.msgs.Pose_V                              |
| sensor_msgs/msg/NavSatFix                   | gz.msgs.NavSat                              |
| nav_msgs/msg/Odometry                       | gz.msgs.Odometry                            |
| sensor_msgs/msg/CameraInfo                  | gz.msgs.CameraInfo                          |

Full list here: [Bridge communication between ROS and Gazebo](https://github.com/gazebosim/ros_gz/tree/humble/ros_gz_bridge)

Refs: [1](https://gazebosim.org/docs/fortress/ros2_integration/#use-ros-2-to-interact-with-gazebo)

### 1. Simple Hello between ROS2 and Gazebo

```
ros2 run ros_gz_bridge parameter_bridge -h
```

Bridge a collection of ROS2 and Gazebo Transport topics and services.

```
  parameter_bridge [ topic_name@ROS2_type@Ign_type ]
```

Topics: The first `@` symbol delimits the topic name from the message types.

Following the first `@` symbol is the ROS message type.

The ROS message type is followed by an `@`, `[`, or `]` symbol where
    `@`  == a bidirectional bridge, 
    `[`  == a bridge from Gazebo to ROS,
    `]`  == a bridge from ROS to Gazebo.
    
Following the direction symbol is the Gazebo Transport message type.

A bidirectional bridge example:
```
ros2 run ros_gz_bridge parameter_bridge /chatter@std_msgs/msg/String@gz.msgs.StringMsg --ros-args --log-level DEBUG
```

**⚠️ IMPORTANT:** The bridge only relays messages - it doesn't create publishers or subscribers. You need:
1. **The bridge running** (creates the topic on both sides)
2. **Something actively publishing** (ROS2 or Gazebo)
3. **Something listening** (on the opposite side)

**Start the bidirectional bridge** (allows communication both ways):

Terminal 1 - Start the bridge (MUST run first):
```bash
ros2 run ros_gz_bridge parameter_bridge /chatter@std_msgs/msg/String@gz.msgs.StringMsg
```

Wait a moment for the bridge to initialize, then proceed with testing.

**Test 1: Publish from ROS2 → Receive in Gazebo**

Terminal 2 - Listen on ROS2 side (start this BEFORE publishing):
```bash
ros2 topic echo /chatter
```

Terminal 3 - Publish from ROS2:
```bash
ros2 topic pub /chatter std_msgs/msg/String '{data: "Hello from ROS2"}'
```

**Note:** Inside Docker, `gz topic` CLI tool doesn't work reliably due to shared memory/discovery issues. Use `ros2 topic echo /chatter` instead to verify the bridge is working. The bridge itself works perfectly, just the `gz topic` command has limitations in Docker.

**Test 2: Publish from Gazebo → Receive in ROS2**

Terminal 2 - Listen on ROS2 side (with bridge from Terminal 1 still running):
```bash
ros2 topic echo /chatter
```

Terminal 3 - In a different terminal (if running Gazebo separately), you can verify by checking ROS2 topics.

**One-way bridge examples:**

Gazebo to ROS only (Gazebo publishes, ROS subscribes):
```bash
ros2 run ros_gz_bridge parameter_bridge /chatter@std_msgs/msg/String]gz.msgs.StringMsg
```

ROS to Gazebo only (ROS publishes, Gazebo subscribes):
```bash
ros2 run ros_gz_bridge parameter_bridge /chatter@std_msgs/msg/String[gz.msgs.StringMsg
```

**Notes:** 
- **Docker limitation:** The `gz topic` CLI tool doesn't work inside Docker containers due to shared memory/discovery issues. However, the bridge and Gazebo plugins work perfectly! To verify the bridge is working in Docker, use `ros2 topic echo /chatter` instead. The `gz topic` commands work correctly when running on the host system.

**Troubleshooting:**

If `ros2 topic echo /chatter` shows nothing:
1. ✅ **Check the bridge is running** - You should see a `parameter_bridge` process
2. ✅ **Verify something is publishing** - Run the `ros2 topic pub` command
3. ✅ **Start listener BEFORE publishing** - The listener needs to be subscribed first
4. ✅ **Check ROS2 is sourced** - The entrypoint should handle this automatically, but verify with `echo $ROS_DISTRO`

If `ros2 topic list` shows `/chatter` but no data flows:
- The bridge is working! You just need to publish messages from one side
- Remember: The bridge doesn't generate messages, it only relays them

If working with an existing simulation (like `sensors_demo.sdf`):
- Don't use `/chatter` - bridge actual sensor topics like `/camera`, `/lidar`, etc.
- Check what topics the simulation publishes: Use `ros2 topic list` after starting Gazebo with the bridge running

Ref: [1](https://index.ros.org/p/ros_gz_bridge/)

### 2. Camera Example 

run:
```
gz sim -v 4  -r sensors_demo.sdf
```

- `-v 4`: This sets the verbosity level of Gazebo to 4. 
- `-r`:  visualize_lidar.sdf: This loads a specific SDF

list topics

```
ros2 topic list
```

bridge:

```
ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image
```

Now open RViz2 to visualize:

```bash
rviz2
```

Refs: [1](https://github.com/gazebosim/ros_gz/tree/humble/ros_gz_sim_demos#camera) 

### 3. Diff Drive Example

```
gz sim -v 4 -r visualize_lidar.sdf
```

list topics:

```
ros2 topic list
```

so mapping between `geometry_msgs/msg/Twist` and `gz.msgs.Twist`

```
ros2 run ros_gz_bridge parameter_bridge /model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist
```
which gives you: 
```
Creating ROS->GZ Bridge: [/model/vehicle_blue/cmd_vel (geometry_msgs/msg/Twist) -> /model/vehicle_blue/cmd_vel (gz.msgs.Twist)]
```

now run: 

```
rqt
```
**very important:** do not forget to clear everything publishing new messages (clicking on the broom)

or you can publish from terminal:

```
ros2 topic pub /model/vehicle_blue/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 5.0}, angular: {z: 0.5}}"
```

or with keyboard:

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/model/vehicle_blue/cmd_vel
```

now to get laser scans:

```
ros2 run ros_gz_bridge parameter_bridge /lidar2/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked
```

**very important**

```
ros2 topic echo /lidar2/points | grep "frame_id"
```

which gives you"
```
frame_id: vehicle_blue/lidar_link/gpu_lidar
```

or simply:

```
ros2 launch ros_gz_sim_demos diff_drive.launch.py
```

### 4. GPU lidar

```
gz sim -r gpu_lidar_sensor.sdf
```
mapping:
```
ros2 run ros_gz_bridge parameter_bridge lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan
```

and 

```
ros2 run ros_gz_bridge parameter_bridge /lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked
```
now run:

```
rviz2
```

**very important:**

```
ros2 topic echo /lidar/points | grep "frame_id"
```

which give us:

```
frame_id: model_with_lidar/link/gpu_lidar
```

or you can run 

```
ros2 launch ros_gz_sim_demos gpu_lidar_bridge.launch.py
```
Refs: [1](https://github.com/gazebosim/ros_gz/tree/humble/ros_gz_sim_demos#gpu-lidar)

### 5. IMU, Magnetometer

```
gz sim -r  sensors.sdf
```

```
ros2 run ros_gz_bridge parameter_bridge /imu@sensor_msgs/msg/Imu@gz.msgs.IMU

ros2 run ros_gz_bridge parameter_bridge /magnetometer@sensor_msgs/msg/MagneticField@gz.msgs.Magnetometer
```

or simply 

```
ros2 launch ros_gz_sim_demos imu.launch.py
ros2 launch ros_gz_sim_demos magnetometer.launch.py
```

### 6. GNSS

```
gz sim -r spherical_coordinates.sdf
```

find /navsat:
```
ros2 topic list
```

bridge:
        
```
ros2 run ros_gz_bridge parameter_bridge /navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat
```

### 7. Robot description publisher (spawning urdf model)

Lets download a simple model [08-macroed.urdf.xacro](https://github.com/ros/urdf_tutorial/blob/ros2/urdf/08-macroed.urdf.xacro)

```
xacro 08-macroed.urdf.xacro > 08-macroed.urdf.urdf
```

```
check_urdf 08-macroed.urdf.urdf
```

```
urdf_to_graphviz 08-macroed.urdf.urdf
```

and for SVG

```
dot -Tsvg macroed.gv -o output.svg
```

more models [here](https://wiki.ros.org/urdf/Examples)

now run:

```
gz sim empty.sdf
```

all worlds available at: `/usr/share/gz/gz-sim6/worlds/`

list all services:

```
gz service -l
```

Look for a create service.

```
/world/empty/create
```

to get the service's request and response message types:

```
gz service -is /world/empty/create
```
 which gives you:

```
gz.msgs.EntityFactory, gz.msgs.Boolean
```

spawns the URDF file model.urdf into the Gazebo Sim world as a model named urdf_model:

```
gz service -s /world/empty/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: "08-macroed.urdf.urdf", name: "08-macroed"'
```
**very important:** in the terminal that you have run `gz sim empty.sdf` you have access to `08-macroed.urdf.urdf` (it should be in the same directory)
 
another example: `rrbot.xacro`

```
xacro rrbot.xacro > rrbot.urdf
```

then run: 

```
gz service -s /world/empty/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req 'sdf_filename: "rrbot.urdf", name: "rrbot"'
```

### 8.Joint States Publisher

```
ros2 launch ros_gz_sim_demos tf_bridge.launch.py
```

in the file: `tf_bridge.launch.py`
 
change:
```
    cmd=[
        'ign', 'gazebo', '-r',
        os.path.join(
            pkg_ros_gz_sim_demos,
            'models',
            'double_pendulum_model.sdf'
        )
    ]
```

to this 

```
cmd=[
        'gz', 'sim', '-r',
        os.path.join(
            pkg_ros_gz_sim_demos,
            'models',
            'double_pendulum_model.sdf'
        )
    ]
```

---

## FAQ & Troubleshooting

### GUI apps don't work in Docker

**Symptoms:** Black windows, "Cannot open display", or GUI apps don't appear.

**Solution:**

1. **Enable X11 forwarding (must be done before running container):**

```bash
xhost +local:docker
```

2. **Ensure your docker run command includes:**

```bash
-e DISPLAY=$DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw
```

**Note:** The mount must be `:rw` (read-write), not `:ro`. Some distributions (Ubuntu 24.04, Fedora, Arch) require write access.

3. **Verify DISPLAY variable:**

```bash
echo $DISPLAY
# Should output something like: :0 or :1

# If empty, set it:
export DISPLAY=:0
```

4. **Test with simple X11 app:**

Inside container:
```bash
xclock
# Should open a clock window if GUI is working
```

### Gazebo is slow/laggy

**Check GPU:**

```bash
# Inside container
nvidia-smi
```

**If GPU not working:**

1. Make sure you used `--gpus all` flag when running container
2. Verify NVIDIA Container Toolkit is installed on host
3. Check Docker runtime configuration: `cat /etc/docker/daemon.json`

**Optimize Gazebo settings:**

```bash
# Use different render engine
gz sim --render-engine ogre2
```

**Shared memory:**

Make sure you're using `--ipc=host` flag (already in our examples).

### "Package not found" errors

**Solution 1: Rebuild image with package**

Add the package to Dockerfile and rebuild:

```bash
cd /path/to/project/src
docker build -t ros2-humble-gazebo-fortress \
  --build-arg USERNAME=$USER \
  --build-arg USER_UID=$(id -u) \
  --build-arg USER_GID=$(id -g) .
```

**Solution 2: Install in running container (if persistent)**

```bash
docker exec -it humble-fortress-permanent bash
sudo apt update && sudo apt install ros-humble-<package-name>
```

### Container won't start or exits immediately

**Check logs:**

```bash
docker logs humble-fortress-permanent
```

**Common issues:**

1. **Port conflicts:** Another container or process is using the same name/ports
   ```bash
   # Check if container name already exists
   docker ps -a | grep humble-fortress
   
   # Remove old container
   docker rm humble-fortress-permanent
   ```

2. **Permission issues:** Check that your user has docker permissions
   ```bash
   # Test docker access
   docker ps
   
   # If permission denied, add user to docker group
   sudo usermod -aG docker $USER
   # Log out and back in
   ```

### ROS2 nodes can't see each other across containers

**Solution:** Use `--network host` flag (already in our examples)

**Alternative:** Set same `ROS_DOMAIN_ID`:

```bash
# Container 1
docker run ... -e ROS_DOMAIN_ID=42 ...

# Container 2  
docker run ... -e ROS_DOMAIN_ID=42 ...
```

### GPU not detected in container

**Verify on host:**

```bash
nvidia-smi
```

**If host works but container doesn't:**

1. Check NVIDIA Container Toolkit is installed and configured
2. Verify `--gpus all` flag is in docker run command
3. Check Docker daemon configuration: `cat /etc/docker/daemon.json`
4. Restart Docker: `sudo systemctl restart docker`

**Inside container:**

```bash
nvidia-smi
# Should show GPU info

nvcc --version
# Should NOT exist (correct - no CUDA toolkit installed)
```

### Can I use Docker on a non-Ubuntu host?

✅ **Yes!** Docker works on:
- Ubuntu, Debian, Fedora, Arch (Linux)
- macOS (with some GUI limitations - requires XQuartz)
- Windows WSL2 (full support with WSLg for GUI)

### Multiple containers

**Running multiple containers:**

You can run multiple containers simultaneously. Just use different names:

```bash
docker run -d --name humble-fortress-1 ...
docker run -d --name humble-fortress-2 ...
```

**Note:** Make sure they use different ROS_DOMAIN_ID if you want them to communicate, or same ROS_DOMAIN_ID if you want them isolated.

---

## Additional Resources

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Documentation](https://gazebosim.org/docs/fortress)
- [ros_gz Bridge](https://github.com/gazebosim/ros_gz)
- [ROS2 Packages Index](https://index.ros.org/packages/)
