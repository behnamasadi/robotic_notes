- [ROS2 Docker](#ros2-docker)




## ROS2 Docker



#### **Official mapping (2024–2025)**

| ROS 2 distro | ros_gz git branch | Required GZ_VERSION value | Actual Gazebo release name |
|--------------|-------------------|----------------------------|----------------------------|
| ROS Jazzy    | `jazzy`           | `harmonic`                 | Gazebo Harmonic            |
| ROS Iron     | `iron`            | `garden`                   | Gazebo Garden              |
| ROS Humble   | `humble`          | `fortress`                 | Gazebo Fortress            |


---

#### 1. Create this file exactly once (put it in your project root or `~/ros2_docker/`)

`Dockerfile` (no extension needed):

```dockerfile
FROM osrf/ros:jazzy-desktop-full

ARG USERNAME=behnam
ARG USER_UID=1000
ARG USER_GID=1000

# Remove the default user/group that ships with UID/GID 1000 (name varies)
# and make sure UID 1000 + GID 1000 are completely free
RUN set -e && \
    # Try to remove any existing user with UID 1000
    (id -u 1000 >/dev/null 2>&1 && userdel -r $(id -un 1000)) || true && \
    # Try to remove any existing group with GID 1000
    (getent group 1000 >/dev/null 2>&1 && groupdel $(getent group 1000 | cut -d: -f1)) || true && \
    # Install sudo if missing (it usually is already there)
    apt-get update && apt-get install -y --no-install-recommends sudo && \
    rm -rf /var/lib/apt/lists/* && \
    # Now safely create your own group and user
    groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} \
            --create-home --shell /bin/bash ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME} && \
    usermod -aG dialout,video ${USERNAME}

# Switch to your user forever
USER ${USERNAME}
WORKDIR /home/${USERNAME}
ENV USER=${USERNAME}

# Nice orange prompt so you know you're inside the container
RUN echo '# Orange ROS2-Jazzy prompt (only when inside container)' >> /etc/bash.bashrc && \
    echo 'export PS1="\[\e[38;5;208m\][ROS2-Jazzy \u@\h] \[\e[0;33m\]\w\[\e[0m\] \$ "' >> /etc/bash.bashrc && \
    echo 'export GZ_VERSION=harmonic' >> /etc/bash.bashrc
```

#### 2. Build it once (takes ~30 seconds)

```bash
docker build -t ros:jazzy \
  --build-arg USER_UID=$(id -u) \
  --build-arg USER_GID=$(id -g) \
  --build-arg USERNAME=$USER .
```

#### 3. Run it 

```bash
# First, allow GUI apps from Docker containers
xhost +local:docker

# (fixed, predictable container name)
docker run -it --rm \
  --name jazzy \
  --privileged \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$HOME":"$HOME":rw \
  --workdir "$PWD" \
  --ipc=host \
  --ulimit nofile=524288 \
  ros:jazzy \
  bash
```  



| Option | Why it matters |
|-----|----------------|
| `-v /tmp/.X11-unix:/tmp/.X11-unix:rw` | Some distributions (Ubuntu 22.04/24.04, Fedora, Arch, etc.) require **read-write** access to the X11 socket, even though it’s technically read-only. Using `:ro` silently fails → black windows or “Cannot open display”. |
| `--ipc=host` | ROS 2 tools (especially rviz2, Gazebo, Foxglove) heavily use shared memory (`/dev/shm`). Without this flag you get crashes or extremely slow rendering. |
| `--ulimit nofile=524288` | Prevents “Too many open files” errors when running large ROS 2 nodes or rviz2 with many topics. |
|`ros:jazzy`      | Image   |
|`jazzy` (or whatever you like)   |Container |




#### Why this is the real best practice today

- Never runs as root  
- Files created inside → owned by you on the host (no more `sudo chown -R`)  
- Proper `$HOME`, bash history, sudo, rosdep, colcon all work perfectly  
- GUI apps (rviz2, rqt, Foxglove Studio) work out of the box  
- Fully reproducible — commit the Dockerfile to your repo and everyone gets the exact same setup  
- Works even if your UID is 1001, 1337, 50000, etc.  



* UID **1000** is clean
* GID **1000** is clean
* Your user (`behnam`) is created without conflicts
* All files inside the mounted host workspace `/home/behnam` stay owned by you
* colcon, rosdep, RViz2, Gazebo, Foxglove all run perfectly
* No surprises caused by “mystery users” that OSRF sometimes adds to base images

Removing the existing UID/GID 1000 (if present) is sometimes the *only* reliable fix when working with OSRF images that already include stub users.

---

####  Why this Dockerfile is the most robust solution

**1. OSRF base images often ship with a default user**

Sometimes:

```
uid=1000(dev)
gid=1000(dev)
```

This version **removes them cleanly**, avoiding collisions.

---

**2. You remove both the user AND the group with UID/GID 1000**

This guarantees those IDs are free:

```
id -u 1000 -> remove user
groupdel $(getent group 1000)
```

This is the strongest way to ensure a deterministic environment.

---

**3. You recreate the group + user cleanly**

You do:

```
groupadd --gid 1000 behnam
useradd --uid 1000 --gid 1000 behnam
```

This ensures the container’s internal user is **identical** to your host user.

This is the foundation for:

* correct permissions
* correct ownership of builds
* correct home directory behavior
* correct colcon + rosdep + pip cache paths

---


## Installing New Packages

The current alias uses `--rm`, so every time you exit the container it disappears → all `apt install`, `pip install`, `rosdep` system packages you installed are **gone** the next time you start it again.

Here are the three real-world solutions people actually use:

#### Option 1 – Recommended

**Re-build the image whenever you need new system packages.**  
It’s only 30–60 seconds and gives you a perfectly clean, reproducible environment forever.

1. Add the missing packages to your Dockerfile (just one extra line):

```dockerfile
# ~/ros2_docker/Dockerfile  ← 100 % working version – November 2025
FROM osrf/ros:jazzy-desktop-full

ARG USERNAME=behnam
ARG USER_UID=1000
ARG USER_GID=1000

# 1. Clean up the default UID/GID 1000 user that the base image ships with
RUN set -e && \
    (id -u 1000 >/dev/null 2>&1 && userdel -r $(id -un 1000)) || true && \
    (getent group 1000 >/dev/null 2>&1 && groupdel $(getent group 1000 | cut -d: -f1)) || true

# 2. Create your own user (same UID/GID as host)
RUN apt-get update && apt-get install -y --no-install-recommends sudo && \
    rm -rf /var/lib/apt/lists/* && \
    groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} \
            --create-home --shell /bin/bash ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME} && \
    usermod -aG dialout,video ${USERNAME}

# 3. Install the system packages you actually need (add more here anytime)
#     → these were the ones that failed for you + a few very common ones
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-libg2o \
        libceres-dev \
        libsuitesparse-dev \
        ros-jazzy-bond \
        ros-jazzy-bondcpp \
        ros-jazzy-nav2-map-server \
        ros-jazzy-nav2-bringup \
        ros-jazzy-navigation2 \
        ros-jazzy-urdf-launch \
        ros-jazzy-simulation-interfaces \
        gz-harmonic \
        python3-pip \
        python3-colcon-common-extensions \
        git \
        wget \
        curl \
    && rm -rf /var/lib/apt/lists/*

# Switch to your user for the rest of eternity
USER ${USERNAME}
WORKDIR /home/${USERNAME}

# ──────────────────────────────────────────────────────────────
# Add a nice orange prompt so you instantly know you're inside the container
# ──────────────────────────────────────────────────────────────
RUN echo '# Orange ROS2-Jazzy prompt (only when inside container)' >> /etc/bash.bashrc && \
    echo 'export PS1="\[\e[38;5;208m\][ROS2-Jazzy \u@\h] \[\e[0;33m\]\w\[\e[0m\] \$ "' >> /etc/bash.bashrc && \
    echo 'export GZ_VERSION=harmonic' >> /etc/bash.bashrc

ENV USER=${USERNAME}
```


2. Re-build once (30–60 seconds):

```bash
cd ~/ros2_docker
docker build -t ros:jazzy \
  \
  --build-arg USERNAME=$USER \
  --build-arg USER_UID=$(id -u) \
  --build-arg USER_GID=$(id -g) .
```

3. Keep using the same fast alias with `--rm`:

```bash
alias jazzy='docker run -it --rm --name jazzy --privileged --network host \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v $HOME:$HOME:rw --workdir "$PWD" ros:jazzy'
```

→ Now every new container already has `libg2o`, `nav2-map-server`, `ceres`, etc. pre-installed.  
You only rebuild when you actually need a new system package → perfect reproducibility.

#### Option 2 – Zero rebuilds ever

Keep a **persistent container** instead of `--rm`.

```bash
# ──────────────────────────────────────────────────────────────
# 1. Run this ONLY ONCE (or when you want to recreate the container)
# ──────────────────────────────────────────────────────────────
xhost +local:docker   # allow GUI from all local containers (safe)

docker run -d \
  --name jazzy-permanent \
  --restart unless-stopped \
  --privileged \
  --network host \
  --ipc=host \
  --ulimit nofile=524288 \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$HOME":"$HOME":rw \
  -v "$PWD":"$PWD":rw \
  --workdir "$PWD" \
  ros:jazzy \
  sleep infinity
```

Now you can `sudo apt install …`, `pip install …` as much as you want — everything survives reboots and new terminals.


You can start:


```bash
docker start -i jazzy-permanent
```
or attach:

```
docker exec -it jazzy-permanent bash
````


---


## Launch Gazebo from ROS 2


```bash
source /opt/ros/jazzy/setup.sh
gz sim
```

or (may not work over docker)

```
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=empty.sdf
```

Refs: [1](https://gazebosim.org/docs/harmonic/ros2_launch_gazebo/)



## ROS2 Interaction With Gazebo

### Bridge communication between ROS and Gazebo


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
| sensor_msgs/msg/Image                       | gz.msgs.Image                               |
| sensor_msgs/msg/MagneticField	              | gz.msgs.Magnetometer                        |
| sensor_msgs/msg/NavSatFix                   | gz.msgs.NavSat                              |
| nav_msgs/msg/Odometry                       | gz.msgs.Odometry                            |
| sensor_msgs/msg/CameraInfo                  | gz.msgs.CameraInfo                          |

	

full list here: [Bridge communication between ROS and Gazebo](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge)

Refs: [1](https://gazebosim.org/docs/harmonic/ros2_integration/#use-ros-2-to-interact-with-gazebo)


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


If the `ros_gz_bridge` node doesn't publish anything, you might happen to have wrong version or gazebo. If you install ROS 2 Humble with the default, the "Ignition Fortress" will be installed. so you need to use (pay attention to `@ignition`):


```
ros2 run ros_gz_bridge parameter_bridge /chatter@std_msgs/msg/String@ignition.msgs.StringMsg --ros-args --log-level DEBUG

```

[read more here](https://docs.px4.io/main/en/ros2/user_guide.html#ros-gz-bridge-not-publishing-on-the-clock-topic)



**Start the bidirectional bridge** (allows communication both ways):

```bash
source /opt/ros/jazzy/setup.sh
```


```bash
ros2 run ros_gz_bridge parameter_bridge /chatter@std_msgs/msg/String@gz.msgs.StringMsg
```

**Test 1: Publish from ROS2 → Receive in Gazebo**

Terminal 1 - Listen on Gazebo side:
```bash
gz topic -e -t /chatter
```

Terminal 2 - Publish from ROS2:
```bash
ros2 topic pub /chatter std_msgs/msg/String '{data: "Hello from ROS2"}'
```

**Test 2: Publish from Gazebo → Receive in ROS2**

Terminal 1 - Listen on ROS2 side:
```bash
ros2 topic echo /chatter
```

Terminal 2 - Publish from Gazebo:
```bash
gz topic -t /chatter -m gz.msgs.StringMsg -p 'data:"Hello from Gazebo"'
```

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
gz topic -l
```
then for `/camera`:


```
gz topic -i -t /camera
```

which gives us:
 
```
Publishers [Address, Message Type]:
  tcp://172.17.0.1:33325, gz.msgs.Image
Subscribers [Address, Message Type]:
  tcp://172.17.0.1:43429, gz.msgs.Image
```
 
 
mapping:

```
ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image
```
	


<img src="images/camera_gazebo_rviz.png" />


Refs: [1](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_sim_demos#camera) 
 


### 3. Diff Drive Example

```
gz sim -v 4 -r visualize_lidar.sdf
```

list topic:

```
gz topic -l
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
<img src="images/rqt_cmd_vel.png" />


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


<img src="images/diff_drive_gazebo_rviz.png" />


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

<img src="images/rviz_lidar.png" />

or you can run 


```
ros2 launch ros_gz_sim_demos gpu_lidar_bridge.launch.py
```
Refs: [1](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_sim_demos#gpu-lidar)



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
gz topic -i -t /navsat
```

which gives us:

```
Publishers [Address, Message Type]:
  tcp://172.17.0.1:43221, gz.msgs.NavSat
No subscribers on topic [/navsat]
```

now mapping:
        
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

<img src="images/08-macroed.svg" />


more models [here](https://wiki.ros.org/urdf/Examples)



now run:


```
gz sim empty.sdf
```

all worlds available at: `/usr/share/gz/gz-sim8/worlds/`


list all services:

```
gz service -l
```

Look for a create service.

```
/world/empty/create
```

to get the service’s request and response message types:


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
**very important:**: in the terminal that you have run `gz sim empty.sdf` you have access to `08-macroed.urdf.urdf` (it should be in the same directory)
 
another example: `rrbot.xacro`

```
<?xml version="1.0"?>
<!-- Revolute-Revolute Manipulator -->
<robot name="rrbot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Constants for robot dimensions -->
  <xacro:property name="PI" value="3.1415926535897931"/>
  <xacro:property name="mass" value="1" /> <!-- arbitrary value for mass -->
  <xacro:property name="width" value="0.1" /> <!-- Square dimensions (widthxwidth) of beams -->
  <xacro:property name="height1" value="2" /> <!-- Link 1 -->
  <xacro:property name="height2" value="1" /> <!-- Link 2 -->
  <xacro:property name="height3" value="1" /> <!-- Link 3 -->
  <xacro:property name="axel_offset" value="0.05" /> <!-- Space btw top of beam and the each joint -->

  <!-- Define colors -->
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>

  <material name="orange">
    <color rgba="${255/255} ${108/255} ${10/255} 1.0"/>
  </material>

  <!-- Used for fixing robot to Gazebo 'base_link' -->
  <link name="world"/>

  <joint name="fixed" type="fixed">
    <parent link="world"/>
    <child link="link1"/>
  </joint>

  <!-- Base Link -->
  <link name="link1">
    <collision>
      <origin xyz="0 0 ${height1/2}" rpy="0 0 0"/>
      <geometry>
        <box size="${width} ${width} ${height1}"/>
      </geometry>
    </collision>

    <visual>
      <origin xyz="0 0 ${height1/2}" rpy="0 0 0"/>
      <geometry>
        <box size="${width} ${width} ${height1}"/>
      </geometry>
      <material name="orange"/>
    </visual>

    <inertial>
      <origin xyz="0 0 ${height1/2}" rpy="0 0 0"/>
      <mass value="${mass}"/>
      <inertia
    ixx="${mass / 12.0 * (width*width + height1*height1)}" ixy="0.0" ixz="0.0"
    iyy="${mass / 12.0 * (height1*height1 + width*width)}" iyz="0.0"
    izz="${mass / 12.0 * (width*width + width*width)}"/>
    </inertial>
  </link>

  <joint name="joint1" type="continuous">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 ${width} ${height1 - axel_offset}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <dynamics damping="0.7"/>
  </joint>

  <!-- Middle Link -->
  <link name="link2">
    <collision>
      <origin xyz="0 0 ${height2/2 - axel_offset}" rpy="0 0 0"/>
      <geometry>
        <box size="${width} ${width} ${height2}"/>
      </geometry>
    </collision>

    <visual>
      <origin xyz="0 0 ${height2/2 - axel_offset}" rpy="0 0 0"/>
      <geometry>
        <box size="${width} ${width} ${height2}"/>
      </geometry>
      <material name="black"/>
    </visual>

    <inertial>
      <origin xyz="0 0 ${height2/2 - axel_offset}" rpy="0 0 0"/>
      <mass value="${mass}"/>
      <inertia
    ixx="${mass / 12.0 * (width*width + height2*height2)}" ixy="0.0" ixz="0.0"
    iyy="${mass / 12.0 * (height2*height2 + width*width)}" iyz="0.0"
    izz="${mass / 12.0 * (width*width + width*width)}"/>
    </inertial>
  </link>

  <joint name="joint2" type="continuous">
    <parent link="link2"/>
    <child link="link3"/>
    <origin xyz="0 ${width} ${height2 - axel_offset*2}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <dynamics damping="0.7"/>
  </joint>

  <!-- Top Link -->
  <link name="link3">
    <collision>
      <origin xyz="0 0 ${height3/2 - axel_offset}" rpy="0 0 0"/>
      <geometry>
        <box size="${width} ${width} ${height3}"/>
      </geometry>
    </collision>

    <visual>
      <origin xyz="0 0 ${height3/2 - axel_offset}" rpy="0 0 0"/>
      <geometry>
        <box size="${width} ${width} ${height3}"/>
      </geometry>
      <material name="orange"/>
    </visual>

    <inertial>
      <origin xyz="0 0 ${height3/2 - axel_offset}" rpy="0 0 0"/>
      <mass value="${mass}"/>
      <inertia
    ixx="${mass / 12.0 * (width*width + height3*height3)}" ixy="0.0" ixz="0.0"
    iyy="${mass / 12.0 * (height3*height3 + width*width)}" iyz="0.0"
    izz="${mass / 12.0 * (width*width + width*width)}"/>
    </inertial>
  </link>

  <!-- Gazebo colors and frictions -->
  <!-- Link1 -->
  <gazebo reference="link1">
    <material>
      <diffuse> 1 0.423529412 0.039215686 1</diffuse>
      <ambient> 1 0.423529412 0.039215686 1</ambient>
      <specular>1 0.423529412 0.039215686 1</specular>
    </material>
  </gazebo>

  <!-- Link2 -->
  <gazebo reference="link2">
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <material>
      <diffuse> 0 0 0 1</diffuse>
      <ambient> 0 0 0 1</ambient>
      <specular>0 0 0 1</specular>
    </material>
  </gazebo>

  <!-- Link3 -->
  <gazebo reference="link3">
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
    <material>
      <diffuse> 1 0.423529412 0.039215686 1</diffuse>
      <ambient> 1 0.423529412 0.039215686 1</ambient>
      <specular>1 0.423529412 0.039215686 1</specular>
    </material>
  </gazebo>

  <!-- Joint states plugin -->
  <gazebo>
    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher"/>
  </gazebo>

</robot>
```

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

<img src="images/tf_bridge.gif" />


