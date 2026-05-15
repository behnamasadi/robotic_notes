# cave_robot

Differential-drive robot in a SubT-style cave for ROS 2 + Gazebo. Runs in
Docker — no host install of ROS or Gazebo needed.

```
src/cave_robot/
├── worlds/cave.sdf            ← world + vehicle + sensors
├── config/bridge.yaml         ← every ROS↔GZ topic, declarative
├── launch/cave_robot.launch.py← gz sim + bridge + RViz + (optional) teleop
├── rviz/cave_robot.rviz       ← TF, Odometry, PointCloud2, Image, IMU
├── docker/sim.Dockerfile      ← osrf/ros + gazebo + ros_gz + xterm
├── docker/sim-entrypoint.sh   ← sources overlay then exec
├── docker-compose.yml         ← host net, X11, NVIDIA GPU, Fuel cache volume
└── .env                       ← ROS_DISTRO / GZ_PKG / GZ_CMD switches
```

## Sensors and topics

| Sensor        | GZ topic           | ROS topic               | Type                          |
|---------------|--------------------|-------------------------|-------------------------------|
| 32-beam lidar | `/lidar/points/points` | `/lidar/points`     | `sensor_msgs/PointCloud2`     |
|               | `/lidar/points`    | `/lidar/scan`           | `sensor_msgs/LaserScan`       |
| RGB camera    | `/camera`          | `/camera/image_raw`     | `sensor_msgs/Image`           |
|               | `/camera_info`     | `/camera/camera_info`   | `sensor_msgs/CameraInfo`      |
| IMU @ 200 Hz  | `/imu`             | `/imu`                  | `sensor_msgs/Imu`             |
| Magnetometer  | `/magnetometer`    | `/magnetometer`         | `sensor_msgs/MagneticField`   |
| DiffDrive cmd | `/cmd_vel`         | `/cmd_vel`              | `geometry_msgs/Twist`         |
| Odometry @ 50 Hz | `/model/vehicle/odometry` | `/model/vehicle/odometry` | `nav_msgs/Odometry` |
| TF            | `/model/vehicle/tf`| `/model/vehicle/tf`     | `tf2_msgs/TFMessage`          |

## Quick start

```bash
cd ~/ros2_ws/src/cave_robot

# Allow X clients from the container (run once per session).
xhost +local:root

# Build + run the full stack (gz sim + bridge + RViz).
docker compose up --build
```

First launch downloads ~50 MB of Fuel cave tiles into a docker volume; later
runs are cached.

### Drive the robot

In a separate terminal **on the host** (anything sharing `ROS_DOMAIN_ID=42`
will see the bus):

```bash
docker compose run --rm sim \
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Or include teleop directly in the launch (opens an `xterm`):

```bash
docker compose run --rm sim \
  ros2 launch cave_robot cave_robot.launch.py teleop:=true
```

### Inspect topics

```bash
docker compose run --rm sim ros2 topic list
docker compose run --rm sim ros2 topic hz /lidar/points
docker compose run --rm sim ros2 topic echo /imu --once
```

## Sensor fusion (next step)

The bridge already exposes everything an EKF needs. Drop a
[`robot_localization`](https://docs.ros.org/en/jazzy/p/robot_localization/)
node into a new launch file:

```python
Node(
  package='robot_localization',
  executable='ekf_node',
  name='ekf',
  parameters=[{
    'use_sim_time': True,
    'frequency': 50.0,
    'odom0': '/model/vehicle/odometry',
    'odom0_config': [True,  True,  False,  # x, y, z
                     False, False, True,   # roll, pitch, yaw
                     True,  True,  False,  # vx, vy, vz
                     False, False, True,   # vroll, vpitch, vyaw
                     False, False, False],
    'imu0': '/imu',
    'imu0_config': [False, False, False,
                    True,  True,  True,
                    False, False, False,
                    True,  True,  True,
                    True,  True,  True],
    'world_frame': 'vehicle/odom',
    'odom_frame':  'vehicle/odom',
    'base_link_frame': 'vehicle/base_link',
  }]
)
```

Add `ros-${ROS_DISTRO}-robot-localization` to `docker/sim.Dockerfile`, then
the EKF will publish the fused `vehicle/odom → vehicle/base_link` TF.

## Switching ROS / Gazebo versions

Edit `.env`:

```bash
ROS_DISTRO=humble
GZ_PKG=ignition-fortress
GZ_CMD=ign gazebo
```

…then in `worlds/cave.sdf` change every `gz-sim-*-system` plugin filename to
`ignition-gazebo-*-system`. Rebuild with `docker compose build`.

## Differences from the original `my_tunnel.sdf` / `tunnel` notes

- **World**: SubT *cave* tiles instead of the long tunnel (matches the
  original "robot in a cave" intent). Five tiles total instead of 119.
- **Vehicle**: minimal box-on-wheels diff drive instead of the 600-line
  EXPLORER_R2 Fuel mesh — much faster to spawn and easier to modify. Swap
  back to R2 by replacing the `<model name="vehicle">` block.
- **Bridge**: declarative YAML instead of one bash command per topic. Fixes
  the `]gz.msgs.Twist` typo on `/cmd_vel`.
- **DiffDrive**: `odom_publish_frequency` is **50 Hz** (was 1 Hz, too slow
  for SLAM/EKF).
- **Lidar**: 1024×32 instead of 2048×32 to keep frame budget tight; range
  pulled to 50 m which is plenty for cave-scale.
- **Camera**: 1280×720 instead of 1920×1080 — same reason.
- **Removed**: the `<plugin name="gz::sim" filename="dummy">` level-streaming
  block (filename was literally "dummy" — no plugin loaded).
- **Frames**: explicit `gz_frame_id` per sensor so RViz TF tree is sane.
