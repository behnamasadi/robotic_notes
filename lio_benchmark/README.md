# lio_benchmark

Gazebo simulator for LiDAR-inertial odometry: a SubT-style rover
(16-beam LiDAR + IMU + stereo cameras), three cave/tunnel worlds,
FAST-LIO wired in, all containerised.

## Layout

```
lio_benchmark/
├── src/explorer_r2_sim/    ROS 2 package — rover model, worlds, launches,
│   ├── models/explorer_r2/ configs, RViz, scripts, Dockerfile + compose
│   ├── worlds/             cave.sdf, tunnel.sdf
│   ├── config/             bridge.yaml, lio.yaml, joy_teleop.yaml
│   ├── launch/             cave.launch.py, lio.launch.py, world.launch.py
│   ├── rviz/               RViz configs
│   ├── scripts/            record_run.sh, analyze_bag.py, gt_to_path.py, …
│   ├── docker/             sim.Dockerfile + entrypoint
│   └── docker-compose.yml
├── third_party/FAST_LIO/   submodule (hku-mars/FAST_LIO, ROS 2 branch)
└── docs/PERFORMANCE.md     gz-sim GPU/render perf notes (EGL pivot)
```

## Quick start

```bash
# fetch the FAST-LIO submodule
cd ~/workspace/robotic_notes
git submodule update --init --recursive lio_benchmark/third_party/FAST_LIO

# build the image (one-time) and bring everything up
cd lio_benchmark/src/explorer_r2_sim
docker compose build
docker compose up
```

`docker compose up` launches `cave.launch.py` which starts:

- **gz sim** with the `tunnel` world (override with `WORLD=cave`,
  `WORLD=rubicon`, or any preset / local SDF / Fuel URL).
- **ros_gz_bridge** wiring IMU, LiDAR, cameras, and ground truth
  onto ROS 2 topics. Config in `config/bridge.yaml`.
- **FAST-LIO** subscribed to `/lidar/points_lio` + `/imu`,
  publishing `/Odometry` + `/path`.
- **gt_to_path** republishing ground truth as a normalised
  `/ground_truth/path` for direct `evo` comparison.

Drive the rover with a joystick (the `joy_node` is on by default) or:

```bash
docker compose exec sim ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Recording a run

```bash
# from a second terminal attached to the container
docker compose exec sim bash
ros2 run explorer_r2_sim gt_to_path.py &
src/explorer_r2_sim/scripts/record_run.sh tunnel_drive_1
# drive, then Ctrl-C the recorder. Bag lands under runs/run_<UTC>/.
```

## Analysis

```bash
python3 lio_benchmark/src/explorer_r2_sim/scripts/analyze_bag.py \
    ~/ros2_ws/runs/run_<stamp>
```

Output: `summary.md` with topic rates, path lengths, end-point
errors, IMU sanity, per-axis plots.

## Tuning reference

- LiDAR sensor + FAST-LIO `lio.yaml` knobs:
  [`../vio_benchmark/docs/PARAMETERS.md` §6](../vio_benchmark/docs/PARAMETERS.md#6-lio--fast-lio-with-the-rover-sim)
- Simulator GPU / rendering performance:
  [`docs/PERFORMANCE.md`](docs/PERFORMANCE.md)

## Trajectory evaluation

The same metrics and methodology used for VIO benchmarking apply:

- APE / RPE / Umeyama: [`../vio_benchmark/docs/ANALYSIS.md`](../vio_benchmark/docs/ANALYSIS.md)
- Diagnostic procedure when results look off: [`../vio_benchmark/docs/VIO_DIAGNOSTIC_GUIDE.md`](../vio_benchmark/docs/VIO_DIAGNOSTIC_GUIDE.md)
