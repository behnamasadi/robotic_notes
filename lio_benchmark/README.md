# lio_benchmark

A self-contained Gazebo-based LiDAR-Inertial Odometry testbed: a
SubT-style rover with a 16-beam LiDAR + IMU, three cave/tunnel worlds,
FAST-LIO wired up out of the box, and an analysis script that produces
per-run trajectory + drift summaries.

Unlike VIO — which is unreliable on simulated data (see
[`../vio_benchmark/docs/VIO_DIAGNOSTIC_GUIDE.md`](../vio_benchmark/docs/VIO_DIAGNOSTIC_GUIDE.md))
— **LIO works correctly on this sim**: FAST-LIO consistently produces
0.3-1.5 % end-point error on the recordings produced here. The
architectural reason is that LIO uses IMU only as a hint for ICP
scan registration, not as the integrator on the position-estimate
path. See the diagnostic guide §3 for the full explanation.

## Layout

```
lio_benchmark/
├── src/explorer_r2_sim/         # ROS 2 package: rover model, worlds, launches
│   ├── models/explorer_r2/      # rover SDF (sensors: IMU, LiDAR, cameras)
│   ├── worlds/                  # cave.sdf, tunnel.sdf
│   ├── config/                  # bridge.yaml, lio.yaml, joy_teleop.yaml
│   ├── launch/                  # cave.launch.py, lio.launch.py, world.launch.py
│   ├── rviz/                    # RViz configs
│   ├── scripts/                 # record_run.sh, analyze_bag.py, gt_to_path.py,
│   │                            # lidar_field_adapter.py, perf_probe.sh
│   ├── docker/                  # sim.Dockerfile + entrypoint
│   └── docker-compose.yml       # one-command bring-up
├── third_party/FAST_LIO/        # submodule (hku-mars/FAST_LIO, ROS 2 branch)
└── docs/
    └── PERFORMANCE.md           # gz-sim GPU/render perf notes (EGL pivot)
```

## Quick start

```bash
# 1. Get the submodule
cd ~/workspace/robotic_notes
git submodule update --init --recursive lio_benchmark/third_party/FAST_LIO

# 2. Build the container image
cd lio_benchmark/src/explorer_r2_sim
docker compose build

# 3. Bring everything up — gz sim + ros_gz_bridge + LIO + ground-truth helper
docker compose up
```

The default `cave.launch.py` brings up:

- **gz sim** with the `tunnel` world (override with `WORLD=cave`,
  `WORLD=rubicon`, or a Fuel URL).
- **ros_gz_bridge** wiring IMU + LiDAR + cameras + ground truth onto
  ROS 2 topics. Bridge config in `config/bridge.yaml`.
- **FAST-LIO** subscribed to `/lidar/points_lio` + `/imu`, publishing
  `/Odometry` + `/path`.
- **gt_to_path** republishing gz's ground-truth pose as a normalised
  `/ground_truth/path` for easy `evo` comparison.

## Recording and analysing a run

```bash
# In a second terminal, attached to the running container:
docker compose exec sim bash
ros2 run explorer_r2_sim gt_to_path.py &
src/explorer_r2_sim/scripts/record_run.sh tunnel_drive_1
# drive via joystick (joy node is on by default) or:
#   ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Ctrl-C the recorder when done. Bag lands under runs/run_<UTC>/.
```

Then on the host:

```bash
python3 lio_benchmark/src/explorer_r2_sim/scripts/analyze_bag.py \
    ~/ros2_ws/runs/run_<stamp>
```

Output: a `summary.md` with topic rates, path lengths, end-point
errors, IMU sanity, and per-axis plots.

## Tuning knobs

Documented separately:

- LiDAR sensor + FAST-LIO config: [`../vio_benchmark/docs/PARAMETERS.md` §6](../vio_benchmark/docs/PARAMETERS.md)
- gz-sim rendering performance (EGL / Cyclone DDS / bridge channel
  trims): [`docs/PERFORMANCE.md`](docs/PERFORMANCE.md)

## Why this is a separate subproject

`lio_benchmark/` and [`../vio_benchmark/`](../vio_benchmark/) live
side-by-side in `robotic_notes/` because they answer different
questions:

| | lio_benchmark | vio_benchmark |
|---|---|---|
| Surface | simulated (this Gazebo setup) | real datasets (EuRoC, TUM-VIO, …) |
| Why | LIO is robust to simulator IMU artefacts | VIO is not (see diagnostic guide §3) |
| What's compared | one estimator (FAST-LIO) at present | OpenVINS, VINS-Fusion (stereo + mono), with ORB-SLAM3 / Kimera planned |
| Grows by | adding worlds, sensors, behaviours, more LIO algorithms | adding estimators and public datasets |

When a second LIO algorithm gets wired in (LIO-SAM, GLIM, DLIO, etc.),
the comparison machinery will start to look like vio_benchmark's
head-to-head + rerun.io setup.
