# VIO/LIO datasets

Reference notes on the real-world public datasets used to benchmark
the estimators in this repo. Downloads are out-of-band — keep the
large bag files outside the repository tree under
`~/datasets/<dataset>/`.

## Why real datasets

VIO is hard to benchmark in a simulator because gz-sim's IMU has no
bandwidth model and gz-sim's diff-drive produces impulsive contact
forces no real platform would generate. Both compound into IMU
signals real hardware can't produce, which break VIO algorithms
tuned on real data — see
[`VIO_DIAGNOSTIC_GUIDE.md` §4`](VIO_DIAGNOSTIC_GUIDE.md#4--on-trusting-simulated-imu-data)
for the full discussion. The fix is to use real datasets for VIO
quality numbers and keep the simulator for LIO and behavioural
work.


## Result on this stack

Stereo OpenVINS, upstream EuRoC reference configuration, no
per-sequence tuning, on EuRoC MH_01_easy (80.99 m laser-tracker GT
path):

```
Path-length ratio (estimator / GT) : 0.9762
Umeyama scale factor                : 0.9875
APE RMSE (rotation+translation)     : 0.226 m
APE RMSE (with scale, Umeyama)      : 0.221 m
Max error                           : 0.581 m
```

For context, sim recordings with the same OpenVINS binary produced
path-length ratios of 3-10×. Same build flags, same calibration —
only the data source differs. The OpenVINS install is sound; the
gap from 0.22 m → published ~0.05 m is per-sequence parameter
tuning, not debugging.

Trajectory plot: [`euroc_mh01_trajectory.png`](euroc_mh01_trajectory.png).


## EuRoC MAV

The de-facto VIO benchmark — 11 sequences across two environments,
hardware-synced stereo + IMU, mm-level Vicon / laser-tracker GT.

| Environment | Sequences | GT |
|---|---|---|
| Machine Hall | `MH_01_easy` → `MH_05_difficult` | Leica laser tracker, position only on `/leica/position` |
| Vicon Room 1 | `V1_01_easy` → `V1_03_difficult` | Vicon, full pose on `/vicon/firefly_sbx/firefly_sbx` |
| Vicon Room 2 | `V2_01_easy` → `V2_03_difficult` | Vicon |

### What's inside an EuRoC bag

```
/imu0               200 Hz   sensor_msgs/Imu          ADIS16448
/cam0/image_raw      20 Hz   sensor_msgs/Image        752×480 mono, MT9V034
/cam1/image_raw      20 Hz   sensor_msgs/Image        right of stereo pair
/leica/position      17 Hz   geometry_msgs/PointStamped   (MH only — position only)
/vicon/...           ~100 Hz geometry_msgs/TransformStamped  (V1/V2 — full pose)
```

### Where to get the bags

The official source at `robotics.ethz.ch` has been intermittently
unreachable. The OpenVINS team maintains a Google Drive mirror that
has been the reliable alternative — and bonus, those bags are
already in rosbag2 format so no conversion step is needed.

Mirror index page: [docs.openvins.com/gs-datasets.html](https://docs.openvins.com/gs-datasets.html)

Direct Google Drive links:

| Sequence | Link |
|---|---|
| MH_01_easy | https://drive.google.com/file/d/1UP4nkuSEOQECZTswwh9BPgfMl-dnDstA/view |
| MH_02_easy | https://drive.google.com/file/d/1wWZgZCqYz6zzzTXS0iqvQCP-cWfFuGLK/view |
| MH_03_medium | https://drive.google.com/file/d/1er07gZ8rso8R3Su00hJMm_GZ4z1n9Rpq/view |
| MH_04_difficult | https://drive.google.com/file/d/1eC8joRXo1rh0wzOpq3e-B4dQ8-w6wZYz/view |
| MH_05_difficult | https://drive.google.com/file/d/1zoN94K1Afrp7HXSduRLkJBiEjPKdk1UA/view |
| V1_01_easy | https://drive.google.com/file/d/1LFrdiMU6UBjtFfXPHzjJ4L7iDIXcdhvh/view |
| V1_02_medium | https://drive.google.com/file/d/1rlGSy7h38ucm8jr8ssH-sJPX84JfkBtX/view |
| V1_03_difficult | https://drive.google.com/file/d/1Gy1zc4LaMlwsLpXBqOIci6Y3cV_5r-0k/view |
| V2_01_easy | https://drive.google.com/file/d/1KAkE8Ptq3eSQlXMozJgzNIAVUBH3h0FP/view |
| V2_02_medium | https://drive.google.com/file/d/1Gj4psmvcAwYwCp4T4CQH-d2ZVJ09d3x2/view |
| V2_03_difficult | https://drive.google.com/file/d/1ohWd0JqDvVhTqjOS5MqHafit5MPdlbff/view |

Original ETH project page: [projects.asl.ethz.ch/datasets/euroc-mav](https://projects.asl.ethz.ch/datasets/euroc-mav/).

Store downloaded bags under `~/datasets/euroc/` so the analysis
scripts can find them. ROS 1 bags need conversion to ROS 2 mcap
(via the `rosbags` Python library); the OpenVINS mirror bags are
already in rosbag2 format and can be played directly.

### Expected APE on EuRoC

Published baselines from the OpenVINS, VINS-Fusion, and Kimera-VIO
papers for stereo+IMU mode:

| Sequence | OpenVINS-stereo | VINS-Fusion-stereo |
|---|---|---|
| MH_01_easy | 0.05–0.15 m | 0.07–0.16 m |
| MH_03_medium | 0.10–0.20 m | 0.10–0.25 m |
| V1_01_easy | 0.05–0.10 m | 0.06–0.15 m |

### Caveats

- **MH_01–05 ground truth (Leica) is position only** — no
  orientation. Vicon-room sequences (V1, V2) have full pose. `evo`'s
  APE handles both; RPE needs pose pairs.
- **Estimator world frame ≠ Leica/Vicon world frame.** Each
  estimator's `global` / `world` / `camera_init` frame is whatever
  it chose at init — typically a yaw rotation away from the dataset
  frame, sometimes mirrored. Always SE(3)-align before computing
  APE; that's `evo_ape -a` (or `-va` to also allow a scale factor).
- **EuRoC IMU and camera timestamps are hardware-synced**, so
  `evo_ape --t_max_diff 0.01` is tight enough.


## Other VIO datasets worth adding

| Dataset | Platform | What it adds | Gotcha |
|---|---|---|---|
| [TUM-VIO](https://vision.in.tum.de/data/datasets/visual-inertial-dataset) | handheld | 26 sequences, different IMU (Bosch BMI160) — validates that estimator numbers are not EuRoC-specific | big downloads (~7 GB total) |
| [M2DGR](https://github.com/SJTU-ViSYS/M2DGR) | ground robot | closest public dataset to a SubT-style rover — ground vehicle + LiDAR + camera + IMU | ROS 1 format, needs conversion |
| [4Seasons](https://www.4seasons-dataset.com/) | BMW autonomous car | day/night/weather/seasonal variation — stress-test conditions | huge (hundreds of GB across all sequences) |
| [NTU-VIRAL](https://ntu-aris.github.io/ntu_viral_dataset/) | UAV | multi-modal: LiDAR + IMU + cameras, comparable across VIO and LIO | smaller community |

### Not appropriate for VIO benchmarking

- **KITTI** — IMU is only 10 Hz (OXTS GPS-INS, not a high-rate
  MEMS). Designed for stereo VO, not VIO. Most VIO papers don't
  report on KITTI.
- **Cityscapes / nuScenes / Waymo** — no high-rate IMU.
- **TUM-RGBD** — no IMU at all.


## Related

- [`COMPARISON.md`](COMPARISON.md) — head-to-head OpenVINS vs
  VINS-Fusion on EuRoC MH_01_easy
- [`ANALYSIS.md`](ANALYSIS.md) — methodology (APE / RPE / Umeyama,
  parallax, mono yaw, KLT vs descriptor)
- [`VIO_DIAGNOSTIC_GUIDE.md`](VIO_DIAGNOSTIC_GUIDE.md) — what to do
  when an estimator is broken
- [`PARAMETERS.md`](PARAMETERS.md) — estimator config-knob
  reference
