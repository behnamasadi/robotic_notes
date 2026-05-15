# vio_benchmark

A reproducible head-to-head comparison framework for **visual-inertial
odometry (VIO)** and **stereo-VO** estimators, evaluated primarily on
real-world public datasets (EuRoC MAV, TUM-VIO, KITTI) with optional
custom-sim scenarios.

The point of this repo: **establish trustworthy comparison numbers**
between modern VIO algorithms (OpenVINS, VINS-Fusion, ORB-SLAM3,
Kimera-VIO, HybVIO, …) on the same data, using the same metrics, with
the same alignment, so the comparison says something about *algorithms*
rather than test-harness differences.

## Why this repo exists

Three reasons not addressed by running estimators directly out of their
own repos:

1. **Unified evaluation pipeline.** Each estimator's upstream repo has
   its own way of running and its own output format. evo + a Python
   harness (see `scripts/`) bring them to a common TUM trajectory format
   for apples-to-apples APE/RPE/Umeyama metrics.

2. **Diagnostic methodology.** When an estimator fails, you need to know
   *why* before you tune anything. `docs/VIO_DIAGNOSTIC_GUIDE.md` is a
   structured 7-step checklist for diagnosing broken VIO — and a long
   side-discussion on why simulated IMU data is currently unreliable
   for VIO benchmarking (with worked examples).

3. **A historical/incident record.** The diagnostic guide and
   `docs/COMPARISON.md` capture not just numbers but *what we tried*,
   *what failed*, and *why* — so revisiting the comparison a year later
   doesn't repeat the same investigation.

## Current status

| | |
|---|---|
| Estimators wired | OpenVINS (stereo+IMU), VINS-Fusion (stereo+IMU) |
| Sequences run | EuRoC MAV MH_01_easy |
| Headline numbers | OpenVINS APE 0.295 m, VINS-Fusion APE 0.248 m |
| Comparison report | [`docs/COMPARISON.md`](docs/COMPARISON.md) |
| Diagnostic guide | [`docs/VIO_DIAGNOSTIC_GUIDE.md`](docs/VIO_DIAGNOSTIC_GUIDE.md) |
| Dataset recipe | [`docs/DATASETS.md`](docs/DATASETS.md) |

## Layout

```
vio_benchmark/
├── docs/                  # comparison reports, diagnostic guide, dataset recipes
├── configs/
│   ├── openvins/          # estimator_config + kalibr chain YAMLs
│   ├── vins/              # vins.yaml + cam0/cam1 calibration
│   └── euroc_groundtruth/ # TUM-format GT trajectories for all 11 EuRoC sequences
├── launch/                # vio.launch.py, vins.launch.py
├── scripts/
│   ├── analyze_bag.py     # bag-level trajectory summary + IMU sanity check
│   └── visualize_rerun.py # rerun.io comparison viewer (frustums + drift + cameras)
├── third_party/           # OpenVINS, VINS-Fusion as submodules
└── runs/                  # gitignored — estimator output recordings live here
```

Datasets themselves are not in the repo and are downloaded out-of-band
to `~/datasets/euroc/` — see [`docs/DATASETS.md`](docs/DATASETS.md)
for download links and conversion recipe.

## Usage

After downloading a sequence into `~/datasets/euroc/<seq>_ros2/`:

```bash
# Analyse a recording (writes summary.md + plots)
python3 scripts/analyze_bag.py runs/<run-dir>

# Interactive comparison in rerun.io
python3 scripts/visualize_rerun.py <run1> <run2> ... \
    --estimator-topic <topic1> <topic2> ... \
    --label <name1> <name2> ... \
    --source-bag ~/datasets/euroc/<seq>_ros2 \
    --output compare.rrd
rerun compare.rrd
```

For the end-to-end recipe (launching estimators, recording outputs,
running APE), see [`docs/COMPARISON.md`](docs/COMPARISON.md) and
[`docs/DATASETS.md`](docs/DATASETS.md).

## Roadmap

In priority order — each is a focused future session:

1. **More EuRoC sequences** (MH_02–05, V1/V2_*) — bumps coverage from
   one sequence to a defensible per-estimator average.
2. **ORB-SLAM3** — third estimator, full SLAM with loop closure,
   community ROS 2 wrapper exists.
3. **Kimera-VIO** — factor-graph perspective. Needs GTSAM built from
   source.
4. **TUM-VIO sequences** — different hardware/scenes; validates that
   numbers are not EuRoC-specific.

Skipped: VINS-Mono (mono-only, ROS 1 only, predecessor of VINS-Fusion).

## Related

- [`../lio_benchmark/`](../lio_benchmark/) — the Gazebo LiDAR-inertial
  testbed. LIO works correctly there; VIO does not (see
  [`docs/VIO_DIAGNOSTIC_GUIDE.md` §3](docs/VIO_DIAGNOSTIC_GUIDE.md#3--why-lio-survives-bad-imu-data-and-vio-doesnt)
  for the architectural reason).
- [`docs/SESSION_2026-05-15.md`](docs/SESSION_2026-05-15.md) — narrative
  log of the VIO investigation: impulses in simulated IMU, filter
  attempts, pivot to real datasets.
- Top-level [README.md](../README.md) — robotic_notes index.
