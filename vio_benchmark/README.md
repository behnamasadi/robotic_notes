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
│   └── euroc_groundtruth/ # TUM-format GT trajectories for all 11 EuRoC sequences
├── scripts/
│   ├── run_euroc.sh       # end-to-end pipeline: download → convert → run → analyze
│   └── analyze_bag.py     # bag-level trajectory summary, IMU sanity check
└── runs/                  # estimator output recordings, symlinked from external storage
```

Datasets themselves are NOT in the repo — they're large (~3 GB per
EuRoC bag) and freely re-downloadable. They live in `~/datasets/euroc/`
on the dev machine.

## Quick start

Reproduce the headline MH_01_easy comparison:

```bash
# 1. Get EuRoC MH_01_easy.bag (ROS 1 bag, ~2.7 GB)
mkdir -p ~/datasets/euroc
wget -P ~/datasets/euroc/ \
    http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.bag

# 2. Convert ROS 1 → ROS 2 mcap
pip install rosbags
rosbags-convert --src ~/datasets/euroc/MH_01_easy.bag \
                --dst ~/datasets/euroc/MH_01_easy_ros2 \
                --dst-storage mcap

# 3. Run estimators against it (requires a ROS 2 Jazzy container with
#    OpenVINS + VINS-Fusion built — see the rover-sim repo's Dockerfile)
scripts/run_euroc.sh

# 4. Compute APE
python3 scripts/analyze_bag.py runs/euroc_mh01_ov_solo
python3 scripts/analyze_bag.py runs/euroc_mh01_vins_solo
```

Full recipe in [`docs/DATASETS.md`](docs/DATASETS.md).

## Roadmap

In priority order — each is a focused future session:

1. **MH_03_medium + V1_01_easy sequences** — bumps to 2 estimators × 3
   sequences. Blocked on ETH's server being reachable (was down 2026-05-15).
2. **ORB-SLAM3** — third estimator, fundamentally different architecture
   (full SLAM with loop closure), ROS 2 wrapper exists. ~half a day.
3. **rerun.io visualization** — replace matplotlib plots with a synced
   3D-pose / camera-image / drift-over-time scene. ~1 hour.
4. **PX4 drone in gz-sim experiment** — test whether wheel-free dynamics
   produces a clean enough IMU stream that sim VIO becomes viable.
   ~half a day.
5. **Kimera-VIO** — needs GTSAM built from source (~30 min build). ~half
   day total. Adds factor-graph perspective.
6. **TUM-VIO sequences** — different hardware/scenes, validates that
   numbers aren't EuRoC-specific.

Skipped: VINS-Mono (mono-only, ROS 1 only, predecessor of VINS-Fusion).

## Related

This repo was now part of `robotic_notes/`. See top-level README.

Originally extracted from the [SubT-rover simulator
repo](../../ros2_ws/) where the VIO investigation originally happened.
The rover sim repo retains:
- The gazebo simulation infrastructure
- LIO comparisons (FAST-LIO works fine in sim, unlike VIO — see the
  diagnostic guide's §4b for why)
- Behavioral / scenario testing
- SubT-style cave/tunnel scenarios

See the rover sim's `docs/SESSION_2026-05-15.md` for the narrative log
of how the VIO investigation unfolded (impulses in simulated IMU,
filter attempts, eventually pivoting to real datasets).
