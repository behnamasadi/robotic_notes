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

Worked example on **EuRoC MH_01_easy** (the sequence the headline numbers
above came from). Download recipe is in [`docs/DATASETS.md`](docs/DATASETS.md);
the bag should land at `~/datasets/euroc/MH_01_easy_ros2/`.

Everything below runs inside the `lio_benchmark` sim container — it
already has ROS 2 Jazzy + an entrypoint that auto-builds anything under
`lio_benchmark/third_party/*` (including OpenVINS and VINS-Fusion).
`~/datasets/` is mounted read-only at `/datasets/` inside the container.

### 0. One-time setup

Expose the estimators to the container by symlinking them from
`vio_benchmark/third_party/` into `lio_benchmark/third_party/` (where
the entrypoint looks):

```bash
cd ~/workspace/robotic_notes/lio_benchmark/third_party
ln -s ../../vio_benchmark/third_party/open_vins         open_vins
ln -s ../../vio_benchmark/third_party/VINS-Fusion-ROS2  VINS-Fusion-ROS2
```

Then build:

```bash
cd ~/workspace/robotic_notes/lio_benchmark/src/explorer_r2_sim
BUILD=force docker compose build
```

### 1. Run OpenVINS on MH_01_easy

Open an interactive shell in the container (overrides the default
`cave.launch.py` so gz-sim doesn't start — we just want the ROS 2
environment for bag replay):

```bash
docker compose run --rm sim bash
```

Inside the container, three panes (use `tmux` or three separate
`docker compose exec sim bash` shells):

```bash
# Pane 1 — estimator
ros2 launch ov_msckf subscribe.launch.py config:=euroc_mav

# Pane 2 — record estimator output + GT
ros2 bag record -o /ws/vio_benchmark/runs/euroc_mh01_ov_solo \
    /ov_msckf/odomimu /leica/position

# Pane 3 — play the EuRoC bag with sim time
ros2 bag play /datasets/euroc/MH_01_easy_ros2 --clock
```

When the bag finishes, SIGINT pane 2's Python child (`pkill -f
ros2_bag_record_python`, **not** the wrapper bash) so it flushes
`metadata.yaml`. The bag lands under
`vio_benchmark/runs/euroc_mh01_ov_solo/` on the host (the workspace is
bind-mounted, so the recording persists outside the container).

For VINS-Fusion: substitute the launch with `ros2 run vins vins_node
/ws/vio_benchmark/configs/vins/euroc_stereo_imu_config.yaml` and record
`/vins_estimator/odometry` into
`/ws/vio_benchmark/runs/euroc_mh01_vins_solo`.

### 2. Analyse the recording

Still inside the container:

```bash
python3 /ws/vio_benchmark/scripts/analyze_bag.py \
    /ws/vio_benchmark/runs/euroc_mh01_ov_solo
```

Writes `vio_benchmark/runs/euroc_mh01_ov_solo/summary.md` with:
- topic rates (e.g. `/ov_msckf/odomimu` ~200 Hz, `/leica/position` ~50 Hz)
- estimator vs GT path lengths
- end-point error
- IMU sanity stats (mean accel, gyro variance, drop count)

### 3. Compare two estimators visually

```bash
python3 /ws/vio_benchmark/scripts/visualize_rerun.py \
    /ws/vio_benchmark/runs/euroc_mh01_ov_solo \
    /ws/vio_benchmark/runs/euroc_mh01_vins_solo \
    --estimator-topic /ov_msckf/odomimu /vins_estimator/odometry \
    --label OpenVINS VINS-Fusion \
    --source-bag /datasets/euroc/MH_01_easy_ros2 \
    --output /ws/vio_benchmark/runs/compare.rrd
```

Open `vio_benchmark/runs/compare.rrd` with `rerun` on the host (rerun
viewer is host-side; the `.rrd` file is portable). Shows both
estimator trajectories + the Leica GT, frustums for the stereo
cameras, and a per-axis drift plot.

### Expected result on MH_01_easy

| estimator   | APE RMSE | matched samples | output topic             |
|-------------|----------|------------------|--------------------------|
| OpenVINS    | 0.295 m  | ~36 000 (200 Hz) | `/ov_msckf/odomimu`      |
| VINS-Fusion | 0.248 m  | ~1 800 (10 Hz)   | `/vins_estimator/odometry` |

Trajectory + APE plots already committed:
[`docs/euroc_mh01_trajectory.png`](docs/euroc_mh01_trajectory.png),
[`docs/euroc_mh01_compare.png`](docs/euroc_mh01_compare.png).

For interpreting the numbers (what APE/RPE/Umeyama mean), see
[`../docs/visual_odometry/trajectory_analysis.ipynb`](../docs/visual_odometry/trajectory_analysis.ipynb).
For deeper end-to-end notes (alternative launch flags, more EuRoC
sequences, common failure modes), see
[`docs/COMPARISON.md`](docs/COMPARISON.md).

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
