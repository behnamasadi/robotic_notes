# Estimator parameters and tuning reference

A single place that documents the **estimator-tuning knobs** in
OpenVINS and VINS-Fusion — what each one is, what value we use, and
*why*. Calibration values (intrinsics, extrinsics, IMU noise) live in
the per-dataset config files under [`../configs/`](../configs/); this
doc covers the algorithm-side knobs.

---

## 1. Coordinate conventions

VIO codebases use two conventions and you have to be explicit about
which is which everywhere:

| Frame                  | Convention                          | Where it appears                            |
|------------------------|-------------------------------------|---------------------------------------------|
| **Body / REP-103**     | X forward, Y left, Z up             | IMU body frame, world frame in most ROS work |
| **Optical (image)**    | X right, Y down, Z forward          | OpenVINS / VINS-Fusion internal camera frame |
| **OpenVINS world**     | `global` (REP-103)                  | `/ov_msckf/*` headers                       |
| **VINS-Fusion world**  | `world` (REP-103)                   | `/odometry` etc. headers                    |
| **FAST-LIO world**     | `camera_init` (REP-103)             | `/Odometry`, `/path`, `/cloud_registered` headers |

The fixed rotation between body and optical frames goes in each
estimator's calibration YAML — see §3 below for OpenVINS' `T_imu_cam`
matrix and §5 for VINS-Fusion's `body_T_cam0`.

---

## 2. IMU calibration — kalibr-style YAMLs

OpenVINS and VINS-Fusion both use the kalibr noise-density model.

### Continuous-time noise densities from per-sample stddev

If your IMU source advertises a per-sample standard deviation `σ_s`
at rate `f`, the continuous-time noise density is

```
density = σ_s / sqrt(f)
```

For a typical Bosch BMI088-class IMU running at 200–250 Hz:

```
gyro_noise_density  ≈ 1.27e-4 rad/s/√Hz   (≈ 0.115 deg/s @ 250 Hz σ_s = 2.0e-3)
accel_noise_density ≈ 1.90e-3 m/s²/√Hz    (≈ 250 Hz σ_s = 3.0e-2)
gyro_random_walk    ≈ 1.0e-5  rad/s²/√Hz
accel_random_walk   ≈ 6.0e-4  m/s³/√Hz
```

These are reasonable defaults for a mid-tier MEMS unit. Multiply
stddev by 5–10× to simulate a cheap MPU-9250-class part.

### OpenVINS `kalibr_imu_chain.yaml`

```yaml
imu0:
  T_i_b:                # IMU body → robot body frame (identity if IMU at origin)
    - [1, 0, 0, 0]
    - [0, 1, 0, 0]
    - [0, 0, 1, 0]
    - [0, 0, 0, 1]
  accelerometer_noise_density: 1.90e-3
  accelerometer_random_walk:   6.00e-4
  gyroscope_noise_density:     1.27e-4
  gyroscope_random_walk:       1.00e-5
  rostopic: /imu0           # whatever your bag publishes
  update_rate: 200.0        # match the source
  model: "kalibr"
  Tw / Ta:                   # intrinsic scale + cross-coupling — identity for perfect IMU
  R_IMUtoACC / R_IMUtoGYRO:  # frame rotation IMU → sensing element — identity unless calibrated
  Tg:                        # gyro g-sensitivity — zero unless calibrated
```

The `kalibr` model in OpenVINS *requires* `Tw`, `Ta`, `R_IMUtoACC`,
`R_IMUtoGYRO`, `Tg` to be present even when identity / zero. Omitting
them produces "unable to parse … Eigen::Matrix" errors and a SIGSEGV
at startup.

### VINS-Fusion `vins.yaml`

```yaml
acc_n: 1.90e-3           # accelerometer measurement noise density  (m/s²/√Hz)
gyr_n: 1.27e-4           # gyroscope noise density                  (rad/s/√Hz)
acc_w: 6.00e-4           # accelerometer bias random walk           (m/s³/√Hz)
gyr_w: 1.00e-5           # gyroscope bias random walk               (rad/s²/√Hz)
g_norm: 9.81             # gravity magnitude
```

Same numbers as OpenVINS, just renamed. Keep them in lockstep — a
discrepancy is a common silent source of "estimator A and B disagree
on the same bag."

---

## 3. Camera calibration

### OpenVINS `kalibr_imucam_chain.yaml`

```yaml
cam0:
  T_imu_cam:                                # camera-optical → IMU, 4×4 SE(3)
    - [ 0.0,  0.0,  1.0,  0.565]            # optical Z (forward)  → IMU X (forward)
    - [-1.0,  0.0,  0.0,  0.000]            # optical X (right)    → IMU −Y
    - [ 0.0, -1.0,  0.0,  0.245]            # optical Y (down)     → IMU −Z
    - [ 0.0,  0.0,  0.0,  1.000]
  camera_model: pinhole
  distortion_model: radtan
  distortion_coeffs: [-0.02, 0.003, 0.0, 0.0]   # k1, k2, p1, p2
  intrinsics: [554.26, 554.26, 320.5, 240.5]    # fu, fv, cu, cv
  resolution: [640, 480]
  rostopic: /cam0/image_raw
```

The fixed rotation block at the top-left 3×3 is the body→optical
convention swap: optical-Z→body-X (forward), optical-X→body-(−Y)
(right→left), optical-Y→body-(−Z) (down→up). Translation goes in the
last column. **Keep `distortion_coeffs` and the source camera's
distortion in lockstep.** A mismatch shows up as systematic drift on
straight-line drives.

### VINS-Fusion `cam0.yaml` + extrinsics in `vins.yaml`

```yaml
# cam0.yaml — pinhole intrinsics + radtan distortion
model_type: PINHOLE
camera_name: cam0
image_width: 640
image_height: 480
distortion_parameters:
   k1: -0.02
   k2:  0.003
   p1:  0.0
   p2:  0.0
projection_parameters:
   fx: 554.26
   fy: 554.26
   cx: 320.5
   cy: 240.5
```

```yaml
# vins.yaml — extrinsics, same SE(3) matrix as OpenVINS' T_imu_cam
body_T_cam0: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [ 0.0,  0.0,  1.0,  0.565,
          -1.0,  0.0,  0.0,  0.000,
           0.0, -1.0,  0.0,  0.245,
           0.0,  0.0,  0.0,  1.000 ]
```

---

## 4. OpenVINS `estimator_config.yaml`

The defaults shipped by OpenVINS are tuned for real-robot data
(noisy sensors, imperfect factory calibration). A few keys *need*
to flip for clean data — without them VIO will drift wildly even
with the platform stationary, or fail to initialise.

### Online calibration — all OFF on clean data

```yaml
calib_cam_extrinsics:    false
calib_cam_intrinsics:    false
calib_cam_timeoffset:    false
calib_imu_intrinsics:    false
calib_imu_g_sensitivity: false
```

**Why:** when every camera/IMU parameter is derived from a known
source (SDF, factory cal, kalibr offline run) there's nothing to
refine. Letting OpenVINS run the online-calibration filter on
already-correct data adds numerical noise that gets integrated into
the pose estimate and the trajectory drifts. On a real robot with
imperfect factory calibration you usually leave these on.

### Initialisation

```yaml
init_window_time:    5.0       # seconds of data to collect for static init
init_imu_thresh:     0.3       # m/s² — accel variance needed to declare "motion"
init_max_disparity:  10.0      # px — pixel disparity max to declare "stationary"
init_max_features:   50        # features tracked during init
init_dyn_use:        true      # fall back to dynamic init if static fails
```

**`init_window_time: 5 s`** averages over enough stationary samples
to nail the gravity direction.

**`init_imu_thresh: 0.3 m/s²`** is the variance of accelerometer
magnitude required within `init_window_time` for OpenVINS to detect
the platform has been "jerked." Default is `1.5 m/s²`, tuned for
real IMUs where the noise floor is several times higher. With clean
data (accel σ ≤ 1e-2 m/s²) `1.5` is so high you have to literally
yank the platform to trigger init; `0.3` triggers reliably on a
gentle drive start.

**`init_dyn_use: true`** enables OpenVINS' dynamic-initialisation
fallback: if the static path fails (no jerk), it bootstraps from
camera-IMU motion + parallax. Important for setups where the
platform might be commanded into smooth motion without an initial
jolt.

### Zero-velocity updates (ZUPT)

```yaml
try_zupt:               true
zupt_chi2_multipler:    0      # 0 = disparity-only detector
zupt_max_velocity:      0.1    # m/s — speed below which ZUPT can activate
zupt_noise_multiplier:  10
zupt_max_disparity:     0.5    # px — pixel disparity max to declare "stationary"
zupt_only_at_beginning: false
```

**What ZUPT does.** Zero-velocity update. When OpenVINS detects the
platform is stationary (small disparity in the camera + small
predicted velocity), it applies a "velocity = 0" measurement to the
filter. This constraint:

- Zeroes out the velocity state → drift stops immediately.
- Re-estimates IMU bias from the no-motion samples → bias error
  decays back toward truth instead of growing.

**Why it matters.** Without ZUPT, while the platform sits still the
IMU's random-walk bias gets integrated forever:

```
drift_position(t) = ∫∫ bias(t') dt' dt'   →   grows quadratically
```

User-visible symptom: VIO trail keeps growing even when the robot
isn't moving. With `try_zupt: true`, the filter detects the
stationary state every few frames and clamps everything.

`zupt_max_disparity: 0.5` is the maximum pixel motion below which we
consider the camera stationary. Tune up if you have lots of
background motion (e.g. wind in trees), down if the camera is very
stable.

### Filter sizing

```yaml
max_clones:          11        # sliding-window clones for MSCKF
max_slam:            50        # SLAM features kept in state
max_slam_in_update:  25        # batch size for SLAM update
max_msckf_in_update: 40        # batch size for MSCKF update
dt_slam_delay:       1         # seconds before initialising SLAM after VIO init
```

Defaults from OpenVINS; haven't needed to touch. Bigger numbers →
more accurate, more CPU. Drop to `max_clones: 8`, `max_slam: 25` on
a Raspberry Pi-class device.

### Tracker (front-end)

```yaml
use_klt:           true        # KLT (true) vs ORB + descriptor matching (false)
num_pts:           200         # features extracted per camera per frame
fast_threshold:    20
grid_x:            5
grid_y:            5
min_px_dist:       10
track_frequency:   21.0        # Hz — sub-rate of the camera
histogram_method:  HISTOGRAM   # NONE / HISTOGRAM / CLAHE
```

**`use_klt: true` vs `false`** — this is the descriptor question. KLT
(Lucas-Kanade pyramidal optical flow on FAST corners) is fast and
excellent for small frame-to-frame motion, but on self-similar
texture (e.g. corridors, tiled floors, hex-lattice walls) it can
drift to a neighbouring identical patch when a feature is partially
occluded. With `use_klt: false`, OpenVINS switches to FAST + ORB
descriptors + brute-force kNN matching with a Lowe's-ratio filter
(`knn_ratio: 0.70`) — ambiguous matches are rejected automatically,
which helps in self-similar scenes specifically. Cost is some extra
CPU per frame. See [`ANALYSIS.md` "Tracker choice: KLT vs descriptor
matching"](ANALYSIS.md#tracker-choice-klt-vs-descriptor-matching)
for the longer discussion.

**Conservative defaults — and a hard lesson.** Tuning a clean
simulator camera with "real-camera" knobs can backfire: lowering
`fast_threshold` to 10, bumping `num_pts` to 400, and switching to
CLAHE on a clean low-noise image stream amplifies the Gaussian
noise floor into "fake structure" that the tracker locks onto. The
filter then integrates nonsense — observed result in past runs:
~22 km of reported motion on a 58 m drive (path-length ratio ≈ 380).

If you're working with a real (noisy, low-light) camera, flip the
knobs in the *opposite* direction: `histogram_method: CLAHE`,
`num_pts: 400`, `fast_threshold: 10`. For pristine sim data, the
conservative defaults are correct.

---

## 5. VINS-Fusion `vins.yaml`

```yaml
imu: 1                          # 1 = use IMU, 0 = visual-only
num_of_cam: 2                   # 1 = mono, 2 = stereo

imu_topic:    /imu0
image0_topic: /cam0/image_raw
image1_topic: /cam1/image_raw

cam0_calib:  cam0.yaml
cam1_calib:  cam1.yaml
image_width: 640
image_height: 480

estimate_extrinsic: 0           # 0 = trust YAML, 1 = init + refine, 2 = estimate from scratch
estimate_td:        0           # 0 = fixed time offset (td), 1 = online estimate
td:                 0.00        # IMU-camera time offset; 0 for hardware-synced data

max_cnt:        200             # max features in tracking
min_dist:       30              # min pixel distance between features
freq:           10              # tracker publish rate (Hz)
F_threshold:    1.0             # RANSAC pixel threshold
show_track:     1               # publish /vins/image_track for RViz
flow_back:      1               # forward+backward optical flow check

max_solver_time:    0.04        # max solver time (s) per iteration — real-time bound
max_num_iterations: 8
keyframe_parallax: 10.0         # keyframe selection threshold (px)
```

**`estimate_extrinsic: 0`** when calibration is known exactly (sim or
recently-kalibr'd hardware). Use `1` if you have a rough initial
guess. `2` if you have no idea — but expect a long, twitchy init.

**`estimate_td: 0`** when IMU and cameras share a hardware clock or
`/clock`. Set to `1` only if you have measurable jitter between the
two streams.

**`freq: 10`** is the *tracker* publish rate, not the camera rate.
VINS-Fusion downsamples internally — useful on resource-limited
platforms. For full-quality benchmarks, set to camera rate (e.g.
`20` for EuRoC's 20 Hz cameras).

**`flow_back: 1`** runs optical flow forward + backward and rejects
features that don't round-trip; this is VINS-Fusion's equivalent of
the kNN-ratio test in descriptor matching. Strongly recommended on.

---

## 6. LIO — FAST-LIO with the rover sim

Unlike VIO, **LIO works fine on simulated data** (see
[`VIO_DIAGNOSTIC_GUIDE.md` §3](VIO_DIAGNOSTIC_GUIDE.md#3--why-lio-survives-bad-imu-data-and-vio-doesnt)
for the architectural reason). The rover sim at `~/ros2_ws/` is the
canonical LIO deployment for this project. The knobs below are for
that setup, not for a public-dataset run.

### LiDAR sensor (rover SDF)

The rover's mast-mounted `gpu_lidar` sensor in
`models/explorer_r2/model.sdf`:

| Param         | Value                       | SDF tag             |
|---------------|-----------------------------|---------------------|
| Type          | `gpu_lidar`                 | `type="gpu_lidar"`  |
| Pose          | `(0, 0, 1.05)`, no rotation | `<pose>`            |
| Update rate   | 15 Hz                       | `<update_rate>`     |
| Horizontal    | 1024 samples × 2π           | `<scan><horizontal>`|
| Vertical      | 16 samples × ±15°           | `<scan><vertical>`  |
| Range         | 0.05 – 100 m                | `<range>`           |
| Range noise σ | 0.01 m                      | `<noise><stddev>`   |

Z = 1.05 m above `base_link` puts the lidar above the payload box;
at z < ~0.65 it raycasts into the chassis.

### Field adapter — `lidar_field_adapter.py`

gz's `gpu_lidar` publishes a basic `sensor_msgs/PointCloud2` with
only `x`, `y`, `z`, `intensity`. FAST-LIO's Ouster preprocessor
refuses to process scans without per-point `ring` (laser-line index)
and `t` (timestamp offset within the scan). The adapter:

- Subscribes to `/lidar/points` (raw gz output).
- Computes `ring` from each point's elevation angle, matching the
  16 vertical samples × ±15° geometry.
- Computes `t` linearly from azimuth angle (15 Hz scan period).
- Republishes as `/lidar/points_lio` with the full
  `ouster_ros::Point` field set (`ring` is `uint8` to match
  FAST-LIO's struct definition; `uint16` silently fails to bind).

`config/lio.yaml`'s `lid_topic` points at `/lidar/points_lio`, not
the raw `/lidar/points`.

### FAST-LIO `config/lio.yaml`

| Key                          | Value                | Why                                                                  |
|------------------------------|----------------------|----------------------------------------------------------------------|
| `common.lid_topic`           | `/lidar/points_lio`  | Adapter output, not the raw gz topic.                                |
| `common.imu_topic`           | `/imu`               | Same as everywhere else in the sim.                                  |
| `preprocess.lidar_type`      | 3                    | Ouster. 1 = Livox, 2 = Velodyne, 3 = Ouster.                         |
| `preprocess.scan_line`       | 16                   | Matches `<scan><vertical><samples>` in the SDF.                      |
| `preprocess.scan_rate`       | 15                   | Hz; matches `<update_rate>`.                                         |
| `preprocess.timestamp_unit`  | 3                    | nanoseconds; matches `t` field emitted by the adapter.               |
| `preprocess.blind`           | 0.5 m                | Points closer than this are discarded (self-hits / wheels).          |
| `mapping.extrinsic_T`        | `[0, 0, 1.05]`       | Lidar position in IMU frame (= base_link), straight from the SDF.    |
| `mapping.extrinsic_R`        | identity             | Lidar has no rotation relative to IMU.                               |
| `mapping.extrinsic_est_en`   | `false`              | Calibration is exact in sim; don't let the filter modify it.         |
| `mapping.acc_cov / gyr_cov`  | 0.1 / 0.01           | Process-noise covariance scales. Loose-ish defaults that work with the gz IMU. |
| `mapping.b_acc_cov / b_gyr_cov` | 0.001 / 8.0e-7    | Bias process-noise. Matches the gz IMU's bias random walk.           |
| `publish.path_en`            | `true`               | `/path` for RViz.                                                    |
| `pcd_save.pcd_save_en`       | `false`              | Use rosbag + evo, not PCD dump.                                      |

This setup gives FAST-LIO 0.3–1.5 % end-point error on rover sim
recordings — competitive with what FAST-LIO reports on KITTI.

---

## Related

- [`ANALYSIS.md`](ANALYSIS.md) — methodology + concepts (parallax,
  KLT vs descriptor, mono unobservable yaw, APE/RPE/Umeyama).
- [`COMPARISON.md`](COMPARISON.md) — head-to-head numbers from
  EuRoC.
- [`VIO_DIAGNOSTIC_GUIDE.md`](VIO_DIAGNOSTIC_GUIDE.md) — 7-step
  diagnostic procedure for broken VIO + LIO-vs-VIO IMU role.
