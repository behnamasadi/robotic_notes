# Parameters and tuning reference

A single place that documents every sensor in the SDF and every
estimator-tuning knob — what it is, what value we use, and *why*.

If you change something in `models/explorer_r2/model.sdf` or in
`config/`, update the matching section here so the README and the
config stay in sync.

---

## 1. Coordinate conventions

| Frame              | Convention                          | Where it appears                                       |
|--------------------|-------------------------------------|--------------------------------------------------------|
| **Gazebo / REP-103** | X forward, Y left, Z up           | World, `explorer_r2/base_link`, all gz sensor frames    |
| **Optical (image)** | X right, Y down, Z forward        | OpenVINS' internal camera frame                         |
| **OpenVINS world**  | `global` (REP-103)                | `/ov_msckf/*` headers                                   |
| **FAST_LIO world**  | `camera_init` (REP-103)           | `/Odometry`, `/path`, `/cloud_registered` headers       |
| **gz world**        | World SDF name (`tunnel`, `cave`, …) | gz scene_broadcaster, bridged into `/ground_truth/pose` |

Two **static identity** transforms wire the estimator world frames into
the sim's TF tree so RViz can draw everything in one orbit view:

| Static TF                 | Parent              | Child           | Started by         |
|---------------------------|---------------------|-----------------|--------------------|
| `vio_world_to_odom`       | `explorer_r2/odom`  | `global`        | `vio.launch.py`    |
| `lio_world_to_odom`       | `explorer_r2/odom`  | `camera_init`   | `lio.launch.py`    |

Static **identity** is intentional: at t=0 the estimators are aligned
with ground truth at the origin, so VIO/LIO drift relative to this
static link is exactly the visual gap between the colored trails and
the yellow GT trail in `rviz/sim.rviz`.

---

## 2. Sensor positions on `base_link`

All sensor poses are derived from `models/explorer_r2/model.sdf` and
expressed in `base_link` (gz/REP-103). The IMU sits at the link origin
with no rotation, so IMU frame = base_link frame.

| Sensor          | Pose (x, y, z, roll, pitch, yaw) [m, rad] | Notes                                  |
|-----------------|--------------------------------------------|-----------------------------------------|
| `imu_sensor`    | `(0.000, 0.000, 0.000, 0, 0, 0)`           | Origin = base_link                      |
| `magnetometer`  | `(0.000, 0.000, 0.000, 0, 0, 0)`           | Same link as IMU                        |
| `navsat`        | `(0.000, 0.000, 0.000, 0, 0, 0)`           | Same link as IMU                        |
| `front_laser`   | `(0.000, 0.000, 1.050, 0, 0, 0)`           | Mast-mounted; clears the payload box    |
| `rs_front`      | `(0.565, 0.000, 0.245, 0, 0, 0)`           | RGBD, forward                            |
| `rs_back`       | `(0.250, 0.000, 0.432, 0, 0, π)`           | RGBD, aft                                |
| `rs_left`       | `(0.365, 0.133, 0.426, 0, 0, +π/2)`        | RGBD, port                               |
| `rs_right`      | `(0.365, −0.133, 0.426, 0, 0, −π/2)`       | RGBD, starboard                          |

(Gz frame convention: x forward, y left, z up. Optical-frame conversion
adds the rotation `R_imu←opt = [[0,0,1],[−1,0,0],[0,−1,0]]` baked into
`config/openvins/kalibr_imucam_chain.yaml`.)

---

## 3. IMU

### SDF (`models/explorer_r2/model.sdf` → `<sensor name="imu_sensor">`)

| Quantity                     | Value                | SDF tag                              |
|------------------------------|----------------------|--------------------------------------|
| Update rate                  | 250 Hz               | `<update_rate>250</update_rate>`     |
| Gyro per-sample stddev       | 2.0 × 10⁻³ rad/s    | `<angular_velocity><…><stddev>`      |
| Gyro bias mean / stddev      | 1.0e-4 / 3.0e-5      | `<bias_mean>`, `<bias_stddev>`       |
| Gyro dynamic bias stddev     | 1.0e-5               | `<dynamic_bias_stddev>`              |
| Gyro bias correlation time   | 500 s                | `<dynamic_bias_correlation_time>`    |
| Accel per-sample stddev      | 3.0 × 10⁻² m/s²     | `<linear_acceleration><…><stddev>`   |
| Accel bias mean / stddev     | 5.0e-3 / 1.0e-3      | `<bias_mean>`, `<bias_stddev>`       |
| Accel dynamic bias stddev    | 2.0e-4               | `<dynamic_bias_stddev>`              |
| Accel bias correlation time  | 200 s                | `<dynamic_bias_correlation_time>`    |

Grade: roughly Bosch BMI088 / Xsens MTi-3 (mid-tier MEMS, what you'd put
on a hobby/research robot). Multiply stddev by 5–10× to simulate a
cheap MPU-9250-class part.

### OpenVINS YAML (`config/openvins/kalibr_imu_chain.yaml`)

Per-sample stddev → continuous noise density:

    density = stddev / sqrt(rate)
    gyro:    2.0e-3 / sqrt(250) ≈ 1.27e-4 rad/s/√Hz
    accel:   3.0e-2 / sqrt(250) ≈ 1.90e-3 m/s²/√Hz

```yaml
imu0:
  T_i_b:                # IMU body in base_link — identity (IMU at link origin)
    - [1, 0, 0, 0]
    - [0, 1, 0, 0]
    - [0, 0, 1, 0]
    - [0, 0, 0, 1]
  accelerometer_noise_density: 1.90e-3
  accelerometer_random_walk:   6.00e-4
  gyroscope_noise_density:     1.27e-4
  gyroscope_random_walk:       1.00e-5
  rostopic: /imu
  update_rate: 250.0
  model: "kalibr"
  Tw / Ta:                    # intrinsic scale + cross-coupling — identity (perfect-sim IMU)
  R_IMUtoACC / R_IMUtoGYRO:   # frame rotation IMU → sensing element — identity
  Tg:                         # gyro g-sensitivity — zero (no correction needed)
```

The `kalibr` model in OpenVINS *requires* `Tw`, `Ta`, `R_IMUtoACC`,
`R_IMUtoGYRO`, `Tg` to be present even when they're identity/zero. Omitting
them produces "unable to parse … Eigen::Matrix" errors and a SIGSEGV at
startup.

---

## 4. Camera — `rs_front`

### SDF (`<sensor name="rs_front" type="rgbd_camera">`)

| Param            | Value                                      | SDF tag                       |
|------------------|--------------------------------------------|-------------------------------|
| Pose (in `base_link`) | `(0.565, 0, 0.245)`, no rotation     | `<pose>`                      |
| Update rate      | 30 Hz                                      | `<update_rate>`               |
| Image width      | 640 px                                     | `<image><width>`              |
| Image height     | 480 px                                     | `<image><height>`             |
| H-FOV            | 1.0472 rad (60°)                           | `<horizontal_fov>`            |
| Format           | `R8G8B8`                                   | `<image><format>`             |
| fx, fy           | 554.26, 554.26                             | `<intrinsics><fx>`, `<fy>`    |
| cx, cy           | 320.5, 240.5                               | `<intrinsics><cx>`, `<cy>`    |
| Distortion model | radial-tangential (`radtan`)               | `<distortion>` block          |
| k1, k2           | −0.02, 0.003 (mild barrel)                 | `<k1>`, `<k2>`                |
| p1, p2           | 0, 0                                       | `<p1>`, `<p2>`                |
| Image noise σ    | 0.01 (norm. 0–1; ≈ 2.5 grey levels of 255) | `<noise><stddev>`             |
| Depth clip       | near 0.1 m, far 10 m                       | `<depth_camera><clip>`        |

A perfect pinhole sim camera is too easy on VIO — useful as a baseline,
but doesn't catch projection-model bugs. The current numbers are
RealSense D435-like (60° HFOV is narrower than the real D435, so
distortion is small but non-zero).

### OpenVINS YAML (`config/openvins/kalibr_imucam_chain.yaml`)

```yaml
cam0:
  T_imu_cam:                                # camera-optical → IMU, 4×4 SE(3)
    - [ 0.0,  0.0,  1.0,  0.565]            # optical Z (forward)  → IMU X (forward)
    - [-1.0,  0.0,  0.0,  0.000]            # optical X (right)    → IMU −Y
    - [ 0.0, -1.0,  0.0,  0.245]            # optical Y (down)     → IMU −Z
    - [ 0.0,  0.0,  0.0,  1.000]
  camera_model: pinhole
  distortion_model: radtan
  distortion_coeffs: [-0.02, 0.003, 0.0, 0.0]   # MUST match the SDF
  intrinsics: [554.26, 554.26, 320.5, 240.5]    # fu, fv, cu, cv  (640×480)
  resolution: [320, 240]
  rostopic: /rs_front/image
```

**Keep `distortion_coeffs` and the SDF `<distortion>` block in lockstep.**
A mismatch shows up as systematic VIO drift on straight-line drives.

---

## 5. LiDAR — `front_laser`

### SDF (`<sensor name="front_laser" type="gpu_lidar">`)

| Param        | Value                | SDF tag                       |
|--------------|----------------------|-------------------------------|
| Type         | `gpu_lidar`          | `type="gpu_lidar"`            |
| Pose         | `(0, 0, 1.05)`, no rotation | `<pose>`                |
| Update rate  | 15 Hz                | `<update_rate>`               |
| Horizontal   | 1024 samples × 2π    | `<scan><horizontal>`          |
| Vertical     | 16 samples × ±15°    | `<scan><vertical>`            |
| Range        | 0.05 – 100 m         | `<range><min>`, `<max>`       |
| Range noise σ| 0.01 m               | `<noise><stddev>`             |

Z = 1.05 m above `base_link` puts the lidar above the payload box; at
z < ~0.65 it raycasts into the chassis.

### Field adapter (`scripts/lidar_field_adapter.py`)

gz `gpu_lidar` publishes a basic `sensor_msgs/PointCloud2` with only
`x`, `y`, `z`, `intensity`. FAST_LIO's Ouster preprocessor refuses to
process scans without per-point `ring` (laser-line index) and `t`
(timestamp offset within the scan). The adapter:

- Subscribes to `/lidar/points` (raw gz output).
- Computes `ring` from each point's elevation angle, matching the 16
  vertical samples × ±15° geometry of the SDF.
- Computes `t` linearly from azimuth angle (15 Hz scan period).
- Republishes as `/lidar/points_lio` with the full `ouster_ros::Point`
  field set (`ring` is `uint8` to match FAST_LIO's struct definition;
  using `uint16` here silently fails to bind).

`config/lio.yaml`'s `lid_topic` points at `/lidar/points_lio`, not the
raw `/lidar/points`.

---

## 6. FAST_LIO config — `config/lio.yaml`

| Key                          | Value                | Why                                                                                                                                                        |
|------------------------------|----------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------|
| `common.lid_topic`           | `/lidar/points_lio`  | Adapter output, not the raw gz topic.                                                                                                                      |
| `common.imu_topic`           | `/imu`               | Same as everywhere else.                                                                                                                                   |
| `preprocess.lidar_type`      | 3                    | Ouster. 1 = Livox, 2 = Velodyne, 3 = Ouster. Picks the preprocessor.                                                                                       |
| `preprocess.scan_line`       | 16                   | Matches `<scan><vertical><samples>` in the SDF.                                                                                                            |
| `preprocess.scan_rate`       | 15                   | Hz; matches `<update_rate>`.                                                                                                                              |
| `preprocess.timestamp_unit`  | 3                    | nanoseconds; matches `t` field emitted by the adapter.                                                                                                     |
| `preprocess.blind`           | 0.5 m                | Points closer than this are discarded (avoid self-hits / wheels).                                                                                          |
| `mapping.extrinsic_T`        | `[0, 0, 1.05]`       | Lidar position in IMU frame (= base_link), straight from the SDF.                                                                                          |
| `mapping.extrinsic_R`        | identity             | Lidar has no rotation relative to IMU.                                                                                                                     |
| `mapping.extrinsic_est_en`   | `false`              | We know the calibration exactly; don't let the filter modify it.                                                                                           |
| `mapping.acc_cov / gyr_cov`  | 0.1 / 0.01           | Process-noise covariance scales. Loose-ish defaults that work with our IMU; if LIO oscillates, bump these up.                                              |
| `mapping.b_acc_cov / b_gyr_cov` | 0.001 / 8.0e-7    | Bias process-noise. Matches the gz IMU's bias random walk.                                                                                                 |
| `publish.path_en`            | `true`               | We want `/path` in RViz.                                                                                                                                   |
| `pcd_save.pcd_save_en`       | `false`              | We use rosbag + evo, not PCD dump.                                                                                                                         |

---

## 7. OpenVINS config — `config/openvins/estimator_config.yaml`

The defaults shipped by OpenVINS are tuned for real-robot data (noisy
sensors, imperfect factory calibration). A few keys *need* to flip for
a clean sim — without them VIO will drift wildly even with the rover
stationary, or fail to initialise.

### Online calibration — all OFF

```yaml
calib_cam_extrinsics: false
calib_cam_intrinsics: false
calib_cam_timeoffset: false
calib_imu_intrinsics: false
calib_imu_g_sensitivity: false
```

**Why:** in this sim every camera/IMU parameter is derived from the SDF
or measured offline. There's nothing to refine. Letting OpenVINS run
the online-calibration filter on already-perfect data adds numerical
noise that gets integrated into the pose estimate, and the trajectory
drifts. On a real robot you'd usually leave these on so the filter
polishes the imperfect factory calibration.

### Initialisation

```yaml
init_window_time: 5.0       # seconds of data to collect for static init
init_imu_thresh:  0.3       # m/s² — accel variance needed to declare "motion"
init_max_disparity: 10.0    # px — pixel disparity max to declare "stationary"
init_max_features: 50       # features tracked during init
init_dyn_use: true          # fall back to dynamic init if static fails
```

Plus a hard delay in `launch/vio.launch.py`:
```python
VIO_START_DELAY_SEC = 8.0   # don't even subscribe to /imu until t+8s
```

**Why the delay matters.** The rover spawns at `z=0.4` and settles at
`z≈0.25` — a brief drop that produces an impact transient on the IMU
during the first ~1 second. Without the delay, OpenVINS' static init
triggers in 0.3 ms on the impact, estimates a wrong gravity vector,
and from that point every accel sample has a constant residual that
gets integrated into position. ZUPT can't save it because the wrong
gravity means the filter always predicts a non-zero velocity, so the
ZUPT velocity gate never fires. The delay lets the impact pass before
VIO collects any samples.

**`init_window_time: 5 s`** then averages over enough stationary
samples to nail the gravity direction. Combined with the 8 s delay
that's 13 s of "clean" pre-init time.

**`init_imu_thresh: 0.3 m/s²`** is the variance of accelerometer
magnitude required, within `init_window_time`, for OpenVINS to detect
that the rover has been "jerked." OpenVINS' default of `1.5 m/s²` is
tuned for real IMUs where the noise floor is several times higher
than ours; with the clean gz IMU (accel σ = 1e-2 m/s²), `1.5` is so
high that you have to literally yank the rover to trigger init.
`0.3` triggers reliably on a gentle drive start.

**`init_dyn_use: true`** enables OpenVINS' dynamic-initialisation
fallback: if the static path fails (no jerk), it bootstraps from
camera-IMU motion + parallax. Important for our setup because the
rover might be commanded into smooth motion without an initial jolt.

### Zero-velocity updates (ZUPT)

```yaml
try_zupt: true
zupt_chi2_multipler: 0      # 0 = disparity-only detector
zupt_max_velocity: 0.1      # m/s — speed below which ZUPT can activate
zupt_noise_multiplier: 10
zupt_max_disparity: 0.5     # px — pixel disparity max to declare "stationary"
zupt_only_at_beginning: false
```

**What is ZUPT?** Zero-velocity update. When OpenVINS detects the rover
is stationary (small disparity in the camera + small predicted velocity),
it applies a "velocity = 0" measurement to the filter. This constraint:

- **Zeroes out the velocity state** → drift stops immediately.
- **Re-estimates IMU bias** from the no-motion samples → bias error
  decays back toward truth instead of growing.

**Why it matters here:** without ZUPT, while the rover sits still the
gz IMU's tiny random-walk bias gets integrated forever:

    drift_position(t) = ∫∫ bias(t') dt' dt'   →   grows quadratically

The user-visible symptom was *"the green VIO trail keeps growing even
when the robot isn't moving."* With `try_zupt: true`, the filter
detects the stationary state every few frames and clamps everything.

`zupt_max_disparity: 0.5` is the maximum pixel motion (in any tracked
feature) below which we consider the camera stationary. Tune up if you
have lots of background motion (e.g., wind in the trees), down if the
camera is super stable.

### Filter sizing

```yaml
max_clones: 11              # sliding-window clones for MSCKF
max_slam: 50                # SLAM features kept in state
max_slam_in_update: 25      # batch size for SLAM update
max_msckf_in_update: 40     # batch size for MSCKF update
dt_slam_delay: 1            # seconds before initialising SLAM after VIO init
```

Defaults from OpenVINS; haven't needed to touch. Bigger numbers → more
accurate, more CPU. Drop to `max_clones: 8`, `max_slam: 25` if you're
on a Raspberry Pi.

### Tracker (front-end)

```yaml
use_klt: true               # KLT (true) vs ORB+descriptor matching (false)
num_pts: 200                # features extracted per camera per frame
fast_threshold: 20
grid_x: 5
grid_y: 5
min_px_dist: 10
track_frequency: 21.0       # Hz — sub-rate of the 30 Hz camera
histogram_method: HISTOGRAM # NONE / HISTOGRAM / CLAHE
```

**`use_klt: true` vs `false`** — this is the descriptor question. KLT
(Lucas-Kanade pyramidal optical flow on FAST corners) is fast and
excellent for small frame-to-frame motion, but on self-similar texture
(e.g. our hexagonal-lattice tunnel walls) it can drift to a
neighbouring identical patch when a feature is partially occluded.
The user's own experience on the KITTI dataset shows the same pattern:
`cv2.goodFeaturesToTrack` + KLT produces visibly worse VO trajectories
than SIFT-based tracking on the same sequences
([behnamasadi/OpenCVProjects/docs/kitti.ipynb](https://github.com/behnamasadi/OpenCVProjects/blob/master/docs/kitti.ipynb)).
With `use_klt: false`, OpenVINS switches to FAST + ORB descriptors +
brute-force kNN matching with the `knn_ratio: 0.70` Lowe's-ratio
filter — ambiguous matches (where two candidates are nearly equally
good) are rejected automatically, which helps in self-similar scenes
specifically. Cost is some extra CPU per frame. See
[`docs/ANALYSIS.md` case study 3.5](ANALYSIS.md#case-study-35--orientation-jitter-and-the-klt-vs-descriptor-question)
for the full discussion and when to consider switching.

**Conservative defaults — and a hard lesson.** A previous iteration of
this file tried to "tune for cave" by lowering `fast_threshold` to 10,
bumping `num_pts` to 400, and switching to CLAHE. That combination
*backfired catastrophically* on our gz camera — see
[`docs/ANALYSIS.md`](ANALYSIS.md) for the side-by-side bag analysis.
The root issue is that real-camera tuning assumes real-camera noise +
real-camera dynamics. Our SDF camera is much cleaner (uniform
lighting, no rolling shutter, only a small σ=0.01 Gaussian noise
floor). CLAHE on that cleanness amplifies the Gaussian noise into
fake structure; `fast_threshold: 10` then accepts the noise as
features; the tracker locks onto ghosts and the filter integrates
nonsense. The combination produced 22 km of reported motion on a 58 m
drive (path-length ratio ≈ 380).

If you ever swap to a real RealSense camera, or to a textured/dim
outdoor Fuel world, flip the knobs in the *opposite* direction:
`histogram_method: CLAHE`, `num_pts: 400`, `fast_threshold: 10`.
For sim, the conservative defaults are correct.

---

## 8. Ground-truth republisher — `scripts/gt_to_path.py`

gz's `/world/<w>/dynamic_pose/info` is bridged into `/ground_truth/pose`
as a `tf2_msgs/TFMessage`. The bridge leaves `frame_id` and
`child_frame_id` *empty strings* (gz doesn't supply names there). The
republisher:

- Tries to match `child_frame_id == target_frame` (default
  `"explorer_r2"`); if none match (true with the empty-name bridge
  output), takes the first transform — safe because
  `dynamic_pose/info` only carries dynamic entities and the rover is
  the only one in our worlds.
- Captures the *first* position as origin and subtracts it from every
  subsequent sample, so the GT trail starts at `(0, 0, 0)` like
  wheel-odom, VIO, and LIO — apples-to-apples visual comparison.
- Publishes both `nav_msgs/Path` (for `evo` + the RViz line) and
  `nav_msgs/Odometry` (for the RViz arrow), in frame
  `explorer_r2/odom`.

---

## 9. GNSS + magnetometer

Both are bridged out of the sim by default but no estimator currently
fuses them. To drive a full GNSS-aided EKF, drop in
`ros-jazzy-robot-localization` and wire `navsat_transform_node` +
`ekf_node` against `/navsat` (`sensor_msgs/NavSatFix` @ 10 Hz),
`/magnetometer` (`sensor_msgs/MagneticField`), and one of the existing
odometry topics.
