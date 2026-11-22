# Exercises for the IMU series

One notebook per video of the series *from "what is an IMU" to sensor fusion* (25 videos). Watch the video, then work through its notebook. Every question can be solved from what the video teaches.

Each notebook has 30–47 questions in the video's order, easy ones first in each section, tagged by kind:
`[explain]` in your own words · `[by hand]` on paper · `[compute]` on the real EuRoC MH_01 data · `[derive]` / `[prove]` · `[what if]` change one assumption, predict, then check · `[code]` implement a piece of the method, including find-the-bug and which-plot-is-which. Each ends with a mini-project that puts the whole video together.

**Try each question before opening its solution.** Solutions are folded under *Solution*; every number in them is printed by code in the notebook, not typed.

Data: EuRoC MH_01 (`imu0` and the ground truth), loaded the same way as the matching notebook in `docs/`. Anything synthetic is labelled.

| # | Video | Questions | Notebook |
|---|---|---|---|
| 1 | What Does an IMU Actually Measure? | 38 | [01_imu_measurement_model_exercises.ipynb](01_imu_measurement_model_exercises.ipynb) |
| 2 | Why Your IMU Reads 8.2 Instead of 9.81 | 33 | [02_imu_rest_paradox_exercises.ipynb](02_imu_rest_paradox_exercises.ipynb) |
| 3 | Roll and Pitch from Gravity — and Why Yaw Is Unobservable | 33 | [03_imu_tilt_attitude_exercises.ipynb](03_imu_tilt_attitude_exercises.ipynb) |
| 4 | Yaw Needs a Compass — the Tilt-Compensated Magnetometer | 31 | [04_mag_yaw_exercises.ipynb](04_mag_yaw_exercises.ipynb) |
| 5 | Integrating a Gyroscope Without Gimbal Lock | 30 | [05_gyro_integration_exercises.ipynb](05_gyro_integration_exercises.ipynb) |
| 6 | Dead Reckoning: Why IMU-Only Position Explodes | 40 | [06_imu_dead_reckoning_exercises.ipynb](06_imu_dead_reckoning_exercises.ipynb) |
| 7 | The Complementary Filter — Two Bad Sensors, One Good Estimate | 34 | [07_complementary_filter_exercises.ipynb](07_complementary_filter_exercises.ipynb) |
| 8 | Madgwick & Mahony — the Attitude Filters Everyone Actually Runs | 31 | [08_madgwick_mahony_exercises.ipynb](08_madgwick_mahony_exercises.ipynb) |
| 9 | Noise Density and Random Walk — Reading an IMU Datasheet | 32 | [09_imu_noise_model_exercises.ipynb](09_imu_noise_model_exercises.ipynb) |
| 10 | Allan Variance — Measure Your IMU's Noise Yourself | 32 | [10_allan_variance_exercises.ipynb](10_allan_variance_exercises.ipynb) |
| 11 | Magnetometer Calibration — Hard Iron, Soft Iron, and the Ellipsoid Fit | 40 | [11_mag_calibration_exercises.ipynb](11_mag_calibration_exercises.ipynb) |
| 12 | A Kalman Filter on an IMU — Tilt Angle and Gyro Bias | 37 | [12_imu_kalman_tilt_exercises.ipynb](12_imu_kalman_tilt_exercises.ipynb) |
| 13 | The Error-State Kalman Filter | 35 | [13_eskf_exercises.ipynb](13_eskf_exercises.ipynb) |
| 14 | GNSS + IMU — a position fix in the error-state Kalman filter | 37 | [14_gnss_imu_exercises.ipynb](14_gnss_imu_exercises.ipynb) |
| 15 | Monocular Scale — Why One Camera Can't Measure Size, and an IMU Can | 35 | [15_vio_scale_exercises.ipynb](15_vio_scale_exercises.ipynb) |
| 16 | Camera–IMU calibration with Kalibr | 40 | [16_kalibr_imu_cam_exercises.ipynb](16_kalibr_imu_cam_exercises.ipynb) |
| 17 | The simplest VIO: loosely coupled fusion on the ESKF | 41 | [17_vio_loose_exercises.ipynb](17_vio_loose_exercises.ipynb) |
| 18 | The MSCKF: using camera features without putting them in the state | 36 | [18_msckf_exercises.ipynb](18_msckf_exercises.ipynb) |
| 19 | Is the Answer Right? Observability, Consistency, and the Overconfident EKF | 41 | [19_observability_consistency_exercises.ipynb](19_observability_consistency_exercises.ipynb) |
| 20 | IMU pre-integration | 47 | [20_imu_preintegration_exercises.ipynb](20_imu_preintegration_exercises.ipynb) |
| 21 | Marginalisation in a Sliding Window | 39 | [21_sliding_window_marginalization_exercises.ipynb](21_sliding_window_marginalization_exercises.ipynb) |
| 22 | How VIO boots up — gravity, scale and gyro bias from a few seconds of motion | 33 | [22_vio_init_exercises.ipynb](22_vio_init_exercises.ipynb) |
| 23 | Inside VINS-Mono: the anatomy of a tightly-coupled VIO | 43 | [23_vins_pipeline_exercises.ipynb](23_vins_pipeline_exercises.ipynb) |
| 24 | The Heading Nobody Measured: why VIO yaw drifts, and what fixes it | 34 | [24_vio_yaw_exercises.ipynb](24_vio_yaw_exercises.ipynb) |
| 25 | LiDAR-inertial odometry: why a LiDAR needs an IMU | 45 | [25_lio_basics_exercises.ipynb](25_lio_basics_exercises.ipynb) |

**917 questions in total.**
