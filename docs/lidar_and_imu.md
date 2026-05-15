# A Stereo Event Camera Dataset for Driving Scenarios DSEC 

Ref: [1](https://github.com/uzh-rpg/DSEC)


# FAST-LIO (Fast LiDAR-Inertial Odometry)

Refs: [1](https://github.com/hku-mars/FAST_LIO)

Deployed as the LIO baseline in the SubT-style rover simulator at
[`~/ros2_ws/`](https://github.com/) — `third_party/FAST_LIO` (submodule),
`src/explorer_r2_sim/config/lio.yaml`,
`src/explorer_r2_sim/launch/lio.launch.py`. End-point error on sim
recordings: ~0.3-1.5 % of path length, while both stereo VIOs blew up
3-10× scale on the same data. The architectural reason is documented in
[`vio_benchmark/docs/VIO_DIAGNOSTIC_GUIDE.md` §4b](../vio_benchmark/docs/VIO_DIAGNOSTIC_GUIDE.md#4b--why-lio-survives-bad-imu-data-and-vio-doesnt):
IMU is on the integrator path for VIO and the hint path for LIO.

#  incremental Generalized Iterative Closest Point (GICP) based tightly-coupled LiDAR-inertial odometry (LIO), iG-LIO

Refs: [1](https://github.com/zijiechenrobotics/ig_lio)


# Direct LiDAR-Inertial Odometry: Lightweight LIO with Continuous-Time Motion Correction

Refs: [1](https://github.com/vectr-ucla/direct_lidar_inertial_odometry)


# Robust Real-time LiDAR-inertial Initialization

Refs: [1](https://github.com/hku-mars/LiDAR_IMU_Init)

# CT-LIO: Continuous-Time LiDAR-Inertial Odometry

Refs: [1](https://github.com/chengwei0427/ct-lio)
