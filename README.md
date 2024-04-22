# Aerial Robotics
- [Aerial Robotics](docs/aerial_robotics.md#aerial-robotics)



# Differential Drive Robots and Wheel odometry

- [Kinematics of Differential Drive Robots and Wheel odometry](docs/differential_drive_robots_and_wheel_odometry.md#kinematics-of-differential-drive-robots-and-wheel-odometry)
- [Velocity-based (dead reckoning)](docs/differential_drive_robots_and_wheel_odometry.md#1-velocity-based--dead-reckoning-)
  - [Forward Kinematics for Differential Drive Robots](docs/differential_drive_robots_and_wheel_odometry.md#11-forward-kinematics-for-differential-drive-robots)
  - [Inverse Kinematics of Differential Drive Robots](docs/differential_drive_robots_and_wheel_odometry.md#12--inverse-kinematics-of-differential-drive-robots)
  - [Odometry-based](docs/differential_drive_robots_and_wheel_odometry.md#2-odometry-based)




# IMU

- [1. Global References](docs/imu.md#1-global-references)
- [2. Accelerometer Model](docs/imu.md#2-accelerometer-model)
- [3. Gyroscope Model](docs/imu.md#3-gyroscope-model)
- [4. Attitude from gravity (Tilt)](docs/imu.md#4-attitude-from-gravity--tilt-)
  * [4.1. Solving R xyz for the Pitch and Roll Angles](docs/imu.md#41-solving-r-xyz-for-the-pitch-and-roll-angles)
  * [4.2 Solving R yxz for the Pitch and Roll Angles](docs/imu.md#42-solving-r-yxz-for-the-pitch-and-roll-angles)
- [Expressing IMU reading with Quaternion](docs/imu.md#expressing-imu-reading-with-quaternion)
- [5. Quaternion from Accelerometer](docs/imu.md#5-quaternion-from-accelerometer)
- [6. Quaternion Integration](docs/imu.md#6-quaternion-integration)
  * [6.1. Numerical Solution](docs/imu.md#61-numerical-solution)
  * [6.2. Closed-form Solution](docs/imu.md#62-closed-form-solution)
- [7.1 Quaternion Derivative](docs/imu.md#71-quaternion-derivative)
- [Relationship Between Euler-Angle Rates and Body-Axis Rates](docs/imu.md#relationship-between-euler-angle-rates-and-body-axis-rates)
- [Complementary Filter](docs/imu.md#complementary-filter)
- [Quaternion-Based Complementary Filter](docs/imu.md#quaternion-based-complementary-filter)
- [Accelerometer-Based Correction](docs/imu.md#accelerometer-based-correction)
- [Attitude from angular rate (Attitude propagation)](docs/imu.md#attitude-from-angular-rate--attitude-propagation-)
- [IMU Integration](docs/imu.md#imu-integration)
- [Noise Spectral Density](docs/imu.md#noise-spectral-density)
- [Signal-to-noise Ratio](docs/imu.md#signal-to-noise-ratio)
- [Allan Variance curve](docs/imu.md#allan-variance-curve)
  * [Variance](docs/imu.md#variance)
  * [M-sample variance](docs/imu.md#m-sample-variance)
  * [Allan Variance](docs/imu.md#allan-variance)
- [Autoregressive model](docs/imu.md#autoregressive-model)
- [Madgwick Orientation Filter](docs/imu.md#madgwick-orientation-filter)
- [Mahony Orientation Filter](docs/imu.md#mahony-orientation-filter)
- [Simulating IMU Measurements](docs/imu.md#simulating-imu-measurements)
- [IMU Propagation Derivations](docs/imu.md#imu-propagation-derivations)
  * [The IMU Noise Model](docs/imu.md#the-imu-noise-model)
    + [Additive "White Noise"](docs/imu.md#additive--white-noise-)
    + [Bias](docs/imu.md#bias)
    + [The Noise Model Parameters in Kalibr](docs/imu.md#the-noise-model-parameters-in-kalibr)
  * [How to Obtain the Parameters for your IMU](docs/imu.md#how-to-obtain-the-parameters-for-your-imu)
    + [From the Datasheet of the IMU](docs/imu.md#from-the-datasheet-of-the-imu)
    + [From the Allan standard deviation (AD)](docs/imu.md#from-the-allan-standard-deviation--ad-)
  * [Kalibr IMU Noise Parameters in Practice](docs/imu.md#kalibr-imu-noise-parameters-in-practice)
- [IMU Noise Model](docs/imu.md#imu-noise-model)
- [The standard deviation of the discrete-time noise process](docs/imu.md#the-standard-deviation-of-the-discrete-time-noise-process)
    + [Example:](docs/imu.md#example-)
    + [Example:](docs/imu.md#example--1)
    + [Note:](docs/imu.md#note-)

# Apriltag
- [Apriltag](docs/apriltag.md#apriltag)
- [Generating Tag](docs/apriltag.md#generating-tag)
- [Apriltag ROS](docs/apriltag.md#apriltag-ros)
- [Detecting Apriltag](docs/apriltag.md#detecting-apriltag)

# Kalibr

- [Datasets and Calibration Targets](docs/kalibr.md#datasets-and-calibration-targets)
- [Supported Camera Models and Distortion](docs/kalibr.md#supported-camera-models-and-distortion)
- [Camera Calibration](docs/kalibr.md#camera-calibration)
- [Camera IMU Calibration](docs/kalibr.md#camera-imu-calibration)

# Topology and Configuration of Robot and Space

- [Configuration of Robot](docs/robot_configuration_dof_topology.md#configuration-of-robot)
- [Configuration  Space - (C-space )](docs/robot_configuration_dof_topology.md#configuration--space----c-space--)
- [Degrees of freedom.](docs/robot_configuration_dof_topology.md#degrees-of-freedom)
- [Task Space](docs/robot_configuration_dof_topology.md#task-space)
- [Work Space](docs/robot_configuration_dof_topology.md#work-space)
- [Dexterous space:](docs/robot_configuration_dof_topology.md#dexterous-space-)
- [dof](docs/robot_configuration_dof_topology.md#dof)
- [Topology](docs/robot_configuration_dof_topology.md#topology)
  * [Different way to represent C-space](docs/robot_configuration_dof_topology.md#different-way-to-represent-c-space)
    + [Explicit](docs/robot_configuration_dof_topology.md#explicit)
    + [Implicit](docs/robot_configuration_dof_topology.md#implicit)
- [Important Lie Groups and Typologies](docs/robot_configuration_dof_topology.md#important-lie-groups-and-typologies)
  * [so(3)](docs/robot_configuration_dof_topology.md#so-3-)
  * [SO(2)](docs/robot_configuration_dof_topology.md#so-2-)
  * [SO(3)](docs/robot_configuration_dof_topology.md#so-3-)
  * [SE(2)](docs/robot_configuration_dof_topology.md#se-2-)
  * [SE(3)](docs/robot_configuration_dof_topology.md#se-3-)


# ROS and Gazebo
- [1. Names](docs/ros_gazebo.md#1-names)
  * [1.1 Graph Resource Names](docs/ros_gazebo.md#11-graph-resource-names)
    + [1.1.1 Resolving Name](docs/ros_gazebo.md#111-resolving-name)
  * [1.2 Package Resource Names](docs/ros_gazebo.md#12-package-resource-names)
- [2. Remapping Arguments](docs/ros_gazebo.md#2-remapping-arguments)
  * [3. Special keys](docs/ros_gazebo.md#3-special-keys)
- [NodeHandles](docs/ros_gazebo.md#nodehandles)
  * [Publishers and Subscribers](docs/ros_gazebo.md#publishers-and-subscribers)
- [Roslaunch](docs/ros_gazebo.md#roslaunch)
- [URDF](docs/ros_gazebo.md#urdf)
- [Publishing the State](docs/ros_gazebo.md#publishing-the-state)
- [Differential Drive Wheel Systems](docs/ros_gazebo.md#differential-drive-wheel-systems)
- [Gazebo](docs/ros_gazebo.md#gazebo)
- [ROS best practices](docs/ros_gazebo.md#ros-best-practices)
- [move_base](docs/ros_gazebo.md#move-base)
  * [ROS Odometery Model](docs/ros_gazebo.md#ros-odometery-model)
- [ROS State Estimation](docs/ros_gazebo.md#ros-state-estimation)
- [EKF Implementations](docs/ros_gazebo.md#ekf-implementations)

# State Estimation
  * [FilterPy](docs/state_estimation.md#filterpy)
- [EKF Implementations](docs/state_estimation.md#ekf-implementations)
- [EKF for Differential Drive Robot](docs/state_estimation.md#ekf-for-differential-drive-robot)
  * [State Space](docs/state_estimation.md#state-space)
- [STATE ESTIMATION FOR ROBOTICS](docs/state_estimation.md#state-estimation-for-robotics)






# [SLAM](#)
[Active Exposure Control for Robust Visual Odometry in HDR Environments](docs/slam/active_exposure_control_HDR_environments.md)  
[Pose Graph SLAM](docs/slam/pose_graph_slam.md)  
[Factor Graph vs Pose Graph](docs/slam/factor_graph_vs_pose_graph.md)  
[g2o](docs/slam/g2o.md)  
[GTSAM](docs/slam/GTSAM.md)  
[Resilient Autonomy in Perceptually-degraded Environments](https://www.youtube.com/watch?v=L0PQKxU8cps)  
[HBA Large-Scale LiDAR Mapping Module](https://github.com/hku-mars/HBA)  
[Hierarchical, multi-resolution volumetric mapping (wavemap)](https://github.com/ethz-asl/wavemap)  
[kiss-icp](https://github.com/PRBonn/kiss-icp?tab=readme-ov-file)  
[TagSLAM SLAM with tags](https://berndpfrommer.github.io/tagslam_web/)  
[OpenDroneMap](docs/slam/open_drone_map.md)  



# VIO
[IMU Propagation Derivations](https://docs.openvins.com/propagation.html)  
[Open Keyframe-based Visual-Inertial SLAM](https://github.com/ethz-asl/okvis)  


# Lidar and IMU
[Lidar and IMU ](docs/slam/lidar_and_imu.md)  


- modern-slam-tutorial-python
Refs: [1](https://github.com/gisbi-kim/modern-slam-tutorial-python)

## Add Apriltag to loop closure

Refs: [1](https://berndpfrommer.github.io/tagslam_web/)



## image-matching-webui

## LightGlue


## DenseSFM

Refs: [1](https://github.com/tsattler/visuallocalizationbenchmark)

## Pixel-Perfect Structure-from-Motion
Refs: [1](https://github.com/cvg/pixel-perfect-sfm)


## AnyLoc: Towards Universal Visual Place Recognition

Refs [1](https://github.com/AnyLoc/AnyLoc)



# Deep Learning based SLAM 
[Gaussian Splatting](docs/slam/gaussian_splatting.md)  
[GANeRF](https://github.com/barbararoessle/ganerf)  
[DSAC*](https://github.com/vislearn/dsacstar)  
[Tracking Any Point (TAP)](https://github.com/google-deepmind/tapnet)  
[image-matching-benchmark](https://github.com/ubc-vision/image-matching-benchmark)  
[Local Feature Matching at Light Speed](https://github.com/cvg/LightGlue)  
[Hierarchical Localization](https://github.com/cvg/Hierarchical-Localization)  
[instant-ngp](docs/slam/instant_ngp.md)  
[NeRF-SLAM](docs/slam/NeRF-SLAM.md)  

[DROID-SLAM](https://github.com/princeton-vl/DROID-SLAM?tab=readme-ov-file)  


# [Procrustes Analysis](docs/shape_analysis.md#procrustes-analysis)  

[Procrustes Analysis](#procrustes-analysis)  
[Wahba's Problem](#wahba-s-problem)
[Quaternion Estimator Algorithm (QUEST)](#quaternion-estimator-algorithm--quest-)  
[Kabsch Algorithm](#kabsch-algorithm)  
[Umeyama Algorithm](#umeyama-algorithm)  
[Iterative Closest Point (ICP)](#iterative-closest-point--icp-)  
[KISS-ICP](#kiss-icp)  


# E-Books and Refs

[Modern Robotics Mechanics, Planning, and Control (Kevin M. Lynch, Frank C. Park)](docs/ebooks/Modern%20Robotics%20Mechanics%2C%20Planning%2C%20and%20Control%20%28Kevin%20M.%20Lynch%20and%20Frank%20C.%20Park%29.pdf)  
[Modern Robotics Mechanics, Planning, and Control (Instructor Solution Manual, Solutions )](docs/ebooks/Modern%20Robotics%20Mechanics%2C%20Planning%2C%20and%20Control%20%28Kevin%20M.%20Lynch%2C%20Frank%20C.%20Park%29%20Exercise%20Solutions.pdf)  
[MODERN ROBOTICS MECHANICS, PLANNING, AND CONTROL (Practice Exercises)](docs/ebooks/MODERN%20ROBOTICS%20MECHANICS%2C%20PLANNING%2C%20AND%20CONTROL%20%28Practice%20Exercises%29.pdf)  
[Basic Knowledge on Visual SLAM: From Theory to Practice, by Xiang Gao, Tao Zhang, Qinrui Yan and Yi Liu](docs/ebooks/slambook-en.pdf)  
[STATE ESTIMATION FOR ROBOTICS (Timothy D. Barfoot)](docs/ebooks/STATE%20ESTIMATION%20FOR%20ROBOTICS%20%28Timothy%20D.%20Barfoot%29.pdf)  
[SLAM for Dummies](docs/ebooks/SLAM%20for%20Dummies.pdf)  

