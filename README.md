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
- [Algebraic topology (playlist)](https://www.youtube.com/playlist?list=PLuFcVFHMIfhJSSX-tlv8XxiAZSAbhv1DA)
 


# ROS1
- [1. Names](docs/ros1.md#1-names)
  * [1.1 Graph Resource Names](docs/ros1.md#11-graph-resource-names)
    + [1.1.1 Resolving Name](docs/ros1.md#111-resolving-name)
  * [1.2 Package Resource Names](docs/ros1.md#12-package-resource-names)
- [2. Remapping Arguments](docs/ros1.md#2-remapping-arguments)
  * [3. Special keys](docs/ros1.md#3-special-keys)
- [NodeHandles](docs/ros1.md#nodehandles)
  * [Publishers and Subscribers](docs/ros1.md#publishers-and-subscribers)
- [Roslaunch](docs/ros1.md#roslaunch)
- [URDF](docs/ros1.md#urdf)
- [Publishing the State](docs/ros1.md#publishing-the-state)
- [ROS best practices](docs/ros1.md#ros-best-practices)
- [move_base](docs/ros1.md#move-base)
- [ROS Odometery Model](docs/ros1.md#ros-odometery-model)
- [ROS State Estimation](docs/ros1.md#ros-state-estimation)
- [EKF Implementations](docs/ros1.md#ekf-implementations)
- [Differential Drive Wheel Systems](docs/ros1.md#differential-drive-wheel-systems)

# ROS2
- [Installation](docs/ros2.md#installation)
- [Configuration](docs/ros2.md#configuration)
  * [Domain ID.](docs/ros2.md#domain-id)
  * [ROS_AUTOMATIC_DISCOVERY_RANGE](docs/ros2.md#ros-automatic-discovery-range)
  * [Remapping](docs/ros2.md#remapping)
  * [Launching nodes](docs/ros2.md#launching-nodes)
- [Colcon](docs/ros2.md#colcon)
- [Using Xacro](docs/ros2.md#using-xacro)
- [Creating a launch file](docs/ros2.md#creating-a-launch-file)
  * [Python launch file](docs/ros2.md#python-launch-file)
  * [C++ launch file](docs/ros2.md#c---launch-file)
  * [Managing launch file](docs/ros2.md#managing-launch-file)
  * [Substitutions](docs/ros2.md#substitutions)
  * [Event handlers](docs/ros2.md#event-handlers)
- [Nav2 - ROS 2 Navigation Stack](docs/ros2.md#nav2---ros-2-navigation-stack)
- [teleop_twist_keyboard](docs/ros2.md#teleop-twist-keyboard)


# Gazebo

- [Gazebo Versions](docs/gazebo.md#gazebo-versions)
- [Installation](docs/gazebo.md#installation)
- [Building a model](docs/gazebo.md#building-a-model)
  * [Links](docs/gazebo.md#links)
    + [Inertia Matrix](docs/gazebo.md#inertia-matrix)
  * [Visual and collision](docs/gazebo.md#visual-and-collision)
  * [Connecting links together (joints)](docs/gazebo.md#connecting-links-together--joints-)
- [Building world](docs/gazebo.md#building-world)
  * [Physics](docs/gazebo.md#physics)
  * [Plugins](docs/gazebo.md#plugins)
    + [Physics](docs/gazebo.md#physics-1)
    + [User-commands](docs/gazebo.md#user-commands)
    + [Scene-broadcaster](docs/gazebo.md#scene-broadcaster)
  * [GUI](docs/gazebo.md#gui)
    + [World control plugin](docs/gazebo.md#world-control-plugin)
    + [World stats plugin](docs/gazebo.md#world-stats-plugin)
    + [Entity tree](docs/gazebo.md#entity-tree)
- [Moving the robot](docs/gazebo.md#moving-the-robot)
- [Sensors](docs/gazebo.md#sensors)
  * [IMU sensor](docs/gazebo.md#imu-sensor)
  * [Lidar sensor](docs/gazebo.md#lidar-sensor)
- [Spawn URDF](docs/gazebo.md#spawn-urdf)
- [ROS 2 integration](docs/gazebo.md#ros-2-integration)


# ROS2 Gazebo Integration
- [Installation of ros_gz](docs/ros2_gazebo_integration.md#installation-of-ros-gz)
  * [Launch Gazebo from ROS 2](docs/ros2_gazebo_integration.md#launch-gazebo-from-ros-2)
- [ROS2 Interaction With Gazebo](docs/ros2_gazebo_integration.md#ros2-interaction-with-gazebo)
  * [Bridge communication between ROS and Gazebo](docs/ros2_gazebo_integration.md#bridge-communication-between-ros-and-gazebo)
  * [1. Simple Hello between ROS2 and Gazebo](docs/ros2_gazebo_integration.md#1-simple-hello-between-ros2-and-gazebo)
  * [2. Camera Example](docs/ros2_gazebo_integration.md#2-camera-example)
  * [3. Diff Drive Example](docs/ros2_gazebo_integration.md#3-diff-drive-example)
  * [4. GPU lidar](docs/ros2_gazebo_integration.md#4-gpu-lidar)
  * [5. IMU, Magnetometer](docs/ros2_gazebo_integration.md#5-imu--magnetometer)
  * [6. GNSS](docs/ros2_gazebo_integration.md#6-gnss)




# State Estimation
- [Bayes filter](docs/state_estimation.md#bayes-filter)  
- [Extended Kalman Filter](docs/state_estimation.md#extended-kalman-filter)  
- [EKF Implementations](docs/state_estimation.md#ekf-implementations)  
- [EKF for Differential Drive Robot](docs/state_estimation.md#ekf-for-differential-drive-robot)  
- [Error State Extended Kalman Filter EFK ES](https://notanymike.github.io/Error-State-Extended-Kalman-Filter/)  
- [Invariant extended Kalman filter EKF](https://en.wikipedia.org/wiki/Invariant_extended_Kalman_filter)
- [Multi-State Constraint Kalman Filter (MSCKF)](https://docs.openvins.com/namespaceov__msckf.html)
- [STATE ESTIMATION FOR ROBOTICS](docs/state_estimation.md#state-estimation-for-robotics)  
- [FilterPy](docs/state_estimation.md#filterpy)
- [Quaternion kinematics for the error-state Kalman filter](https://arxiv.org/pdf/1711.02508)

# Bag of Words
[FBOW (Fast Bag of Words)](https://github.com/rmsalinas/fbow)



# [SLAM](#)
[Active Exposure Control for Robust Visual Odometry in HDR Environments](docs/slam/active_exposure_control_HDR_environments.md)  
[Pose Graph SLAM](docs/slam/pose_graph_slam.md)  
[Factor Graph vs Pose Graph](docs/slam/factor_graph_vs_pose_graph.md)  
[g2o](docs/slam/g2o.md)  
[GTSAM](docs/slam/GTSAM.md)  
- [Factor Graph - 5 Minutes with Cyrill](docs/slam/GTSAM.md#factor-graph---5-minutes-with-cyrill)  
- [Georgia Tech Smoothing and Mapping Library](docs/slam/GTSAM.md#georgia-tech-smoothing-and-mapping-library)  
- [iSAM: Incremental Smoothing and Mapping](docs/slam/GTSAM.md#isam--incremental-smoothing-and-mapping)  
- [python-graphslam](docs/slam/GTSAM.md#python-graphslam)  
- [GTSAM python Tutorial Robust Pose-graph Optimization](docs/slam/GTSAM.md#gtsam-python-tutorial-robust-pose-graph-optimization)  
[Resilient Autonomy in Perceptually-degraded Environments](https://www.youtube.com/watch?v=L0PQKxU8cps)  
[HBA Large-Scale LiDAR Mapping Module](https://github.com/hku-mars/HBA)  
[Hierarchical, multi-resolution volumetric mapping (wavemap)](https://github.com/ethz-asl/wavemap)  
[kiss-icp](https://github.com/PRBonn/kiss-icp?tab=readme-ov-file)  
[TagSLAM SLAM with tags](https://berndpfrommer.github.io/tagslam_web/)  
[OpenDroneMap](docs/slam/open_drone_map.md)  
[Lidar SLAM for Automated Driving (MATLAB learning)](https://www.youtube.com/watch?v=n4tazoEcBGo)  
[Interactive SLAM](https://github.com/SMRT-AIST/interactive_slam)  
[Volumetric TSDF Fusion of Multiple Depth Maps](https://github.com/andyzeng/tsdf-fusion)  
[Euclidean Signed Distance Field (ESDF)](https://github.com/HKUST-Aerial-Robotics/FIESTA?tab=readme-ov-file)  
[Lidar odometry smoothing using ES EKF and KissICP for Ouster sensors with IMUs](https://capsulesbot.com/blog/2024/02/05/esekf-smoothing-ouster-lidar-with-imu-using-kiss.html)
[OpenVINS](docs/slam/open_vins.md)  
- [OpenVINS Multi-Camera Extension](docs/slam/open_vins.md#openvins-multi-camera-extension)  

[Multisensor-aided Inertial Navigation System (MINS)](https://github.com/rpng/MINS)  
[Benchmark Comparison of Monocular Visual-Inertial Odometry Algorithms for Flying Robots](docs/slam/visual_Inertial_SLAM_comparison.md)  
[Visual-Inertial Navigation Systems: An Introduction](https://www.youtube.com/watch?v=dXN2E38jvQM)  
[nano-pgo](https://github.com/gisbi-kim/nano-pgo)



# SLAM Dataset
[rvp group](https://rvp-group.net/slam-dataset.html)  

# Visual and Inertial Odometry VIO
[IMU Propagation Derivations openvin](https://docs.openvins.com/propagation.html)  
[Open Keyframe-based Visual-Inertial SLAM okvis](https://github.com/ethz-asl/okvis)  


# Lidar and IMU LIO
- [A Stereo Event Camera Dataset for Driving Scenarios DSEC](docs/slam/lidar_and_imu.md#a-stereo-event-camera-dataset-for-driving-scenarios-dsec)  
- [FAST-LIO (Fast LiDAR-Inertial Odometry)](docs/slam/lidar_and_imu.md#fast-lio--fast-lidar-inertial-odometry-)  
- [incremental Generalized Iterative Closest Point (GICP) based tightly-coupled LiDAR-inertial odometry (LIO), iG-LIO](docs/slam/lidar_and_imu.md#incremental-generalized-iterative-closest-point--gicp--based-tightly-coupled-lidar-inertial-odometry--lio---ig-lio)  
- [Direct LiDAR-Inertial Odometry: Lightweight LIO with Continuous-Time Motion Correction](docs/slam/lidar_and_imu.md#direct-lidar-inertial-odometry--lightweight-lio-with-continuous-time-motion-correction)  
- [Robust Real-time LiDAR-inertial Initialization](docs/slam/lidar_and_imu.md#robust-real-time-lidar-inertial-initialization)  
- [CT-LIO: Continuous-Time LiDAR-Inertial Odometry](docs/slam/lidar_and_imu.md#ct-lio--continuous-time-lidar-inertial-odometry)  

# Lidar-Monocular Visual Odometry
[Lidar-Monocular Visual Odometry](https://github.com/johannes-graeter/limo)



# Radar SLAM
[Navtech-Radar-SLAM](https://github.com/gisbi-kim/navtech-radar-slam)

# Lie theory
[manif: A small header-only library for Lie theory ](https://github.com/artivis/manif)   
[Sophus](https://github.com/strasdat/Sophus)  

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
[ACE0](https://github.com/nianticlabs/acezero)  
[A Hierarchical 3D Gaussian Representation for Real-Time Rendering of Very Large Datasets]  
[DoubleTake:Geometry Guided Depth Estimation](https://nianticlabs.github.io/doubletake/)  
[Mitigating Motion Blur in Neural Radiance Fields with Events and Frames](https://github.com/uzh-rpg/EvDeblurNeRF)  
[LEAP-VO: Long-term Effective Any Point Tracking for Visual Odometry](https://chiaki530.github.io/projects/leapvo/)  
[MegaScenes: Scene-Level View Synthesis at Scale](https://megascenes.github.io/)  
[Intrinsic Image Diffusion for Indoor Single-view Material Estimation](https://github.com/Peter-Kocsis/IntrinsicImageDiffusion)  
[Vidu4D: Single Generated Video to High-Fidelity 4D Reconstruction with Dynamic Gaussian Surfels](https://vidu4d-dgs.github.io/)  


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
[VSLAM Handbook](https://github.com/tussedrotten/vslam-handbook)  


