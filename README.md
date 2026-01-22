# Installation and Requirement 


![Ubuntu](https://github.com/behnamasadi/robotic_notes/actions/workflows/build.yml/badge.svg)
![alt text](https://img.shields.io/badge/license-BSD-blue.svg)
![GitHub Issues or Pull Requests](https://img.shields.io/github/issues/behnamasadi/robotic_notes)
<!-- 
![GitHub Release](https://img.shields.io/github/v/release/behnamasadi/robotic_notes)
-->
![GitHub Repo stars](https://img.shields.io/github/stars/behnamasadi/robotic_notes)
![GitHub forks](https://img.shields.io/github/forks/behnamasadi/robotic_notes)




### C++ Dependencies


```
cd /home/$USER/workspace/
git clone git@github.com:behnamasadi/robotic_notes.git
```

vcpkg is configured as a git submodule. Initialize it:

```
cd /home/$USER/workspace/robotic_notes
git submodule update --init --recursive
```

set the path:

```
export VCPKG_ROOT=$PWD/vcpkg
export PATH=$VCPKG_ROOT:$PATH
```
Setting `VCPKG_ROOT` tells vcpkg where your vcpkg instance is located.

Install required system dependencies for vcpkg (on Linux):

```
sudo apt-get install -y bison flex build-essential cmake autoconf autoconf-archive automake libtool libltdl-dev libx11-dev libxft-dev libxext-dev libxtst-dev libxrandr-dev ninja-build pkg-config
```


Now you can run:


```
cmake -S . -B build \
  -DCMAKE_TOOLCHAIN_FILE=./vcpkg/scripts/buildsystems/vcpkg.cmake \
  -DCMAKE_BUILD_TYPE=Release \
  -DVCPKG_TARGET_TRIPLET=x64-linux-release
```

The `VCPKG_TARGET_TRIPLET=x64-linux-release` option ensures vcpkg only builds release packages, which significantly reduces build time (especially for large packages like OpenCV) and disk space usage. This is already configured in CMakeLists.txt, but you can explicitly set it as shown above.


```
cmake --build build --parallel
```

### Python Dependencies

```
conda create -n robotic_notes
conda activate robotic_notes
conda install python=3.13
cd /home/$USER/anaconda3/envs/robotic_notes/
```

Create this soft link.

```
ln -s /home/$USER/workspace/robotic_notes /home/$USER/anaconda3/envs/robotic_notes/src
```

Install the python packages:
```
pip3 install rerun-sdk
conda install -c conda-forge opencv
pip install graphslam
conda install conda-forge::gtsam
conda install conda-forge::matplotlib
conda install conda-forge::plotly
conda install -c conda-forge jupyterlab
pip install gradio_rerun
pip install ahrs
pip install pyceres
pip install liegroups
```



# [Lie Group and Lie Algebra](#)
- [Lie Group and Lie Algebra](docs/lie_group_lie_algebra.ipynb)   
- [manif](docs/manif.ipynb)   
  * [SE2 localization](docs/se2_localization.ipynb)
  * [SE2 SAM](docs/se2_sam.ipynb)
- [Sophus](https://github.com/strasdat/Sophus)  

# [Topology and Configuration of Robot and Space](#)
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
- [Algebraic topology (playlist)](https://www.youtube.com/playlist?list=PLuFcVFHMIfhJSSX-tlv8XxiAZSAbhv1DA)
- [Non-Holonomic Constraints, Pfaffian Constraints and Holonomic Constraints](docs/non_holonomic_pfaffian_constraint.ipynb)


# [Aerial Robotics](#)
- [Aerial Robotics](docs/aerial_robotics.md#aerial-robotics)

# [PX4](#)
- [PX4](docs/px4.md)


# [Differential Drive Robots and Wheel odometry](#)

- [Kinematics of Differential Drive Robots and Wheel odometry](docs/differential_drive_robots_and_wheel_odometry.md#kinematics-of-differential-drive-robots-and-wheel-odometry)
- [Velocity-based (dead reckoning)](docs/differential_drive_robots_and_wheel_odometry.md#1-velocity-based--dead-reckoning-)
  - [Forward Kinematics for Differential Drive Robots](docs/differential_drive_robots_and_wheel_odometry.md#11-forward-kinematics-for-differential-drive-robots)
  - [Inverse Kinematics of Differential Drive Robots](docs/differential_drive_robots_and_wheel_odometry.md#12--inverse-kinematics-of-differential-drive-robots)
  - [Odometry-based](docs/differential_drive_robots_and_wheel_odometry.md#2-odometry-based)
- [Nonlinear uncertainty model associated with a robot's position over time (The Banana Distribution is Gaussian)](docs/nonlinear_uncertainty_model_associated_with_robot_position_banana_shape_.ipynb)

# [IMU](#)

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

# [Apriltag](#)
- [Apriltag](docs/apriltag.md#apriltag)
- [Generating Tag](docs/apriltag.md#generating-tag)
- [Apriltag ROS](docs/apriltag.md#apriltag-ros)
- [Detecting Apriltag](docs/apriltag.md#detecting-apriltag)

# [Kalibr](#)

- [Datasets and Calibration Targets](docs/kalibr.md#datasets-and-calibration-targets)
- [Supported Camera Models and Distortion](docs/kalibr.md#supported-camera-models-and-distortion)
- [Camera Calibration](docs/kalibr.md#camera-calibration)
- [Camera IMU Calibration](docs/kalibr.md#camera-imu-calibration)



# [ROS1](#)
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

# [ROS2](#)
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


# [Gazebo](#)

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


# [ROS2 Gazebo Integration](#)
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


# [MarsLogger](#)
- [MarsLogger](docs/mars_logger.md)

# [State Estimation](#)
- [Bayes Filter](docs/bayes_filter.ipynb)
- [Kalman Filter](docs/kalman_filter.ipynb)
- [Extended Kalman Filter](docs/extended_kalman_filter.ipynb)
- [Extended Kalman Filter for Differential Drive Robot](docs/extended_kalman_filter.ipynb#EKF-for-Differential-Drive-Robot)
- [Error State Extended Kalman Filter](docs/error_state_extended_kalman_filter_vio.ipynb)
- [Error State Extended Kalman Filter(IMU, a GNSS, and a LiDAR)](https://github.com/enginBozkurt/Error-State-Extended-Kalman-Filter)  
- [Multi-State Constraint Kalman Filter (MSCKF)](https://docs.openvins.com/namespaceov__msckf.html)  
- [Quaternion kinematics for the error-state Kalman filter](https://arxiv.org/pdf/1711.02508)


# [Bag of Words](#)
- [FBOW (Fast Bag of Words)](https://github.com/rmsalinas/fbow)
- [AnyLoc: Towards Universal Visual Place Recognition](https://github.com/AnyLoc/AnyLoc)

# [Rerun](#)
- [Rerun](docs/rerun.ipynb)


# [SLAM](#)
- [Active Exposure Control for Robust Visual Odometry in HDR Environments](docs/active_exposure_control_HDR_environments.md)
- [Pose Graph SLAM from Scratch](docs/pose_graph_slam.ipynb)
- [nano-pgo](https://github.com/gisbi-kim/nano-pgo)
- [g2o](docs/g2o.md)  
- [Factor Graph GTSAM iSAM2](docs/factor_graph_gtsam_isam2.ipynb)  
- [Resilient Autonomy in Perceptually-degraded Environments](https://www.youtube.com/watch?v=L0PQKxU8cps)  
- [HBA Large-Scale LiDAR Mapping Module](https://github.com/hku-mars/HBA)  
- [Hierarchical, multi-resolution volumetric mapping (wavemap)](https://github.com/ethz-asl/wavemap)  
- [kiss-icp](https://github.com/PRBonn/kiss-icp?tab=readme-ov-file)  
- [TagSLAM SLAM with tags](https://berndpfrommer.github.io/tagslam_web/)  
- [OpenDroneMap](docs/open_drone_map.md)  
- [Interactive SLAM](https://github.com/SMRT-AIST/interactive_slam)  
- [Volumetric TSDF Fusion of Multiple Depth Maps](https://github.com/andyzeng/tsdf-fusion)  
- [Euclidean Signed Distance Field (ESDF)](https://github.com/HKUST-Aerial-Robotics/FIESTA?tab=readme-ov-file)  
- [Lidar odometry smoothing using ES EKF and KissICP for Ouster sensors with IMUs](https://capsulesbot.com/blog/2024/02/05/esekf-smoothing-ouster-lidar-with-imu-using-kiss.html)
- [Multisensor-aided Inertial Navigation System (MINS)](https://github.com/rpng/MINS)  
- [GLOMAP explained](https://www.youtube.com/watch?v=lYC-oMSCNOE)
- [Zero-Shot Point Cloud Registration](https://github.com/MIT-SPARK/BUFFER-X)
- [Add Apriltag to loop closure](https://berndpfrommer.github.io/tagslam_web/)
- [Navtech Radar SLAM](https://github.com/gisbi-kim/navtech-radar-slam)


# [Shape Analysis](#)
- [Procrustes Analysis](shape_analysis.ipynb#procrustes-analysis)
- [Wahba's Problem](shape_analysis.ipynb#wahba-s-problem)
- [Quaternion Estimator Algorithm (QUEST)](shape_analysis.ipynb#quaternion-estimator-algorithm--quest-)
- [Kabsch Algorithm](shape_analysis.ipynb#kabsch-algorithm)
- [Umeyama Algorithm](shape_analysis.ipynb#umeyama-algorithm)
- [Iterative Closest Point (ICP)](shape_analysis.ipynb#iterative-closest-point--icp-)



# [Ceres](#)
- [Non-linear Least Squares](docs/ceres.ipynb#Residuals-for-Bundle-Adjustment)
- [Bundle Adjustment with Ceres](docs/ceres.ipynb#Residuals-for-Bundle-Adjustment)


# [Visual and Inertial Odometry VIO](#)

- [Open Keyframe-based Visual-Inertial SLAM okvis](https://github.com/ethz-asl/okvis)  
- [HybVIO](https://github.com/SpectacularAI/HybVIO/)  
- [SVO Pro](https://github.com/uzh-rpg/rpg_svo_pro_open)  
- [OpenVINS](docs/open_vins.md)  
  - [OpenVINS Multi-Camera Extension](docs/open_vins.md#openvins-multi-camera-extension)  
  - [Visual-Inertial Navigation Systems: An Introduction](https://www.youtube.com/watch?v=dXN2E38jvQM)  
  - [IMU Propagation Derivations openvin](https://docs.openvins.com/propagation.html)  
- [Error State Kalman Filter VIO (ESKF-VIO)](docs/eskf_vio.ipynb)
- [Kimera-VIO](https://github.com/MIT-SPARK/Kimera-VIO)  
- [3D Mapping Library For Autonomous Robots](https://github.com/Zhefan-Xu/map_manager)  

# [SLAM Benchmark ](#)
- [Benchmark Comparison of Monocular Visual-Inertial Odometry Algorithms for Flying Robots](docs/visual_Inertial_SLAM_comparison.md)  
- [A Comparison of Modern General-Purpose Visual SLAM Approaches](https://arxiv.org/pdf/2107.07589)  
- [ETH3D](https://www.eth3d.net/slam_overview)  
- [rvp group](https://rvp-group.net/slam-dataset.html)  


# [Lidar and IMU LIO](#)
- [A Stereo Event Camera Dataset for Driving Scenarios DSEC](docs/lidar_and_imu.md#a-stereo-event-camera-dataset-for-driving-scenarios-dsec)  
- [FAST-LIO (Fast LiDAR-Inertial Odometry)](docs/lidar_and_imu.md#fast-lio--fast-lidar-inertial-odometry-)  
- [incremental Generalized Iterative Closest Point (GICP) based tightly-coupled LiDAR-inertial odometry (LIO), iG-LIO](docs/lidar_and_imu.md#incremental-generalized-iterative-closest-point--gicp--based-tightly-coupled-lidar-inertial-odometry--lio---ig-lio)  
- [Direct LiDAR-Inertial Odometry: Lightweight LIO with Continuous-Time Motion Correction](docs/lidar_and_imu.md#direct-lidar-inertial-odometry--lightweight-lio-with-continuous-time-motion-correction)  
- [Robust Real-time LiDAR-inertial Initialization](docs/lidar_and_imu.md#robust-real-time-lidar-inertial-initialization)  
- [CT-LIO: Continuous-Time LiDAR-Inertial Odometry](docs/lidar_and_imu.md#ct-lio--continuous-time-lidar-inertial-odometry)  
- [Lidar SLAM for Automated Driving (MATLAB learning)](https://www.youtube.com/watch?v=n4tazoEcBGo)  
- [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM/tree/ros2)  
- [GLIM](https://github.com/koide3/glim)  
- [Lidar-Monocular Visual Odometry](https://github.com/johannes-graeter/limo)




# [Structure-from-Motion](#)
- [Structure from Motion from Scratch](docs/sfm.ipynb)
- [Robust Rotation Averaging](https://www.youtube.com/watch?v=oAR-LMStRS4)
- [Bundler](https://www.cs.cornell.edu/~snavely/bundler/bundler-v0.4-manual.html)
- [Noah Snavely Reprojection Error](docs/noah_snavely_reprojection_error.ipynb)
- [Global Structure-from-Motion Revisited](https://arxiv.org/pdf/2407.20219)
- [LightGlue](https://github.com/cvg/LightGlue)
- [DenseSFM](https://github.com/tsattler/visuallocalizationbenchmark)
- [Pixel-Perfect Structure-from-Motion](https://github.com/cvg/pixel-perfect-sfm)
- [image-matching-webui](https://huggingface.co/spaces/Realcat/image-matching-webui)


# [Deep Learning based SLAM](#)
- [Gaussian Splatting](docs/gaussian_splatting.md)  
- [GANeRF](https://github.com/barbararoessle/ganerf)  
- [DSAC*](https://github.com/vislearn/dsacstar)  
- [Tracking Any Point (TAP)](https://github.com/google-deepmind/tapnet)  
- [image-matching-benchmark](https://github.com/ubc-vision/image-matching-benchmark)  
- [Local Feature Matching at Light Speed](https://github.com/cvg/LightGlue)  
- [Hierarchical Localization](https://github.com/cvg/Hierarchical-Localization)  
- [instant-ngp](docs/instant_ngp.md)  
- [NeRF-SLAM](docs/NeRF-SLAM.md)  
- [DROID-SLAM](https://github.com/princeton-vl/DROID-SLAM?tab=readme-ov-file)  
- [ACE0](https://github.com/nianticlabs/acezero)  
- [A Hierarchical 3D Gaussian Representation for Real-Time Rendering of Very Large Datasets](https://arxiv.org/pdf/2406.12080)  - 
- [DoubleTake:Geometry Guided Depth Estimation](https://nianticlabs.github.io/doubletake/)  
- [Mitigating Motion Blur in Neural Radiance Fields with Events and Frames](https://github.com/uzh-rpg/EvDeblurNeRF)  
- [LEAP-VO: Long-term Effective Any Point Tracking for Visual Odometry](https://chiaki530.github.io/projects/leapvo/)  
- [MegaScenes: Scene-Level View Synthesis at Scale](https://megascenes.github.io/)  
- [Intrinsic Image Diffusion for Indoor Single-view Material Estimation](https://github.com/Peter-Kocsis/IntrinsicImageDiffusion)  
- [Vidu4D: Single Generated Video to High-Fidelity 4D Reconstruction with Dynamic Gaussian Surfels](https://vidu4d-dgs.github.io/)  
- [Detector-Free Structure from Motion](https://hxy-123.github.io/)  
- [Continuous 3D Perception Model with Persistent State](https://github.com/CUT3R/CUT3R)  
- [MegaSam: Accurate, Fast and Robust Casual Structure and Motion from Casual Dynamic Videos](https://github.com/mega-sam/mega-sam)  
- [Mast3r Slam with Rerun](https://github.com/rerun-io/mast3r-slam)  
- [Stereo Any Video:Temporally Consistent Stereo Matching](https://tomtomtommi.github.io/StereoAnyVideo/)  
- [Multi-view Reconstruction via SfM-guided Monocular Depth Estimation](https://github.com/zju3dv/Murre)  
- [UniK3D: Universal Camera Monocular 3D Estimation](https://github.com/lpiccinelli-eth/unik3d)  
- [Depth Any Camera: Zero-Shot Metric Depth Estimation from Any Camera](https://github.com/yuliangguo/depth_any_camera)  
- [Fast3R: Towards 3D Reconstruction of 1000+ Images in One Forward Pass](https://github.com/facebookresearch/fast3r)  
- [VGGT: Visual Geometry Grounded Transformer](https://github.com/facebookresearch/vggt)  
- [MAGiC-SLAM: Multi-Agent Gaussian Globally Consistent SLAM](https://vladimiryugay.github.io/magic_slam/index.html)  
- [Speedy MASt3R](https://arxiv.org/html/2503.10017v1)  
- [CURL-MAP](https://github.com/SenseRoboticsLab/CURL-MAP)  


# [Gaussian Splat](#)
- [Morpheus Text-Driven 3D Gaussian Splat Shape and Color Stylization](https://nianticlabs.github.io/morpheus/)  



# [Object Pose and Shape Estimation](#)
- [RF-DETR: SOTA Real-Time Object Detection Model](https://github.com/roboflow/rf-detr)  


# [Visualization](#)
- [Layered Image Vectorization via Semantic Simplification](https://szuviz.github.io/layered_vectorization/)  
- [Potree: WebGL based point cloud renderer ](https://github.com/potree/potree/)  

# [Procrustes Analysis](docs/shape_analysis.md#procrustes-analysis)  

- [Procrustes Analysis](#procrustes-analysis)  
- [Wahba's Problem](#wahba-s-problem)
- [Quaternion Estimator Algorithm (QUEST)](#quaternion-estimator-algorithm--quest-)  
- [Kabsch Algorithm](#kabsch-algorithm)  
- [Umeyama Algorithm](#umeyama-algorithm)  
- [Iterative Closest Point (ICP)](#iterative-closest-point--icp-)  
- [KISS-ICP](#kiss-icp)  


# [Lidar-Camera Calibration](#)
- [MATLAB Lidar-Camera Calibration](https://www.mathworks.com/help/lidar/ug/lidar-camera-calibration.html)  
- [ROS2 Camera Lidar Fusion](https://github.com/CDonosoK/ros2_camera_lidar_fusion)  
- [Awesome-LiDAR-Camera-Calibration](https://github.com/Deephome/Awesome-LiDAR-Camera-Calibration)  





# [E-Books and Refs](#)

- [Modern Robotics Mechanics, Planning, and Control (Kevin M. Lynch, Frank C. Park)](docs/ebooks/Modern%20Robotics%20Mechanics%2C%20Planning%2C%20and%20Control%20%28Kevin%20M.%20Lynch%20and%20Frank%20C.%20Park%29.pdf)  
- [Modern Robotics Mechanics, Planning, and Control (Instructor Solution Manual, Solutions )](docs/ebooks/Modern%20Robotics%20Mechanics%2C%20Planning%2C%20and%20Control%20%28Kevin%20M.%20Lynch%2C%20Frank%20C.%20Park%29%20Exercise%20Solutions.pdf)  
- [MODERN ROBOTICS MECHANICS, PLANNING, AND CONTROL (Practice Exercises)](docs/ebooks/MODERN%20ROBOTICS%20MECHANICS%2C%20PLANNING%2C%20AND%20CONTROL%20%28Practice%20Exercises%29.pdf)  
- [Basic Knowledge on Visual SLAM: From Theory to Practice, by Xiang Gao, Tao Zhang, Qinrui Yan and Yi Liu](docs/ebooks/slambook-en.pdf)  
- [STATE ESTIMATION FOR ROBOTICS (Timothy D. Barfoot)](docs/ebooks/STATE%20ESTIMATION%20FOR%20ROBOTICS%20%28Timothy%20D.%20Barfoot%29.pdf)  
- [SLAM for Dummies](docs/ebooks/SLAM%20for%20Dummies.pdf)  
- [VSLAM Handbook](https://github.com/tussedrotten/vslam-handbook)  
- [SLAM Handbook](https://github.com/SLAM-Handbook-contributors/slam-handbook-public-release)
- [Matrix Calculus (for Machine Learning and Beyond)](https://arxiv.org/pdf/2501.14787)
- [Reinforcement Learning: A Comprehensive Overview](https://arxiv.org/pdf/2412.05265)