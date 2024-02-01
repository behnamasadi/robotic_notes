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
- [Estate Estimation](docs/ros_gazebo.md#estate-estimation)
  * [State Space](docs/ros_gazebo.md#state-space)
  * [Jacobian Matrix of Motion](docs/ros_gazebo.md#jacobian-matrix-of-motion)
- [IMU](docs/ros_gazebo.md#imu)
  * [IMU Noise Model](docs/ros_gazebo.md#imu-noise-model)
  * [MPU-9250](docs/ros_gazebo.md#mpu-9250)
- [ROS State Estimation](docs/ros_gazebo.md#ros-state-estimation)
- [EKF Implementations](docs/ros_gazebo.md#ekf-implementations)
- [Visual Odometry](docs/ros_gazebo.md#visual-odometry)
- [Online Resources](docs/ros_gazebo.md#online-resources)
- [Gauss-Newton](docs/ros_gazebo.md#gauss-newton)
- [Conditional Distribution of Y Given X](docs/ros_gazebo.md#conditional-distribution-of-y-given-x)
- [Hand-Eye Calibration](docs/ros_gazebo.md#hand-eye-calibration)
  * [Eliminating Duplicate Solutions by Limiting the Roll and Pitch Ranges](docs/ros_gazebo.md#eliminating-duplicate-solutions-by-limiting-the-roll-and-pitch-ranges)
- [Quaternion estimator algorithm (QUEST)](docs/ros_gazebo.md#quaternion-estimator-algorithm--quest-)
- [Closed-form solution of absolute orientation using unit quaternions](docs/ros_gazebo.md#closed-form-solution-of-absolute-orientation-using-unit-quaternions)
- [Wahba's problem](docs/ros_gazebo.md#wahba-s-problem)
- [Angular Velocity Vector Transformation](docs/ros_gazebo.md#angular-velocity-vector-transformation)



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

# Kalibr

- [Datasets and Calibration Targets](docs/kalibr.md#datasets-and-calibration-targets)
- [Supported Camera Models and Distortion](docs/kalibr.md#supported-camera-models-and-distortion)
- [Camera Calibration](docs/kalibr.md#camera-calibration)
- [Camera IMU Calibration](docs/kalibr.md#camera-imu-calibration)

# Apriltag

- [Apriltag](docs/apriltag.md#apriltag)
- [Generating Tag](docs/apriltag.md#generating-tag)
- [Apriltag ROS](docs/apriltag.md#apriltag-ros)
- [Detecting Apriltag](docs/apriltag.md#detecting-apriltag)

# Configuration Space
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

# Joystick
- [Joystick](docs/joystick.md)

# Geometry Transformation
- [Homogeneous Transformations](docs/geometry_transformation.md#homogeneous-transformations)
- [Translation, Scaling, and Rotations Matrices](docs/geometry_transformation.md#translation--scaling--and-rotations-matrices)
- [Euler Angles](docs/geometry_transformation.md#euler-angles)
- [Quaternions](docs/geometry_transformation.md#quaternions)









