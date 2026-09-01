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

The Rerun C++ SDK (and its Arrow dependency) is **disabled by default** because the SDK is downloaded and built via `FetchContent`, which is heavy. The Python `rerun-sdk` package covers the same use cases for the notebooks. To opt in and build the rerun-dependent C++ targets, add the following flags:

```
cmake -S . -B build \
  -DCMAKE_TOOLCHAIN_FILE=./vcpkg/scripts/buildsystems/vcpkg.cmake \
  -DCMAKE_BUILD_TYPE=Release \
  -DVCPKG_TARGET_TRIPLET=x64-linux-release \
  -DBUILD_RERUN_EXAMPLES=ON \
  -DVCPKG_MANIFEST_FEATURES=rerun
```


```
cmake --build build --parallel
```

### Python Dependencies

```
conda create -n robotic_notes  python=3.12 -y
conda activate robotic_notes
cd /home/$USER/anaconda3/envs/robotic_notes/
```

Create this soft link.

```
ln -s /home/$USER/workspace/robotic_notes /home/$USER/anaconda3/envs/robotic_notes/src
```

Install the python packages:
```
conda install -c conda-forge opencv
pip install graphslam
conda install conda-forge::gtsam
conda install conda-forge::matplotlib
conda install conda-forge::plotly
conda install -c conda-forge jupyterlab
pip install ahrs
pip install pyceres
pip install liegroups
pip install rerun-sdk[notebook]==0.29.2
pip install "gradio_rerun==0.29.2"
pip install "gradio==6.5.1"
pip install ipykernel
pip install jupyterlab
```






# [Continuous Integration](#)
- [Running GitHub Actions locally with `act` (test CI before you push)](docs/run_github_actions_locally_with_act.md)

# [Robotic System Design & Design Patterns](#)
- [Robotic System Design & Design Patterns](docs/robotic_system_design.md)
  * [1. High-Level Architecture of a Humanoid Robot](docs/robotic_system_design.md#1-high-level-architecture-of-a-humanoid-robot)
  * [2. Perception Pipeline: Consistency, Robustness, Synchronization](docs/robotic_system_design.md#2-perception-pipeline-consistency-robustness-synchronization)
  * [3. High-Frequency Sensor Pipelines in C++](docs/robotic_system_design.md#3-high-frequency-sensor-pipelines-in-c)
  * [4. Concurrency & Real-Time: SPMC Sensor Distribution](docs/robotic_system_design.md#4-concurrency--real-time-spmc-sensor-distribution)
  * [5. Task Execution: State Machine vs Behavior Tree](docs/robotic_system_design.md#5-task-execution-state-machine-vs-behavior-tree)
  * [6. Debugging a Regressing Perception Model in Production](docs/robotic_system_design.md#6-debugging-a-regressing-perception-model-in-production)
  * [7. Time Synchronization](docs/robotic_system_design.md#7-time-synchronization)
  * [8. Safety & Fault Tolerance](docs/robotic_system_design.md#8-safety--fault-tolerance)
  * [9. Logging, Observability & Replay](docs/robotic_system_design.md#9-logging-observability--replay)

- [Designing a Geometry & Imaging Library in C++](docs/geometry_library_design.md)
  * [1. The three libraries, and what each one got right](docs/geometry_library_design.md#1-the-three-libraries-and-what-each-one-got-right)
  * [2. The type razor: what earns a type](docs/geometry_library_design.md#2-the-type-razor-what-earns-a-type)
  * [3. Storage: stack or heap, and who decides](docs/geometry_library_design.md#3-storage-stack-or-heap-and-who-decides)
  * [4. The three-way split: owner, view, expression](docs/geometry_library_design.md#4-the-three-way-split-owner-view-expression)
  * [5. Ownership: `unique_ptr`, `shared_ptr`, raw, view](docs/geometry_library_design.md#5-ownership-unique_ptr-shared_ptr-raw-view)
  * [6. Templates: what to parameterise on, and what not to](docs/geometry_library_design.md#6-templates-what-to-parameterise-on-and-what-not-to)
  * [7. Inheritance: CRTP, virtual, and type erasure](docs/geometry_library_design.md#7-inheritance-crtp-virtual-and-type-erasure)
  * [8. Rotations: matrix, quaternion, angle-axis, Euler](docs/geometry_library_design.md#8-rotations-matrix-quaternion-angle-axis-euler)
  * [9. Lie groups and Lie algebras: two types, not one](docs/geometry_library_design.md#9-lie-groups-and-lie-algebras-two-types-not-one)
  * [10. Inversion: when to transpose, and when to genuinely invert](docs/geometry_library_design.md#10-inversion-when-to-transpose-and-when-to-genuinely-invert)
  * [11. Putting frames in the type system](docs/geometry_library_design.md#11-putting-frames-in-the-type-system)
  * [12. The class map: hierarchy, composition, dependency](docs/geometry_library_design.md#12-the-class-map-hierarchy-composition-dependency)
  * [13. Images and volumes](docs/geometry_library_design.md#13-images-and-volumes)
  * [14. Applying a transform to an image: why warping *does* invert](docs/geometry_library_design.md#14-applying-a-transform-to-an-image-why-warping-does-invert)
  * [15. Point clouds: AoS vs SoA](docs/geometry_library_design.md#15-point-clouds-aos-vs-soa)
  * [16. Rotating fast, without copying](docs/geometry_library_design.md#16-rotating-fast-without-copying)
  * [17. Algorithms and extensibility: plugging in a solver](docs/geometry_library_design.md#17-algorithms-and-extensibility-plugging-in-a-solver)
  * [18. Decision tables: which representation, when](docs/geometry_library_design.md#18-decision-tables-which-representation-when)
  * [19. Files, headers and build structure](docs/geometry_library_design.md#19-files-headers-and-build-structure)
  * [20. Modern C++: what the language now does for you](docs/geometry_library_design.md#20-modern-c-what-the-language-now-does-for-you)
  * [21. Specification of a good design](docs/geometry_library_design.md#21-specification-of-a-good-design)
  * [22. Worked examples: what the calls actually look like](docs/geometry_library_design.md#22-worked-examples-what-the-calls-actually-look-like)
  * [23. Anti-patterns: the mistakes people actually make](docs/geometry_library_design.md#23-anti-patterns-the-mistakes-people-actually-make)
  * [24. Do / Don't](docs/geometry_library_design.md#24-do--dont)
  * [25. The whole picture](docs/geometry_library_design.md#25-the-whole-picture)
  * companion code: [`src/geometry_library_design/`](src/geometry_library_design/) — `tiny_geometry.hpp` + `demo.cpp`, dependency-free, build with `g++ -std=c++17 -O2 demo.cpp -o demo` or the CMake target `tiny_geometry_demo`


# [Lie Group and Lie Algebra](#)
- [Lie Group and Lie Algebra](docs/lie_group_lie_algebra.ipynb)   
- [manif](docs/manif.ipynb)   
  * [SE2 localization](docs/se2_localization.ipynb)
  * [SE2 SAM](docs/se2_sam.ipynb)
- [Sophus](https://github.com/strasdat/Sophus)  

# [Topology and Configuration of Robot and Space](#)
- [Robot Configuration, Degrees of Freedom, and Topology](docs/robot_configuration_dof_topology.md)
  * [Configuration](docs/robot_configuration_dof_topology.md#configuration)
  * [Configuration space (C-space)](docs/robot_configuration_dof_topology.md#configuration-space-c-space)
  * [Degrees of freedom](docs/robot_configuration_dof_topology.md#degrees-of-freedom)
    + [Joint table](docs/robot_configuration_dof_topology.md#joint-table)
    + [Worked examples](docs/robot_configuration_dof_topology.md#worked-examples)
    + [When Grübler's formula fails](docs/robot_configuration_dof_topology.md#when-grüblers-formula-fails)
  * [Task space](docs/robot_configuration_dof_topology.md#task-space)
  * [Workspace](docs/robot_configuration_dof_topology.md#workspace)
  * [Topology of C-space](docs/robot_configuration_dof_topology.md#topology-of-c-space)
  * [Representations of C-space](docs/robot_configuration_dof_topology.md#representations-of-c-space)
    + [Explicit (minimal) representation](docs/robot_configuration_dof_topology.md#explicit-minimal-representation)
    + [Implicit (constrained) representation](docs/robot_configuration_dof_topology.md#implicit-constrained-representation)
    + [Configuration vs velocity constraints](docs/robot_configuration_dof_topology.md#configuration-vs-velocity-constraints)
  * [Why this matters](docs/robot_configuration_dof_topology.md#why-this-matters)
- [Algebraic topology (playlist)](https://www.youtube.com/playlist?list=PLuFcVFHMIfhJSSX-tlv8XxiAZSAbhv1DA)
- [Non-Holonomic Constraints, Pfaffian Constraints and Holonomic Constraints](docs/non_holonomic_pfaffian_constraint.ipynb)



# [PX4](#)
- [PX4](docs/px4.md)


# [Differential Drive Robots and Wheel odometry](#)

- [Kinematics of Differential Drive Robots and Wheel odometry](docs/differential_drive_robots_and_wheel_odometry.ipynb#kinematics-of-differential-drive-robots-and-wheel-odometry)
- [1. Velocity-based (dead reckoning)](docs/differential_drive_robots_and_wheel_odometry.ipynb#1-velocity-based-dead-reckoning)
  - [1.1 Forward Kinematics](docs/differential_drive_robots_and_wheel_odometry.ipynb#11-forward-kinematics)
  - [1.2 Inverse Kinematics of Differential Drive Robots](docs/differential_drive_robots_and_wheel_odometry.ipynb#12-inverse-kinematics-of-differential-drive-robots)
- [2. Odometry-based Motion Model](docs/differential_drive_robots_and_wheel_odometry.ipynb#2-odometry-based-motion-model)
- [Nonlinear uncertainty model associated with a robot's position over time (The Banana Distribution is Gaussian)](docs/nonlinear_uncertainty_model_associated_with_robot_position_banana_shape_.ipynb)

# [IMU](#)

- [Coordinate Frame Conventions](docs/coordinate_frame_conventions.ipynb) — ENU, NED, FRD, FLU, RDF, RUB
- [Global frame conventions](docs/imu.ipynb#global-frame-conventions)
  * [World/Navigation frame conventions](docs/imu.ipynb#worldnavigation-frame-conventions)
- [1. Frames](docs/imu.ipynb#1-frames)
- [2. What is stored in the ESKF state?](docs/imu.ipynb#2-what-is-stored-in-the-eskf-state)
- [3. What does the IMU measure?](docs/imu.ipynb#3-what-does-the-imu-measure)
- [Accelerometer model](docs/imu.ipynb#accelerometer-model)
- [Gyroscope model](docs/imu.ipynb#gyroscope-model)
- [What the accelerometer measures](docs/imu.ipynb#what-the-accelerometer-measures)
- [Rest (no motion) numeric example: why you see ±9.81](docs/imu.ipynb#rest-no-motion-numeric-example-why-you-see-981)
- [“Add gravity” in the velocity update (numeric)](docs/imu.ipynb#add-gravity-in-the-velocity-update-numeric)
- [Random walk](docs/imu.ipynb#random-walk)
- [Attitude estimation: two flavors](docs/imu.ipynb#attitude-estimation-two-flavors)
- [Attitude from gravity (tilt sensing)](docs/imu.ipynb#attitude-from-gravity-tilt-sensing)
  * [Solving R_xyz = Rx(φ) Ry(θ) Rz(ψ) for roll and pitch](docs/imu.ipynb#r_xyz--r_xphir_ythetar_zpsi)
  * [Solving R_yxz = Ry(θ) Rx(φ) Rz(ψ) for roll and pitch](docs/imu.ipynb#r_yxz--r_ythetar_xphir_zpsi)
- [Numerical Example](docs/imu.ipynb#numerical-example)
- [Uniqueness of R_BW](docs/imu.ipynb#uniqueness-of-r_bw)
- [Quaternion integration](docs/imu.ipynb#quaternion-integration)
  * [First-order numerical update](docs/imu.ipynb#first-order-numerical-update)
  * [Closed-form (exact for constant ω over Δt)](docs/imu.ipynb#closed-form-exact-for-constant-boldsymbolomega-over-delta-t)
- [Quaternion from a single accelerometer reading](docs/imu.ipynb#quaternion-from-a-single-accelerometer-reading)
- [Euler-angle rates from body-axis rates](docs/imu.ipynb#euler-angle-rates-from-body-axis-rates)
- [Complementary filter](docs/imu.ipynb#complementary-filter)
- [Yaw from a magnetometer (tilt-compensated compass)](docs/imu.ipynb#yaw-from-a-magnetometer-tilt-compensated-compass)
- [4. Orientation propagation (Body → World)](docs/imu.ipynb#4-orientation-propagation-body--world)
- [5. Why accelerometer must be rotated](docs/imu.ipynb#5-why-accelerometer-must-be-rotated)
- [6. Why gravity is added](docs/imu.ipynb#6-why-gravity-is-added)
- [What does the IMU measure here?](docs/imu.ipynb#what-does--imu-measure-here)
- [Why measured acceleration at IMU rest was not 9.8](docs/imu.ipynb#why-measured-acceleration-at-imu-rest-was-not-98)
- [7. Velocity propagation (world frame)](docs/imu.ipynb#7-velocity-propagation-world-frame)
- [8. Position propagation (world frame)](docs/imu.ipynb#8-position-propagation-world-frame)
- [9. Bias propagation (random walk)](docs/imu.ipynb#9-bias-propagation-random-walk)
- [Kalibr-style IMU noise model](docs/imu.ipynb#kalibr-style-imu-noise-model)
  * [Additive white noise](docs/imu.ipynb#additive-white-noise)
  * [Bias random walk](docs/imu.ipynb#bias-random-walk)
  * [Kalibr YAML reference](docs/imu.ipynb#kalibr-yaml-reference)
  * [Obtaining the parameters](docs/imu.ipynb#obtaining-the-parameters)
- [Allan variance background](docs/imu.ipynb#allan-variance-background)
- [EuRoC noise params → discrete-time numbers you use in code](docs/imu.ipynb#euroc-noise-params--discrete-time-numbers-you-use-in-code)
- [Further references](docs/imu.ipynb#further-references)


# [Fiducial Markers](#)
- [Square binary fiducials (ArUco, AprilTag](docs/apriltag.md#2-square-binary-fiducials--the-family-relevant-to-robotics)
- [Composite calibration boards (ChArUco, AprilGrid)](docs/apriltag.md#3-composite-calibration-boards)
- [Which marker should I use?](docs/apriltag.md#4-decision-shortcut--which-marker-should-i-use)
- [AprilTag — coordinate frame and measurements](docs/apriltag.md#5-coordinate-frame-and-measurements)
- [Generating AprilTags](docs/apriltag.md#6-generating-tags)
- [Detecting AprilTags (C++, Python, ROS)](docs/apriltag.md#7-detecting-apriltags)
- [Common pitfalls](docs/apriltag.md#8-common-pitfalls)
- [AprilTag vs ArUco (and ChArUco / AprilGrid)](docs/apriltag_vs_aruco.md)
  * [What the family / dictionary names mean (tag36h11, DICT_6X6_250)](docs/apriltag_vs_aruco.md#2-what-the-family--dictionary-names-mean)
  * [Calibration boards — ChArUco vs AprilGrid (with generation)](docs/apriltag_vs_aruco.md#3-the-real-fork-in-the-road-calibration-boards)
  * [Decision guide](docs/apriltag_vs_aruco.md#4-decision-guide)

# [Kalibr](#)

- [Choosing a calibration target](docs/kalibr.md#1-choosing-a-calibration-target)
- [Calibration target files (AprilGrid PDF, YAMLs)](docs/kalibr.md#2-calibration-target-files)
- [Supported camera models and distortion](docs/kalibr.md#3-supported-camera-models-and-distortion)
- [Building the Kalibr Docker image](docs/kalibr.md#4-building-the-kalibr-docker-image)
- [Recording the calibration bag](docs/kalibr.md#5-recording-the-calibration-bag)
- [Camera-only calibration](docs/kalibr.md#6-camera-only-calibration)
- [Camera–IMU calibration](docs/kalibr.md#7-cameraimu-calibration)
- [Common pitfalls](docs/kalibr.md#8-common-pitfalls)



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
- [move_base](docs/ros1.md#move_base)
- [ROS Odometery Model](docs/ros1.md#ros-odometery-model)
- [ROS State Estimation](docs/ros1.md#ros-state-estimation)
- [EKF Implementations](docs/ros1.md#ekf-implementations)
- [Differential Drive Wheel Systems](docs/ros1.md#differential-drive-wheel-systems)

# [ROS2](#)
- [Choosing a distro](docs/ros2.md#0-which-distro-should-you-actually-use)
- [Pull the image](docs/ros2.md#1-pull-the-image)
- [Run a persistent container](docs/ros2.md#3-run-a-persistent-container)
- [Extending the image with a Dockerfile](docs/ros2.md#6-extending-the-image-with-a-dockerfile)
- [Docker Compose for multi-container setups](docs/ros2.md#7-docker-compose-for-multi-container-setups)
- [ROS 2 ↔ ROS 1 bridge](docs/ros2.md#8-ros-2--ros-1-bridge-also-dockerized)
- [ROS 2 inside the container](docs/ros2.md#9-ros-2-inside-the-container)
  * [Domain ID](docs/ros2.md#domain-id)
  * [ROS_AUTOMATIC_DISCOVERY_RANGE](docs/ros2.md#ros_automatic_discovery_range)
  * [QoS (Quality of Service)](docs/ros2.md#qos-quality-of-service) → full doc: [ros2_qos.md](docs/ros2_qos.md)
  * [Remapping](docs/ros2.md#remapping)
- [Colcon](docs/ros2.md#10-colcon-inside-the-container)
- [Packages](docs/ros2.md#11-packages)
- [Xacro](docs/ros2.md#12-xacro)
- [Launch files](docs/ros2.md#13-launch-files)
  * [Python launch file](docs/ros2.md#python-launch-file)
  * [C++ launch file](docs/ros2.md#c-launch-file)
  * [Substitutions](docs/ros2.md#substitutions)
  * [Event handlers](docs/ros2.md#event-handlers)
- [Nav2](docs/ros2.md#14-nav2)
- [teleop_twist_keyboard](docs/ros2.md#15-teleop_twist_keyboard)
- [Useful container shortcuts](docs/ros2.md#16-useful-container-shortcuts)


# [Gazebo](#)

- [Gazebo Versions](docs/gazebo.md#gazebo-versions)
- [Installation](docs/gazebo.md#installation)
- [Building a model](docs/gazebo.md#building-a-model)
  * [Links](docs/gazebo.md#links)
    + [Inertia Matrix](docs/gazebo.md#inertia-matrix)
  * [Visual and collision](docs/gazebo.md#visual-and-collision)
  * [Connecting links together (joints)](docs/gazebo.md#connecting-links-together-joints)
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
- [Distro / Gazebo mapping](docs/ros2_gazebo_integration.md#1-distro--gazebo-mapping)
- [Dockerfile (Jazzy + Harmonic + ros_gz)](docs/ros2_gazebo_integration.md#2-dockerfile-jazzy--harmonic--ros_gz)
- [Running the container](docs/ros2_gazebo_integration.md#3-running-the-container)
- [GPU access (NVIDIA)](docs/ros2_gazebo_integration.md#4-gpu-access-nvidia)
- [Launching Gazebo](docs/ros2_gazebo_integration.md#5-launching-gazebo)
- [`ros_gz_bridge` essentials](docs/ros2_gazebo_integration.md#6-ros_gz_bridge-essentials)
- [Demo recipes](docs/ros2_gazebo_integration.md#7-demo-recipes)
  * [Camera](docs/ros2_gazebo_integration.md#71-camera)
  * [Differential drive](docs/ros2_gazebo_integration.md#72-differential-drive)
  * [GPU LiDAR](docs/ros2_gazebo_integration.md#73-gpu-lidar)
  * [IMU + Magnetometer](docs/ros2_gazebo_integration.md#74-imu--magnetometer)
  * [GNSS](docs/ros2_gazebo_integration.md#75-gnss)
  * [Spawn a URDF model](docs/ros2_gazebo_integration.md#76-spawn-a-urdf-model)
- [Troubleshooting](docs/ros2_gazebo_integration.md#8-troubleshooting)


# [MarsLogger](#)
- [MarsLogger](docs/mars_logger.md)

# [State Estimation](#)
- [Probability Review (Marginals/conditionals, probability vs likelihood, conditional Gaussians ) ](docs/probability_review.ipynb)(foundational primer)
- [Bayes Filter](docs/bayes_filter.ipynb)
- [Maximum A-Posteriori (MAP) Estimation](docs/map.ipynb)
- [Information Form vs Covariance Form](docs/information_vs_covariance_form.md) — which parameterization to carry, and why
  * [The underlying operation is multiplication](docs/information_vs_covariance_form.md#2-the-underlying-operation-is-multiplication)
  * [The Kalman gain is the price of moment coordinates](docs/information_vs_covariance_form.md#3-the-kalman-gain-is-the-price-of-moment-coordinates)
  * [Then why does anyone use the covariance form?](docs/information_vs_covariance_form.md#4-then-why-does-anyone-use-the-covariance-form)
  * [Why SLAM back-ends live in information space](docs/information_vs_covariance_form.md#5-why-slam-back-ends-live-in-information-space)
  * [Why $H$ is literally an information matrix](docs/information_vs_covariance_form.md#6-why-h-is-literally-an-information-matrix)
  * [Marginalization: the bill information form eventually pays](docs/information_vs_covariance_form.md#7-marginalization-the-bill-information-form-eventually-pays)
  * [Decision table](docs/information_vs_covariance_form.md#9-decision-table)
- [Kalman Filter](docs/kalman_filter.ipynb)
- [Extended Kalman Filter](docs/extended_kalman_filter.ipynb)
- [Extended Kalman Filter for Differential Drive Robot](docs/extended_kalman_filter.ipynb#ekf-for-differential-drive-robot)
- [Error State Extended Kalman Filter](docs/error_state_extended_kalman_filter_vio.ipynb)
- [Error State Extended Kalman Filter(IMU, a GNSS, and a LiDAR)](https://github.com/enginBozkurt/Error-State-Extended-Kalman-Filter)  
- [Multi-State Constraint Kalman Filter (MSCKF)](https://docs.openvins.com/namespaceov__msckf.html)  
- [Quaternion kinematics for the error-state Kalman filter](https://arxiv.org/pdf/1711.02508)


# [Place Recognition](#)
- [FBOW (Fast Bag of Words)](https://github.com/rmsalinas/fbow)
- [AnyLoc: Towards Universal Visual Place Recognition](https://github.com/AnyLoc/AnyLoc)

# [Rerun](#)
- [Rerun](docs/rerun.ipynb)
- [gradio-rerun viewer](docs/gradio_rerun_viewer.md)


# [SLAM](#)
- [Active Exposure Control for Robust Visual Odometry in HDR Environments](docs/active_exposure_control_HDR_environments.md)
- [Pose Graph SLAM from Scratch](docs/pose_graph_slam.ipynb)
- [nano-pgo](https://github.com/gisbi-kim/nano-pgo)
- [General Graph Optimization g2o explained](docs/g2o.md)  
- [Factor Graphs, Bayes Trees, and iSAM2 (with GTSAM)](docs/factor_graph.ipynb)  
- [Resilient Autonomy in Perceptually-degraded Environments](https://www.youtube.com/watch?v=L0PQKxU8cps)  
- [HBA Large-Scale LiDAR Mapping Module](https://github.com/hku-mars/HBA)  
- [Hierarchical, multi-resolution volumetric mapping (wavemap)](https://github.com/ethz-asl/wavemap)  
- [kiss-icp](https://github.com/PRBonn/kiss-icp?tab=readme-ov-file)  
- [TagSLAM SLAM with tags](https://berndpfrommer.github.io/tagslam_web/)  
- [OpenDroneMap](https://github.com/OpenDroneMap/ODM)  
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
- [1. Orthogonal Procrustes](docs/shape_analysis.ipynb#1-orthogonal-procrustes)
- [2. Wahba's problem](docs/shape_analysis.ipynb#2-wahbas-problem)
- [3. Davenport's q-method and QUEST](docs/shape_analysis.ipynb#3-davenports-q-method-and-quest)
- [4. Kabsch algorithm](docs/shape_analysis.ipynb#4-kabsch-algorithm)
- [5. Umeyama algorithm](docs/shape_analysis.ipynb#5-umeyama-algorithm)
- [6. Iterative Closest Point (ICP)](docs/shape_analysis.ipynb#6-iterative-closest-point-icp)
- [Rotation Averaging — the same SVD/Procrustes trick applied to N estimates of one rotation](docs/rotation_averaging.md#1-single-rotation-averaging)



# [Ceres](#)
- [1. Gauss-Newton and Levenberg-Marquardt](docs/ceres.ipynb#1-gaussnewton-and-levenbergmarquardt)
- [3. Bundle Adjustment Example](docs/ceres.ipynb#3-bundle-adjustment-example)


# [Visual and Inertial Odometry VIO](#)

### vio_benchmark — head-to-head VIO comparison on real datasets

The [`vio_benchmark/`](vio_benchmark/) subproject is a self-contained
framework for comparing visual-inertial odometry estimators against
public datasets (EuRoC MAV, TUM-VIO, and others). It bundles OpenVINS
and VINS-Fusion as submodules, ships per-dataset calibration files
and ground-truth references, and includes pipeline scripts plus a
rerun.io visualisation that overlays multiple trajectories with the
live stereo camera projected inside each estimator's frustum. First
head-to-head result: OpenVINS 0.295 m APE vs VINS-Fusion 0.248 m on
EuRoC MH_01_easy.

- [vio_benchmark README and pipeline](vio_benchmark/README.md)
- [Head-to-head comparison report](vio_benchmark/docs/COMPARISON.md)

### Concepts and methodology
- [Trajectory analysis methodology and case studies](docs/visual_odometry/trajectory_analysis.ipynb)
- [Estimator consistency — NEES, NIS, and auditing the covariance](docs/visual_odometry/estimator_consistency.ipynb)
- [Benchmark methodology — run-to-run variance and paired significance](docs/visual_odometry/benchmark_methodology.ipynb)
- [Runtime evaluation — latency, throughput, and tail costs](docs/visual_odometry/runtime_evaluation.ipynb)
- [Diagnostic procedure for broken VIO](vio_benchmark/docs/VIO_DIAGNOSTIC_GUIDE.md)
- [Estimator parameter reference](vio_benchmark/docs/PARAMETERS.md)
- [Visual-Inertial Navigation Systems: An Introduction](https://www.youtube.com/watch?v=dXN2E38jvQM)
- [IMU Propagation Derivations (OpenVINS)](https://docs.openvins.com/propagation.html)
- [Error State Kalman Filter VIO (ESKF-VIO)](docs/error_state_extended_kalman_filter_vio.ipynb)

### Estimators
- [OpenVINS](docs/open_vins.md)
  - [OpenVINS Multi-Camera Extension](docs/open_vins.md#openvins-multi-camera-extension)
- [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
- [Kimera-VIO](https://github.com/MIT-SPARK/Kimera-VIO)
- [HybVIO](https://github.com/SpectacularAI/HybVIO/)
- [SVO Pro](https://github.com/uzh-rpg/rpg_svo_pro_open)
- [OKVIS — Open Keyframe-based Visual-Inertial SLAM](https://github.com/ethz-asl/okvis)

# [Datasets](#)

### KITTI odometry
- [Overview](docs/visual_odometry/KITTI.ipynb)
- [Sensor setup](docs/visual_odometry/KITTI.ipynb#sensor-setup)
- [Calibration files and projection matrices](docs/visual_odometry/KITTI.ipynb#calibration-files-and-projection-matrices)
- [Ground-truth poses](docs/visual_odometry/KITTI.ipynb#ground-truth-poses)
- [Displaying ground-truth poses in Rerun](docs/visual_odometry/KITTI.ipynb#display-ground-truth-poses-in-rerun)
- [Stereo vision](docs/visual_odometry/KITTI.ipynb#stereo-vision)
- [Sparse/dense reconstruction from known poses with COLMAP](docs/visual_odometry/KITTI.ipynb#reconstruct-sparsedense-model-from-known-camera-poses-with-colmap)
- [Understanding KITTI poses](docs/understanding_poses.md)

### Dataset collections
- [Dataset recipes — EuRoC, TUM-VIO, M2DGR, 4Seasons](vio_benchmark/docs/DATASETS.md)
- [M3DGR — Multi-sensor, Multi-scenario, Massive-baseline SLAM Dataset for Ground Robots](https://github.com/sjtuyinjie/M3DGR)
- [rvp group SLAM datasets](https://rvp-group.net/slam-dataset.html)
- [DSEC — Stereo Event Camera Dataset for Driving Scenarios](docs/lidar_and_imu.md#a-stereo-event-camera-dataset-for-driving-scenarios-dsec)


# [SLAM Benchmark ](#)

### This repo's results
- [Head-to-head — OpenVINS vs VINS-Fusion on EuRoC MH_01_easy](vio_benchmark/docs/COMPARISON.md)
- [Debug session log](vio_benchmark/docs/SESSION_2026-05-15.md)

### External benchmarks
- [Hilti x Trimble SLAM Challenge 2026](https://hilti-trimble-challenge.com/)
- [Benchmark Comparison of Monocular Visual-Inertial Odometry Algorithms for Flying Robots](docs/visual_Inertial_SLAM_comparison.md)
- [A Comparison of Modern General-Purpose Visual SLAM Approaches](https://arxiv.org/pdf/2107.07589)
- [ETH3D](https://www.eth3d.net/slam_overview)


# [Lidar and IMU LIO](#)

### lio_benchmark — Gazebo LIO testbed

The [`lio_benchmark/`](lio_benchmark/) subproject is a self-contained
Gazebo simulator for LiDAR-inertial odometry development. It ships a
SubT-style rover with a 16-beam LiDAR + IMU + cameras, three
cave/tunnel worlds, and FAST-LIO wired into the rover by default.
One-line bring-up via `docker compose`. Unlike VIO, LIO works
correctly on this sim (FAST-LIO consistently produces 0.3–1.5 %
end-point error on rover recordings) — the architectural reason is in
the [VIO diagnostic guide §3](vio_benchmark/docs/VIO_DIAGNOSTIC_GUIDE.md#3--why-lio-survives-bad-imu-data-and-vio-doesnt).

- [lio_benchmark README and quick start](lio_benchmark/README.md)
- [Simulator GPU/rendering performance notes](lio_benchmark/docs/PERFORMANCE.md)
- [FAST-LIO + LiDAR tuning reference](vio_benchmark/docs/PARAMETERS.md#6-lio--fast-lio-with-the-rover-sim)

### Concepts and methodology
- [Why LIO is robust to bad IMU but VIO isn't](vio_benchmark/docs/VIO_DIAGNOSTIC_GUIDE.md#3--why-lio-survives-bad-imu-data-and-vio-doesnt)
- [Robust Real-time LiDAR-inertial Initialization](docs/lidar_and_imu.md#robust-real-time-lidar-inertial-initialization)
- [Lidar SLAM for Automated Driving (MATLAB)](https://www.youtube.com/watch?v=n4tazoEcBGo)

### Estimators
- [FAST-LIO](docs/lidar_and_imu.md#fast-lio-fast-lidar-inertial-odometry)
- [iG-LIO — incremental GICP tightly-coupled LIO](docs/lidar_and_imu.md#incremental-generalized-iterative-closest-point-gicp-based-tightly-coupled-lidar-inertial-odometry-lio-ig-lio)
- [DLIO — Direct LiDAR-Inertial Odometry with Continuous-Time Motion Correction](docs/lidar_and_imu.md#direct-lidar-inertial-odometry-lightweight-lio-with-continuous-time-motion-correction)
- [CT-LIO — Continuous-Time LiDAR-Inertial Odometry](docs/lidar_and_imu.md#ct-lio-continuous-time-lidar-inertial-odometry)
- [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM/tree/ros2)
- [GLIM — versatile and extensible range-based 3D mapping framework](https://github.com/koide3/glim)
- [LIMO — Lidar-Monocular Visual Odometry](https://github.com/johannes-graeter/limo)




# [Structure-from-Motion](#)
- [Structure from Motion from Scratch](docs/sfm.ipynb)
- [Rotation Averaging](docs/rotation_averaging.md)
  - [Single rotation averaging — chordal/SVD mean, quaternion (Markley) mean, Karcher mean, Weiszfeld median](docs/rotation_averaging.md#1-single-rotation-averaging)
  - [Multiple rotation averaging (rotation synchronization) — the global-SfM problem, Govindu → Martinec–Pajdla → Chatterjee–Govindu IRLS → Shonan](docs/rotation_averaging.md#2-multiple-rotation-averaging--the-sfm-problem)
  - [Why outlier relative rotations, not the optimizer, are the hard part](docs/rotation_averaging.md#the-practical-note)
  - [Robust Rotation Averaging (talk)](https://www.youtube.com/watch?v=oAR-LMStRS4)
- [Bundler](https://www.cs.cornell.edu/~snavely/bundler/bundler-v0.4-manual.html)
- [Noah Snavely Reprojection Error](docs/noah_snavely_reprojection_error.ipynb)
- [Global Structure-from-Motion Revisited](https://arxiv.org/pdf/2407.20219)
- [LightGlue](https://github.com/cvg/LightGlue)
- [XRefine: Attention-Guided Keypoint Match Refinement](https://github.com/boschresearch/xrefine)
- [DenseSFM](https://github.com/tsattler/visuallocalizationbenchmark)
- [Pixel-Perfect Structure-from-Motion](https://github.com/cvg/pixel-perfect-sfm)
- [image-matching-webui](https://huggingface.co/spaces/Realcat/image-matching-webui)


# [Deep Learning based SLAM](#)

### Self-supervised DepthNet + PoseNet
- [Implementation tutorial](docs/visual_odometry/KITTI.ipynb#self-supervised-monocular-vo-on-kitti--practical-implementation-tutorial)
- [Depth + pose training objective](docs/visual_odometry/KITTI.ipynb#1-self-supervised-monocular-vo-depth--pose)
- [Supervised depth add-on](docs/visual_odometry/KITTI.ipynb#2-supervised-depth-optional-add-on)
- [Shapes, sizes and VRAM knobs](docs/visual_odometry/KITTI.ipynb#shapes-sizes-and-vram-knobs)
- [Minimal architectures](docs/visual_odometry/KITTI.ipynb#minimal-architectures-proven--tiny)
- [Training recipe](docs/visual_odometry/KITTI.ipynb#training-recipe-self-sup-monocular-vo-kitti)
- [Evaluation](docs/visual_odometry/KITTI.ipynb#evaluation)
- [Baselines for a 4 GB GPU](docs/visual_odometry/KITTI.ipynb#concrete-baselines-to-run-on-4-gb)
- [Multi-scale output](docs/visual_odometry/KITTI.ipynb#multi-scale-output)
- [Implementation tips](docs/visual_odometry/KITTI.ipynb#handy-implementation-tips)

### Deep-learning VO architectures
- [Overview of the main families](docs/visual_odometry/vit_monocular_vo.ipynb#overview-of-approaches-and-deep-learning-architecture-for-visual-odometry)
- [Pose regression heads — CNN vs ViT](docs/visual_odometry/vit_monocular_vo.ipynb#21-pose-regression-heads--cnn-style-vs-vit-style)
- [ViT-based VO architectures](docs/visual_odometry/vit_monocular_vo.ipynb#22-representative-vit-based-vo-architectures)
- [Temporal modeling](docs/visual_odometry/vit_monocular_vo.ipynb#23-temporal-modeling)
- [Input and output representations](docs/visual_odometry/vit_monocular_vo.ipynb#24-input-and-output-representations)
- [Self-supervised VO](docs/visual_odometry/vit_monocular_vo.ipynb#iii-unsupervised--self-supervised-vo)

### Feed-forward depth estimation
- [Depth-Anything-3](https://github.com/ByteDance-Seed/Depth-Anything-3)  
- [Depth Any Camera](https://github.com/yuliangguo/depth_any_camera)  
- [UniK3D](https://github.com/lpiccinelli-eth/unik3d)  
- [DoubleTake](https://nianticlabs.github.io/doubletake/)  
- [Murre](https://github.com/zju3dv/Murre)  
- [Stereo Any Video](https://tomtomtommi.github.io/StereoAnyVideo/)  

### Two-view pointmap regression — DUSt3R / MASt3R
- [DUSt3R & MASt3R notebook](docs/visual_odometry/dust3r_mast3r.ipynb)
- [The paper landscape](docs/visual_odometry/dust3r_mast3r.ipynb#1-the-paper-landscape)
- [The 2026 role inversion](docs/visual_odometry/dust3r_mast3r.ipynb#the-2026-role-inversion-feed-forward-front-end-classical-back-end)
- [VGGT-SLAM loop closure](docs/visual_odometry/dust3r_mast3r.ipynb#vggt-slam-how-loop-closure-works-at-submap-level)
- [Persistent state as a learned filter](docs/visual_odometry/dust3r_mast3r.ipynb#persistent-state-the-learned-filter)
- [The pointmap](docs/visual_odometry/dust3r_mast3r.ipynb#21-the-pointmap)
- [Pointmap head](docs/visual_odometry/dust3r_mast3r.ipynb#23-from-tokens-to-xyz-what-the-head-actually-regresses)
- [The canonical frame](docs/visual_odometry/dust3r_mast3r.ipynb#24-the-canonical-frame-is-just-camera-1)
- [Global alignment, not bundle adjustment](docs/visual_odometry/dust3r_mast3r.ipynb#27-multi-view-global-alignment-this-is-not-bundle-adjustment)
- [Matching — MASt3R's descriptor head](docs/visual_odometry/dust3r_mast3r.ipynb#41-matching--mast3rs-descriptor-head)
- [Recovering intrinsics](docs/visual_odometry/dust3r_mast3r.ipynb#42-recovering-intrinsics--the-focal-from-a-single-pointmap)
- [Recovering the relative pose](docs/visual_odometry/dust3r_mast3r.ipynb#43-recovering-the-relative-camera-pose)
- [Sparse global alignment (MASt3R-SfM)](docs/visual_odometry/dust3r_mast3r.ipynb#5-from-a-pair-to-a-scene-sparse-global-alignment-mast3r-sfm-style)
- [Speedy MASt3R](https://arxiv.org/html/2503.10017v1)  
- [MASt3R-SLAM with Rerun](https://github.com/rerun-io/mast3r-slam)  

### End-to-end multi-view — VGGT
- [VGGT](https://github.com/facebookresearch/vggt)  
- [MapAnything](https://github.com/facebookresearch/map-anything)  
- [Fast3R](https://github.com/facebookresearch/fast3r)  
- [VGGSfM](https://github.com/facebookresearch/vggsfm)  
- [CUT3R](https://github.com/CUT3R/CUT3R)  

### SLAM with learned front-ends
- [DROID-SLAM](https://github.com/princeton-vl/DROID-SLAM?tab=readme-ov-file)  
- [SLAM3R](https://github.com/PKU-VCL-3DV/SLAM3R)  
- [MegaSaM](https://github.com/mega-sam/mega-sam)  
- [MAGiC-SLAM](https://vladimiryugay.github.io/magic_slam/index.html)  
- [NeRF-SLAM](docs/NeRF-SLAM.md)  
- [CURL-MAP](https://github.com/SenseRoboticsLab/CURL-MAP)  

### Feature matching and point tracking
- [MatchAnything](docs/match_anything/index.ipynb)  
- [LightGlue](https://github.com/cvg/LightGlue)  
- [Tracking Any Point (TAP)](https://github.com/google-deepmind/tapnet)  
- [LEAP-VO](https://chiaki530.github.io/projects/leapvo/)  

### Learned SfM and visual localization
- [Detector-Free Structure from Motion](https://hxy-123.github.io/)  
- [InstantSfM](https://github.com/cre185/InstantSfM)  
- [ACE0](https://github.com/nianticlabs/acezero)  
- [Hierarchical Localization](https://github.com/cvg/Hierarchical-Localization)  

### Neural rendering and mapping
- [Gaussian Splatting](docs/gaussian_splatting.md)  
- [instant-ngp](docs/instant_ngp.md)  
- [Hierarchical 3D Gaussians for large datasets](https://arxiv.org/pdf/2406.12080)  
- [EvDeblurNeRF](https://github.com/uzh-rpg/EvDeblurNeRF)  
- [MegaScenes](https://megascenes.github.io/)  
- [Morpheus](https://nianticlabs.github.io/morpheus/)  

### Semantic and multimodal 3D perception
- [Human3R](https://github.com/fanegg/Human3R)  
- [Describe Anything, Anywhere, at Any Moment (DAAAM)](https://github.com/MIT-SPARK/DAAAM)  
- [Puffin — camera-centric multimodal model](https://github.com/KangLiao929/Puffin)  


# [Trajectory and Pose Evaluation](#)

### Trajectory metrics
- [Trajectory analysis](docs/visual_odometry/trajectory_analysis.ipynb)  
- [Why naive comparison fails — the alignment problem](docs/visual_odometry/trajectory_analysis.ipynb#1-why-naive-comparison-fails--the-alignment-problem)  
- [Absolute Pose Error (APE / ATE)](docs/visual_odometry/trajectory_analysis.ipynb#2-absolute-pose-error-ape--ate)  
- [Relative Pose Error (RPE)](docs/visual_odometry/trajectory_analysis.ipynb#3-relative-pose-error-rpe)  
- [Alignment — Umeyama Sim(3), SE(3), yaw-only, posyaw](docs/visual_odometry/trajectory_analysis.ipynb#4-alignment--umeyama-and-friends)  
- [Sub-trajectory drift](docs/visual_odometry/trajectory_analysis.ipynb#5-sub-trajectory-drift--the-recommended-evaluation)  
- [Which statistics to report](docs/visual_odometry/trajectory_analysis.ipynb#6-which-statistics-to-report)  
- [Tools — `evo` and `rpg_trajectory_evaluation`](docs/visual_odometry/trajectory_analysis.ipynb#7-tools--evo-and-rpg_trajectory_evaluation)  
- [Reporting recipe](docs/visual_odometry/trajectory_analysis.ipynb#8-reporting-recipe-for-a-new-voslam-method)  

### Diagnostics
- [Parallax and forward driving](docs/visual_odometry/trajectory_analysis.ipynb#9-parallax--and-why-forward-driving-is-the-worst-case)  
- [Mono VIO has unobservable yaw](docs/visual_odometry/trajectory_analysis.ipynb#10-mono-vio-has-unobservable-yaw)  
- [KLT vs descriptor matching](docs/visual_odometry/trajectory_analysis.ipynb#11-tracker-choice--klt-vs-descriptor-matching)  

### Benchmark scores — Image Matching Challenge
- [Evaluation metrics for poses and trajectories](docs/vo_evaluation_metrics.ipynb)  
- [The one residual behind ATE and mAA](docs/vo_evaluation_metrics.ipynb#1-the-one-residual-behind-ate-and-maa)  
- [Same Umeyama, different robustness](docs/vo_evaluation_metrics.ipynb#2-same-umeyama-different-robustness)  
- [The convention trap: $C = -R^\top t$](docs/vo_evaluation_metrics.ipynb#3-the-convention-trap-c---rtop-t)  
- [Two different metrics are both called mAA](docs/vo_evaluation_metrics.ipynb#4-two-different-metrics-are-both-called-maa)  
- [image-matching-benchmark](https://github.com/ubc-vision/image-matching-benchmark)  

### Estimator consistency — is the reported covariance honest
- [Estimator consistency](docs/visual_odometry/estimator_consistency.ipynb)  
- [Accuracy and consistency are different failure modes](docs/visual_odometry/estimator_consistency.ipynb#1-accuracy-and-consistency-are-different-failure-modes)  
- [NEES — Normalised Estimation Error Squared](docs/visual_odometry/estimator_consistency.ipynb#2-nees--normalised-estimation-error-squared)  
- [The gauge trap — why absolute NEES is usually meaningless](docs/visual_odometry/estimator_consistency.ipynb#3-the-gauge-trap--the-reason-absolute-nees-is-usually-meaningless)  
- [ANEES — and why one run tests almost nothing](docs/visual_odometry/estimator_consistency.ipynb#4-anees--and-why-one-run-tests-almost-nothing)  
- [NIS — consistency without ground truth](docs/visual_odometry/estimator_consistency.ipynb#5-nis--the-consistency-test-that-needs-no-ground-truth)  
- [Diagnosis — FEJ, double-counting, sparsification](docs/visual_odometry/estimator_consistency.ipynb#6-diagnosis--what-actually-causes-each-direction)  
- [Where P comes from — and the convention trap](docs/visual_odometry/estimator_consistency.ipynb#7-where-p-comes-from--and-the-convention-trap)  
- [Reporting recipe](docs/visual_odometry/estimator_consistency.ipynb#8-reporting-recipe)  

### Benchmark methodology — variance, significance, honest comparison
- [Benchmark methodology](docs/visual_odometry/benchmark_methodology.ipynb)  
- ["Deterministic" algorithms are not](docs/visual_odometry/benchmark_methodology.ipynb#1-deterministic-algorithms-are-not)  
- [How many runs, and what to do with them](docs/visual_odometry/benchmark_methodology.ipynb#2-how-many-runs-and-what-to-do-with-them)  
- [Compare paired, not unpaired](docs/visual_odometry/benchmark_methodology.ipynb#3-compare-paired-not-unpaired)  
- [Bootstrap confidence intervals](docs/visual_odometry/benchmark_methodology.ipynb#4-confidence-intervals-without-the-normality-assumption)  
- [Aggregation traps — failures, weighting, normalisation](docs/visual_odometry/benchmark_methodology.ipynb#5-aggregation-traps--where-the-ranking-silently-changes)  
- [Cherry-picking and leaderboard overfitting](docs/visual_odometry/benchmark_methodology.ipynb#6-cherry-picking-and-leaderboard-overfitting)  
- [Reporting checklist](docs/visual_odometry/benchmark_methodology.ipynb#7-reporting-checklist)  

### Runtime evaluation — latency, throughput, memory
- [Runtime evaluation](docs/visual_odometry/runtime_evaluation.ipynb)  
- [Three numbers that get called "speed"](docs/visual_odometry/runtime_evaluation.ipynb#1-three-numbers-that-get-called-speed)  
- [The mean is the wrong statistic — report the tail](docs/visual_odometry/runtime_evaluation.ipynb#2-the-mean-is-the-wrong-statistic--report-the-tail)  
- [Per-thread budgets](docs/visual_odometry/runtime_evaluation.ipynb#3-one-number-per-system-is-not-enough--report-per-thread-budgets)  
- [Measurement traps — build type, playback rate, throttling](docs/visual_odometry/runtime_evaluation.ipynb#4-measurement-traps)  
- [Memory and energy](docs/visual_odometry/runtime_evaluation.ipynb#5-memory-and-energy--the-constraints-that-actually-kill-deployments)  
- [Tooling — perf, Tracy, ros2 topic delay](docs/visual_odometry/runtime_evaluation.ipynb#6-tooling)  
- [Reporting template](docs/visual_odometry/runtime_evaluation.ipynb#7-reporting-template)  


# [VO and Depth Loss Functions](#)

### Overview
- [Loss functions for depth + pose estimation](docs/vo_loss_functions.ipynb)  
- [Recommended combination](docs/vo_loss_functions.ipynb#5-recommended-combination-for-your-project)  

### Self-supervised losses
- [Photometric reprojection loss](docs/photometric_reprojection_loss.ipynb)  
- [Photometric loss mathematics](docs/photometric_loss_mathematics.md)  
- [SSIM](docs/ssim.ipynb)  
- [LPIPS](docs/ssim.ipynb#lpips)  
- [Edge-aware depth smoothness](docs/edge_aware_depth_smoothness.ipynb)  

### Pose losses
- [Types of loss functions used in VO](docs/visual_odometry/vit_monocular_vo.ipynb#i-types-of-loss-functions-used-in-vo)  
- [Rotation loss overview](docs/visual_odometry/vit_monocular_vo.ipynb#111-rotation-loss-overview)  
- [Quaternion loss](docs/visual_odometry/vit_monocular_vo.ipynb#112-quaternion-loss-naïve-euclidean-loss)  
- [Geodesic loss](docs/visual_odometry/vit_monocular_vo.ipynb#113-geodesic-loss-rotation-angle-loss)  
- [Full transformation loss $SE(3)$](docs/visual_odometry/vit_monocular_vo.ipynb#12-full-transformation-loss-se3)  
- [How to pick the weights](docs/visual_odometry/vit_monocular_vo.ipynb#124-how-to-pick-weights-very-important)  


# [Object Pose and Shape Estimation](#)
- [RF-DETR: SOTA Real-Time Object Detection Model](https://github.com/roboflow/rf-detr)  


# [Visualization](#)
- [Layered Image Vectorization via Semantic Simplification](https://szuviz.github.io/layered_vectorization/)  
- [Potree: WebGL based point cloud renderer ](https://github.com/potree/potree/)  

# [Lidar-Camera Calibration](#)
- [MATLAB Lidar-Camera Calibration](https://www.mathworks.com/help/lidar/ug/lidar-camera-calibration.html)  
- [ROS2 Camera Lidar Fusion](https://github.com/CDonosoK/ros2_camera_lidar_fusion)  
- [Awesome-LiDAR-Camera-Calibration](https://github.com/Deephome/Awesome-LiDAR-Camera-Calibration)  


# [Courses](#)
- [MIT16.485 - Visual Navigation for Autonomous Vehicles](https://vnav.mit.edu/)  


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
- [Linear Algebra for Computer Vision, Robotics, and Machine Learning](https://www.cis.upenn.edu/~cis5150/linalg-I-f.pdf)
