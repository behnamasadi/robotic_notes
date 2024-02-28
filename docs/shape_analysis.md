- [Procrustes Analysis](#procrustes-analysis)
- [Wahba's Problem](#wahba-s-problem)
- [Quaternion Estimator Algorithm (QUEST)](#quaternion-estimator-algorithm--quest-)
- [Kabsch Algorithm](#kabsch-algorithm)
- [Umeyama Algorithm](#umeyama-algorithm)
- [Iterative Closest Point (ICP)](#iterative-closest-point--icp-)
- [KISS-ICP](#kiss-icp)

# Procrustes Analysis
# Wahba's Problem
# Quaternion Estimator Algorithm (QUEST)
# Kabsch Algorithm
# Umeyama Algorithm
# Iterative Closest Point (ICP)
# KISS-ICP



ROS integration:

```
cmake -DWITH_ROS=ON ../superbuild  
cmake --build . --config Release -DWITH_ROS=ON -DWITH_ROS=ON
```

```
git clone https://github.com/PRBonn/kiss-icp
colcon build
source ./install/setup.bash
```

### ROS2

```
ros2 launch kiss_icp odometry.launch.py bagfile:=<path_to_rosbag> topic:=<topic_name>

```
