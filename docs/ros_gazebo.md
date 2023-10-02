# 1. Names
## 1.1 Graph Resource Names
Graph Resource Names provide a hierarchical naming structure that is used for all resources in a ROS Computation Graph, such as:
- Nodes
- Parameters
- Topics
- Services
### 1.1.1 Resolving Name
There are four types of Graph Resource Names in ROS with the following syntax

- base
- relative/name
- /global/name
- ~private/name


## 1.2 Package Resource Names
Package Resource Names are used in ROS with Filesystem-Level concepts to simplify the process of referring to files and data types on disk. Examples:

- Message (msg) types. For example, the name "std_msgs/String" refers to the "String" message type in the "std_msgs" Package.
- Service (srv) types
- Node types

# 2. Remapping Arguments
Any **Graph Resource Names** within a node can be remapped when it is launched at the command-line, This  lets you launch the same node with multiple configurations from the command-line. 
 syntax is `name:=new_name`
 
 
This will remap teh topic chat to chatter 
```
rosrun tutorials talker  chat:=/chatter
```

remapping so that the new node ends up subscribing to `/needed_topic `when it thinks it is subscribing to `/different_topic`
```
<remap from="/different_topic" to="/needed_topic"/>
```


Refs: [1](https://wiki.ros.org/roslaunch/XML/remap)
 
## 3. Special keys 

- __name: name of the node
- __log: designates the location that the node's log file
- __ip: and __hostname: substitutes for `ROS_IP` and `ROS_HOSTNAME`.
- __master: substitute for `ROS_MASTER_URI`.
- __ns: substitute for `ROS_NAMESPACE`. The `ROS_NAMESPACE` lets you push down a Node into a namespace. All of the names in the Node will be resolved relative to this value, including remapped names.


Example: This let you launch robot1 and robot2 start sedning their message (i.e. their position)
```
rosrun tutorials talker  __ns:=robot1  
rosrun tutorials talker  __ns:=robot2

rosrun tutorials listener __ns:=robot1
rosrun tutorials listener __ns:=robot2
```

or you can run a node in a namespace, simply add the `ns` attribute to a <node> tag. For example

```
<node pkg="foo" type="bar" name="my_node" ns="my_namespace" />
```

Refs: [1](https://wiki.ros.org/Names)

# NodeHandles


1.  public: /namespace/topic
```cpp
ros::NodeHandle nh=ros::NodeHandle();
```

This will look for your parameter in the global namespace,it will find /my_param




2. private: /namespace/node/topic
```cpp
ros::NodeHandle nh=ros::NodeHandle("~my_private_namespace");
```


This will look under the nested namespaces of the node itself and will find /namespace/my_node_name/my_param.


3. namespaced: /namespace/node/topic
```cpp
ros::NodeHandle nh=ros::NodeHandle("my_namespace");
```

4. global: /topic
```cpp
ros::NodeHandle nh=ros::NodeHandle("/my_global_namespace");
```


Refs: [1](http://wiki.ros.org/roscpp/Overview/NodeHandles), [2](http://wiki.ros.org/roscpp_tutorials/Tutorials/AccessingPrivateNamesWithNodeHandle), [3](http://wiki.ros.org/roscpp/Overview/NodeHandles)

## Publishers and Subscribers

Refs: [1](http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers#Callback_Signature)

# Roslaunch

```
roslaunch tutorials display.launch model:=urdf/01-myfirst.urdf
```

# URDF




```
rosrun xacro xacro path/to/file.xacro > model.urdf
```

```
check_urdf model.urdf
```


Frames are attached to the links. Every link has an origin located in its center of mass which is called **link origin **. For instance for a cylinder it is in the center of that. 
Visual, inertia and collision can have offset relative to that.
If a link is child of a joint, the **link origin ** is the ** joint origin **. joint origin is its parent link origin.

For instance here we have `right_leg` 

```
    <link name="right_leg">
        <visual>
            <geometry>
                <box size="0.6 0.1 0.2"/>
            </geometry>
            <origin rpy="0 0 0" xyz="1 0 0"/>
        </visual>
    </link>
```

and 


```
    <joint name="base_to_right_leg" type="fixed">
        <parent link="base_link"/>
        <child link="right_leg"/>
        <origin rpy="0 0 0" xyz="0 1 1"/>
    </joint>
```


<img src="images/rviz_screenshot_2022_10_22-12_04_59.png"  with="491" height="341"    />
<br/>

<img src="images/rviz_screenshot_2022_10_22-12_05_45.png"  with="491" height="341" />
<br/>

<img src="images/rviz_screenshot_2022_10_22-12_06_06.png"  with="491" height="341" />
<br/>


<img src="images/inertial.png" with="200" height="108" />
<br/>

<img src="images/joint.png" with="200" height="183"/>
<br/>





```
roslaunch tutorials complete_model.launch
```

Refs: [1](https://wiki.ros.org/roslaunch), [2](https://github.com/ros/urdf_sim_tutorial), [3](https://github.com/ros/urdf_tutorial) , [4](http://wiki.ros.org/urdf/Tutorials/Using%20a%20URDF%20in%20Gazebo) 

# Publishing the State

Refs: [1](http://wiki.ros.org/urdf/Tutorials/Using%20urdf%20with%20robot_state_publisher), [2](http://wiki.ros.org/robot_state_publisher/Tutorials/Using%20the%20robot%20state%20publisher%20on%20your%20own%20robot)

# Differential Drive Wheel Systems

Refs: [1](https://github.com/ros-controls/ros_controllers/tree/noetic-devel/diff_drive_controller), [2](http://wiki.ros.org/diff_drive_controller)


# Gazebo

Refs: [1](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins#IMU(GazeboRosImu))



# ROS best practices

Refs: [1](https://github.com/leggedrobotics/ros_best_practices/tree/master/ros_package_template), [2](https://github.com/leggedrobotics/ros_best_practices/wiki)


# move_base

Refs: [1](http://wiki.ros.org/move_base)



## ROS Odometery Model

Refs: [1](https://answers.ros.org/question/359950/is-ros-using-velocity-motion-model-or-odometry-motion-model/), [2](https://github.com/ros-planning/navigation/blob/melodic-devel/amcl/src/amcl/sensors/amcl_odom.cpp#L113), [3](https://www.mathworks.com/help/nav/ref/odometrymotionmodel.html#d124e109215)


# Estate Estimation


There are several solutions for fusion data
- madgwick filter
- mahony filter
- complementary filter


<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}x\prime&space;\\y\prime\\\phi&space;\prime\end{bmatrix}=\underbrace{\begin{bmatrix}\frac{V}{\omega}\sin(\phi&plus;\omega&space;\delta&space;t)&space;&space;&space;-\frac{V}{\omega}&space;\sin(\phi)&space;&space;\\-\frac{V}{\omega}&space;\cos(\phi&space;&plus;\omega&space;\delta&space;t)&space;&space;&plus;\frac{V}{\omega}&space;\cos(\phi)\\\phi\end{bmatrix}&plus;\begin{bmatrix}x&space;&space;\\y&space;\\\omega&space;\delta&space;t\end{bmatrix}}_{g(u_t,\mu_{t-1})}&space;" title="https://latex.codecogs.com/svg.image?\begin{bmatrix}x\prime \\y\prime\\\phi \prime\end{bmatrix}=\underbrace{\begin{bmatrix}\frac{V}{\omega}\sin(\phi+\omega \delta t) -\frac{V}{\omega} \sin(\phi) \\-\frac{V}{\omega} \cos(\phi +\omega \delta t) +\frac{V}{\omega} \cos(\phi)\\\phi\end{bmatrix}+\begin{bmatrix}x \\y \\\omega \delta t\end{bmatrix}}_{g(u_t,\mu_{t-1})} " />



## State Space
So at the beginning we have only robot pose (robot start in its own frame) and orientation and zero land mark, as the robot explor we will add landmarks and increase the state space, but here we assume that all landmark are known beforehand, so for a map with  <img src="https://latex.codecogs.com/svg.image?n" title="https://latex.codecogs.com/svg.image?n" /> landmarks: <img src="https://latex.codecogs.com/svg.image?(3&plus;2n)" title="https://latex.codecogs.com/svg.image?(3+2n)" />  dimensional Gaussian

<img src="https://latex.codecogs.com/svg.image?x_t=\begin{pmatrix}x&space;\\y&space;\\\theta&space;\\&space;m_{1,x}&space;\\m_{1,y}&space;\\\vdots&space;\\m_{n,x}&space;\\m_{n,y}&space;\\\end{pmatrix}" title="https://latex.codecogs.com/svg.image?x_t=\begin{pmatrix}x \\y \\\theta \\ m_{1,x} \\m_{1,y} \\\vdots \\m_{n,x} \\m_{n,y} \\\end{pmatrix}" />

<br/>
<br/>


Compact Representation:

<br/>
<br/>


<img src="https://latex.codecogs.com/svg.image?\underbrace{\begin{pmatrix}x&space;\\m\end{pmatrix}}_\mu&space;" title="https://latex.codecogs.com/svg.image?\underbrace{\begin{pmatrix}x \\m\end{pmatrix}}_\mu " />


<br/>
<br/>


<img src="https://latex.codecogs.com/svg.image?\underbrace{\begin{pmatrix}\Sigma_{xx}&space;&&space;\Sigma_{xm}&space;\\\Sigma_{mx}&space;&&space;\Sigma_{mm}\end{pmatrix}}_\Sigma&space;" title="https://latex.codecogs.com/svg.image?\underbrace{\begin{pmatrix}\Sigma_{xx} & \Sigma_{xm} \\\Sigma_{mx} & \Sigma_{mm}\end{pmatrix}}_\Sigma " />


<br/>
<br/>



Refs: [1](https://www.youtube.com/watch?v=hN8dL55rP5I), [2](https://www.mathworks.com/help/fusion/ug/pose-estimation-from-asynchronous-sensors.html#d124e16816), [3](https://www.mathworks.com/help/fusion/ug/imu-and-gps-fusion-for-inertial-navigation.html)


## Jacobian Matrix of Motion 


# IMU

## IMU Noise Model

Refs: [1](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model)
## MPU-9250
Refs: [1](https://medium.com/@niru5/hands-on-with-rpi-and-mpu9250-part-3-232378fa6dbc)



# ROS State Estimation
Refs: [1](https://kapernikov.com/the-ros-robot_localization-package/), [2](https://automaticaddison.com/sensor-fusion-using-the-ros-robot-pose-ekf-package/), [3](http://wiki.ros.org/robot_pose_ekf), [4](https://github.com/ros-planning/robot_pose_ekf/blob/master/include/robot_pose_ekf/odom_estimation.h), [5](https://github.com/ros-planning/robot_pose_ekf/blob/master/src/odom_estimation.cpp), [6]()

# EKF Implementations

Refs: [1](https://github.com/Sina-Baharlou/Pose-Estimation-EKF), [2](https://orocos.org/bfl.html)

# Visual Odometry

# Online Resources

[aerial-robotics](https://aerial-robotics-iitk.gitbook.io/wiki)
<br/>
[ros-mobile-robots](https://ros-mobile-robots.com/)

# Gauss-Newton

Taylor series of an arbitrary function:

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?f(x)\approx&space;f(a)&plus;(x-a)f'(a)" title="https://latex.codecogs.com/svg.image?f(x)\approx f(a)+(x-a)f'(a)" />

<br/>
<br/>

this will approximate a function by a line, if we set it to zero, meaning the point that line became zero is an approximation of the where the function became zero, this point would be a start point for our next iteration.

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?x=a-\frac{f(a)}{f'(a)}" title="https://latex.codecogs.com/svg.image?x=a-\frac{f(a)}{f'(a)}" />
<br/>
<br/>



<img src="https://latex.codecogs.com/svg.image?x_{n&plus;1}=x_n-\frac{f(x_n)}{f'(x_n)}" title="https://latex.codecogs.com/svg.image?x_{n+1}=x_n-\frac{f(x_n)}{f'(x_n)}" />

<br/>
<br/>



If we linearize the derivative of the function and set it zero we are looking for its extreme points

<img src="https://latex.codecogs.com/svg.image?f'(x)\approx&space;f(a)'&plus;(x-a)f''(a)" title="https://latex.codecogs.com/svg.image?f'(x)\approx f'(a)+(x-a)f''(a)" />


<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?x=a-\frac{f'(a)}{f''(a)}" title="https://latex.codecogs.com/svg.image?x=a-\frac{f'(a)}{f''(a)}" />
<br/>
<br/>


<img src="https://latex.codecogs.com/svg.image?x_{n&plus;1}=x_n-\frac{f'(x_n)}{f''(x_n)}" title="https://latex.codecogs.com/svg.image?x_{n+1}=x_n-\frac{f'(x_n)}{f''(x_n)}" />

<br/>
<br/>
for multi dimensional data: 
<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?X_{n&plus;1}=X_n-(\nabla^2f(X_n))^{-1}\nabla&space;f(X_n)" title="https://latex.codecogs.com/svg.image?X_{n+1}=X_n-(\nabla^2f(X_n))^{-1}\nabla f(X_n)" />




<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?\nabla&space;f(X_n)=J(X_n)^Tr(X_n)" title="https://latex.codecogs.com/svg.image?\nabla f(X_n)=J(X_n)^Tr(X_n)" />
<br/>
<br/>


<img src="https://latex.codecogs.com/svg.image?\nabla&space;^2f(X_n)=J(X_n)^TJ(X_n)" title="https://latex.codecogs.com/svg.image?\nabla ^2f(X_n)=J(X_n)^TJ(X_n)" />
<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?J^TJ" title="https://latex.codecogs.com/svg.image?J^TJ" />is a reasonable the approximation of Hessian <img src="https://latex.codecogs.com/svg.image?H" title="https://latex.codecogs.com/svg.image?H" />

<br/>
<br/>


<img src="https://latex.codecogs.com/svg.image?X_{n&plus;1}=X_n-&space;(J(X_n)^TJ(X_n))^{-1}J(X_n)^Tr(X_n)" title="https://latex.codecogs.com/svg.image?X_{n+1}=X_n- (J(X_n)^TJ(X_n))^{-1}J(X_n)^Tr(X_n)" />

Refs: [1](https://math.stackexchange.com/questions/2349026/why-is-the-approximation-of-hessian-jtj-reasonable)

# Conditional Distribution of Y Given X

Refs: [1](https://online.stat.psu.edu/stat414/lesson/21/21.1)

# Hand-Eye Calibration
Refs: [1](https://support.zivid.com/en/latest/academy/applications/hand-eye.html), [2](https://wiki.ros.org/ensenso_driver/Tutorials/HandEyeCalibration)






## Eliminating Duplicate Solutions by Limiting the Roll and Pitch Ranges



# Quaternion estimator algorithm (QUEST)
Refs: [1](https://en.wikipedia.org/wiki/Quaternion_estimator_algorithm), [2](https://ahrs.readthedocs.io/en/latest/filters/quest.html)

# Closed-form solution of absolute orientation using unit quaternions
Refs: [1](https://web.stanford.edu/class/cs273/refs/Absolute-OPT.pdf)

# Wahba's problem
Refs [1](https://en.wikipedia.org/wiki/Wahba%27s_problem)


# Angular Velocity Vector Transformation

Refs: [1](https://physics.stackexchange.com/questions/429081/rotational-kinematics-and-angular-velocity-vector-transformation)


# Dynamic Bayesian network



