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





# Kinematics of Differential Drive Robots and Wheel odometry

## 1. Velocity-based (dead reckoning)
Linear Velocity:

<img src="https://latex.codecogs.com/svg.image?V=R\omega" title="https://latex.codecogs.com/svg.image?V=R\omega" />

<br/>
<br/>

so for the right wheel:
<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?V_r=\omega(R&plus;\frac{L}{2})" title="https://latex.codecogs.com/svg.image?V_r=\omega(R+\frac{L}{2})" />
<br/>
<br/>

and for the left wheel:
<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?V_l=\omega(R-\frac{L}{2})" title="https://latex.codecogs.com/svg.image?V_l=\omega(R-\frac{L}{2})" />


if we add and subtract the above equation:

<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?\omega=&space;\frac{V_l&space;&plus;&space;V_r}{2R}" title="https://latex.codecogs.com/svg.image?\omega= \frac{V_l + V_r}{2R}" />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?\omega=&space;\frac{V_l&space;-&space;V_r}{L}" title="https://latex.codecogs.com/svg.image?\omega= \frac{V_l - V_r}{L}" />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?\omega=L&space;\frac{V_l&space;&plus;&space;V_r}{2(V_r-V_l)}" title="https://latex.codecogs.com/svg.image?\omega=L \frac{V_l + V_r}{2(V_r-V_l)}" />

<br/>
<br/>



<img src="https://latex.codecogs.com/svg.image?I_{CC}=\left\{\begin{matrix}I_{CC_X}=x-R&space;\cos(\frac{\pi}{2}-\phi)&space;=x-R\sin(\phi)\\I_{CC_Y}=y-R&space;\sin(\frac{\pi}{2}-\phi)&space;=y&plus;R\cos(\phi)\end{matrix}\right." title="https://latex.codecogs.com/svg.image?I_{CC}=\left\{\begin{matrix}I_{CC_X}=x-R \cos(\frac{\pi}{2}-\phi) =x-R\sin(\phi)\\I_{CC_Y}=y-R \sin(\frac{\pi}{2}-\phi) =y+R\cos(\phi)\end{matrix}\right." />


<br/>
<br/>

<img src="images/Differential_Drive_Kinematics_of_a_Wheeled_Mobile_Robot.svg" />

<br/>
<br/>

### Forward Kinematics for Differential Drive Robots

so we express the pose of the robot in <img src="https://latex.codecogs.com/svg.image?ICC" title="https://latex.codecogs.com/svg.image?ICC" /> frame, we rotate it around that <img src="https://latex.codecogs.com/svg.image?\omega&space;\delta&space;t" title="https://latex.codecogs.com/svg.image?\omega \delta t" /> degree and finally express it our world coordinate frame:


<img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}x\prime&space;\\y\prime\end{bmatrix}=\begin{bmatrix}\cos(\omega&space;\delta&space;t)&space;&&space;-\sin(\omega&space;\delta&space;t)&space;&space;\\\sin(\omega&space;\delta&space;t)&space;&space;&&space;&space;\cos(\omega&space;\delta&space;t)&space;\\\end{bmatrix}\begin{bmatrix}x-I_{CC_X}\\y-I_{CC_Y}\end{bmatrix}&plus;\begin{bmatrix}I_{CC_X}&space;\\I_{CC_y}\end{bmatrix}" title="https://latex.codecogs.com/svg.image?\begin{bmatrix}x\prime \\y\prime\end{bmatrix}=\begin{bmatrix}\cos(\omega \delta t) & -\sin(\omega \delta t) \\\sin(\omega \delta t) & \cos(\omega \delta t) \\\end{bmatrix}\begin{bmatrix}x-I_{CC_X}\\y-I_{CC_Y}\end{bmatrix}+\begin{bmatrix}I_{CC_X} \\I_{CC_y}\end{bmatrix}" />

<br/>
<br/>

If we add orientation and write all positions in our world coordinate:
<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}x\prime&space;\\y\prime\\\phi&space;\prime\end{bmatrix}=\begin{bmatrix}\cos(\omega&space;\delta&space;t)&space;&&space;-\sin(\omega&space;\delta&space;t)&space;&&space;0\\\sin(\omega&space;\delta&space;t)&space;&&space;\cos(\omega&space;\delta&space;t\theta&space;&&space;0&space;\\0&space;&&space;1&space;&&space;1\end{bmatrix}\begin{bmatrix}x-x&plus;R&space;\sin&space;\phi&space;\\y-y-R&space;\cos&space;\phi&space;\\\phi\end{bmatrix}&plus;\begin{bmatrix}x-R&space;\sin&space;\phi&space;\\y&plus;R&space;\cos&space;\phi&space;\\\omega&space;\delta&space;t\end{bmatrix}" title="https://latex.codecogs.com/svg.image?\begin{bmatrix}x\prime \\y\prime\\\phi \prime\end{bmatrix}=\begin{bmatrix}\cos(\omega \delta t) & -\sin(\omega \delta t) & 0\\\sin(\omega \delta t) & \cos(\omega \delta t\theta & 0 \\0 & 1 & 1\end{bmatrix}\begin{bmatrix}x-x+R \sin \phi \\y-y-R \cos \phi \\\phi\end{bmatrix}+\begin{bmatrix}x-R \sin \phi \\y+R \cos \phi \\\omega \delta t\end{bmatrix}" />
<br/>
<br/>









<img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}x\prime&space;\\y\prime\\\phi&space;\prime\end{bmatrix}=\begin{bmatrix}R&space;(\cos&space;(\omega&space;\delta&space;t)&space;&space;\sin&space;(\phi&space;)&plus;&space;\sin&space;(\omega&space;\delta&space;t&space;)\cos&space;(\phi)&space;&space;)&space;\\-R&space;(\sin(\omega&space;\delta&space;t)&space;\sin(\phi)&space;&plus;&space;cos(\omega&space;\delta&space;t)&space;cos(\phi))&space;&space;\\0&space;\end{bmatrix}&plus;\begin{bmatrix}x-R&space;\sin(\phi)&space;&space;\\y&plus;R&space;\cos(\phi)&space;\\\omega&space;\delta&space;t\end{bmatrix}" title="https://latex.codecogs.com/svg.image?\begin{bmatrix}x\prime \\y\prime\\\phi \prime\end{bmatrix}=\begin{bmatrix}R (\cos (\omega \delta t) \sin (\phi )+ \sin (\omega \delta t )\cos (\phi) ) \\-R (\sin(\omega \delta t) \sin(\phi) + cos(\omega \delta t) cos(\phi)) \\0 \end{bmatrix}+\begin{bmatrix}x-R \sin(\phi) \\y+R \cos(\phi) \\\omega \delta t\end{bmatrix}" />


<br/>
<br/>


<img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}x\prime&space;\\y\prime\\\phi&space;\prime\end{bmatrix}=\begin{bmatrix}R\sin(\phi&plus;\omega&space;\delta&space;t)&space;&space;&space;-R&space;\sin(\phi)&space;&space;\\-R&space;\cos(\phi&space;&plus;\omega&space;\delta&space;t)&space;&space;&plus;R&space;\cos(\phi)\\\phi\end{bmatrix}&plus;\begin{bmatrix}x&space;&space;\\y&space;\\\omega&space;\delta&space;t\end{bmatrix}" title="https://latex.codecogs.com/svg.image?\begin{bmatrix}x\prime \\y\prime\\\phi \prime\end{bmatrix}=\begin{bmatrix}R\sin(\phi+\omega \delta t) -R \sin(\phi) \\-R \cos(\phi +\omega \delta t) +R \cos(\phi)\\\phi\end{bmatrix}+\begin{bmatrix}x \\y \\\omega \delta t\end{bmatrix}" />



<br/>
<br/>
since we had :
<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?R=\frac{V}{\omega}" title="https://latex.codecogs.com/svg.image?R=\frac{V}{\omega}" />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}x\prime&space;\\y\prime\\\phi&space;\prime\end{bmatrix}=\begin{bmatrix}\frac{V}{\omega}\sin(\phi&plus;\omega&space;\delta&space;t)&space;&space;&space;-\frac{V}{\omega}&space;\sin(\phi)&space;&space;\\-\frac{V}{\omega}&space;\cos(\phi&space;&plus;\omega&space;\delta&space;t)&space;&space;&plus;\frac{V}{\omega}&space;\cos(\phi)\\\phi\end{bmatrix}&plus;\begin{bmatrix}x&space;&space;\\y&space;\\\omega&space;\delta&space;t\end{bmatrix}" title="https://latex.codecogs.com/svg.image?\begin{bmatrix}x\prime \\y\prime\\\phi \prime\end{bmatrix}=\begin{bmatrix}\frac{V}{\omega}\sin(\phi+\omega \delta t) -\frac{V}{\omega} \sin(\phi) \\-\frac{V}{\omega} \cos(\phi +\omega \delta t) +\frac{V}{\omega} \cos(\phi)\\\phi\end{bmatrix}+\begin{bmatrix}x \\y \\\omega \delta t\end{bmatrix}" />

<br/>
<br/>

###  Inverse Kinematics of Differential Drive Robots



Refs: [1](https://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf)



## 2. Odometry-based


<img src="images/odometry_model.jpg" width="461" height="192">

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?\delta_{translation}=\sqrt{(\bar{x^\prime}&space;-\bar{x})^2&space;&plus;&space;(\bar{y^\prime}&space;-\bar{y})^2&space;}" title="https://latex.codecogs.com/svg.image?\delta_{translation}=\sqrt{(\bar{x^\prime} -\bar{x})^2 + (\bar{y^\prime} -\bar{y})^2 }" />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?\delta_{rot1}=atan2(&space;&space;\bar{y^\prime}&space;-\bar{y}&space;,&space;&space;\bar{x^\prime}&space;-\bar{x})&space;&space;)&space;&space;&space;-\bar\theta&space;" title="https://latex.codecogs.com/svg.image?\delta_{rot1}=atan2( \bar{y^\prime} -\bar{y} , \bar{x^\prime} -\bar{x}) ) -\bar\theta " />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?\delta_{rot2}=\bar\theta&space;^\prime&space;&space;&space;-\bar\theta&space;-\delta_{rot1}" title="https://latex.codecogs.com/svg.image?\delta_{rot2}=\bar\theta ^\prime -\bar\theta -\delta_{rot1}" />


<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?\left\{\begin{matrix}x_t=x_{t-1}&plus;\delta_{translation}&space;&space;&space;\cos(&space;\theta_{t-1}&space;&plus;\delta{rot1}&space;)&space;\\y_t=y_{t-1}&plus;\delta_{translation}&space;&space;&space;\sin(&space;\theta_{t-1}&space;&plus;\delta{rot1}&space;)&space;\\\theta_t=\theta_{t-1}&plus;\delta_{rot1}&plus;\delta_{rot2}\end{matrix}\right.&space;" title="https://latex.codecogs.com/svg.image?\left\{\begin{matrix}x_t=x_{t-1}+\delta_{translation} \cos( \theta_{t-1} +\delta{rot1} ) \\y_t=y_{t-1}+\delta_{translation} \sin( \theta_{t-1} +\delta{rot1} ) \\\theta_t=\theta_{t-1}+\delta_{rot1}+\delta_{rot2}\end{matrix}\right. " />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?g(u_t,\mu_{t-1})=\begin{pmatrix}x_t&space;\\y_t&space;\\\theta_t\end{pmatrix}&space;=\begin{pmatrix}x_{t-1}\\y_{t-1}&space;\\\theta_{t-1}\end{pmatrix}&space;&plus;\begin{pmatrix}&space;\delta_{translation}&space;\cos(&space;\theta_{t-1}&space;&plus;\delta{rot1}&space;&space;&space;\\&space;\delta_{translation}&space;\sin(&space;\theta_{t-1}&space;&plus;\delta{rot1}&space;&space;\\&space;\delta_{rot1}&plus;\delta_{rot2}\end{pmatrix}&space;" title="https://latex.codecogs.com/svg.image?g(u_t,\mu_{t-1})=\begin{pmatrix}x_t \\y_t \\\theta_t\end{pmatrix} =\begin{pmatrix}x_{t-1}\\y_{t-1} \\\theta_{t-1}\end{pmatrix} +\begin{pmatrix} \delta_{translation} \cos( \theta_{t-1} +\delta{rot1} \\ \delta_{translation} \sin( \theta_{t-1} +\delta{rot1} \\ \delta_{rot1}+\delta_{rot2}\end{pmatrix} " />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?G_t=&space;\frac{\partial}{\partial(x,y,\theta)^T}&space;\begin{bmatrix}&space;\begin{pmatrix}x\\y&space;\\\theta\end{pmatrix}&space;&plus;\begin{pmatrix}&space;\delta_{translation}&space;\cos(&space;\theta&space;&plus;\delta{rot1}&space;)\\&space;\delta_{translation}&space;\sin(&space;\theta&space;&plus;\delta{rot1}&space;)\\&space;\delta_{rot1}&plus;\delta_{rot2}\end{pmatrix}&space;\end{bmatrix}" title="https://latex.codecogs.com/svg.image?G_t= \frac{\partial}{\partial(x,y,\theta)^T} \begin{bmatrix} \begin{pmatrix}x\\y \\\theta\end{pmatrix} +\begin{pmatrix} \delta_{translation} \cos( \theta +\delta{rot1} )\\ \delta_{translation} \sin( \theta +\delta{rot1} )\\ \delta_{rot1}+\delta_{rot2}\end{pmatrix} \end{bmatrix}" />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?G_t=I&plus;&space;\frac{\partial}{\partial(x,y,\theta)^T}&space;\begin{bmatrix}&space;\begin{pmatrix}&space;\delta_{translation}&space;\cos(&space;\theta&space;&plus;\delta{rot1}&space;)\\&space;\delta_{translation}&space;\sin(&space;\theta&space;&plus;\delta{rot1}&space;)\\&space;\delta_{rot1}&plus;\delta_{rot2}\end{pmatrix}&space;\end{pmatrix}" title="https://latex.codecogs.com/svg.image?G_t=I+ \frac{\partial}{\partial(x,y,\theta)^T} \begin{bmatrix} \begin{pmatrix} \delta_{translation} \cos( \theta +\delta{rot1} )\\ \delta_{translation} \sin( \theta +\delta{rot1} )\\ \delta_{rot1}+\delta_{rot2}\end{pmatrix} \end{pmatrix}" />



<br/>
<br/>
Not sure about the followings:
<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?G_t=I&plus;&space;\begin{pmatrix}0&space;&&space;0&space;&&space;-\delta_{translation}&space;\sin(\theta&plus;\delta&space;rot1)&space;\\0&space;&&space;0&space;&&space;\delta_{translation}&space;\cos(\theta&plus;\delta&space;rot1)&space;\\0&space;&&space;0&space;&&space;0&space;\\\end{pmatrix}" title="https://latex.codecogs.com/svg.image?G_t=I+ \begin{pmatrix}0 & 0 & -\delta_{translation} \sin(\theta+\delta rot1) \\0 & 0 & \delta_{translation} \cos(\theta+\delta rot1) \\0 & 0 & 0 \\\end{pmatrix}" />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?G_t=\begin{pmatrix}1&space;&&space;0&space;&&space;-\delta_{translation}&space;\sin(\theta&plus;\delta&space;rot1)&space;\\0&space;&&space;1&space;&&space;\delta_{translation}&space;\cos(\theta&plus;\delta&space;rot1)&space;\\0&space;&&space;0&space;&&space;1&space;\\\end{pmatrix}" title="https://latex.codecogs.com/svg.image?G_t=\begin{pmatrix}1 & 0 & -\delta_{translation} \sin(\theta+\delta rot1) \\0 & 1 & \delta_{translation} \cos(\theta+\delta rot1) \\0 & 0 & 1 \\\end{pmatrix}" />
<br/>
<br/>



<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?h(\bar{\mu}_t,j)=z_t^i=\begin{pmatrix}r_i^t&space;\\\phi_i^t\end{pmatrix}=\begin{pmatrix}\sqrt{(&space;\bar{\mu}_{j,x}&space;&space;-\bar{\mu}_{t,x}&space;)^2&space;&plus;&space;(&space;\bar{\mu}_{j,y}&space;&space;-\bar{\mu}_{t,y}&space;&space;&space;)^2&space;}&space;&space;&space;\\atan2(&space;\bar{\mu}_{j,y}&space;-\bar{\mu}_{t,y}&space;&space;&space;&space;,&space;&space;&space;\bar{\mu}_{j,x}-&space;\bar{\mu}_{t,x}&space;&space;)-&space;\bar{\mu}_{t,\theta}&space;)\end{pmatrix}" title="https://latex.codecogs.com/svg.image?h(\bar{\mu}_t,j)=z_t^i=\begin{pmatrix}r_i^t \\\phi_i^t\end{pmatrix}=\begin{pmatrix}\sqrt{( \bar{\mu}_{j,x} -\bar{\mu}_{t,x} )^2 + ( \bar{\mu}_{j,y} -\bar{\mu}_{t,y} )^2 } \\atan2( \bar{\mu}_{j,y} -\bar{\mu}_{t,y} , \bar{\mu}_{j,x}- \bar{\mu}_{t,x} )- \bar{\mu}_{t,\theta} )\end{pmatrix}" />


<br/>
<br/>

The pose of the <img src="https://latex.codecogs.com/svg.image?j^{th}" title="https://latex.codecogs.com/svg.image?j^{th}" /> landmark

<img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}\bar{\mu}_{j,x}&space;\\\bar{\mu}_{j,y}\end{bmatrix}" title="https://latex.codecogs.com/svg.image?\begin{bmatrix}\bar{\mu}_{j,x} \\\bar{\mu}_{j,y}\end{bmatrix}" />

<br/>
<br/>

the pose of the robot at time t:
<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}&space;\bar{\mu}_{t,x}&space;\\&space;\bar{\mu}_{t,y}&space;\\&space;\bar{\mu}_{t,\theta}\end{bmatrix}&space;" title="https://latex.codecogs.com/svg.image?\begin{bmatrix} \bar{\mu}_{t,x} \\ \bar{\mu}_{t,y} \\ \bar{\mu}_{t,\theta}\end{bmatrix} " />
<br/>
<br/>


observed range and bearing of the landmark:


<img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}r_t^i&space;\\\phi_t^i\end{bmatrix}&space;" title="https://latex.codecogs.com/svg.image?\begin{bmatrix}r_t^i \\\phi_t^i\end{bmatrix} " />

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


# Accelerometer


Accelerometer sensors measure the difference between any linear acceleration in the accelerometer’s reference frame and can be used to determine the accelerometer pitch and roll orientation angles.
the accelerometer will give us <img src="https://latex.codecogs.com/svg.latex?G_p" alt="https://latex.codecogs.com/svg.latex?G_p" />
and we are looking for <img src="https://latex.codecogs.com/svg.latex?R" alt="https://latex.codecogs.com/svg.latex?R" /> 
that relate gravity to out measurement to extract roll and pitch.


**Since the gravity express as <img src="https://latex.codecogs.com/svg.latex?%5Cbegin%7Bbmatrix%7D%200%5C%5C%200%5C%5C%201%20%5Cend%7Bbmatrix%7D" alt="https://latex.codecogs.com/svg.latex?\begin{bmatrix} 0\\  0\\  1 \end{bmatrix}" /> it means that the `Z` axis of  world coordinate frame is up, East, North, Up (ENU), used in geography (z is up and x is in the direction of move, y is pointing left)**


```
          ▲ z
          |    ▲
          |   / x
          |  /
y         | /
◀---------|/
```


<img src="https://latex.codecogs.com/svg.latex?G_p%3D%5Cbegin%7Bbmatrix%7D%20G_%7Bpx%7D%20%5C%5C%20G_%7Bpy%7D%20%5C%5C%20G_%7Bpz%7D%20%5Cend%7Bbmatrix%7D%3DRg%3DR%5Cbegin%7Bbmatrix%7D%200%20%5C%5C%200%20%5C%5C%201%20%5Cend%7Bbmatrix%7D" alt="https://latex.codecogs.com/svg.latex?G_p=\begin{bmatrix} G_{px} \\ G_{py} \\  G_{pz} \end{bmatrix}=Rg=R\begin{bmatrix} 0 \\ 0 \\  1 \end{bmatrix}" />





<img src="https://latex.codecogs.com/svg.latex?%5C%5C%20R_x%28%5Cphi%29%3D%5Cbegin%7Bpmatrix%7D%201%20%26%200%20%260%20%5C%5C%200%20%26cos%28%5Cphi%29%20%26%20sin%28%5Cphi%29%20%5C%5C%200%20%26%20-sin%28%5Cphi%29%20%26%20cos%28%5Cphi%29%5C%5C%20%5Cend%7Bpmatrix%7D%20%5C%5C%20R_y%28%5Ctheta%29%3D%5Cbegin%7Bpmatrix%7D%20cos%28%5Ctheta%29%20%260%20%26-sin%28%5Ctheta%29%5C%5C%200%26%201%26%200%5C%5C%20sin%28%5Ctheta%29%20%260%20%26cos%28%5Ctheta%29%20%5C%5C%20%5Cend%7Bpmatrix%7D%20%5C%5C%20R_z%28%5Cpsi%29%3D%5Cbegin%7Bpmatrix%7D%20cos%28%5Cpsi%29%20%26%20sin%28%5Cpsi%29%20%260%20%5C%5C%20-sin%28%5Cpsi%29%20%26%20cos%28%5Cpsi%29%20%260%20%5C%5C%200%20%26%200%20%26%201%5C%5C%20%5Cend%7Bpmatrix%7D" alt="\\R_x(\phi)=\begin{pmatrix} 
1 & 0 &0 \\  0 &cos(\phi)  & sin(\phi) \\ 0 & -sin(\phi) & cos(\phi)\\ \end{pmatrix}\\ R_y(\theta)=\begin{pmatrix}  cos(\theta) &0 &-sin(\theta)\\  0& 1& 0\\ sin(\theta) &0 &cos(\theta) \\ \end{pmatrix} \\ R_z(\psi)=\begin{pmatrix} cos(\psi) & sin(\psi) &0 \\  -sin(\psi) & cos(\psi) &0 \\ 0 & 0 & 1\\ \end{pmatrix}" />




We have six different rotation matrix depending on the order of rotation around axis:

<img src="https://latex.codecogs.com/svg.latex?R_%7Bxyz%7D%2CR_%7Bxzy%7D%2CR_%7Byxz%7D%2CR_%7Byzx%7D%2CR_%7Bzyx%7D%2CR_%7Bzxy%7D%2C" alt="https://latex.codecogs.com/svg.latex?R_{xyz},R_{xzy},R_{yxz},R_{yzx},R_{zyx},R_{zxy}" /> and in 4 of them we will have to determine 
<img src="https://latex.codecogs.com/svg.image?\phi,&space;\theta,&space;\psi" title="https://latex.codecogs.com/svg.image?\phi, \theta, \psi" />, 

<br/>

For instance in 
<br/>
<br/>

<img src="https://latex.codecogs.com/svg.latex?R_zR_yR_x%5Cbegin%7Bbmatrix%7D%200%5C%5C%200%5C%5C%201%20%5Cend%7Bbmatrix%7D%20%3D%5Cdisplaystyle%20%5Cleft%5B%5Cbegin%7Bmatrix%7D%5Ccos%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%20%26%20%5Csin%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Csin%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20&plus;%20%5Csin%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%26%20%5Csin%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Csin%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20-%20%5Csin%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%5C%5C-%20%5Csin%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%20%26%20-%20%5Csin%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Csin%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20%5Csin%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%20&plus;%20%5Ccos%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20%26%20%5Csin%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20&plus;%20%5Csin%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20%5Csin%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%5C%5C%5Csin%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%20%26%20-%20%5Csin%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%20%26%20%5Ccos%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%5Cend%7Bmatrix%7D%5Cright%5D%5Cbegin%7Bbmatrix%7D%200%5C%5C%200%5C%5C%201%20%5Cend%7Bbmatrix%7D" alt="https://latex.codecogs.com/svg.latex?R_zR_yR_x\begin{bmatrix} 0\\ 0\\  1 \end{bmatrix} =\displaystyle \left[\begin{matrix}\cos{\left(\psi \right)} \cos{\left(\theta \right)} & \sin{\left(\phi \right)} \sin{\left(\theta \right)} \cos{\left(\psi \right)} + \sin{\left(\psi \right)} \cos{\left(\phi \right)} & \sin{\left(\phi \right)} \sin{\left(\psi \right)} - \sin{\left(\theta \right)} \cos{\left(\phi \right)} \cos{\left(\psi \right)}\\- \sin{\left(\psi \right)} \cos{\left(\theta \right)} & - \sin{\left(\phi \right)} \sin{\left(\psi \right)} \sin{\left(\theta \right)} + \cos{\left(\phi \right)} \cos{\left(\psi \right)} & \sin{\left(\phi \right)} \cos{\left(\psi \right)} + \sin{\left(\psi \right)} \sin{\left(\theta \right)} \cos{\left(\phi \right)}\\\sin{\left(\theta \right)} & - \sin{\left(\phi \right)} \cos{\left(\theta \right)} & \cos{\left(\phi \right)} \cos{\left(\theta \right)}\end{matrix}\right]\begin{bmatrix} 0\\ 0\\  1 \end{bmatrix} "/>


<br/>
<br/>



<img src="https://latex.codecogs.com/svg.latex?%3D%5Cdisplaystyle%20%5Cleft%5B%5Cbegin%7Bmatrix%7D%5Csin%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Csin%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20-%20%5Csin%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%5C%5C%5Csin%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20&plus;%20%5Csin%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20%5Csin%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%5C%5C%5Ccos%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%5Cend%7Bmatrix%7D%5Cright%5D" alt="https://latex.codecogs.com/svg.latex?=\displaystyle \left[\begin{matrix}\sin{\left(\phi \right)} \sin{\left(\psi \right)} - \sin{\left(\theta \right)} \cos{\left(\phi \right)} \cos{\left(\psi \right)}\\\sin{\left(\phi \right)} \cos{\left(\psi \right)} + \sin{\left(\psi \right)} \sin{\left(\theta \right)} \cos{\left(\phi \right)}\\\cos{\left(\phi \right)} \cos{\left(\theta \right)}\end{matrix}\right]" />

<br/>
<br/>
However in the following rotation matrix, we have to determine only <img src="https://latex.codecogs.com/svg.image?\phi,&space;\theta" title="https://latex.codecogs.com/svg.image?\phi, \theta" />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.latex?R_%7Bxyz%7D%5Cbegin%7Bbmatrix%7D0%5C%5C%200%5C%5C%201%5Cend%7Bbmatrix%7D%3D%5Cdisplaystyle%20%5Cleft%5B%5Cbegin%7Bmatrix%7D%5Ccos%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%20%26%20%5Csin%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%20%26%20-%20%5Csin%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%5C%5C%5Csin%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Csin%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20-%20%5Csin%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%26%20%5Csin%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Csin%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20%5Csin%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%20&plus;%20%5Ccos%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20%26%20%5Csin%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%5C%5C%5Csin%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Csin%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20&plus;%20%5Csin%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20%26%20-%20%5Csin%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20&plus;%20%5Csin%7B%5Cleft%28%5Cpsi%20%5Cright%29%7D%20%5Csin%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%26%20%5Ccos%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%5Cend%7Bmatrix%7D%5Cright%5D%5Cbegin%7Bbmatrix%7D%200%5C%5C%200%5C%5C%201%5Cend%7Bbmatrix%7D" alt="https://latex.codecogs.com/svg.latex?R_{xyz}\begin{bmatrix}0\\ 0\\ 1\end{bmatrix}=\displaystyle \left[\begin{matrix}\cos{\left(\psi \right)} \cos{\left(\theta \right)} & \sin{\left(\psi \right)} \cos{\left(\theta \right)} & - \sin{\left(\theta \right)}\\\sin{\left(\phi \right)} \sin{\left(\theta \right)} \cos{\left(\psi \right)} - \sin{\left(\psi \right)} \cos{\left(\phi \right)} & \sin{\left(\phi \right)} \sin{\left(\psi \right)} \sin{\left(\theta \right)} + \cos{\left(\phi \right)} \cos{\left(\psi \right)} & \sin{\left(\phi \right)} \cos{\left(\theta \right)}\\\sin{\left(\phi \right)} \sin{\left(\psi \right)} + \sin{\left(\theta \right)} \cos{\left(\phi \right)} \cos{\left(\psi \right)} & - \sin{\left(\phi \right)} \cos{\left(\psi \right)} + \sin{\left(\psi \right)} \sin{\left(\theta \right)} \cos{\left(\phi \right)} & \cos{\left(\phi \right)} \cos{\left(\theta \right)}\end{matrix}\right]\begin{bmatrix} 0\\  0\\  1\end{bmatrix}" />



<img src="https://latex.codecogs.com/svg.latex?=%5Cdisplaystyle%20%5Cleft%5B%5Cbegin%7Bmatrix%7D-%20%5Csin%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%5C%5C%5Csin%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%5C%5C%5Ccos%7B%5Cleft%28%5Cphi%20%5Cright%29%7D%20%5Ccos%7B%5Cleft%28%5Ctheta%20%5Cright%29%7D%5Cend%7Bmatrix%7D%5Cright%5D" alt="https://latex.codecogs.com/svg.latex?=\displaystyle \left[\begin{matrix}- \sin{\left(\theta \right)}\\\sin{\left(\phi \right)} \cos{\left(\theta \right)}\\\cos{\left(\phi \right)} \cos{\left(\theta \right)}\end{matrix}\right]" />



<br/>
<br/>

Therefore if we write R as <img src="https://latex.codecogs.com/svg.latex?R_%7Bxyz%7D%20%5Ctext%7B%20or%20%7D%20R_%7Byxz%7D" alt="https://latex.codecogs.com/svg.latex?R_{xyz} \text{ or }  R_{yxz}"/>:


<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}&space;G_x\\&space;G_y&space;\\&space;G_z\end{bmatrix}=R_{xyz}\begin{bmatrix}&space;0\\&space;0\\1\end{bmatrix}=R_x(\phi)R_y(\theta)R_z(\psi)\begin{bmatrix}&space;0\\&space;0\\1\end{bmatrix}=\begin{bmatrix}&space;-sin\theta\\&space;cos\theta&space;sin\phi\\cos\theta&space;cos\phi\end{bmatrix}" title="https://latex.codecogs.com/svg.image?\begin{bmatrix} G_x\\ G_y \\ G_z\end{bmatrix}=R_{xyz}\begin{bmatrix} 0\\ 0\\1\end{bmatrix}=R_x(\phi)R_y(\theta)R_z(\psi)\begin{bmatrix} 0\\ 0\\1\end{bmatrix}=\begin{bmatrix} -sin\theta\\ cos\theta sin\phi\\cos\theta cos\phi\end{bmatrix}" />

or 

<img src="https://latex.codecogs.com/svg.image?\begin{bmatrix}&space;G_x\\&space;G_y&space;\\&space;G_z\end{bmatrix}=R_{yxz}\begin{bmatrix}&space;0\\&space;0\\1\end{bmatrix}=R_y(\theta)R_x(\phi)R_z(\psi)\begin{bmatrix}&space;0\\&space;0\\1\end{bmatrix}=\begin{bmatrix}&space;-sin\theta&space;cos\phi&space;\\sin\phi&space;\\cos\theta&space;cos\phi\end{bmatrix}" title="https://latex.codecogs.com/svg.image?\begin{bmatrix} G_x\\ G_y \\ G_z\end{bmatrix}=R_{yxz}\begin{bmatrix} 0\\ 0\\1\end{bmatrix}=R_y(\theta)R_x(\phi)R_z(\psi)\begin{bmatrix} 0\\ 0\\1\end{bmatrix}=\begin{bmatrix} -sin\theta cos\phi \\sin\phi \\cos\theta cos\phi\end{bmatrix}" />


The lack of any dependence on the yaw rotation angle <img src="https://latex.codecogs.com/svg.image?&space;\psi" title="https://latex.codecogs.com/svg.image? \psi" /> is easy to understand physically
since the first rotation is in yaw <img src="https://latex.codecogs.com/svg.image?&space;\psi" title="https://latex.codecogs.com/svg.image? \psi" /> around the smartphone z-axis which is initially aligned with the
gravitational field and pointing downwards. All accelerometers are completely insensitive to rotations
about the gravitational field vector and cannot be used to determine such a rotation.

It is conventional therefore to select either the rotation sequence <img src="https://latex.codecogs.com/svg.image?R_{xyz}" title="https://latex.codecogs.com/svg.image?R_{xyz}" /> or <img src="https://latex.codecogs.com/svg.image?R_{yxz}" title="https://latex.codecogs.com/svg.image?R_{yxz}" />

<br/>
<br/>

## Solving R xyz for the Pitch and Roll Angles

<img src="https://latex.codecogs.com/svg.image?R_{xyz}" title="https://latex.codecogs.com/svg.image?R_{xyz}" /> will give us:

<img src="https://latex.codecogs.com/svg.image?\frac{G}{||G&space;||}&space;=\begin{bmatrix}&space;-sin\theta\\&space;cos\theta&space;sin\phi\\cos\theta&space;cos\phi\end{bmatrix}" title="https://latex.codecogs.com/svg.image?\frac{G}{||G ||} =\begin{bmatrix} -sin\theta\\ cos\theta sin\phi\\cos\theta cos\phi\end{bmatrix}" />


<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?tan\phi=\frac{G_y}{G_z}" title="https://latex.codecogs.com/svg.image?tan\phi=\frac{G_y}{G_z}" />

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?tan\theta=\frac{-G_x}{&space;\sqrt{&space;G_z^2&space;&plus;&space;G_y^2}}" title="https://latex.codecogs.com/svg.image?tan\theta=\frac{-G_x}{ \sqrt{ G_z^2 + G_y^2}}" />
<br/>
<br/>


## Solving R yxz for the Pitch and Roll Angles

<br/>
<br/>

<img src="https://latex.codecogs.com/svg.image?R_{yxz}" title="https://latex.codecogs.com/svg.image?R_{yxz}" /> will give us:

<img src="https://latex.codecogs.com/svg.image?tan\theta=\frac{-G_x}{G_z}" title="https://latex.codecogs.com/svg.image?tan\theta=\frac{-G_x}{G_z}" />
<br/>
<br/>
<img src="https://latex.codecogs.com/svg.image?tan\theta=\frac{G_y}{&space;\sqrt{&space;G_z^2&space;&plus;&space;G_x^2}}" title="https://latex.codecogs.com/svg.image?tan\theta=\frac{G_y}{ \sqrt{ G_z^2 + G_x^2}}" />

<br/>
<br/>

**The order of rotations is important and must always be specified when referring to specific orientation angles.**

Refs: [1](https://www.nxp.com/files-static/sensors/doc/app_note/AN3461.pdf)

# Accelerometer Model


<img src="https://latex.codecogs.com/svg.latex?%5Cbegin%7Bbmatrix%7D%20a_x%20%5C%5C%20a_y%5C%5C%20a_z%20%5Cend%7Bbmatrix%7D%3D%20%5Cunderbrace%7B%5Cfrac%7Bdv%7D%7Bdt%7D%7D_%7B%5Ctext%7Blinear%7D%7D%20&plus;%20%5Cunderbrace%7B%5Comega_b%20%5Ctimes%20%5Cnu%7D_%7B%5Ctext%7Btranslation%2C%20rotation%7D%7D%20-%5Cunderbrace%7BR%5Cbegin%7Bbmatrix%7D0%5C%5C%200%5C%5C%20g%20%5Cend%7Bbmatrix%7D%7D_%7B%5Ctext%7Bgravity%7D%7D%20&plus;%20%5Cunderbrace%7B%5Cbeta%28t%29%7D_%7B%5Ctext%7Bbias%7D%7D%20&plus;%20%5Cunderbrace%7B%20%5Cmu%28t%29%7D_%7B%5Ctext%7Bnoise%7D%7D" alt="https://latex.codecogs.com/svg.latex?\begin{bmatrix} a_x \\  a_y\\  a_z \end{bmatrix}= \underbrace{\frac{dv}{dt}}_{\text{linear}} +  \underbrace{\omega_b \times \nu}_{\text{translation, rotation}}  -\underbrace{R\begin{bmatrix}0\\ 0\\  g \end{bmatrix}}_{\text{gravity}}  + \underbrace{\beta(t)}_{\text{bias}} + \underbrace{ \mu(t)}_{\text{noise}}" />

# Gyroscope Model

<img src="https://latex.codecogs.com/svg.latex?%5Comega%3D%5Comega_%7Btrue%7D%20&plus;%20%5Cunderbrace%7B%5Cbeta%28t%29%7D_%7B%5Ctext%7Bbias%7D%7D%20&plus;%20%5Cunderbrace%7B%20%5Cmu%28t%29%7D_%7B%5Ctext%7Bnoise%7D%7D" alt="https://latex.codecogs.com/svg.latex?\omega=\omega_{true}  + \underbrace{\beta(t)}_{\text{bias}} + \underbrace{ \mu(t)}_{\text{noise}} " />


# Relationship Between Euler-Angle Rates and Body-Axis Rates

<img src="https://latex.codecogs.com/svg.latex?%5Cbegin%7Bbmatrix%7D%20%5Cdot%7B%5Cphi%7D%20%5C%5C%20%5Cdot%7B%5Ctheta%7D%5C%5C%20%5Cdot%7B%5Cpsi%7D%20%5Cend%7Bbmatrix%7D%3D%5Cbegin%7Bbmatrix%7D%201%20%26%20sin%28%5Cphi%29tan%28%5Ctheta%29%20%26%20cos%28%5Cphi%29%5Ctan%28%5Ctheta%29%20%5C%5C%200%20%26%20cos%28%5Cphi%29%20%26%20-sin%28%5Cphi%29%5C%5C%200%20%26%20sin%28%5Cphi%29sec%28%5Ctheta%29%20%26%20cos%28%5Cphi%29sec%28%5Ctheta%29%20%5Cend%7Bbmatrix%7D%5Cbegin%7Bbmatrix%7D%20p%5C%5C%20q%5C%5C%20r%20%5Cend%7Bbmatrix%7D" alt="https://latex.codecogs.com/svg.latex?\begin{bmatrix}
\dot{\phi} \\ 
\dot{\theta}\\ 
\dot{\psi}
\end{bmatrix}=\begin{bmatrix}
1 & sin(\phi)tan(\theta) & cos(\phi)\tan(\theta) \\ 
0 & cos(\phi) & -sin(\phi)\\ 
0 & sin(\phi)sec(\theta) & cos(\phi)sec(\theta) 
\end{bmatrix}\begin{bmatrix}
p\\ 
q\\ 
r
\end{bmatrix}" />
<br/>
<br/>
# Complementary Filter

Accelerometer gives estimate in non accelerating conditions (they have problem when they are in acceleration motion) and Gyroscope gives estimate of short period of times (bias drift)


<img src="https://latex.codecogs.com/svg.latex?%5Cunderbrace%7B%5Chat%7B%5Cphi%7D_%7Bn&plus;1%7D%20%7D_%7B%5Ctext%7Bcurrent%20angle%20estimate%7D%7D%20%3D%5Cunderbrace%7B%5Chat%7B%5Cphi%7D_%7Bacc%2Cn%7D%20%7D_%7B%5Ctext%7Bcalculated%20from%20accelerometer%7D%7D.%5Calpha%20&plus;%281-%5Calpha%29%5B%5Cunderbrace%7B%5Chat%7B%5Cphi%7D_%7Bn%7D%20%7D_%7B%5Ctext%7Bprevious%20angle%20estimate%7D%7D%20&plus;%20%5Cunderbrace%7BT.%5Cdot%7B%5Cphi%7D_%7Bgyro%2Cn%7D%20%7D_%7B%5Ctext%7Bchanges%20in%20angle%7D%7D%20%5D" alt="https://latex.codecogs.com/svg.latex?\underbrace{\hat{\phi}_{n+1}  }_{\text{current angle estimate}} =\underbrace{\hat{\phi}_{acc,n}  }_{\text{calculated from accelerometer}}.\alpha +(1-\alpha)[\underbrace{\hat{\phi}_{n}  }_{\text{previous angle estimate}} + \underbrace{T.\dot{\phi}_{gyro,n}  }_{\text{changes in angle}} ]"/>




<img src="https://latex.codecogs.com/svg.latex?%5Cunderbrace%7B%5Chat%7B%5Ctheta%7D_%7Bn&plus;1%7D%20%7D_%7B%5Ctext%7Bcurrent%20angle%20estimate%7D%7D%20%3D%5Cunderbrace%7B%5Chat%7B%5Ctheta%7D_%7Bacc%2Cn%7D%20%7D_%7B%5Ctext%7Bcalculated%20from%20accelerometer%7D%7D.%5Calpha%20&plus;%281-%5Calpha%29%5B%5Cunderbrace%7B%5Chat%7B%5Ctheta%7D_%7Bn%7D%20%7D_%7B%5Ctext%7Bprevious%20angle%20estimate%7D%7D%20&plus;%20%5Cunderbrace%7BT.%5Cdot%7B%5Ctheta%7D_%7Bgyro%2Cn%7D%20%7D_%7B%5Ctext%7Bchanges%20in%20angle%7D%7D%20%5D" alt="https://latex.codecogs.com/svg.latex?\underbrace{\hat{\theta}_{n+1}  }_{\text{current angle estimate}} =\underbrace{\hat{\theta}_{acc,n}  }_{\text{calculated from accelerometer}}.\alpha +(1-\alpha)[\underbrace{\hat{\theta}_{n}  }_{\text{previous angle estimate}} + \underbrace{T.\dot{\theta}_{gyro,n}  }_{\text{changes in angle}} ]" />


<br/>
<br/>

## Eliminating Duplicate Solutions by Limiting the Roll and Pitch Ranges

