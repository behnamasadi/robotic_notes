## Gazebo Versions

1. Gazebo Classic: is the original version of Gazebo. The last stable release of Gazebo Classic is Gazebo 11, which was released in 2020 and will be supported until 2025. It has integration with ROS1 and ROS2.

2. Gazebo Ignition: Ignition Gazebo is the next-generation version of Gazebo, built with modularity, flexibility, and modern design in mind. Designed to integrate better with ROS2 and other middleware.

Versions of Ignition Gazebo:

 - Ignition Gazebo 1: Acropolis
 - Ignition Gazebo 2: Blueprint
 - Ignition Gazebo 3: Citadel
 - Ignition Gazebo 4: Dome
 - Ignition Gazebo 5: Edifice
 - Ignition Gazebo 6: Fortress
 - Gazebo Sim 7: Garden
 - Gazebo Sim 8: Harmonic


**Gazebo Sim (GZ Sim)**: The name change from Ignition Gazebo to GZ Sim (or simply GZ Sim),  It's a continuation of Ignition, but with the name transition.


<img src="images/gazebo_timeline.svg" width="100%" />


Refs: [1](https://gazebosim.org/about)


## Installation
Gazebo Harmonic: Harmonic binaries are provided for **Ubuntu Jammy (22.04)** and **Ubuntu Noble (24.04)**.

dependencies: 
```
sudo apt-get update
sudo apt-get install curl lsb-release gnupg
```


Then install Gazebo Harmonic:

```
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
```

remove:

```
sudo apt remove gz-harmonic && sudo apt autoremove
```

you can run it by:

```
gz sim
```

Ref: [1](https://gazebosim.org/docs/latest/install_ubuntu/)


## Building a model

Defining the model: 

```
<model name='vehicle_blue' canonical_link='chassis'>
    <pose relative_to='world'>0 0 0 0 0 0</pose>
```

`canonical_link`: Each model may have one link designated as the canonical_link, the implicit frame of the model is attached to this link. If not defined, the first <link> will be chosen as the


`pose`: The <pose> tag is used to define the position and orientation of our model, If relative_to is not defined, the model’s <pose> will be relative to the world.

### Links
Every model is a group of `links` connected together with `joints`.

#### Inertia Matrix 
Inertia is a property of an object that resists changes in its motion, specifically rotational motion in this case. For a 3D object, the inertia of the object is represented by a 3x3 matrix called the **inertia tensor**. The terms you mentioned, <img src="https://latex.codecogs.com/svg.image?&space;I_{xx}" alt="I_{xx}" />, <img src="https://latex.codecogs.com/svg.image?&space;I_{xy}" alt="I_{xy}" />, <img src="https://latex.codecogs.com/svg.image?&space;I_{xz}" alt="I_{xz}" />, <img src="https://latex.codecogs.com/svg.image?&space;I_{yy}" alt="I_{yy}" />, <img src="https://latex.codecogs.com/svg.image?&space;I_{yz}" alt="I_{yz}" />, <img src="https://latex.codecogs.com/svg.image?&space;I_{zz}" alt="I_{zz}" />, are the components of this tensor.

Simple explanation using a 3D object, such as a **box**.

**Inertia Tensor Components**:

- **<img src="https://latex.codecogs.com/svg.image?&space;I_{xx}" alt="I_{xx}" />, <img src="https://latex.codecogs.com/svg.image?&space;I_{yy}" alt="I_{yy}" />, <img src="https://latex.codecogs.com/svg.image?&space;I_{zz}" alt="I_{zz}" />:** These represent the moments of inertia about the x, y, and z axes, respectively. These describe how difficult it is to rotate the object around each of these axes.
    - Example: If you try to rotate a box around its **x-axis**, <img src="https://latex.codecogs.com/svg.image?&space;I_{xx}" alt="I_{xx}" /> will tell you how much the box resists that rotation.

- **<img src="https://latex.codecogs.com/svg.image?&space;I_{xy}" alt="I_{xy}" />, <img src="https://latex.codecogs.com/svg.image?&space;I_{xz}" alt="I_{xz}" />, <img src="https://latex.codecogs.com/svg.image?&space;I_{yz}" alt="I_{yz}" />:** These represent the products of inertia. These terms describe how much the mass of the object is "coupled" between two different axes.
    - Example: If you try to rotate the box around the **x-axis**, but its mass distribution is uneven, you might also induce some rotation around the **y-axis** or **z-axis**. <img src="https://latex.codecogs.com/svg.image?&space;I_{xy}" alt="I_{xy}" />, <img src="https://latex.codecogs.com/svg.image?&space;I_{xz}" alt="I_{xz}" />, and <img src="https://latex.codecogs.com/svg.image?&space;I_{yz}" alt="I_{yz}" /> describe this coupling between axes.


---
**Simple Example** 

Imagine you have a rectangular box. Its moments of inertia about the **x**, **y**, and **z** axes can be calculated based on how its mass is distributed along those axes. Here’s a breakdown:

- <img src="https://latex.codecogs.com/svg.image?&space;I_{xx}" alt="I_{xx}" /> depends on the mass distribution away from the **x-axis** (how far the mass is from the axis).
- <img src="https://latex.codecogs.com/svg.image?&space;I_{yy}" alt="I_{yy}" /> depends on the mass distribution away from the **y-axis**.
- <img src="https://latex.codecogs.com/svg.image?&space;I_{zz}" alt="I_{zz}" /> depends on the mass distribution away from the **z-axis**.

Now, if the box is perfectly symmetrical and aligned with the axes, <img src="https://latex.codecogs.com/svg.image?&space;I_{xy}" alt="I_{xy}" />, <img src="https://latex.codecogs.com/svg.image?&space;I_{xz}" alt="I_{xz}" />, and <img src="https://latex.codecogs.com/svg.image?&space;I_{yz}" alt="I_{yz}" /> would be zero, meaning there's no coupling. But if the box is tilted or its mass is unevenly distributed, these terms would have values, indicating that rotating about one axis would also induce some motion around the others.

In Summary

- **Diagonal terms** (<img src="https://latex.codecogs.com/svg.image?&space;I_{xx}" alt="I_{xx}" />, <img src="https://latex.codecogs.com/svg.image?&space;I_{yy}" alt="I_{yy}" />, <img src="https://latex.codecogs.com/svg.image?&space;I_{zz}" alt="I_{zz}" />) show resistance to rotation about each principal axis.
- **Off-diagonal terms** (<img src="https://latex.codecogs.com/svg.image?&space;I_{xy}" alt="I_{xy}" />, <img src="https://latex.codecogs.com/svg.image?&space;I_{xz}" alt="I_{xz}" />, <img src="https://latex.codecogs.com/svg.image?&space;I_{yz}" alt="I_{yz}" />) show coupling between the axes, where rotating around one axis can also affect rotation around others. 
---
### Visual and collision



### Connecting links together (joints)
The joint tag connects two links together and defines how they will move with respect to each other, we need to define the two links to connect and their relations (way of movement)


## Building world

### Physics
```
<physics name="1ms" type="ignored">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
</physics>
```

### Plugins
#### Physics



- `type`: is the type of the dynamic engine (Ode, Bullet, Simbody and Dar), choosing the type of the physics engine is **not** done through this tag yet
- `max_step_size`:  maximum time at which every system in simulation can interact with the states of the world

```
<plugin
    filename="gz-sim-user-commands-system"
    name="gz::sim::systems::UserCommands">
</plugin>
```


#### User-commands
```
<plugin
    filename="gz-sim-user-commands-system"
    name="gz::sim::systems::UserCommands">
</plugin>
```
#### Scene-broadcaster

### GUI
#### World control plugin
#### World stats plugin
#### Entity tree

## Moving the robot



## Sensors

### IMU sensor
This code defines the IMU sensor plugin to be used in our world
```
<plugin filename="gz-sim-imu-system"
        name="gz::sim::systems::Imu">
</plugin>
```

Now we can add the IMU sensor to our robot as follows:

```
<sensor name="imu_sensor" type="imu">
    <always_on>1</always_on>
    <update_rate>1</update_rate>
    <visualize>true</visualize>
    <topic>imu</topic>
</sensor>
```


### Lidar sensor

Add this plugin under the <world> tag, to be able to use the lidar sensor:

```
    <plugin
      filename="gz-sim-sensors-system"
      name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
```

add the lidar sensor 

```
<sensor name='gpu_lidar' type='gpu_lidar'>"
    <pose relative_to='lidar_frame'>0 0 0 0 0 0</pose>
    <topic>lidar</topic>
    <update_rate>10</update_rate>
    <ray>
        <scan>
            <horizontal>
                <samples>640</samples>
                <resolution>1</resolution>
                <min_angle>-1.396263</min_angle>
                <max_angle>1.396263</max_angle>
            </horizontal>
            <vertical>
                <samples>1</samples>
                <resolution>0.01</resolution>
                <min_angle>0</min_angle>
                <max_angle>0</max_angle>
            </vertical>
        </scan>
        <range>
            <min>0.08</min>
            <max>10.0</max>
            <resolution>0.01</resolution>
        </range>
    </ray>
    <always_on>1</always_on>
    <visualize>true</visualize>
</sensor>
```

## Spawn URDF
Lets download a simple model [08-macroed.urdf.xacro](https://github.com/ros/urdf_tutorial/blob/ros2/urdf/08-macroed.urdf.xacro)


```
xacro 08-macroed.urdf.xacro > 08-macroed.urdf.urdf
```

```
check_urdf 08-macroed.urdf.urdf
```

```
urdf_to_graphviz 08-macroed.urdf.urdf
```

and for SVG

```
dot -Tsvg macroed.gv -o output.svg
```

<img src="images/08-macroed.svg" />


more models [here](https://wiki.ros.org/urdf/Examples)



Now 


```
gz sim empty.sdf
```

list all services:

```
gz service -l
```



## ROS 2 integration

- Use ROS to launch Gazebo
- Use ROS to interact with Gazebo topics via the [ros_gz](https://github.com/gazebosim/ros_gz): Gazebo has its own middleware, Gazebo Transport, that exposes a set of topics and services quite similar to ROS. The ros_gz bridge allows you to create a bridge between Gazebo and your ROS system, that translates between Gazebo Transport and ROS 2 as needed.






ROS 2 version | Gazebo version | Branch | Binaries hosted at
-- | -- | -- | --
Foxy | Citadel | [foxy](https://github.com/gazebosim/ros_gz/tree/foxy) | https://packages.ros.org
Foxy | Edifice | [foxy](https://github.com/gazebosim/ros_gz/tree/foxy) | only from source
Galactic | Edifice | [galactic](https://github.com/gazebosim/ros_gz/tree/galactic) | https://packages.ros.org
Galactic | Fortress | [galactic](https://github.com/gazebosim/ros_gz/tree/galactic) | only from source
Humble | Fortress | [humble](https://github.com/gazebosim/ros_gz/tree/humble) | https://packages.ros.org
Humble | Garden | [humble](https://github.com/gazebosim/ros_gz/tree/humble) | [gazebo packages](https://gazebosim.org/docs/latest/ros_installation#gazebo-garden-with-ros-2-humble-iron-or-rolling-use-with-caution-)[^1]
Humble | Harmonic | [humble](https://github.com/gazebosim/ros_gz/tree/humble) | [gazebo packages](https://gazebosim.org/docs/harmonic/ros_installation#-gazebo-harmonic-with-ros-2-humble-iron-or-rolling-use-with-caution-)[^1]
Iron | Fortress | [humble](https://github.com/gazebosim/ros_gz/tree/iron) | https://packages.ros.org
Iron | Garden | [humble](https://github.com/gazebosim/ros_gz/tree/iron) | only from source
Iron | Harmonic | [humble](https://github.com/gazebosim/ros_gz/tree/iron) | only from source
Jazzy | Garden | [ros2](https://github.com/gazebosim/ros_gz/tree/ros2) | only from source
Jazzy | Harmonic | [jazzy](https://github.com/gazebosim/ros_gz/tree/jazzy) | https://packages.ros.org
Rolling | Fortress | [humble](https://github.com/gazebosim/ros_gz/tree/humble) | https://packages.ros.org
Rolling | Garden | [ros2](https://github.com/gazebosim/ros_gz/tree/ros2) | only from source
Rolling | Harmonic | [ros2](https://github.com/gazebosim/ros_gz/tree/ros2) | only from source


