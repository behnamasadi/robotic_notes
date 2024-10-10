## Installation

- Rolling Ridley (progressing since June 2020)
- Jazzy Jalisco (May 2029, Ubuntu Linux - Noble Numbat (24.04))
- Humble Hawksbill (EOL date May 2027, Ubuntu Linux - Jammy Jellyfish (22.04))




## Configuration


```
source /opt/ros/$ROS_DISTRO/setup.sh
printenv | grep -i ROS
```


### Domain ID.
As the default middleware that ROS 2 uses for communication is DDS. In DDS, you can have different logical networks share a physical network is known as the `Domain ID`. ROS 2 nodes on the same domain can freely discover and send messages to each other. All ROS 2 nodes use domain ID 0 by default. 

```
export ROS_DOMAIN_ID=<your_domain_id>
```

### ROS_AUTOMATIC_DISCOVERY_RANGE
By default, ROS 2 communication is not limited to localhost. `ROS_AUTOMATIC_DISCOVERY_RANGE` environment variable allows you to limit ROS 2 discovery range


### Remapping

```
ros2 run turtlesim turtle_teleop_key
```
will publish `/turtle1/cmd_vel` 


we can remap the the topic:
```
ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel
```
will publish `/turtle2/cmd_vel` 

this will rename the node itself:
```
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
```


to see the message definition:

```
ros2 interface show geometry_msgs/msg/Twist
```
will give you 


```
    Vector3  linear
            float64 x
            float64 y
            float64 z
    Vector3  angular
            float64 x
            float64 y
            float64 z
```

Load parameter file on node startup

```
ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
```
  
### Launching nodes

      

## Colcon

Installation:
```
sudo apt install python3-colcon-common-extensions
```
If you do not want to build a specific package place an empty file named `COLCON_IGNORE` in the directory and it will not be indexed.

or you can specify your package

```
colcon build --packages-select my_package
```


#### Underlay
An underlay workspace is a workspace that has already been built and sourced.


```
source /opt/ros/$ROS_DISTRO/setup.sh
```

#### Overlay

An overlay workspace is where you are actively developing or testing new ROS 2 packages, on top of the existing underlay.
```
source ~/ros2_ws/install/setup.bash
```

By sourcing both the underlay and the overlay, your environment will recognize both the ROS 2 core packages (underlay) and any new or updated packages you have built in the overlay


#### Resolve dependencies

In the root of your workspace:

```
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
```

1. `--from-paths` src specifies the path to check for package.xml files to resolve keys for
2. `-y` means to default yes to all prompts from the package manager to install without prompts
3. `--ignore-src` means to ignore installing dependencies, even if a rosdep key exists, if the package itself is also in the workspace.


`rosdep` is not a package manager, it uses its own knowledge of the system and the dependencies to find the appropriate package to install on a particular platform.  e.g. apt on Debian/Ubuntu

The dependencies in the package.xml file are generally referred to as `rosdep keys`, more [here](https://ros.org/reps/rep-0149.html)

##### <depend>
These are dependencies that should be provided at both build time and run time for your package.
For C++ packages, if in doubt, use this tag.


#### <build_depend>
If you only use a particular dependency for building your package, and not at execution time, for example

```
  <build_depend>gtest</build_depend>
  <build_depend>ament_cmake</build_depend>

```

#### <build_export_depend>

It is used when a dependency is needed not only during the build phase of the package but also needs to be made available for other packages that depend on this package. This is especially important if your package **exposes some headers** or libraries that other packages might use at build time.


examples:  If your package uses a header-only library such as Eigen3

```
  <build_export_depend>Eigen3</build_export_depend>
```

#### <exec_depend>
This tag declares dependencies for shared libraries, executables, Python modules, launch scripts and other files required when running your package. For packages with launch files, it is a good idea to add an exec_depend dependency on the ros2launch package in your package’s package.xml:

```
<exec_depend>ros2launch</exec_depend>
```
This helps make sure that the ros2 launch command is available after building your package. It also ensures that all launch file formats are recognized.



#### How to find packages that goes into package.xml

1. For ROS-based: find a list of all released ROS packages `https://github.com/ros/rosdistro/<distro>/distribution.yaml`
2. For non-ROS package: for [apt system dependencies](https://github.com/ros/rosdistro/blob/master/rosdep/base.yaml) and for [Python dependencies](https://github.com/ros/rosdistro/blob/master/rosdep/python.yaml)



### Package
Package creation in ROS 2 uses `ament` as its **build system** and `colcon` as its **build tool**

ROS 2  `CMake` package minimum required contents:

- CMakeLists.txt file that describes how to build the code within the package
- include/<package_name> directory containing the public headers for the package
- package.xml file containing meta information about the package
- src directory containing the source code for the package



ROS 2 `Python` package minimum required contents:


- package.xml file containing meta information about the package
- resource/<package_name> marker file for the package
- setup.cfg is required when a package has executables, so ros2 run can find them
- setup.py containing instructions for how to install the package
- <package_name> - a directory with the same name as your package, used by ROS 2 tools to find your package, contains __init__.py



```
ros2 pkg create --build-type ament_cmake --license Apache-2.0 <package_name>
```
or 

```
ros2 pkg create --build-type ament_python --license Apache-2.0 <package_name>
```
```
ros2 pkg
  create       Create a new ROS 2 package
  executables  Output a list of package specific executables
  list         Output a list of available packages
  prefix       Output the prefix path of a package
  xml          Output the XML of the package manifest or a specific tag
```



### Workspace 

A single workspace can contain as many packages. You **cannot** have nested packages.


Refs [1](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)

## Using Xacro


Refs: [1](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html)


## Creating a launch file
Launch files written in Python, XML, or YAML

### Python launch file

launch file name needs to end with `launch.py` to be recognized and autocompleted by ros2 launch.
launch file should define the generate_launch_description() function which returns a launch.LaunchDescription()

```
import launch
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])
```

When launching the two turtlesim nodes, the only difference between them is their namespace values. Unique namespaces allow the system to start two nodes without node name or topic name conflicts. Both turtles in this system receive commands over the same topic and publish their pose over the same topic.

```
ros2 node list
/mimic
/turtlesim1/sim
/turtlesim2/sim
```

and
 
```
ros2 topic list 
/parameter_events
/rosout
/turtlesim1/turtle1/cmd_vel
/turtlesim1/turtle1/color_sensor
/turtlesim1/turtle1/pose
/turtlesim2/turtle1/cmd_vel
/turtlesim2/turtle1/color_sensor
/turtlesim2/turtle1/pose
```






After creating launch file you need to update the `setup.py` to install them, for instance add this: 

```
('share/' + package_name + '/launch', ['launch/turtlesim_mimic_launch.py']),
```

so you get this:

```
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/turtlesim_mimic_launch.py']),
    ]
```

or you can add all launch files:


```
import os
from glob import glob

    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),

    ]
```

### C++ launch file
adjusting the `CMakeLists.txt` file by adding:

```
# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
```

and in the `launch/my_script_launch.py`

```
import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='cpp_pubsub',
            executable='talker',
            name='talker'),
        launch_ros.actions.Node(
            package='cpp_pubsub',
            executable='listener',
            name='talker'),            
  ])
```

### Managing launch file
Refs: [1](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html)



[Architecture of launch](https://github.com/ros2/launch/blob/humble/launch/doc/source/architecture.rst)


Refs: [1](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html), [2](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-system.html)

### Substitutions
Launch files are used to start nodes, and services and execute processes. This set of actions may have arguments, which affect their behavior. Substitutions can be used in arguments to provide more flexibility when describing reusable launch files.

### Event handlers
Launch in ROS 2 is a system that executes and manages user-defined processes. It is responsible for monitoring the state of processes it launched, as well as reporting and reacting to changes in the state of those processes. These changes are called events and can be handled by registering an event handler with the launch system.


## Nav2 - ROS 2 Navigation Stack

Refs [1](https://neobotix-docs.de/ros/ros2/autonomous_navigation.html#:~:text=Nav2%20is%20the%20navigation%20stack,it%20provides%20to%20the%20user.), [2](https://github.com/ros-navigation/navigation2)



## ROS2 Interaction With Gazebo

```
ros2 run ros_gz_bridge parameter_bridge -h
```

Bridge a collection of ROS2 and Gazebo Transport topics and services.

```
  parameter_bridge [ topic_name@ROS2_type@Ign_type ]
```

Topics: The first `@` symbol delimits the topic name from the message types.

Following the first `@` symbol is the ROS message type.

The ROS message type is followed by an `@`, `[`, or `]` symbol where
    `@`  == a bidirectional bridge, 
    `[`  == a bridge from Gazebo to ROS,
    `]`  == a bridge from ROS to Gazebo.
    
Following the direction symbol is the Gazebo Transport message type.

A bidirectional bridge example:
```
ros2 run ros_gz_bridge parameter_bridge /chatter@std_msgs/msg/String@gz.msgs.StringMsg
```
or 

```
ros2 run ros_gz_bridge parameter_bridge /chatter@std_msgs/msg/String@ignition.msgs.StringMsg --ros-args --log-level DEBUG

```
then publish from ROS:


```
ros2 topic pub /chatter std_msgs/msg/String '{data: "Hello from ROS"}'
```
receive from gazebo: 

```
ign  topic -e -t /chatter
```


then publish from gazebo:

```
ign topic -t /chatter -m ignition.msgs.StringMsg -p 'data:"Hello from Gazebo"'
```

receive in ROS: 
```
ros2 topic echo /chatter
```

A bridge from Gazebo to ROS example:

```
ros2 run ros_gz_bridge parameter_bridge  /chatter@std_msgs/msg/String[ignition.msgs.StringMsg
                                                         
```


A bridge from ROS to Gazebo example:
```
ros2 run ros_gz_bridge parameter_bridge  /chatter@std_msgs/msg/String]ignition.msgs.StringMsg
```

Ref: [1](https://index.ros.org/p/ros_gz_bridge/)




```
ign gazebo -v 4 -r visualize_lidar.sdf
```



gz topic -l


gz service -l



```
ign gazebo -v 4 -r visualize_lidar.sdf
```

- `-v 4`: This sets the verbosity level of Gazebo to 4. 
- `-r`:  visualize_lidar.sdf: This loads a specific SDF

```
ros2 run ros_gz_bridge parameter_bridge /model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist
```

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/model/vehicle_blue/cmd_vel
```

```
ros2 run ros_gz_bridge parameter_bridge /lidar2@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan --ros-args -r /lidar2:=/laser_scan
```

```
rviz2
```



[Bridge communication between ROS and Gazebo](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge)

Refs: [1](https://gazebosim.org/docs/harmonic/ros2_integration/#use-ros-2-to-interact-with-gazebo)


## Obtain the ground truth position

```
gz topic -e -t /model/tugbot/pose
```
`-e` : for echo.
`-t` : specify the topic.




## teleop_twist_keyboard

In `ROS2`'s `teleop_twist_keyboard`, the keys on the keyboard are mapped to specific movements by publishing messages to the `/cmd_vel` topic. These messages contain geometry messages of type `Twist`, which is a data structure that consists of two vectors: `linear` and `angular` velocities.

Here's how the key mappings work in terms of movement:

```
   u    i    o
   j    k    l
   m    ,    .
```

### Breakdown of movement commands:

- `i`: Move forward (linear velocity along the x-axis, no angular velocity).
- `,`: Move backward (linear velocity along the x-axis, but in the opposite direction, negative).
- `j`: Turn left (positive angular velocity around the z-axis, no linear velocity).
- `l`: Turn right (negative angular velocity around the z-axis, no linear velocity).
- `u`: Move forward while turning left (combination of linear x velocity and angular z velocity).
- `o`: Move forward while turning right (combination of linear x velocity and negative angular z velocity).
- `m`: Move backward while turning left (combination of negative linear x velocity and positive angular z velocity).
- `.`: Move backward while turning right (combination of negative linear x velocity and negative angular z velocity).
- `k`: Stop (no movement, both linear and angular velocities are set to zero).

### How it works:
- Each key corresponds to a specific combination of linear and angular velocities. When you press a key, a `Twist` message is published, which controls the robot's velocity. For example:
  - If you press `i`, the robot's `linear.x` is set to a positive value (move forward).
  - If you press `j`, the robot's `angular.z` is set to a positive value (turn left).
  - The combination keys like `u` and `o` adjust both the `linear` and `angular` values to move in a curve.

This method gives simple control over robot movement by adjusting these velocity commands. The robot then executes the motion as long as the velocity command is being received from the `teleop_twist_keyboard`.

Let me know if you'd like more specific details about the implementation!
