## Building PX4

On Ubuntu 22.04, ROS2 `humble`  and Gazebo `Harmonic` (`gz sim --version`)


```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```


##  Build (Using a Simulator)

Gazebo SITL (Software In The Loop)

Run the followings:
```
make px4_sitl gz_x500
```

The full syntax to call make with a particular configuration and initialization file is:
```
make [VENDOR_][MODEL][_VARIANT] [VIEWER_MODEL_DEBUGGER_WORLD]
```

### VENDOR_MODEL_VARIANT: (also known as CONFIGURATION_TARGET)

- VENDOR: `px4`, `aerotenna`, `airmind`, etc
- MODEL: The board model "model": `sitl`, `fmu-v2`, `fmu-v3`, `fmu-v4`, `fmu-v5`, `navio2`, etc.
- VARIANT: particular configurations,Most commonly this is default, and may be omitted.


list of all available `CONFIGURATION_TARGET` options:

```
make list_config_targets
```

### VIEWER_MODEL_DEBUGGER_WORLD

- VIEWER: `gz`, `gazebo`, `jmavsim`, `none`. if you want to launch PX4 and wait for a simulator (jmavsim, Gazebo, Gazebo Classic, or some other simulator). For example, `make px4_sitl none_iris` launches PX4 without a simulator (but with the iris airframe).

```
make px4_sitl jmavsim
```

- MODEL: The vehicle model to use (e.g. `iris` (default), `rover`, `tailsitter`, etc). The environment variable PX4_SIM_MODEL will be set to the selected model

- WORLD: (Gazebo Classic only).



Refs: [1](https://docs.px4.io/main/en/dev_setup/building_px4.html#px4-make-build-targets)



You can also specify the world using the `PX4_GZ_WORLD` environment variable:

```
PX4_GZ_WORLD=windy make px4_sitl gz_x500
```

List of some of the avilable vehicles

- `make px4_sitl gz_x500`
- `make px4_sitl gz_x500_depth`
- `make px4_sitl gz_x500_vision`
- `make px4_sitl gz_x500_lidar`
- `make px4_sitl gz_standard_vtol`
- `make px4_sitl gz_rc_cessna`
- `make px4_sitl gz_advanced_plane`
- `make px4_sitl gz_r1_rover`
- `make px4_sitl gz_rover_ackermann`

All vehicle models (and worlds) are included as a submodule from the Gazebo Models Repository repository.


Refs: [1](https://docs.px4.io/main/en/sim_gazebo_gz/#running-the-simulation)

### Standalone Mode
In this mode PX4 SITL and Gazebo are started separately in their own terminals.

```
cd ~/workspace/PX4-Autopilot/Tools/simulation/gz/
python simulation-gazebo
```
or 

```
python simulation-gazebo --world walls
```


then 



```
PX4_GZ_STANDALONE=1 make px4_sitl gz_x500
```

more [here](https://docs.px4.io/main/en/sim_gazebo_gz/gazebo_models.html#basic-usage)



## ROS2


```
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```
installation:

```
make -j20 install DESTDIR=~/
```

LD_LIBRARY_PATH=~/usr/local/lib

```
cd ~/usr/local/bin

./MicroXRCEAgent udp4 -p 8888
```


First

```
cd ~/ros2_ws/src/
```

then 

```
git clone git@github.com:PX4/px4_msgs.git
git@github.com:PX4/px4_ros_com.git
```

and finally:

```
cd ~/ros2_ws/
colcon build
```
 


```
gz topic -i -t /world/default/model/x500_lidar_0/link/link/sensor/lidar_2d_v2/scan/points
```

/world/default/model/x500_lidar_0/link/link/sensor/lidar_2d_v2/scan






# Remove the wrong version (for Ignition Fortress)
sudo apt remove ros-humble-ros-gz

# Install the version for Gazebo Garden
sudo apt install ros-humble-ros-gzharmonic






Refs: [1](https://docs.px4.io/main/en/ros2/user_guide.html#humble)











https://docs.px4.io/main/en/test_and_ci/docker.html

# enable access to xhost from the container
xhost +

# Run docker and open bash shell
docker run -it --privileged \
--env=LOCAL_USER_ID="$(id -u)" \
-v ~/workspace/PX4-Autopilot:/src/PX4-Autopilot/:rw \
-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
-e DISPLAY=$DISPLAY \
--network host \
--name=px4-ros px4io/px4-dev-base-jammy:latest bash


cd src/PX4-Autopilot    #This is <container_src>
make px4_sitl_default gazebo-classic





# start the container
docker start -i px4-ros
# open a new bash shell in this container
docker exec -it px4-ros bash


cd /src/PX4-Autopilot/
source /opt/ros/noetic/setup.sh

https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/quick_start.html


https://docs.px4.io/main/en/simulation/ros_interface.html

roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"


https://www.youtube.com/watch?v=jBTikChu02E&list=PLYy2pGCdhu7xEaNN8krzAKxv74L1mD4OV&index=5





export GZ_SIM_RESOURCE_PATH=/home/behnam/workspace/PX4-Autopilot/Tools/simulation/gz/worlds:$GZ_SIM_RESOURCE_PATH


PX4_SITL_WORLD=/home/behnam/workspace/PX4-Autopilot/Tools/simulation/gz/worlds/baylands.sdf 
make px4_sitl gz_x500_depth


PX4_SITL_WORLD=/home/behnam/workspace/PX4-Autopilot/Tools/simulation/gz/worlds/baylands.sdf 
make px4_sitl gz_x500_depth















ign topic -l


 ign topic -i -t /camera 
Publishers [Address, Message Type]:
  tcp://172.17.0.1:46765, gz.msgs.Image



ros2 run ros_ign_bridge parameter_bridge /example_topic@std_msgs/msg/String[ignition.msgs.StringMsg








roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"












