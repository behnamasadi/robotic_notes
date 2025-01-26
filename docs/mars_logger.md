## MarsLogger

First visit the [project page](https://github.com/OSUPCVLab/mobile-ar-sensor-logger/wiki/Transfer-Android-Ubuntu)

```
sudo apt-get update
sudo apt-get install android-tools-adb
```

adb shell
cd /sdcard/Android/data/edu.osu.pcv.marslogger/files/data
ls
exit


MARS_DIR=/sdcard/Android/data/edu.osu.pcv.marslogger/files/data
OUTPUT_DIR=/home/behnam/Desktop/MarsLogger
cd $OUTPUT_DIR
adb pull $MARS_DIR

## Convert recording to ROS bag
in  /home/behnam/anaconda3/envs/bagwriter/venv/bin/python


BAG_PYTHON=/home/behnam/workspace/vio_common/python/kalibr_bagcreater.py

ANDROID_DATA_DIR=/home/behnam/data/2023_03_07_10_16_29
python $BAG_PYTHON --video $ANDROID_DATA_DIR/movie.mp4 \
--imu $ANDROID_DATA_DIR/gyro_accel.csv \
--video_time_file $ANDROID_DATA_DIR/frame_timestamps.txt \
--output_bag $ANDROID_DATA_DIR/movie.bag



BAG_PYTHON=/home/behnam/workspace/vio_common/python/kalibr_bagcreater.py
ANDROID_DATA_DIR=/home/behnam/data/2023_03_07_09_14_07
python $BAG_PYTHON --video $ANDROID_DATA_DIR/movie.mp4  --max_video_frame_height=10000 --imu $ANDROID_DATA_DIR/gyro_accel.csv --video_time_file $ANDROID_DATA_DIR/frame_timestamps.txt --output_bag $ANDROID_DATA_DIR/movie.bag 


=================================kalibr =================================

'pinhole-radtan', 'pinhole-equi', 'pinhole-fov', 'omni-none', 'omni-radtan', 'eucm-none', 'ds-none'

python bag_from_recording.py --height 1280


FOLDER=/home/behnam/anaconda3/envs/bagwriter/src/kalibr_fly

xhost +local:root
docker run -it -e "DISPLAY" -e "QT_X11_NO_MITSHM=1" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v "$FOLDER:/data" kalibr

source devel/setup.bash
rosrun kalibr kalibr_calibrate_cameras     --bag /data/fly.bag --target /data/april_6x6.yaml     --models pinhole-equi --topics /camera/image_raw

rosrun kalibr kalibr_calibrate_imu_camera --bag /data/cam_april.bag --cam  /data/cam_april-camchain.yaml  --imu  /data/imu.yaml  --target /data/april_6x6.yaml




TargetViewTable]: Tried to add second view to a given cameraId & timestamp. Maybe try to reduce the approximate syncing tolerance..


=================================kalibr android =================================
FOLDER=/home/behnam/Desktop/MarsLogger/data/2023_03_06_15_20_28
=================================kalibr android =================================

xhost +local:root
docker run -it -e "DISPLAY" -e "QT_X11_NO_MITSHM=1" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v "$FOLDER:/data" kalibr
    
    
    
source devel/setup.bash
rosrun kalibr kalibr_calibrate_cameras \
    --bag /data/cam_april.bag --target /data/april_6x6.yaml \
    --models pinhole-radtan --topics /cam0/image_raw  
    
    
    
rosrun kalibr kalibr_calibrate_imu_camera --bag /data/cam_april.bag --cam  /data/cam_april-camchain.yaml  --imu  /data/imu.yaml  --target /data/april_6x6.yaml


================================= ORB_SLAM3 =================================
xxxxxxxxxxxxxxxxxxxxxx fails xxxxxxxxxxxxxxxxxxxxxxxxxx
rosrun ORB_SLAM3 Mono /home/behnam/workspace/ORB_SLAM3/Vocabulary/ORBvoc.txt  /home/behnam/Desktop/MarsLogger/data/2023_03_06_15_20_28/ORB3_EuRoC.yaml


rosrun ORB_SLAM3 Mono_Inertial /home/behnam/workspace/ORB_SLAM3/Vocabulary/ORBvoc.txt      /home/behnam/Desktop/MarsLogger/data/2023_03_06_15_20_28/ORB3_EuRoC.yaml
cd /home/behnam/Desktop/MarsLogger/data/2023_03_07_10_16_29
rosbag play movie.bag /cam0/image_raw:=/camera/image_raw  /imu0:=/imu


================================= vins =================================

roslaunch vins vins_rviz.launch
rosrun vins vins_node /home/behnam/Desktop/MarsLogger/data/2023_03_06_15_20_28/android_imu_config.yaml

roslaunch vins vins_rviz.launch
rosrun vins vins_node /home/behnam/Desktop/MarsLogger/data/2023_03_06_15_20_28/gazebo_firefly.yaml


