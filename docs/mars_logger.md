# MarsLogger — Android sensor capture for VIO/SLAM

[Mars Logger](https://github.com/OSUPCVLab/mobile-ar-sensor-logger) records synchronized **camera frames + accelerometer + gyroscope** from an Android phone in a format suitable for visual-inertial odometry / SLAM benchmarks.

## 1. Install the app on the phone

1. Grab the latest `.apk` from the project's [transfer guide](https://github.com/OSUPCVLab/mobile-ar-sensor-logger/wiki/Transfer-Android-Ubuntu) and install it.
2. Enable **Developer Options**: Settings → About → tap *Build number* 7 times.
3. Enable **USB debugging** under Developer Options.

## 2. Pull recordings from the phone

Install ADB on the host:

```bash
sudo apt-get update
sudo apt-get install -y android-tools-adb
```

Recordings are written to `/sdcard/Android/data/edu.osu.pcv.marslogger/files/data`. Browse first:

```bash
adb shell
cd /sdcard/Android/data/edu.osu.pcv.marslogger/files/data
ls
exit
```

Pull a session to the host:

```bash
MARS_DIR=/sdcard/Android/data/edu.osu.pcv.marslogger/files/data
OUTPUT_DIR=$HOME/Desktop/MarsLogger
mkdir -p "$OUTPUT_DIR" && cd "$OUTPUT_DIR"
adb pull "$MARS_DIR"
```

A session directory contains:
- `movie.mp4` — H.264 video
- `gyro_accel.csv` — IMU samples (gyro + accel, hardware-timestamped)
- `frame_timestamps.txt` — per-frame capture timestamps
- `intrinsics.yaml`, `*.csv` metadata

## 3. Convert a recording to a ROS bag

Use `kalibr_bagcreater.py` from [vio_common](https://github.com/JzHuai0108/vio_common). It produces `/cam0/image_raw` and `/imu0` topics with frame timestamps aligned to the IMU clock.

```bash
BAG_PYTHON=$HOME/workspace/vio_common/python/kalibr_bagcreater.py
ANDROID_DATA_DIR=$HOME/Desktop/MarsLogger/data/2023_03_07_10_16_29

python "$BAG_PYTHON" \
  --video           "$ANDROID_DATA_DIR/movie.mp4" \
  --imu             "$ANDROID_DATA_DIR/gyro_accel.csv" \
  --video_time_file "$ANDROID_DATA_DIR/frame_timestamps.txt" \
  --output_bag      "$ANDROID_DATA_DIR/movie.bag"
```

For high-resolution captures, allow taller frames:

```bash
python "$BAG_PYTHON" \
  --video                  "$ANDROID_DATA_DIR/movie.mp4" \
  --max_video_frame_height 10000 \
  --imu                    "$ANDROID_DATA_DIR/gyro_accel.csv" \
  --video_time_file        "$ANDROID_DATA_DIR/frame_timestamps.txt" \
  --output_bag             "$ANDROID_DATA_DIR/movie.bag"
```

---

## 4. Camera + IMU calibration with Kalibr

[Kalibr](https://github.com/ethz-asl/kalibr) is most easily run from its official Docker image.

Supported camera models:

```
pinhole-radtan, pinhole-equi, pinhole-fov,
omni-none,     omni-radtan,
eucm-none, ds-none
```

### 4.1 Launch the container

```bash
FOLDER=$HOME/Desktop/MarsLogger/data/2023_03_06_15_20_28

xhost +local:root
docker run -it \
  -e DISPLAY -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$FOLDER:/data" \
  kalibr
```

Inside the container:

```bash
source devel/setup.bash
```

### 4.2 Camera intrinsics

```bash
rosrun kalibr kalibr_calibrate_cameras \
  --bag    /data/cam_april.bag \
  --target /data/april_6x6.yaml \
  --models pinhole-radtan \
  --topics /cam0/image_raw
```

Output: `cam_april-camchain.yaml`.

### 4.3 Camera–IMU extrinsics

```bash
rosrun kalibr kalibr_calibrate_imu_camera \
  --bag    /data/cam_april.bag \
  --cam    /data/cam_april-camchain.yaml \
  --imu    /data/imu.yaml \
  --target /data/april_6x6.yaml
```

### 4.4 Common errors

> `[TargetViewTable] Tried to add second view to a given cameraId & timestamp. Maybe try to reduce the approximate syncing tolerance.`

Two image messages share a timestamp (Kalibr de-dupes by `(camera_id, timestamp)`). Either re-record at a lower frame rate, or pass `--dont-show-report --time-calibration --max-iter <N>` plus a tighter sync tolerance.

---

## 5. ORB-SLAM3

```bash
cd $HOME/Desktop/MarsLogger/data/2023_03_07_10_16_29

# Mono
rosrun ORB_SLAM3 Mono \
  $HOME/workspace/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  ./ORB3_EuRoC.yaml

# Mono + IMU
rosrun ORB_SLAM3 Mono_Inertial \
  $HOME/workspace/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  ./ORB3_EuRoC.yaml

# Replay the bag with topics remapped to ORB-SLAM3's defaults
rosbag play movie.bag /cam0/image_raw:=/camera/image_raw /imu0:=/imu
```

The YAML must contain the camera intrinsics + camera-IMU extrinsics produced by Kalibr in §4.

## 6. VINS-Fusion

```bash
roslaunch vins vins_rviz.launch
rosrun vins vins_node $HOME/Desktop/MarsLogger/data/2023_03_06_15_20_28/android_imu_config.yaml

# In another terminal
rosbag play movie.bag
```

The VINS config YAML expects `body_T_cam0`, accelerometer/gyroscope noise densities, and the topic names that match the bag.

---

## References

- [Mars Logger repo](https://github.com/OSUPCVLab/mobile-ar-sensor-logger)
- [Mars Logger transfer guide](https://github.com/OSUPCVLab/mobile-ar-sensor-logger/wiki/Transfer-Android-Ubuntu)
- [vio_common (`kalibr_bagcreater.py`)](https://github.com/JzHuai0108/vio_common)
- [Kalibr](https://github.com/ethz-asl/kalibr)
- [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
