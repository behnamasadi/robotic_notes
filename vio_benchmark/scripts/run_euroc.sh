#!/usr/bin/env bash
# Run two EuRoC sequences (MH_03, V1_01) end-to-end:
#   wait for downloads → convert each bag → for each sequence:
#       OpenVINS solo run, then VINS-Fusion solo run, recording outputs
# Designed to run unattended ~30 min.
set -e

CONTAINER=explorer_r2_sim-sim-1

# 1. Wait for both downloads
echo "[1/4] waiting for downloads..."
for f in MH_03_medium V1_01_easy; do
    while ! ls -la ~/datasets/euroc/${f}.bag >/dev/null 2>&1; do sleep 10; done
    # Wait until size is stable (not still growing)
    PREV=0
    while true; do
        CURR=$(stat -c %s ~/datasets/euroc/${f}.bag 2>/dev/null)
        if [ "$CURR" = "$PREV" ] && [ "$CURR" -gt 100000000 ]; then break; fi
        PREV=$CURR
        sleep 5
    done
    echo "  $f.bag ready ($(du -h ~/datasets/euroc/${f}.bag | awk '{print $1}'))"
done

# 2. Convert to ROS 2 mcap
echo "[2/4] converting bags..."
for f in MH_03_medium V1_01_easy; do
    if [ ! -d ~/datasets/euroc/${f}_ros2 ]; then
        rosbags-convert --src ~/datasets/euroc/${f}.bag --dst ~/datasets/euroc/${f}_ros2 --dst-storage mcap
        echo "  converted $f"
    else
        echo "  $f_ros2 already exists, skipping"
    fi
done

# 3. For each sequence, run BOTH estimators sequentially
for SEQ in MH_03_medium V1_01_easy; do
    echo "[3/4] sequence: $SEQ"
    SEQ_TAG=$(echo $SEQ | tr '_' '_')

    # 3a. OpenVINS solo
    docker exec $CONTAINER bash -lc "pkill -KILL -f run_subscribe_msckf 2>/dev/null; pkill -KILL -f vins_node 2>/dev/null; sleep 2; true"
    docker exec -d $CONTAINER bash -lc "
        source /opt/ros/jazzy/setup.bash
        source /ws/install/setup.bash
        ros2 launch ov_msckf subscribe.launch.py \
            config_path:=/ws/install/ov_msckf/share/ov_msckf/config/euroc_mav/estimator_config.yaml \
            use_stereo:=true max_cameras:=2 verbosity:=INFO > /tmp/ov_${SEQ_TAG}.log 2>&1
    "
    sleep 5
    docker exec -d $CONTAINER bash -lc "
        source /opt/ros/jazzy/setup.bash
        cd /ws/runs && rm -rf euroc_${SEQ_TAG}_ov && \
        ros2 bag record -o euroc_${SEQ_TAG}_ov -s mcap \
            /ov_msckf/odomimu /ov_msckf/pathimu \
            /leica/position /vicon/firefly_sbx/firefly_sbx \
            /imu0 > /tmp/rec_ov_${SEQ_TAG}.log 2>&1
    "
    sleep 3
    docker exec -d $CONTAINER bash -lc "
        source /opt/ros/jazzy/setup.bash
        ros2 bag play /datasets/euroc/${SEQ}_ros2/ --clock > /tmp/play_ov_${SEQ_TAG}.log 2>&1
    "
    echo "  OpenVINS running on $SEQ..."
    while docker exec $CONTAINER pgrep -f "ros2 bag play" >/dev/null 2>&1; do sleep 10; done
    sleep 3
    # Stop recorder (kill the python child, not the wrapper)
    docker exec $CONTAINER bash -lc "for pid in \$(pgrep -f 'ros2 bag record'); do kill -INT \$pid; done; sleep 5"
    echo "  OpenVINS run on $SEQ done"

    # 3b. VINS-Fusion solo
    docker exec $CONTAINER bash -lc "pkill -KILL -f run_subscribe_msckf 2>/dev/null; sleep 2; true"
    docker exec -d $CONTAINER bash -lc "
        source /opt/ros/jazzy/setup.bash
        source /ws/install/setup.bash
        ros2 run vins vins_node /ws/install/vins/share/vins/config/euroc/euroc_stereo_imu_config.yaml > /tmp/vins_${SEQ_TAG}.log 2>&1
    "
    sleep 5
    docker exec -d $CONTAINER bash -lc "
        source /opt/ros/jazzy/setup.bash
        cd /ws/runs && rm -rf euroc_${SEQ_TAG}_vins && \
        ros2 bag record -o euroc_${SEQ_TAG}_vins -s mcap \
            /odometry /path /camera_pose /keyframe_pose \
            /leica/position /vicon/firefly_sbx/firefly_sbx \
            /imu0 > /tmp/rec_vins_${SEQ_TAG}.log 2>&1
    "
    sleep 3
    docker exec -d $CONTAINER bash -lc "
        source /opt/ros/jazzy/setup.bash
        ros2 bag play /datasets/euroc/${SEQ}_ros2/ --clock > /tmp/play_vins_${SEQ_TAG}.log 2>&1
    "
    echo "  VINS-Fusion running on $SEQ..."
    while docker exec $CONTAINER pgrep -f "ros2 bag play" >/dev/null 2>&1; do sleep 10; done
    sleep 3
    docker exec $CONTAINER bash -lc "for pid in \$(pgrep -f 'ros2 bag record'); do kill -INT \$pid; done; sleep 5"
    docker exec $CONTAINER bash -lc "pkill -KILL -f vins_node 2>/dev/null; sleep 2; true"
    echo "  VINS-Fusion run on $SEQ done"
done

echo "[4/4] all runs complete. Recorded:"
ls -1d /home/behnam/ros2_ws/runs/euroc_*_ov /home/behnam/ros2_ws/runs/euroc_*_vins 2>/dev/null
