#!/usr/bin/env bash
# Record a self-contained rosbag for offline VIO/LIO evaluation.
#
# Records both raw sensor inputs and live estimator outputs, so the bag
# can be (a) analysed as-is with scripts/analyze_bag.py, and (b) replayed
# through a different estimator config later without re-driving.
#
# Topics recorded
#   Raw sensors (for replay):
#     /imu, /lidar/points,
#     /rs_front/image,        /rs_front/camera_info,
#     /rs_front_right/image,  /rs_front_right/camera_info,
#     /tf, /tf_static
#   Live estimator outputs (for immediate analysis):
#     /ov_msckf/odomimu, /ov_msckf/pathimu      (OpenVINS)
#     /vins_estimator/odometry, /vins_estimator/path  (VINS-Fusion)
#     /Odometry, /path                          (FAST-LIO)
#   Ground-truth + commands:
#     /ground_truth/path, /ground_truth/odom,
#     /model/explorer_r2/odometry, /cmd_vel
#
# Why this exact list (and not `ros2 bag record -a`):
# `-a` will silently miss any topic that isn't yet publishing when record
# starts. The 2026-05-15 recording captured /rs_front/image but not
# /rs_front_right/image for exactly that reason — the right camera came
# up a fraction of a second later. An explicit list with --include-hidden
# fails loud if a topic is missing.
#
# Why --max-cache-size 1024MB:
# The default 100 MB cache fills in ~1 s with two 640×480 RGB streams at
# 28 Hz. When it fills, reliable QoS drops frames on the recorder's
# subscriber queue → fewer frames in the bag than were actually published.
# The 2026-05-15 run captured /rs_front/image at 10 Hz despite the live
# sim running at 28 Hz for this reason. 1 GB cache buys us ~10 s of
# burst-absorption before the writer would have to start dropping.
#
# Usage (inside the sim container, with sim + estimators already up):
#   ros2 run explorer_r2_sim gt_to_path.py &       # publishes /ground_truth/path
#   src/explorer_r2_sim/scripts/record_run.sh slow_loop
#
# Output: $OUT_DIR/run_<UTC-stamp>/  (defaults to ~/ros2_ws/runs/)
set -e

TAG="${1:-run}"
# Default to /ws/runs because that's the bind-mounted workspace on the host
# (~/ros2_ws/runs). $HOME inside the container is /root and would land the
# bag in container-local storage that gets thrown away with the container.
OUT_DIR="${OUT_DIR:-/ws/runs}"
mkdir -p "${OUT_DIR}"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
BAG_PATH="${OUT_DIR}/run_${STAMP}"

echo "[record_run] tag=${TAG}"
echo "[record_run] Writing rosbag to ${BAG_PATH}"
echo "[record_run] Stop with Ctrl-C when the drive scenario is over."

exec ros2 bag record \
    --storage mcap \
    --max-cache-size 1073741824 \
    -o "${BAG_PATH}" \
    /imu /imu_filtered \
    /cmd_vel_in \
    /lidar/points \
    /rs_front/image        /rs_front/camera_info \
    /rs_front_right/image  /rs_front_right/camera_info \
    /tf /tf_static \
    /ov_msckf/odomimu /ov_msckf/pathimu \
    /vins_estimator/odometry /vins_estimator/path \
    /Odometry /path \
    /ground_truth/path /ground_truth/odom \
    /model/explorer_r2/odometry \
    /cmd_vel
