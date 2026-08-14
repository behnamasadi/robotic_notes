#!/usr/bin/env bash
# Compare LIO estimator trajectories against an external ground-truth TUM file.
# Designed for the `bag` compose profile (real-world rosbag replay) where the
# GT is shipped separately from the rosbag — e.g. Newer College Dataset GT
# CSVs under .../ground_truth/tum_format/.
#
# For the in-sim flow where /ground_truth/path is in the recorded bag, use
# eval.sh (sibling script) instead.
#
# Usage:
#   eval_bag_replay.sh <recorded-rosbag2-dir> <gt-tum-file> [out-dir]
#
# <recorded-rosbag2-dir> must contain one of these topics:
#   /Odometry                  (FAST-LIO)
#   /dlio/odom_node/odom       (DLIO)
#   /kiss/odometry             (KISS-ICP)
#   /lio_sam/mapping/odometry  (LIO-SAM)
#
# <gt-tum-file> = whitespace-separated TUM (timestamp tx ty tz qx qy qz qw).
#
# Produces in <out-dir> (default: <bag>/eval/):
#   {fast_lio,dlio,kiss_icp,lio_sam}.tum  — extracted trajectories
#   summary.txt                            — APE per estimator
#   trajectories_*.png                     — comparison plots
set -e

BAG="${1:?usage: eval_bag_replay.sh <bag-dir> <gt-tum-file> [out-dir]}"
GT="${2:?usage: eval_bag_replay.sh <bag-dir> <gt-tum-file> [out-dir]}"
OUT="${3:-${BAG}/eval}"

mkdir -p "${OUT}"

# 1. Extract odometry topics → TUM (works for mcap or sqlite3 rosbag2).
python3 - "${BAG}" "${OUT}" <<'PY'
import sys, os, glob
bag, out = sys.argv[1], sys.argv[2]
TOPICS = {
    "/Odometry":                  "fast_lio",
    "/dlio/odom_node/odom":       "dlio",
    "/kiss/odometry":             "kiss_icp",
    "/lio_sam/mapping/odometry":  "lio_sam",
}
# Try mcap first (preferred), fall back to rosbag2_py for sqlite3.
mcap_files = glob.glob(os.path.join(bag, "*.mcap"))
if mcap_files:
    from mcap_ros2.reader import read_ros2_messages
    handles = {t: open(os.path.join(out, f"{s}.tum"), "w") for t, s in TOPICS.items()}
    counts  = {t: 0 for t in TOPICS}
    for m in read_ros2_messages(mcap_files[0], topics=list(TOPICS)):
        msg = m.ros_msg
        ts  = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        handles[m.channel.topic].write(
            f"{ts:.6f} {p.x:.6f} {p.y:.6f} {p.z:.6f} "
            f"{q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n")
        counts[m.channel.topic] += 1
    for h in handles.values(): h.close()
else:
    # sqlite3 fallback — needs ROS 2 sourced (rclpy + rosbag2_py).
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id="sqlite3"),
           ConverterOptions(input_serialization_format="cdr",
                            output_serialization_format="cdr"))
    type_map = {t.name: t.type for t in r.get_all_topics_and_types()}
    handles = {t: open(os.path.join(out, f"{s}.tum"), "w") for t, s in TOPICS.items()}
    counts  = {t: 0 for t in TOPICS}
    while r.has_next():
        topic, data, _ = r.read_next()
        if topic not in TOPICS: continue
        Msg = get_message(type_map[topic])
        m = deserialize_message(data, Msg)
        ts = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p, q = m.pose.pose.position, m.pose.pose.orientation
        handles[topic].write(
            f"{ts:.6f} {p.x:.6f} {p.y:.6f} {p.z:.6f} "
            f"{q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n")
        counts[topic] += 1
    for h in handles.values(): h.close()
for t, s in TOPICS.items():
    print(f"  {s+'.tum':<15} n={counts[t]:<6} from {t}")
PY

# 2. Run evo_ape per estimator, capture summary.
SUMMARY="${OUT}/summary.txt"
: > "${SUMMARY}"
echo "ground truth: ${GT}" >> "${SUMMARY}"
echo "bag:          ${BAG}" >> "${SUMMARY}"
echo                          >> "${SUMMARY}"

# Per-estimator APE.
for est in fast_lio dlio kiss_icp lio_sam; do
    TRAJ="${OUT}/${est}.tum"
    if [ ! -s "${TRAJ}" ]; then
        echo "----- ${est} -----"                                | tee -a "${SUMMARY}"
        echo "  (no messages recorded — skipping)"               | tee -a "${SUMMARY}"
        continue
    fi
    echo "----- ${est} -----"                                    | tee -a "${SUMMARY}"
    evo_ape tum "${GT}" "${TRAJ}" -a --t_max_diff 0.05 --no_warnings 2>&1 \
        | tee -a "${SUMMARY}" \
        | grep -E "max|mean|median|min|rmse|sse|std"             || true
done

# 3. Overlay XY trajectories of all estimators + GT into one PNG.
# Skip empty .tum files (e.g. LIO-SAM when NCD's 6-axis IMU prevents it
# from publishing) — evo_traj rejects 0-line trajectory files.
TRAJS=$(find "${OUT}" -maxdepth 1 -name '*.tum' -not -name 'gt.tum' \
         -not -name 'ground_truth.tum' -size +0c | sort)
if [ -n "${TRAJS}" ]; then
    evo_traj tum ${TRAJS} --ref="${GT}" -a --t_max_diff 0.05 \
        --save_plot "${OUT}/trajectories.png" --no_warnings 2>&1 \
        | tail -3
fi

echo
echo "Wrote ${SUMMARY}"
echo "Wrote ${OUT}/trajectories_*.png"
