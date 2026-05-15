#!/usr/bin/env bash
# Run evo metrics on a recorded rosbag and print a one-line summary per
# estimator. Produces PNG plots alongside the bag.
#
# Reference trajectory: /ground_truth/path (TUM-converted)
# Estimator trajectories:
#   - /ov_msckf/pathimu  (OpenVINS VIO)
#   - /path              (FAST_LIO)
#
# Usage (on host or inside the container — needs `evo` installed):
#   pip install evo --user
#   ros2 run explorer_r2_sim eval.sh ~/.local/share/evo/slow_loop/20260514T120000Z
#
# Produces:
#   <bag>/gt.tum, <bag>/vio.tum, <bag>/lio.tum
#   <bag>/vio_ape.png, <bag>/lio_ape.png
#   <bag>/summary.txt — RMSE / mean / median per estimator
set -e

BAG="${1:?usage: eval.sh <bag-dir>}"
if [ ! -d "${BAG}" ]; then
    echo "[eval] No such bag dir: ${BAG}" >&2
    exit 1
fi

cd "${BAG}"
SUMMARY="${BAG}/summary.txt"
: > "${SUMMARY}"

echo "[eval] Extracting trajectories from $(basename "${BAG}")"
evo_traj bag2 . --topic_names /ground_truth/path --save_as_tum \
    --logfile /dev/null 2>/dev/null || true
mv -f ground_truth_path.tum gt.tum 2>/dev/null || true

evo_traj bag2 . --topic_names /ov_msckf/pathimu --save_as_tum \
    --logfile /dev/null 2>/dev/null || true
mv -f ov_msckf_pathimu.tum vio.tum 2>/dev/null || true

evo_traj bag2 . --topic_names /path --save_as_tum \
    --logfile /dev/null 2>/dev/null || true
mv -f path.tum lio.tum 2>/dev/null || true

if [ ! -f gt.tum ]; then
    echo "[eval] No ground truth in bag — was gt_to_path.py running?" >&2
    exit 2
fi

run_metric() {
    local label="$1" est="$2"
    if [ ! -f "${est}" ]; then
        echo "[eval] Skip ${label}: ${est} not in bag"
        return
    fi
    echo "===== ${label} =====" | tee -a "${SUMMARY}"
    evo_ape tum gt.tum "${est}" -va --save_plot "${label}_ape.png" 2>&1 \
        | tee -a "${SUMMARY}" \
        | grep -E "rmse|mean|median|std|min|max" || true
    evo_rpe tum gt.tum "${est}" --delta 1 --delta_unit m -va 2>&1 \
        | tee -a "${SUMMARY}" \
        | grep -E "rmse|mean|median|std|min|max" || true
}

run_metric vio vio.tum
run_metric lio lio.tum

echo "[eval] Wrote ${SUMMARY} and ${BAG}/*_ape.png"
