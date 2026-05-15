#!/usr/bin/env bash
# recording_health.sh — one-shot live sanity check during a recording.
#
# Verifies the three things that bit us in run_20260515T103115Z:
#   1) /rs_front_right/image is actually publishing (not just /rs_front/image)
#   2) OpenVINS is wired up in stereo (uses both image topics)
#   3) GPU is being used — confirms gz-rendering is on NVIDIA EGL,
#      not falling back to llvmpipe
#
# Run from the host (uses docker exec + nvidia-smi):
#   src/explorer_r2_sim/scripts/recording_health.sh
#
# 10-second probe by default; pass an integer arg to override:
#   src/explorer_r2_sim/scripts/recording_health.sh 30
#
# Designed to be safe to run during an active recording — it only reads.
set -eu

WINDOW=${1:-10}
SIM_CONTAINER="${SIM_CONTAINER:-explorer_r2_sim-sim-1}"

if ! docker ps --format '{{.Names}}' | grep -q "^${SIM_CONTAINER}$"; then
    # Fall back to whatever sim container is up
    SIM_CONTAINER=$(docker ps --format '{{.Names}}' | grep -E '(sim|explorer)' | head -n 1 || true)
    if [ -z "${SIM_CONTAINER}" ]; then
        echo "ERROR: no sim container is running" >&2
        exit 1
    fi
fi
echo "============================================================"
echo "recording_health.sh   container=${SIM_CONTAINER}   window=${WINDOW}s"
echo "============================================================"

echo
echo "--- (1) STEREO IMAGES PUBLISHING? ---"
docker exec "${SIM_CONTAINER}" bash -lc "
source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash 2>/dev/null
echo '/rs_front/image:'
timeout ${WINDOW} ros2 topic hz /rs_front/image 2>&1 | tail -n 2
echo
echo '/rs_front_right/image:'
timeout ${WINDOW} ros2 topic hz /rs_front_right/image 2>&1 | tail -n 2
"

echo
echo "--- (2) OPENVINS WIRED UP IN STEREO? ---"
docker exec "${SIM_CONTAINER}" bash -lc "
source /opt/ros/jazzy/setup.bash 2>/dev/null
# subscribers on the right-camera topic; ov_msckf should be in the list
echo 'Subscribers on /rs_front_right/image (ov_msckf should be present):'
ros2 topic info /rs_front_right/image --verbose 2>&1 | grep -A2 'Subscription count'
ros2 topic info /rs_front_right/image --verbose 2>&1 | grep -E 'Node name|Topic type' | head -n 10
"

echo
echo "--- (3) RENDERING ON THE GPU? ---"
echo 'nvidia-smi (host):'
nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total --format=csv,noheader || echo '  (nvidia-smi not available)'
echo
echo 'eglinfo renderer (inside container):'
docker exec "${SIM_CONTAINER}" bash -lc "eglinfo 2>/dev/null | grep -E 'EGL vendor|OpenGL core profile renderer' | head -n 4"
echo
echo 'Top CPU consumers inside container (expect ruby=gz-sim near the top):'
docker exec "${SIM_CONTAINER}" bash -lc "top -bn1 -o %CPU | head -n 12 | tail -n 8"

echo
echo "============================================================"
echo "INTERPRETATION:"
echo "  - Both image streams should be >20 Hz."
echo "  - /rs_front_right/image should have an 'ov_msckf' subscriber."
echo "  - nvidia-smi GPU util should be >10% (was 40% on 2026-05-15"
echo "    headless test). If 0%, gz-sim is software-rendering on CPU."
echo "  - eglinfo should say 'NVIDIA', not 'Mesa' / 'llvmpipe'."
echo "============================================================"
