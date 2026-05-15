#!/usr/bin/env bash
# perf_probe.sh — 10-second snapshot of where gz-sim is spending time.
#
# Run while the sim + estimators are up. Captures three signals in one
# screen so you don't have to chase them in separate terminals:
#
#   1) Top CPU consumers (top -bn1)
#   2) Measured image rate on /rs_front/image  (ros2 topic hz)
#   3) GPU utilization                          (nvidia-smi, host-side)
#
# Usage:
#   src/explorer_r2_sim/scripts/perf_probe.sh                # default 10s
#   src/explorer_r2_sim/scripts/perf_probe.sh 30             # custom window
#
# If run from outside the sim container the CPU/GPU probes target the
# host; the ros2 topic-hz probe is run inside the container via
# `docker compose exec sim`. If run *inside* the container, only the
# CPU + topic-hz probes are useful (no nvidia-smi).
#
# Companion to docs/PERFORMANCE.md.

set -euo pipefail

WINDOW_SEC=${1:-10}

in_container() { [[ -f /.dockerenv ]]; }

probe_top() {
    echo "--- top CPU consumers (1-shot, sorted by %CPU) ---"
    top -bn1 -o %CPU | head -n 18
}

probe_topic_hz() {
    local topic=${1:-/rs_front/image}
    echo "--- ros2 topic hz $topic   (${WINDOW_SEC}s window) ---"
    if in_container; then
        timeout "${WINDOW_SEC}" ros2 topic hz "$topic" 2>&1 | tail -n 6 || true
    else
        docker compose -f ~/ros2_ws/src/explorer_r2_sim/docker-compose.yml \
            exec -T sim timeout "${WINDOW_SEC}" ros2 topic hz "$topic" 2>&1 | tail -n 6 || true
    fi
}

probe_gpu() {
    if ! command -v nvidia-smi >/dev/null 2>&1; then
        echo "--- nvidia-smi not available, skipping GPU probe ---"
        return
    fi
    echo "--- nvidia-smi (utilization sampled every 1s for ${WINDOW_SEC}s) ---"
    nvidia-smi --query-gpu=utilization.gpu,utilization.memory,memory.used,memory.total \
               --format=csv,noheader -lms 1000 \
               -i 0 -c "$WINDOW_SEC" 2>/dev/null || \
    timeout "${WINDOW_SEC}" nvidia-smi --query-gpu=utilization.gpu,utilization.memory \
               --format=csv,noheader -lms 1000 || true
}

echo "============================================================"
echo "perf_probe.sh  window=${WINDOW_SEC}s  in_container=$(in_container && echo yes || echo no)"
echo "============================================================"
probe_top
echo
probe_topic_hz /rs_front/image &
TOPIC_PID=$!
probe_gpu
wait $TOPIC_PID || true
echo
echo "Done. Compare against expectations:"
echo "  - /rs_front/image should be at the SDF nominal rate (30 Hz currently)."
echo "  - parameter_bridge should not peg a whole core per image topic."
echo "  - nvidia-smi util should be <30%% on a robust GPU; if higher the GUI is rendering."
echo "  - If RTF is low but everything looks idle, suspect DDS / lockstep — see docs/PERFORMANCE.md."
