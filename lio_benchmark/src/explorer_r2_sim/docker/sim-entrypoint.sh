#!/usr/bin/env bash
# Sim container entrypoint.
#
# Expects the host's ros2_ws to be bind-mounted at /ws (see compose.yml).
# Steps each container start does:
#   1. Wire optional submodules from /ws/third_party/* into /ws/src/* via
#      symlinks so colcon picks them up.
#   2. Apply Jazzy-compatibility header patches to OpenVINS in-place
#      (idempotent — sed is a no-op once .hpp form is in the files).
#   3. Run `colcon build` if /ws/install isn't populated yet, or if the
#      user passes BUILD=force in the env. Otherwise reuse the cached
#      build.
#   4. Source the overlay and exec whatever was passed as CMD.
#
# Env knobs:
#   BUILD=force   rebuild from scratch even if install/ exists
#   BUILD=skip    never build (assume install/ is up to date)
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"

# Persist Fuel cache between runs so cave tiles aren't re-downloaded.
mkdir -p /root/.gz/fuel /root/.ignition/fuel

cd /ws
mkdir -p src

PKGS=(explorer_r2_sim)

# ─── Optional: OpenVINS (VIO) ───────────────────────────────────────────
if [ -e third_party/open_vins/ov_msckf ]; then
    # Reset the patched files first so the sed patches stay idempotent
    # across container restarts. The naive `\.h` → `.hpp` pattern below
    # would match inside an already-patched `.hpp` substring and keep
    # appending `p`s — past bug. Resetting from git before each patch
    # makes the whole step deterministic.
    git -C third_party/open_vins checkout -- \
        ov_msckf/src/ros/ROS2Visualizer.h \
        ov_msckf/src/ros/ROS1Visualizer.h \
        ov_msckf/src/ros/ROSVisualizerHelper.h \
        ov_core/src/test_tracking.cpp 2>/dev/null || true

    sed -i 's|image_transport/image_transport\.h>|image_transport/image_transport.hpp>|' \
        third_party/open_vins/ov_msckf/src/ros/ROS2Visualizer.h \
        third_party/open_vins/ov_msckf/src/ros/ROS1Visualizer.h
    sed -i 's|tf2_geometry_msgs/tf2_geometry_msgs\.h>|tf2_geometry_msgs/tf2_geometry_msgs.hpp>|' \
        third_party/open_vins/ov_msckf/src/ros/ROSVisualizerHelper.h \
        third_party/open_vins/ov_msckf/src/ros/ROS2Visualizer.h \
        third_party/open_vins/ov_msckf/src/ros/ROS1Visualizer.h
    sed -i 's|cv_bridge/cv_bridge\.h>|cv_bridge/cv_bridge.hpp>|' \
        third_party/open_vins/ov_msckf/src/ros/ROS1Visualizer.h \
        third_party/open_vins/ov_msckf/src/ros/ROS2Visualizer.h \
        third_party/open_vins/ov_core/src/test_tracking.cpp
    ln -sfn /ws/third_party/open_vins /ws/src/open_vins
    PKGS+=(ov_core ov_init ov_msckf)
else
    rm -f /ws/src/open_vins
fi

# ─── Optional: VINS-Fusion (alternative VIO) ────────────────────────────
# zinuok/VINS-Fusion-ROS2 ships four ament packages under one repo:
# camera_models, vins, loop_fusion, global_fusion. Symlink the whole
# repo so colcon picks them up; we only need camera_models + vins for
# the basic stereo VIO. loop_fusion + global_fusion are optional.
#
# VINS-Fusion writes pose-graph + logs to output_path at runtime;
# create it before vins_node starts.
if [ -e third_party/VINS-Fusion-ROS2/vins/package.xml ]; then
    ln -sfn /ws/third_party/VINS-Fusion-ROS2/camera_models  /ws/src/vins_camera_models
    ln -sfn /ws/third_party/VINS-Fusion-ROS2/vins           /ws/src/vins
    # loop_fusion + global_fusion are present but not required for the
    # basic stereo VIO comparison. Uncomment to add them.
    # ln -sfn /ws/third_party/VINS-Fusion-ROS2/loop_fusion   /ws/src/vins_loop_fusion
    # ln -sfn /ws/third_party/VINS-Fusion-ROS2/global_fusion /ws/src/vins_global_fusion
    mkdir -p /ws/runs/vins_output
    PKGS+=(camera_models vins)
else
    rm -f /ws/src/vins_camera_models /ws/src/vins
fi

# ─── Optional: FAST_LIO (LIO) ───────────────────────────────────────────
if [ -e third_party/FAST_LIO/package.xml ]; then
    # FAST_LIO has a nested submodule (include/ikd-Tree). If the user
    # cloned the parent submodule without --recursive, the nested one
    # is missing and the build fails with "Cannot find source file:
    # include/ikd-Tree/ikd_Tree.cpp". Auto-fix.
    if [ ! -f third_party/FAST_LIO/include/ikd-Tree/ikd_Tree.cpp ]; then
        echo "[entrypoint] Fetching nested submodules under third_party/FAST_LIO/"
        git -C third_party/FAST_LIO submodule update --init --recursive
    fi
    # FAST_LIO hard-codes C++14 in CMakeLists.txt (set 5 different ways).
    # rclcpp on Jazzy uses C++17 features (std::is_convertible_v) inside
    # the RCLCPP_INFO/WARN macros, so the build fails. Bump to C++17.
    # Idempotent — sed on already-patched file is a no-op.
    sed -i 's/c++14/c++17/g; s/CXX_STANDARD 14/CXX_STANDARD 17/g' \
        third_party/FAST_LIO/CMakeLists.txt
    ln -sfn /ws/third_party/FAST_LIO /ws/src/fast_lio
    # FAST_LIO unconditionally find_package()'s livox_ros_driver2. We
    # don't have a Livox device — we feed it /lidar/points from the gz
    # gpu_lidar (lidar_type: 3 / Ouster) — but we still need the
    # CustomMsg type to be defined for FAST_LIO to compile. The shim
    # at src/livox_ros_driver2_msgs provides only the message types
    # (no SDK), which is enough.
    PKGS+=(livox_ros_driver2 fast_lio)
else
    rm -f /ws/src/fast_lio
fi

# ─── Build ──────────────────────────────────────────────────────────────
# Is the install/ tree complete? Check every required package's install
# dir, not just install/setup.bash — a previously-failed build can leave
# the setup script in place while the package itself is missing.
needs_build=false
if [ ! -f install/setup.bash ]; then
    needs_build=true
else
    for pkg in "${PKGS[@]}"; do
        if [ ! -d "install/${pkg}" ]; then
            echo "[entrypoint] install/ is incomplete (missing ${pkg}) — will rebuild"
            needs_build=true
            break
        fi
    done
fi

case "${BUILD:-auto}" in
    skip)
        echo "[entrypoint] BUILD=skip — using existing install/ (no completeness check)"
        ;;
    force)
        echo "[entrypoint] BUILD=force — full rebuild: ${PKGS[*]}"
        rm -rf build install log
        colcon build --symlink-install --packages-select "${PKGS[@]}"
        ;;
    *)
        if [ "${needs_build}" = "true" ]; then
            echo "[entrypoint] Building: ${PKGS[*]}"
            colcon build --symlink-install --packages-select "${PKGS[@]}"
        else
            echo "[entrypoint] install/ is complete — skipping build (set BUILD=force to rebuild)"
        fi
        ;;
esac

source install/setup.bash
exec "$@"
