# LIO-SAM on ROS Jazzy. Built specifically to match the sim container's
# ROS distribution so cross-container DDS delivers messages (no
# Humble↔Jazzy type-hash incompatibility — that's the whole reason for
# this port).
#
# GTSAM 4.1 + unstable is built from source because (a) LIO-SAM uses
# `gtsam_unstable::IncrementalFixedLagSmoother` (in src/imuPreintegration.cpp),
# (b) Noble's apt ships libgtsam-dev 4.2 only and no libgtsam-unstable-dev,
# (c) the borglab PPA only releases for Jammy.

FROM osrf/ros:jazzy-desktop

ARG DEBIAN_FRONTEND=noninteractive

# Build deps for GTSAM + LIO-SAM.
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      curl git python3-colcon-common-extensions \
      cmake build-essential \
      libboost-all-dev libeigen3-dev libtbb-dev \
      libceres-dev libopencv-dev libpcl-dev \
      ros-${ROS_DISTRO}-pcl-ros \
      ros-${ROS_DISTRO}-pcl-conversions \
      ros-${ROS_DISTRO}-perception-pcl \
      ros-${ROS_DISTRO}-pcl-msgs \
      ros-${ROS_DISTRO}-tf2-eigen \
      ros-${ROS_DISTRO}-tf2-geometry-msgs \
      ros-${ROS_DISTRO}-tf2-sensor-msgs \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
      ros-${ROS_DISTRO}-xacro \
 && rm -rf /var/lib/apt/lists/*

# Build GTSAM 4.1 + unstable from source. -DGTSAM_USE_SYSTEM_EIGEN=ON
# so we link against Noble's libeigen3-dev (same Eigen as everything
# else in the workspace). -DGTSAM_BUILD_UNSTABLE=ON because LIO-SAM's
# imuPreintegration includes IncrementalFixedLagSmoother from
# gtsam_unstable. Tests/examples/Python wrapper turned off to save
# build time and disk.
RUN git clone --depth 1 --branch 4.1.1 https://github.com/borglab/gtsam.git /tmp/gtsam \
 && mkdir /tmp/gtsam/build \
 && cd /tmp/gtsam/build \
 && cmake \
      -DCMAKE_BUILD_TYPE=Release \
      -DGTSAM_BUILD_TESTS=OFF \
      -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
      -DGTSAM_BUILD_PYTHON=OFF \
      -DGTSAM_INSTALL_GEOGRAPHICLIB=OFF \
      -DGTSAM_USE_SYSTEM_EIGEN=ON \
      -DGTSAM_BUILD_UNSTABLE=ON \
      -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
      .. \
 && make -j$(nproc) install \
 && ldconfig \
 && cd / && rm -rf /tmp/gtsam

SHELL ["/bin/bash", "-c"]

# Clone LIO-SAM and apply the Eigen→Eigen3 patches the upstream
# CMakeLists needs (their bare "Eigen" doesn't resolve on Ubuntu where
# the package is "Eigen3"). Same idempotent-sed pattern we use for
# FAST-LIO's C++14 → C++17 fix in sim-entrypoint.sh.
RUN mkdir -p /root/ros2_ws/src \
 && cd /root/ros2_ws/src \
 && git clone --branch ros2 https://github.com/TixiaoShan/LIO-SAM.git \
 && sed -i \
      -e 's/find_package(Eigen REQUIRED)/find_package(Eigen3 REQUIRED)/' \
      -e 's/ GTSAM Eigen)/ GTSAM Eigen3)/' \
      /root/ros2_ws/src/LIO-SAM/CMakeLists.txt \
 && cd /root/ros2_ws \
 && source /opt/ros/${ROS_DISTRO}/setup.bash \
 && colcon build

# Default to Cyclone at runtime so both containers speak the same RMW.
# (Now redundant since both are Jazzy and the type-hash issue is gone,
# but staying consistent with the sim container's choice.)
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WORKDIR /root/ros2_ws
