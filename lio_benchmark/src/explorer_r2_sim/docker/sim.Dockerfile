# Dev container: ROS 2 desktop + Gazebo + ros_gz_bridge + RViz + teleop.
# Source code lives on the host in ~/ros2_ws and is bind-mounted into /ws at
# runtime by docker-compose.yml. Colcon build happens inside the container
# on first start (see docker/sim-entrypoint.sh). Optional submodules
# (third_party/open_vins, third_party/FAST_LIO) are picked up automatically
# if they've been initialised on the host.
ARG ROS_DISTRO=jazzy
FROM osrf/ros:${ROS_DISTRO}-desktop-full

ARG ROS_DISTRO
ARG GZ_PKG=gz-harmonic
ARG DEBIAN_FRONTEND=noninteractive

# OSRF Gazebo apt repo + Gazebo + ros_gz packages.
# xterm: cave.launch.py teleop:=true opens teleop_twist_keyboard in an xterm.
# git + sed: used by the entrypoint when patching OpenVINS for Jazzy.
# OpenVINS build-time deps: libceres, libeigen3, libboost, image_transport,
#   cv_bridge, tf2_geometry_msgs.
# FAST_LIO build-time deps: libpcl, pcl_conversions, pcl_ros.
RUN apt-get update && apt-get install -y --no-install-recommends \
      curl gnupg lsb-release ca-certificates xterm git \
 && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list \
 && apt-get update && apt-get install -y --no-install-recommends \
      ${GZ_PKG} \
      ros-${ROS_DISTRO}-ros-gz-bridge \
      ros-${ROS_DISTRO}-ros-gz-sim \
      ros-${ROS_DISTRO}-ros-gz-image \
      ros-${ROS_DISTRO}-teleop-twist-keyboard \
      ros-${ROS_DISTRO}-joy \
      ros-${ROS_DISTRO}-teleop-twist-joy \
      ros-${ROS_DISTRO}-rqt-robot-steering \
      ros-${ROS_DISTRO}-rqt \
      ros-${ROS_DISTRO}-rqt-graph \
      libceres-dev libeigen3-dev libboost-all-dev \
      ros-${ROS_DISTRO}-image-transport \
      ros-${ROS_DISTRO}-image-transport-plugins \
      ros-${ROS_DISTRO}-tf2-geometry-msgs \
      ros-${ROS_DISTRO}-cv-bridge \
      libpcl-dev \
      ros-${ROS_DISTRO}-pcl-conversions \
      ros-${ROS_DISTRO}-pcl-ros \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
 && rm -rf /var/lib/apt/lists/*

# GLVND + EGL + GL + glxinfo so gz-rendering's Ogre2 backend can reach
# the NVIDIA driver libraries that nvidia-container-toolkit mounts at
# runtime. Without libglvnd0 the EGL loader can't find the nvidia vendor
# ICD at /usr/share/glvnd/egl_vendor.d/10_nvidia.json and Ogre2 silently
# falls back to Mesa's llvmpipe (software rasterizer) — the 2026-05-15
# measurement had the 3090 at 0 % util while `ruby` (gz-sim) burned 200 %
# CPU. mesa-utils ships glxinfo / eglinfo for diagnosing regressions.
RUN apt-get update && apt-get install -y --no-install-recommends \
      libglvnd0 libglvnd-dev libegl1 libgl1 libgles2 libglx0 \
      mesa-utils \
 && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=${ROS_DISTRO}
WORKDIR /ws

# The bind-mounted workspace is owned by the host user (uid 1000), but the
# container runs as root. Git's "dubious ownership" check then refuses to
# operate on third_party/* submodules. Allow git to trust any directory —
# safe in this container because it's a single-user dev environment.
RUN git config --global --add safe.directory '*'

# Source ROS (and the workspace overlay if it exists) in /root/.bashrc so
# `docker compose exec sim bash` drops you into a shell with `ros2` and
# the explorer_r2_sim package already on the path. The entrypoint also
# sources these for the CMD process, but compose exec bypasses the
# entrypoint — without this, exec'd commands hit "ros2: not found".
RUN printf '%s\n' \
      'source /opt/ros/${ROS_DISTRO}/setup.bash' \
      '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' \
      >> /root/.bashrc

COPY docker/sim-entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "explorer_r2_sim", "cave.launch.py"]
