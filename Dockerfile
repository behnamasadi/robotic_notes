# Dockerfile for ROS2 Humble with Gazebo Fortress
# Based on osrf/ros:humble-desktop (Ubuntu 22.04)

FROM osrf/ros:humble-desktop

# Add a non-root user with a specified UID and GID matching the host system
ARG USERNAME=behnam
ARG USER_UID=1000
ARG USER_GID=$USER_UID

SHELL ["/bin/bash", "-c"]

# 1. Remove default UID/GID 1000 user (if exists)
RUN set -e && \
    (id -u 1000 >/dev/null 2>&1 && userdel -r $(id -un 1000)) || true && \
    (getent group 1000 >/dev/null 2>&1 && groupdel $(getent group 1000 | cut -d: -f1)) || true

# 2. Create your user
RUN apt-get update && apt-get install -y --no-install-recommends sudo && \
    rm -rf /var/lib/apt/lists/* && \
    groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} \
            --create-home --shell /bin/bash ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME} && \
    usermod -aG dialout,video ${USERNAME}

# Preemptively fix permissions for the user during build
RUN mkdir -p /workspace && \
    chown -R ${USER_UID}:${USER_GID} /workspace

# Set the working directory and user
USER ${USERNAME}
WORKDIR /home/${USERNAME}
ENV USER=${USERNAME}

# Install development tools and common ROS packages
RUN sudo apt update && sudo apt upgrade -y && \
    sudo apt install -y --no-install-recommends \
    ros-dev-tools \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-argcomplete \
    git \
    build-essential \
    vim \
    nano \
    curl \
    wget \
    lsb-release \
    gnupg \
    x11-apps \
    xauth \
    mesa-utils \
    ros-humble-libg2o \
    libceres-dev \
    libsuitesparse-dev \
    ros-humble-bond \
    ros-humble-bondcpp \
    ros-humble-nav2-map-server \
    ros-humble-nav2-bringup \
    ros-humble-navigation2 \
    ros-humble-urdf-launch \
    ros-humble-simulation-interfaces \
    ros-humble-rviz2 \
    ros-humble-slam-toolbox \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-tf2-tools \
    ros-humble-tf-transformations \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins && \
    sudo apt clean && sudo rm -rf /var/lib/apt/lists/*

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Add Gazebo Fortress repository and install
RUN sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    sudo apt update && \
    sudo apt install -y --no-install-recommends \
    libignition-gazebo6-dev \
    libignition-gazebo6 \
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-interfaces && \
    sudo apt clean && sudo rm -rf /var/lib/apt/lists/*

# Set up Gazebo version environment variable and custom prompt
RUN echo "export GZ_VERSION=fortress" >> ~/.bashrc && \
    sudo bash -c 'echo "# Orange ROS2-Humble prompt" >> /etc/bash.bashrc && \
    echo "export PS1=\"\[\e[38;5;208m\][ROS2-Humble \u@\h] \[\e[0;33m\]\w\[\e[0m\] \$ \"" >> /etc/bash.bashrc && \
    echo "export GZ_VERSION=fortress" >> /etc/bash.bashrc'

# Create gz wrapper script for backward compatibility (gz -> ign)
# Maps gz sim -> ign gazebo, gz model -> ign model, etc.
RUN sudo bash -c 'echo "#!/bin/bash" > /usr/local/bin/gz && \
    echo "# Wrapper to map gz commands to ign commands for Gazebo Fortress compatibility" >> /usr/local/bin/gz && \
    echo "if [ \"\$1\" = \"sim\" ]; then" >> /usr/local/bin/gz && \
    echo "    shift" >> /usr/local/bin/gz && \
    echo "    exec ign gazebo \"\$@\"" >> /usr/local/bin/gz && \
    echo "else" >> /usr/local/bin/gz && \
    echo "    exec ign \"\$@\"" >> /usr/local/bin/gz && \
    echo "fi" >> /usr/local/bin/gz && \
    chmod +x /usr/local/bin/gz'

# Create entrypoint script to source ROS2 setup
RUN sudo bash -c 'echo "#!/bin/bash" > /entrypoint.sh && \
    echo "source /opt/ros/humble/setup.bash" >> /entrypoint.sh && \
    echo "export GZ_VERSION=fortress" >> /entrypoint.sh && \
    echo "if [ -f /workspace/install/setup.bash ]; then" >> /entrypoint.sh && \
    echo "    source /workspace/install/setup.bash" >> /entrypoint.sh && \
    echo "fi" >> /entrypoint.sh && \
    echo "exec \"\$@\"" >> /entrypoint.sh && \
    chmod +x /entrypoint.sh'

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]