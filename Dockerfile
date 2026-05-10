# ROS 2 + Gazebo dev image, parameterized via build ARGs.
# Defaults below match a `docker build .` invocation; `docker compose build`
# overrides them with values from .env.
#
# Tested pairings:
#   ROS_DISTRO=humble  GZ_PKG=ignition-fortress  GZ_VERSION=fortress
#   ROS_DISTRO=jazzy   GZ_PKG=gz-harmonic        GZ_VERSION=harmonic

ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop

ARG ROS_DISTRO
ARG GZ_PKG=ignition-fortress
ARG GZ_VERSION=fortress

# Add a non-root user with a UID/GID matching the host
ARG USERNAME=behnam
ARG USER_UID=1000
ARG USER_GID=$USER_UID

SHELL ["/bin/bash", "-c"]

# 1. Remove default UID/GID 1000 user (if exists)
RUN set -e && \
    (id -u 1000 >/dev/null 2>&1 && userdel -r $(id -un 1000)) || true && \
    (getent group 1000 >/dev/null 2>&1 && groupdel $(getent group 1000 | cut -d: -f1)) || true

# 2. Create the user
RUN apt-get update && apt-get install -y --no-install-recommends sudo && \
    rm -rf /var/lib/apt/lists/* && \
    groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} \
            --create-home --shell /bin/bash ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME} && \
    usermod -aG dialout,video ${USERNAME}

RUN mkdir -p /workspace && \
    chown -R ${USER_UID}:${USER_GID} /workspace

USER ${USERNAME}
WORKDIR /home/${USERNAME}
ENV USER=${USERNAME}

# Development tools and common ROS 2 packages
RUN sudo apt update && sudo apt upgrade -y && \
    sudo apt install -y --no-install-recommends \
      ros-dev-tools \
      python3-pip \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-vcstool \
      python3-argcomplete \
      git build-essential vim nano curl wget lsb-release gnupg \
      x11-apps xauth mesa-utils \
      libceres-dev libsuitesparse-dev \
      ros-${ROS_DISTRO}-libg2o \
      ros-${ROS_DISTRO}-bond \
      ros-${ROS_DISTRO}-bondcpp \
      ros-${ROS_DISTRO}-nav2-map-server \
      ros-${ROS_DISTRO}-nav2-bringup \
      ros-${ROS_DISTRO}-navigation2 \
      ros-${ROS_DISTRO}-urdf-launch \
      ros-${ROS_DISTRO}-rviz2 \
      ros-${ROS_DISTRO}-slam-toolbox \
      ros-${ROS_DISTRO}-robot-state-publisher \
      ros-${ROS_DISTRO}-xacro \
      ros-${ROS_DISTRO}-tf2-tools \
      ros-${ROS_DISTRO}-tf-transformations \
      ros-${ROS_DISTRO}-teleop-twist-keyboard \
      ros-${ROS_DISTRO}-rqt \
      ros-${ROS_DISTRO}-rqt-common-plugins && \
    sudo apt clean && sudo rm -rf /var/lib/apt/lists/*

RUN rosdep update

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Add OSRF Gazebo apt repo + install Gazebo and the matching ros_gz binding
RUN sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
        --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    sudo apt update && \
    sudo apt install -y --no-install-recommends \
      ${GZ_PKG} \
      ros-${ROS_DISTRO}-ros-gz \
      ros-${ROS_DISTRO}-ros-gz-bridge \
      ros-${ROS_DISTRO}-ros-gz-sim \
      ros-${ROS_DISTRO}-ros-gz-interfaces && \
    sudo apt clean && sudo rm -rf /var/lib/apt/lists/*

# GZ_VERSION + colored prompt for both interactive shells and non-login shells
RUN echo "export GZ_VERSION=${GZ_VERSION}" >> ~/.bashrc && \
    sudo bash -c "echo '# ROS 2 ${ROS_DISTRO} prompt' >> /etc/bash.bashrc && \
    echo 'export PS1=\"\[\e[38;5;208m\][ROS2-${ROS_DISTRO} \u@\h] \[\e[0;33m\]\w\[\e[0m\] \$ \"' >> /etc/bash.bashrc && \
    echo 'export GZ_VERSION=${GZ_VERSION}' >> /etc/bash.bashrc"

# Fortress used the `ign` CLI; Garden+ use `gz`. When building a Fortress image,
# install a tiny shim so `gz sim` etc. keeps working in the docs/examples.
RUN if [ "${GZ_VERSION}" = "fortress" ]; then \
      sudo bash -c 'echo "#!/bin/bash"                                                       >  /usr/local/bin/gz && \
                    echo "# Maps gz sim -> ign gazebo for Gazebo Fortress compatibility"     >> /usr/local/bin/gz && \
                    echo "if [ \"\$1\" = \"sim\" ]; then"                                    >> /usr/local/bin/gz && \
                    echo "    shift"                                                         >> /usr/local/bin/gz && \
                    echo "    exec ign gazebo \"\$@\""                                       >> /usr/local/bin/gz && \
                    echo "else"                                                              >> /usr/local/bin/gz && \
                    echo "    exec ign \"\$@\""                                              >> /usr/local/bin/gz && \
                    echo "fi"                                                                >> /usr/local/bin/gz && \
                    chmod +x /usr/local/bin/gz' ; \
    fi

# Entrypoint: source ROS, set GZ_VERSION, source workspace overlay if present
RUN sudo bash -c "echo '#!/bin/bash'                                            >  /entrypoint.sh && \
                  echo 'source /opt/ros/${ROS_DISTRO}/setup.bash'                >> /entrypoint.sh && \
                  echo 'export GZ_VERSION=${GZ_VERSION}'                         >> /entrypoint.sh && \
                  echo 'if [ -f /workspace/install/setup.bash ]; then'           >> /entrypoint.sh && \
                  echo '    source /workspace/install/setup.bash'                >> /entrypoint.sh && \
                  echo 'fi'                                                      >> /entrypoint.sh && \
                  echo 'exec \"\$@\"'                                            >> /entrypoint.sh && \
                  chmod +x /entrypoint.sh"

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
