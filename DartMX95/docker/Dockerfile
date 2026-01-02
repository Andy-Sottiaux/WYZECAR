# Ubuntu 22.04 + ROS2 Humble environment for DART-MX95 development
ARG UBUNTU_VERSION=22.04
FROM ubuntu:${UBUNTU_VERSION}

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=humble \
    WORKSPACE_DIR=/workspace/wyzecar_ws

# Base tooling and locale setup
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        locales \
        curl \
        gnupg2 \
        lsb-release \
        build-essential \
        git \
        python3-pip \
        sudo && \
    locale-gen en_US en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*

# ROS 2 apt repository and packages
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-desktop \
        python3-argcomplete \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool && \
    rm -rf /var/lib/apt/lists/*

# Initialize rosdep database (ignore if already initialized)
RUN rosdep init 2>/dev/null || true

# Default workspace location inside container
WORKDIR ${WORKSPACE_DIR}

# Entry point sets up ROS environment automatically
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
