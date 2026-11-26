# ==============================================================================
# Single xArm 1S Color-Sorting Robot - Development Container
# Base: ROS 2 Humble Hawksbill (Ubuntu 22.04 LTS)
# Multi-arch support: x86_64 (Laptop with Intel graphics) and aarch64 (Jetson)
# ==============================================================================
FROM osrf/ros:humble-desktop

ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Install core development tools, Mesa OpenGL for Intel GPU, OpenCV, and hardware serial tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    curl \
    git \
    gnupg2 \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    locales \
    lsb-release \
    mesa-utils \
    net-tools \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-serial \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-v4l2-camera \
    sudo \
    udev \
    usbutils \
    v4l-utils \
    wget \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# Install common Python robotics/vision libraries
RUN pip3 install --no-cache-dir \
    numpy \
    opencv-python-headless \
    pyserial \
    pytest \
    scipy

# Create a non-root user matching the host UID/GID to preserve file ownership on mounted volumes
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && usermod -aG dialout,video,plugdev $USERNAME \
    && (getent group render >/dev/null && usermod -aG render $USERNAME || true)

# Configure bashrc to automatically source ROS 2 and workspace overlay
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo 'if [ -f "/workspace/ros2_ws/install/setup.bash" ]; then source "/workspace/ros2_ws/install/setup.bash"; fi' >> /home/$USERNAME/.bashrc

# Initialize rosdep for the container user
USER $USERNAME
RUN rosdep update || true

# Entrypoint setup
USER root
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER $USERNAME
WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
