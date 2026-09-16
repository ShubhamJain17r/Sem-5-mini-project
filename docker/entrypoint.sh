#!/bin/bash
set -e

# Ensure hardware device permissions for user
sudo chmod 666 /dev/hidraw* /dev/ttyUSB* /dev/ttyACM* /dev/video* 2>/dev/null || true

# Source ROS 2 Humble core environment
source /opt/ros/humble/setup.bash

# Source local workspace overlay if built
if [ -f "/workspace/ros2_ws/install/setup.bash" ]; then
    source "/workspace/ros2_ws/install/setup.bash"
fi

# If no command provided, default to bash
if [ $# -eq 0 ]; then
    exec bash
else
    exec "$@"
fi
