#!/bin/bash
set -e

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
