#!/bin/bash
set -e

# Ensure hardware device permissions for user
sudo chmod 666 /dev/hidraw* /dev/ttyUSB* /dev/ttyACM* /dev/video* 2>/dev/null || true

# Source ROS 2 Humble core environment
source /opt/ros/humble/setup.bash

# Ensure pytest wrapper binary is available in /usr/local/bin
if ! command -v pytest >/dev/null 2>&1; then
    sudo sh -c 'printf "#!/bin/sh\nexec python3 -m pytest \"\$@\"\n" > /usr/local/bin/pytest && chmod +x /usr/local/bin/pytest' 2>/dev/null || true
fi

# Ensure workspace root is in PYTHONPATH for Python imports across packages
if [ -z "$PYTHONPATH" ]; then
    export PYTHONPATH="/workspace"
else
    case ":$PYTHONPATH:" in
        *:/workspace:*) ;;
        *) export PYTHONPATH="/workspace:$PYTHONPATH" ;;
    esac
fi

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
