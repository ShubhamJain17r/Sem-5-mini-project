#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
IMAGE_NAME="xarm-sorting:humble"

# Allow local X11 display connections for GUI applications (RViz2, OpenCV)
if command -v xhost >/dev/null 2>&1; then
    xhost +local:root >/dev/null 2>&1 || true
    xhost +local:"$(whoami)" >/dev/null 2>&1 || true
fi

# Intel GPU / Mesa DRI options
DRI_FLAGS=()
if [ -d "/dev/dri" ]; then
    DRI_FLAGS=(--device /dev/dri:/dev/dri)
fi

# Video / Camera options (Logitech C270/C920)
VIDEO_FLAGS=()
for dev in /dev/video*; do
    if [ -e "$dev" ]; then
        VIDEO_FLAGS+=(--device "$dev":"$dev")
    fi
done

# Serial / UART options (xArm controllers)
SERIAL_FLAGS=()
for dev in /dev/ttyUSB* /dev/ttyACM* /dev/xarm_*; do
    if [ -e "$dev" ]; then
        SERIAL_FLAGS+=(--device "$dev":"$dev")
    fi
done

echo "Starting ROS 2 Humble container for ${REPO_ROOT}..."

docker run -it --rm \
    --net=host \
    --ipc=host \
    --privileged \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    "${DRI_FLAGS[@]}" \
    "${VIDEO_FLAGS[@]}" \
    "${SERIAL_FLAGS[@]}" \
    -v "${REPO_ROOT}:/workspace" \
    -w /workspace \
    --name xarm_sorting_env \
    "${IMAGE_NAME}" \
    "$@"
