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

# Serial / UART & USB HID options (xArm controllers)
SERIAL_FLAGS=()
for dev in /dev/ttyUSB* /dev/ttyACM* /dev/xarm_* /dev/hidraw*; do
    if [ -e "$dev" ]; then
        SERIAL_FLAGS+=(--device "$dev":"$dev")
    fi
done

CONTAINER_NAME="xarm_sorting_env"

# If container is already running, open a new shell inside it
if [ "$(docker inspect -f '{{.State.Running}}' "${CONTAINER_NAME}" 2>/dev/null)" = "true" ]; then
    echo "Container '${CONTAINER_NAME}' is already running. Opening new terminal session inside container..."
    if [ $# -eq 0 ]; then
        exec docker exec -it "${CONTAINER_NAME}" /entrypoint.sh bash
    else
        exec docker exec -it "${CONTAINER_NAME}" /entrypoint.sh "$@"
    fi
fi

# Auto-clean any stopped/stale container with the same name before starting
docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true

echo "Starting ROS 2 Humble container for ${REPO_ROOT}..."

docker run -it --rm \
    --net=host \
    --ipc=host \
    --privileged \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /dev:/dev \
    "${DRI_FLAGS[@]}" \
    "${VIDEO_FLAGS[@]}" \
    "${SERIAL_FLAGS[@]}" \
    -v "${REPO_ROOT}:/workspace" \
    -w /workspace \
    --name xarm_sorting_env \
    "${IMAGE_NAME}" \
    "$@"
