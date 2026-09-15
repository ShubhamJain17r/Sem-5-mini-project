#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

IMAGE_NAME="xarm-sorting:humble"
USER_ID="$(id -u)"
GROUP_ID="$(id -g)"
USER_NAME="$(whoami)"

echo "=========================================================="
echo " Building Project-Level ROS 2 Humble Docker Image"
echo " Tag:      ${IMAGE_NAME}"
echo " User:     ${USER_NAME} (UID: ${USER_ID}, GID: ${GROUP_ID})"
echo " Repo:     ${REPO_ROOT}"
echo "=========================================================="

docker build \
    --build-arg USERNAME="${USER_NAME}" \
    --build-arg USER_UID="${USER_ID}" \
    --build-arg USER_GID="${GROUP_ID}" \
    -t "${IMAGE_NAME}" \
    -f "${SCRIPT_DIR}/Dockerfile" \
    "${REPO_ROOT}"

echo "=========================================================="
echo " Docker build complete: ${IMAGE_NAME}"
echo " Run interactive shell with: ./docker/run.sh"
echo "=========================================================="
