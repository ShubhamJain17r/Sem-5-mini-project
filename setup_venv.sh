#!/usr/bin/env bash
# ==============================================================================
# Setup / Activate Python Virtual Environment (venv) for Single xArm
# Note: Host venv is used ONLY for standalone Python scripts (Web Tool, Telemetry).
# All ROS 2 commands must run inside the Docker container (./docker/run.sh).
# ==============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${SCRIPT_DIR}/venv"

if [ ! -d "${VENV_DIR}" ]; then
    echo "Creating Python virtual environment in ${VENV_DIR}..."
    python3 -m venv "${VENV_DIR}"
fi

echo "Activating virtual environment..."
source "${VENV_DIR}/bin/activate"

echo "Installing / updating dependencies from requirements.txt..."
pip install --upgrade pip
pip install -r "${SCRIPT_DIR}/requirements.txt"

echo ""
echo "========================================================================"
echo "  ✅ Python Virtual Environment Ready!"
echo ""
echo "  ⚠️  STRICT TWO-ENVIRONMENT EXECUTION POLICY:"
echo "     1. ROS 2 commands MUST run inside Docker: ./docker/run.sh"
echo "     2. Python commands MUST run in virtualenv: source venv/bin/activate"
echo ""
echo "  To activate in your current terminal session, run:"
echo "    source venv/bin/activate"
echo "========================================================================"

