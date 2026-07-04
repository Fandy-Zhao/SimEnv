#!/usr/bin/env bash
set -euo pipefail

# Build the SimEnv catkin workspace with the project Python venv.
#
# Expected layout:
#   <repo>/
#     .venv/bin/python
#     src/
#     tools/build_with_venv.sh
#
# Usage:
#   ./tools/build_with_venv.sh
#
# Notes:
# - The venv should be created with:
#     python3 -m venv --system-site-packages .venv
# - This script intentionally uses the venv Python for CMake:
#     -DPYTHON_EXECUTABLE="$PWD/.venv/bin/python"
# - Sourcing devel/setup.bash inside this script does not modify the parent shell.
#   After the script finishes, run:
#     source devel/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

cd "$REPO_ROOT"

ROS_SETUP="/opt/ros/noetic/setup.bash"
VENV_ACTIVATE="$REPO_ROOT/.venv/bin/activate"
VENV_PYTHON="$REPO_ROOT/.venv/bin/python"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ERROR: ROS Noetic setup file not found: $ROS_SETUP" >&2
  echo "Install ROS Noetic or adjust this script for your ROS distribution." >&2
  exit 1
fi

if [[ ! -x "$VENV_PYTHON" ]]; then
  echo "ERROR: venv Python not found or not executable: $VENV_PYTHON" >&2
  echo "Create the project venv first:" >&2
  echo "  python3 -m venv --system-site-packages .venv" >&2
  echo "  source .venv/bin/activate" >&2
  echo "  python -m pip install -U pip setuptools wheel" >&2
  echo "  python -m pip install numpy pyyaml rospkg catkin_pkg empy" >&2
  exit 1
fi

if [[ ! -f "$VENV_ACTIVATE" ]]; then
  echo "ERROR: venv activate script not found: $VENV_ACTIVATE" >&2
  exit 1
fi

if [[ ! -d "$REPO_ROOT/src" ]]; then
  echo "ERROR: catkin workspace src directory not found: $REPO_ROOT/src" >&2
  exit 1
fi

echo "[build_with_venv] Repository root: $REPO_ROOT"
echo "[build_with_venv] ROS setup: $ROS_SETUP"
echo "[build_with_venv] Python executable: $VENV_PYTHON"

# shellcheck source=/opt/ros/noetic/setup.bash
source "$ROS_SETUP"

# shellcheck source=/dev/null
source "$VENV_ACTIVATE"

echo "[build_with_venv] Active python: $(command -v python)"
python --version

echo "[build_with_venv] Starting catkin_make..."
catkin_make -DPYTHON_EXECUTABLE="$VENV_PYTHON" "$@"

if [[ -f "$REPO_ROOT/devel/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "$REPO_ROOT/devel/setup.bash"
  echo "[build_with_venv] Sourced devel/setup.bash inside script."
fi

echo "[build_with_venv] Build finished."
echo "[build_with_venv] To use the workspace in your current shell, run:"
echo "  source devel/setup.bash"
