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
# Environment overrides (optional):
#   SIMENV_CC=/path/to/gcc   SIMENV_CXX=/path/to/g++   ./tools/build_with_venv.sh
#   CUDA_HOME=/path/to/cuda  ./tools/build_with_venv.sh
#
# Notes:
# - The venv should be created with:
#     python3 -m venv --system-site-packages .venv
# - This script intentionally uses the venv Python for CMake:
#     -DPYTHON_EXECUTABLE="$PWD/.venv/bin/python"
# - For CUDA builds, gcc-11/g++-11 are preferred because CUDA 11.x is
#   incompatible with gcc 12+. The script auto-detects them.
# - Sourcing devel/setup.bash inside this script does not modify the parent shell.
#   After the script finishes, run:
#     source devel/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

cd "$REPO_ROOT"

# ---------------------------------------------------------------------------
# Prerequisite checks
# ---------------------------------------------------------------------------
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

# ---------------------------------------------------------------------------
# Compiler selection (prefer gcc-11 / g++-11 for CUDA 11.x compatibility)
# ---------------------------------------------------------------------------
SELECTED_CC="${SIMENV_CC:-}"
SELECTED_CXX="${SIMENV_CXX:-}"

if [[ -z "$SELECTED_CC" ]]; then
  if command -v gcc-11 &>/dev/null && gcc-11 -print-prog-name=cc1plus 2>/dev/null | grep -q /; then
    SELECTED_CC="$(command -v gcc-11)"
  else
    SELECTED_CC="$(command -v gcc)"
  fi
fi

if [[ -z "$SELECTED_CXX" ]]; then
  if command -v g++-11 &>/dev/null && g++-11 -print-prog-name=cc1plus 2>/dev/null | grep -q /; then
    SELECTED_CXX="$(command -v g++-11)"
  else
    SELECTED_CXX="$(command -v g++)"
  fi
fi

# Validate selected compilers
if ! "$SELECTED_CC" -print-prog-name=cc1plus 2>/dev/null | grep -q /; then
  echo "WARN: Selected CC ($SELECTED_CC) cannot find cc1plus. C++ compilation may fail." >&2
fi
if ! "$SELECTED_CXX" -print-prog-name=cc1plus 2>/dev/null | grep -q /; then
  echo "WARN: Selected CXX ($SELECTED_CXX) cannot find cc1plus. C++ compilation may fail." >&2
fi

export CC="$SELECTED_CC"
export CXX="$SELECTED_CXX"
export CUDAHOSTCXX="$SELECTED_CXX"

# ---------------------------------------------------------------------------
# CUDA detection
# ---------------------------------------------------------------------------
CUDA_HOME="${CUDA_HOME:-/usr/local/cuda-11.8}"

if [[ -d "$CUDA_HOME" ]] && [[ -x "$CUDA_HOME/bin/nvcc" ]]; then
  export PATH="$CUDA_HOME/bin:$PATH"
  echo "[build_with_venv] CUDA_HOME: $CUDA_HOME"
else
  echo "[build_with_venv] CUDA_HOME not found or nvcc missing ($CUDA_HOME)." >&2
  echo "[build_with_venv] CUDA-dependent packages may fail to configure." >&2
fi

# ---------------------------------------------------------------------------
# Source ROS and venv
# ---------------------------------------------------------------------------
echo "[build_with_venv] Repository root: $REPO_ROOT"
echo "[build_with_venv] Selected CC : $SELECTED_CC"
echo "[build_with_venv] Selected CXX: $SELECTED_CXX"
echo "[build_with_venv] CUDAHOSTCXX : $CUDAHOSTCXX"
echo "[build_with_venv] ROS setup: $ROS_SETUP"
echo "[build_with_venv] Python executable: $VENV_PYTHON"

# shellcheck source=/opt/ros/noetic/setup.bash
source "$ROS_SETUP"

# shellcheck source=/dev/null
source "$VENV_ACTIVATE"

echo "[build_with_venv] Active python: $(command -v python)"
python --version

# ---------------------------------------------------------------------------
# Torch CMake prefix detection
# ---------------------------------------------------------------------------
TORCH_CMAKE_PREFIX="$("$VENV_PYTHON" - <<'PY'
try:
    import torch
    print(torch.utils.cmake_prefix_path)
except Exception:
    pass
PY
)"

# ---------------------------------------------------------------------------
# Build catkin_make arguments
# ---------------------------------------------------------------------------
CATKIN_CMAKE_ARGS=(
  -DPYTHON_EXECUTABLE="$VENV_PYTHON"
  "-DCMAKE_C_COMPILER=$SELECTED_CC"
  "-DCMAKE_CXX_COMPILER=$SELECTED_CXX"
  "-DCMAKE_CUDA_HOST_COMPILER=$SELECTED_CXX"
  -DCMAKE_BUILD_TYPE=Release
  -DCMAKE_CXX_STANDARD=17
  -DCMAKE_CXX_STANDARD_REQUIRED=ON
  "-DCMAKE_EXE_LINKER_FLAGS=-no-pie"
)

# CUDA toolkit root
if [[ -d "$CUDA_HOME" ]]; then
  CATKIN_CMAKE_ARGS+=("-DCUDA_TOOLKIT_ROOT_DIR=$CUDA_HOME")
  CATKIN_CMAKE_ARGS+=("-DCUDAToolkit_ROOT=$CUDA_HOME")
  CATKIN_CMAKE_ARGS+=("-DCMAKE_CUDA_FLAGS=-ccbin=$SELECTED_CXX")
fi

# Accumulate CMake prefix entries (ROS, Torch, CUDA)
CMAKE_PREFIX_ENTRIES=()

if [[ -n "${CMAKE_PREFIX_PATH:-}" ]]; then
  CMAKE_PREFIX_ENTRIES+=("$CMAKE_PREFIX_PATH")
fi

if [[ -n "${TORCH_CMAKE_PREFIX:-}" ]]; then
  if find "$TORCH_CMAKE_PREFIX" \( -name "TorchConfig.cmake" -o -name "torch-config.cmake" \) 2>/dev/null | grep -q .; then
    CMAKE_PREFIX_ENTRIES+=("$TORCH_CMAKE_PREFIX")
    CATKIN_CMAKE_ARGS+=("-DTorch_DIR=$TORCH_CMAKE_PREFIX/Torch")
    echo "[build_with_venv] Torch CMake prefix: $TORCH_CMAKE_PREFIX"
    echo "[build_with_venv] Torch_DIR: $TORCH_CMAKE_PREFIX/Torch"
  else
    echo "WARN: torch import works but TorchConfig.cmake was not found under: $TORCH_CMAKE_PREFIX" >&2
  fi
else
  echo "WARN: Python torch is not importable from venv." >&2
  echo "      If a package needs LibTorch, install a compatible torch wheel:" >&2
  echo "      source .venv/bin/activate" >&2
  echo "      python -m pip install torch==2.0.1 torchvision==0.15.2 torchaudio==2.0.2" >&2
fi

if [[ -d "${CUDA_HOME:-}" ]]; then
  CMAKE_PREFIX_ENTRIES+=("$CUDA_HOME")
fi

if [[ ${#CMAKE_PREFIX_ENTRIES[@]} -gt 0 ]]; then
  CMAKE_PREFIX_JOINED="$(IFS=';'; echo "${CMAKE_PREFIX_ENTRIES[*]}")"
  CATKIN_CMAKE_ARGS+=("-DCMAKE_PREFIX_PATH=$CMAKE_PREFIX_JOINED")
  echo "[build_with_venv] CMake prefix path: $CMAKE_PREFIX_JOINED"
fi

# ---------------------------------------------------------------------------
# Warn about CMake cache if compilers may have changed
# ---------------------------------------------------------------------------
if [[ -f "$REPO_ROOT/build/CMakeCache.txt" ]]; then
  CACHED_CC="$(grep CMAKE_C_COMPILER: "$REPO_ROOT/build/CMakeCache.txt" | cut -d= -f2 2>/dev/null || true)"
  CACHED_CXX="$(grep CMAKE_CXX_COMPILER: "$REPO_ROOT/build/CMakeCache.txt" | cut -d= -f2 2>/dev/null || true)"
  if [[ -n "$CACHED_CC" ]] && [[ "$CACHED_CC" != "$SELECTED_CC" ]]; then
    echo "[build_with_venv] WARN: CMakeCache.txt has C compiler  : $CACHED_CC" >&2
    echo "[build_with_venv] WARN: Now selecting                   : $SELECTED_CC" >&2
    echo "[build_with_venv] WARN: CMake may refuse to reconfigure. If the build fails" >&2
    echo "[build_with_venv]       with 'compiler changed' errors, clean build/devel:" >&2
    echo "[build_with_venv]         rm -rf build devel" >&2
  fi
fi

# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------
echo "[build_with_venv] Starting catkin_make..."
catkin_make "${CATKIN_CMAKE_ARGS[@]}" "$@"

if [[ -f "$REPO_ROOT/devel/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "$REPO_ROOT/devel/setup.bash"
  echo "[build_with_venv] Sourced devel/setup.bash inside script."
fi

echo "[build_with_venv] Build finished."
echo "[build_with_venv] To use the workspace in your current shell, run:"
echo "  source devel/setup.bash"
