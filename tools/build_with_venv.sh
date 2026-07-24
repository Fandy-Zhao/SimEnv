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
LIVOX_SDK_EXPECTED="9306596a2bf15c1343bc023b497465ed0a32909d"
LIVOX_SDK_INSTALL="${SIMENV_LIVOX_SDK_INSTALL:-/home/zzf/search_ws/shared_ros_deps/Livox-SDK/${LIVOX_SDK_EXPECTED}/install}"

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

if [[ -x "$REPO_ROOT/tools/prepare_shared_ros_deps.sh" ]]; then
  echo "[build_with_venv] Checking shared ROS dependency links..."
  "$REPO_ROOT/tools/prepare_shared_ros_deps.sh" --check-only
fi

if [[ ! -f "$LIVOX_SDK_INSTALL/include/livox_sdk.h" ]]; then
  echo "ERROR: Livox-SDK header not found: $LIVOX_SDK_INSTALL/include/livox_sdk.h" >&2
  echo "Run tools/prepare_shared_ros_deps.sh before building." >&2
  exit 21
fi

if [[ ! -f "$LIVOX_SDK_INSTALL/lib/liblivox_sdk_static.a" ]]; then
  echo "ERROR: Livox-SDK static library not found: $LIVOX_SDK_INSTALL/lib/liblivox_sdk_static.a" >&2
  echo "Run tools/prepare_shared_ros_deps.sh before building." >&2
  exit 21
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
  -DBUILD_LIVOX_DRIVER_NODE=OFF
)


# ---------------------------------------------------------------------------
# Catkin package selection
# ---------------------------------------------------------------------------
# Default: build the runtime profile used by auto.sh: FAST-LIO2, the Livox
# Mid-360 Gazebo sensor plugin, the Unitree controller, and Gazebo control /
# contact plugins. This avoids unrelated, locally added packages (for example
# the legacy ps3joy sixpair utility, which requires libusb-0.1) from blocking
# normal simulation builds.
#
# Override examples:
#   SIMENV_CATKIN_WHITELIST="" ./tools/build_with_venv.sh
#   SIMENV_CATKIN_WHITELIST="livox_ros_driver;fast_lio" ./tools/build_with_venv.sh
#
# Notes:
# - livox_ros_driver is kept for CustomMsg / CustomPoint message definitions.
# - BUILD_LIVOX_DRIVER_NODE=OFF means the real Livox hardware driver node is skipped.
if [[ -z "${SIMENV_CATKIN_WHITELIST+x}" ]]; then
  SIMENV_CATKIN_WHITELIST="livox_ros_driver;livox_laser_simulation;fast_lio;simenv_fast_lio2_integration;unitree_legged_msgs;unitree_guide;unitree_legged_control;unitree_gazebo;catkin_simple;kdtree;minkindr;octomap_msgs;octomap_ros;minkindr_conversions;volumetric_msgs;volumetric_map_base;octomap_world;misc_utils;graph_utils;graph_planner;dsvplanner;dsvp_launch;local_planner;simenv_navigation_bridge;simenv_navigation_bringup"
fi

if [[ -n "$SIMENV_CATKIN_WHITELIST" ]]; then
  CATKIN_CMAKE_ARGS+=("-DCATKIN_WHITELIST_PACKAGES=$SIMENV_CATKIN_WHITELIST")
  echo "[build_with_venv] Catkin whitelist: $SIMENV_CATKIN_WHITELIST"
else
  echo "[build_with_venv] Catkin whitelist disabled: building all packages."
fi

# CUDA toolkit root
if [[ -d "$CUDA_HOME" ]]; then
  CATKIN_CMAKE_ARGS+=("-DCUDA_TOOLKIT_ROOT_DIR=$CUDA_HOME")
  CATKIN_CMAKE_ARGS+=("-DCUDAToolkit_ROOT=$CUDA_HOME")
  CATKIN_CMAKE_ARGS+=("-DCMAKE_CUDA_FLAGS=-ccbin=$SELECTED_CXX")
fi

# Accumulate CMake prefix entries (ROS, Torch, CUDA) and export them as an
# environment variable.  ROS setup.bash already seeded CMAKE_PREFIX_PATH with
# /opt/ros/noetic.  Exporting additional prefixes via the environment (rather
# than -DCMAKE_PREFIX_PATH) keeps ROS's own prefixes intact and avoids
# overriding per-package find_package() paths (e.g. unitree_guide's
# NO_DEFAULT_PATH for its own LibTorch distribution).
CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH:-}"
CMAKE_LIBRARY_PATH="${CMAKE_LIBRARY_PATH:-}"
CPATH="${CPATH:-}"
CPLUS_INCLUDE_PATH="${CPLUS_INCLUDE_PATH:-}"
LIBRARY_PATH="${LIBRARY_PATH:-}"

if [[ -n "${TORCH_CMAKE_PREFIX:-}" ]]; then
  if find "$TORCH_CMAKE_PREFIX" \( -name "TorchConfig.cmake" -o -name "torch-config.cmake" \) 2>/dev/null | grep -q .; then
    CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}:${TORCH_CMAKE_PREFIX}"
    echo "[build_with_venv] Torch CMake prefix appended: $TORCH_CMAKE_PREFIX"
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
  CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}:${CUDA_HOME}"
fi

CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}:${LIVOX_SDK_INSTALL}"
CMAKE_LIBRARY_PATH="${LIVOX_SDK_INSTALL}/lib:${CMAKE_LIBRARY_PATH}"
CPATH="${LIVOX_SDK_INSTALL}/include:${CPATH}"
CPLUS_INCLUDE_PATH="${LIVOX_SDK_INSTALL}/include:${CPLUS_INCLUDE_PATH}"
LIBRARY_PATH="${LIVOX_SDK_INSTALL}/lib:${LIBRARY_PATH}"

export CMAKE_PREFIX_PATH
export CMAKE_LIBRARY_PATH
export CPATH
export CPLUS_INCLUDE_PATH
export LIBRARY_PATH
echo "[build_with_venv] CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH"
echo "[build_with_venv] CMAKE_LIBRARY_PATH=$CMAKE_LIBRARY_PATH"
echo "[build_with_venv] Livox-SDK install: $LIVOX_SDK_INSTALL"

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
# Shared dependency safety checks
# ---------------------------------------------------------------------------
SHARED_LIVOX_CMAKELISTS="$REPO_ROOT/src/external/livox_ros_driver/livox_ros_driver/CMakeLists.txt"
if [[ -f "$SHARED_LIVOX_CMAKELISTS" ]]; then
  if ! grep -q "BUILD_LIVOX_DRIVER_NODE" "$SHARED_LIVOX_CMAKELISTS"; then
    if ! find "$LIVOX_SDK_INSTALL/lib" /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu -maxdepth 2 \
      -name "liblivox_sdk_static.a" 2>/dev/null | grep -q .; then
      echo "[build_with_venv] ERROR: shared livox_ros_driver requires Livox-SDK for its hardware node." >&2
      echo "[build_with_venv] ERROR: its CMakeLists.txt has no BUILD_LIVOX_DRIVER_NODE guard and would try to clone/build Livox-SDK inside the shared source checkout." >&2
      echo "[build_with_venv] ERROR: refusing to modify /home/zzf/search_ws/livox_ros_driver from this worktree." >&2
      echo "[build_with_venv] ERROR: use a clean message-only upstream commit/patch strategy that does not mutate the shared source, or install a real Livox-SDK system library." >&2
      exit 20
    fi
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
