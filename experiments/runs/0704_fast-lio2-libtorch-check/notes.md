# Experiment Notes: 0704 fast-lio2 libtorch check

## Goal

Fix `tools/build_with_venv.sh` to auto-detect PyTorch C++ CMake prefix from pip-installed torch, passing it to catkin_make without overwriting ROS CMAKE_PREFIX_PATH.

## What Changed

`tools/build_with_venv.sh` now:
1. After sourcing venv, attempts `import torch` and reads `torch.utils.cmake_prefix_path`
2. Checks if `TorchConfig.cmake` or `torch-config.cmake` exists at that path
3. If found, appends the torch cmake prefix to `CMAKE_PREFIX_PATH` (preserving ROS prefix)
4. If not found, prints clear warnings and suggested commands

## Libtorch Status

| Item | Status | Detail |
|------|--------|--------|
| Python torch import | ✅ OK | torch 2.0.1+cu118 |
| torch cmake prefix | ✅ OK | `.venv/lib/python3.10/site-packages/torch/share/cmake` |
| TorchConfig.cmake | ✅ FOUND | at torch cmake prefix |
| CMAKE_PREFIX_PATH append | ✅ OK | `/opt/ros/noetic;.../torch/share/cmake` |
| CUDA toolkit | ❌ MISSING | torch is cu118 variant, needs CUDA |
| livox_ros_driver C++11 | ❌ CODE_ISSUE | C++11 compilation errors in Livox SDK |

## Build Result

- Previous error: `Could not find TorchConfig.cmake` → **FIXED**
- New error 1: `Caffe2: CUDA cannot be found` → **ENV_MISSING** (no CUDA toolkit)
- New error 2: `livox_ros_driver` C++11 `shared_ptr`/`make_shared` errors → **CODE_ISSUE** (pre-existing, in new external package)

## Resolution Path

1. Install CUDA toolkit to satisfy torch cu118 CMake expectation, OR
2. Use CPU-only torch (`torch==2.0.1` without `+cu118`), OR
3. Set `Torch_DIR` to a CPU-only LibTorch install and use CPU-only torch for Python
