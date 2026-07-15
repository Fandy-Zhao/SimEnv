# Experiment Notes: 0704 fast-lio2 build errors

## Build Diagnosis

| Item | Status |
|------|--------|
| workspace root | SimEnv ✅ |
| simenv_fast_lio2_integration | rospack find OK ✅ |
| FAST_LIO | rospack find OK ✅ |
| livox_ros_driver | rospack find OK ✅ |
| unitree_guide | Torch found, PIE fixed ✅ |
| uav_simulator | Not yet reached; packages built to 38% |
| Torch_DIR | .venv/lib/.../torch/share/cmake/Torch ✅ |
| CUDA_HOME | /usr/local/cuda-11.8 ✅ |
| selected CC/CXX | gcc-11/g++-11 ✅ |

## Fixes Applied (6 files)

1. `tools/build_with_venv.sh`: Added C++17, -no-pie linker, CUDA_FLAGS
2. `FAST_LIO/CMakeLists.txt`: C++14 → C++17 (log4cxx compat)
3. `livox_ros_driver/livox_ros_driver/CMakeLists.txt`: C++11 → C++17
4. `lddc.cpp`: `publish(cloud)` → `publish(*cloud)` for PCL PointCloud::Ptr (std::shared_ptr)
5. `timesync.h`: Added `#include <memory>` (GCC 11 no longer implicitly includes)
6. `thread_base.h`: Added `#include <memory>` (same root cause)

## Remaining Blocker

- **Livox-SDK static library**: `LIVOX_SDK_LIBRARY-NOTFOUND` was cached in CMakeCache.txt. The SDK's `CHECK_CXX_SOURCE_COMPILES` tests failed during initial configure because `<memory>` wasn't included. The cache has been cleaned (`build/livox_ros_driver/` removed, `LIVOX_SDK_LIBRARY` cache entry deleted). Rebuild needed.

## uav_simulator Status

- NOT yet a blocker — build stuck at livox_ros_driver (43%)
- uav packages (quadrotor_msgs, etc.) built successfully to 38%
- No CATKIN_IGNORE created yet

## Error Classification Summary

| Category | Status |
|----------|--------|
| Torch/TorchConfig | FIXED |
| CUDA Toolkit | FIXED |
| nvcc/cc1plus/gcc | FIXED |
| unitree PIE linker | FIXED |
| FAST_LIO C++ std | FIXED |
| livox_ros_driver serialization | FIXED |
| livox SDK includes | FIXED |
| livox SDK linkage | PENDING (cache) |
| uav_simulator | NOT REACHED |

## Recommended Next Commands

```bash
# Clean only livox build artifacts
rm -rf build/livox_ros_driver
rm -rf devel/lib/livox_ros_driver
sed -i '/LIVOX_SDK_LIBRARY/d' build/CMakeCache.txt

# Rebuild
./tools/build_with_venv.sh
```
