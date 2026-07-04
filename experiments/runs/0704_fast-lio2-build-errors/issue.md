# Issue: 定位并修复 SimEnv / FAST-LIO2 编译报错

## Task Goal

定位并解决当前 SimEnv / FAST-LIO2 编译报错。如果错误来自 `uav_simulator`，跳过该部分编译。

## Modification Scope

- `tools/build_with_venv.sh` — C++17, -no-pie linker, CUDA flags
- `src/FAST_LIO/CMakeLists.txt` — C++14 → C++17
- `src/livox_ros_driver/livox_ros_driver/CMakeLists.txt` — C++11 → C++17
- `src/livox_ros_driver/.../lddc.cpp` — PCL shared_ptr dereference fix
- `src/livox_ros_driver/.../timesync.h` — missing `<memory>`
- `src/livox_ros_driver/.../thread_base.h` — missing `<memory>`
- 测试记录和治理文档

## Non-Scope

- 不重装 Torch/CUDA
- 不改系统 gcc alternatives
- 不自动删除 build/devel
- 不修复 uav_simulator 源码
- 不 push/不 merge main/master

## Build Blockers Fixed

| # | Blocker | Status |
|---|---------|--------|
| 1 | unitree_legged_sdk PIE linker | FIXED: `-no-pie` |
| 2 | FAST_LIO C++14 log4cxx | FIXED: C++14→17 |
| 3 | livox_ros_driver C++11 compile | FIXED: C++11→17 |
| 4 | lddc.cpp std::shared_ptr serialization | FIXED: dereference |
| 5 | timesync.h missing `<memory>` | FIXED |
| 6 | thread_base.h missing `<memory>` | FIXED |
| 7 | Livox-SDK build cache NOTFOUND | PENDING: needs clean build |
