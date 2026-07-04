# Issue: 修复 FAST-LIO2 mapping 编译中的 libtorch C++ SDK 路径问题

## Task Goal

修复或增强 `tools/build_with_venv.sh`，使它在 `.venv` 中安装了 Python torch 的情况下，可以自动发现 PyTorch C++ CMake 路径 (`torch.utils.cmake_prefix_path`)，并将其传递给 `catkin_make`。

## Modification Scope

- `tools/build_with_venv.sh` — 添加 torch CMake 路径自动检测
- `experiments/runs/0704_fast-lio2-libtorch-check/` — 测试记录
- 项目治理文档

## Non-Scope

- 不安装 torch
- 不下载 LibTorch
- 不修改 FAST_LIO 源码
- 不做导航探索
- 不 push

## Acceptance Criteria

1. 脚本能检测 torch import 状态
2. 脚本能读取 `torch.utils.cmake_prefix_path`
3. 如果 Torch CMake 路径存在，脚本将其追加给 catkin_make
4. 不覆盖 ROS 的 `CMAKE_PREFIX_PATH`
5. 如果 torch 或 TorchConfig.cmake 缺失，给出清晰错误/warning
6. catkin_make 不再因 TorchConfig.cmake 缺失而失败

## Risk Points

- Python torch 版本与系统 CUDA/GCC/GLIBC 不匹配
- pip torch 自带 LibTorch 路径可能不可用
- `CMAKE_PREFIX_PATH` 覆盖会破坏 ROS 包发现
