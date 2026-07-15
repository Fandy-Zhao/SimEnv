# Issue: 修复 CUDA 11.8 host compiler 问题

## Task Goal

修复 CUDA 11.8 使用默认 gcc 12 导致 cc1plus / host compiler 失败的问题。通过强制使用 gcc-11/g++-11 构建。

## Modification Scope

- `tools/build_with_venv.sh` — 编译器选择和 CUDA 路径逻辑
- 测试记录和治理文档

## Non-Scope

- 不安装 gcc-11/g++-11
- 不修改系统 alternatives
- 不自动删除 build/devel
- 不改 FAST_LIO 算法
- 不 push

## Acceptance Criteria

1. 脚本能检测 gcc-11/g++-11 和 cc1plus ✅
2. 脚本能设置 CC/CXX/CUDAHOSTCXX ✅
3. 脚本传递 C/C++/CUDA host compiler 给 catkin_make ✅
4. 保留 Torch CMake prefix 逻辑 ✅
5. 不覆盖 ROS CMAKE_PREFIX_PATH ✅
6. nvcc + g++-11 最小测试通过 ✅
7. CUDA host compiler 错误已消除 ✅
