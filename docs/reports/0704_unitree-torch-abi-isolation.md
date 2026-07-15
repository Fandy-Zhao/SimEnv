# Task Report — 0704 unitree Torch ABI Isolation

## Branch
- 工作分支: fix/0704-unitree-torch-abi-isolation
- 任务类型: 编译修复 (fix)
- 是否 push: 否
- 是否 merge 回 dev: 待用户确认

## Summary
- 修复了 `junior_ctrl` 的 `_GLIBCXX_USE_CXX11_ABI=0` 污染问题
- 移除了全局 Torch flag 注入，恢复 ROS Noetic ABI=1 编译
- Torch/RL policy 默认 OFF，可通过 CMake option 重新启用
- 排除了 3 个 torch 依赖源文件 + 守卫了 2 个头文件引用

## ABI Diagnosis

| 项目 | 结果 | 说明 |
|------|------|------|
| junior_ctrl target | ✅ 已定位 | CMakeLists.txt:167 |
| Torch usage files | State_RL_test.cpp, State_Trotting.cpp, State_move_base.cpp | 直接或间接依赖 torch |
| TORCH_CXX_FLAGS source | find_package(Torch) in CMakeLists.txt:13 | 注入 `-D_GLIBCXX_USE_CXX11_ABI=0` |
| ABI=0 evidence before | ✅ 确认 | `CXX_FLAGS = ... -D_GLIBCXX_USE_CXX11_ABI=0 ...` |
| ABI=0 evidence after | ✅ CLEAN | No ABI flag in compile flags |
| ROS undefined reference | ✅ FIXED | junior_ctrl links successfully |

## CMake Changes

| 文件 | 修改 | 原因 |
|------|------|------|
| `unitree_guide/.../CMakeLists.txt` | 添加 `UNITREE_ENABLE_TORCH_POLICY` option (OFF) | 隔离 Torch ABI |
| 同上 | find_package(Torch) → 条件化 | 只在 Torch ON 时调用 |
| 同上 | 排除 3 个 torch 依赖 .cpp 文件 | 防止编译失败 |
| 同上 | 添加 `-DUNITREE_DISABLE_TORCH_POLICY` define | 守卫头文件 |
| 同上 | TORCH_LIBRARIES 链接条件化 | 避免 ABI=0 库链接 |
| 同上 | CMAKE_CXX_STANDARD 17 无条件设置 | log4cxx 需要 C++17 |
| `FSM.h` | 守卫 torch 依赖头文件 | 防止 transitive include 失败 |
| `FSM.cpp` | 守卫 torch 依赖实例化 | 防止编译错误 |

## CATKIN_IGNORE Handling
无 CATKIN_IGNORE 文件需要处理。

## Build Results

| 测试 | 结果 | 日志路径 |
|------|------|---------|
| rospack find unitree_guide | ✅ PASS | — |
| unitree_guide build Torch OFF | ✅ PASS | build_unitree_torch_off.log |
| Full build Torch OFF | ⚠️ uav_simulator fails | build_full_torch_off.log (pre-existing mockamap issue) |
| ABI=0 in flags after fix | ✅ CLEAN | — |
| Torch ON optional check | Not attempted | Requires libtorch + CUDA |

## Documentation Updated
- issue: `experiments/runs/0704_unitree-torch-abi-isolation/issue.md`
- notes: `experiments/runs/0704_unitree-torch-abi-isolation/notes.md`
- report: `docs/reports/0704_unitree-torch-abi-isolation.md`
- ADR: `docs/decisions/ADR-0704-unitree-torch-abi-isolation.md`
- governance: PROJECT_STATE.md, CHANGELOG.md, docs/module_status.md (pending commit)

## Risks
- Torch policy OFF: RL 控制和 trotting (torch quaternion) 不可用
- 如需恢复 Torch policy: `cmake -DUNITREE_ENABLE_TORCH_POLICY=ON` + libtorch + CUDA 11.8
- 同一 executable 中同时链接 ROS ABI=1 和 Torch ABI=0 仍有深层冲突风险
- 长期方案: 进程隔离 (ROS 控制节点 ↔ Torch 推理节点)
