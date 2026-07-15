# ADR-0704-unitree-torch-abi-isolation: Isolate Torch ABI from ROS Controller

## Status
Proposed

## Context
`unitree_guide/junior_ctrl` failed to link ROS `roscpp` symbols (`ros::init`, `ros::NodeHandle`, etc.) because `find_package(Torch)` injected `-D_GLIBCXX_USE_CXX11_ABI=0` into global CMAKE_CXX_FLAGS. ROS Noetic uses the new C++11 ABI (`_GLIBCXX_USE_CXX11_ABI=1`). The ABI mismatch caused `std::string`-based ROS APIs to produce undefined references at link time.

## Decision

### 1. Make Torch optional, default OFF
**Decision**: Add `UNITREE_ENABLE_TORCH_POLICY` option, default OFF. When OFF, `find_package(Torch)` is not called, and `_GLIBCXX_USE_CXX11_ABI=0` is never added to compile flags.
**Reason**: The immediate goal is to restore `junior_ctrl` ROS compilation. RL policy inference and torch-based trotting are not required for basic robot control in simulation.

### 2. Exclude torch-dependent source files
**Decision**: When Torch OFF, exclude `State_RL_test.cpp`, `State_Trotting.cpp`, and `State_move_base.cpp` from compilation via `list(FILTER ... EXCLUDE REGEX ...)`.
**Reason**: These files include `<torch/torch.h>` either directly or transitively. Compiling them without torch headers would fail. Excluding them is the minimal change — they can be re-enabled by setting the option ON.

### 3. Guard transitive header includes
**Decision**: Add `#ifndef UNITREE_DISABLE_TORCH_POLICY` guards around `State_Trotting.h` and `State_RL_test.h` includes in `FSM.h`, and around corresponding struct members and instantiation sites in `FSM.h`/`FSM.cpp`.
**Reason**: `FSM.h` includes all state headers unconditionally. Without guards, `State_Trotting.h` → `<torch/torch.h>` would fail to compile even though the .cpp file was excluded.

### 4. Not implementing process isolation now
**Decision**: Record process isolation (`fix/0704-unitree-torch-process-isolation`) as a future task, not implemented here.
**Reason**: Full ABI isolation within a single executable requires deeper refactoring. Process isolation (ROS node for control ↔ separate node for RL inference) is the robust long-term solution but exceeds the scope of this minimal CMake fix.

## Alternatives Considered
1. **Remove TORCH_CXX_FLAGS from CMAKE_CXX_FLAGS but keep Torch linking**: Rejected — Torch C++ libraries are compiled with old ABI and would still cause link conflicts with ROS libraries using new ABI.
2. **Recompile Torch with new ABI**: Rejected — requires rebuilding PyTorch from source, impractical for deployment.
3. **Static library isolation**: Rejected — same-linker-symbol conflict persists.

## Consequences
- `junior_ctrl` compiles and links with ROS Noetic without ABI conflicts
- RL policy inference and torch-based trotting are disabled by default
- To re-enable: `-DUNITREE_ENABLE_TORCH_POLICY=ON` (requires libtorch + CUDA 11.8 + gcc-11)
- Future task: process isolation for robust Torch+ROS coexistence
