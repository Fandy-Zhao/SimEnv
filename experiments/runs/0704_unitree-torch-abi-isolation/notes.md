# Experiment Notes — 0704 unitree Torch ABI Isolation

## Date
2026-07-06

## Branch
fix/0704-unitree-torch-abi-isolation (from develop)

## ABI Diagnosis (Before Fix)

| Item | Finding |
|------|---------|
| junior_ctrl target | `src/unitree_guide/unitree_guide/unitree_guide/CMakeLists.txt:167` |
| Torch injection | Line 13: `find_package(Torch REQUIRED PATHS ...)` |
| ABI=0 evidence | `CXX_FLAGS = ... -D_GLIBCXX_USE_CXX11_ABI=0 -std=gnu++17` in flags.make |
| CATKIN_IGNORE | None found |
| Torch-dependent sources | `State_RL_test.cpp`, `State_Trotting.cpp` |
| Torch-transitive sources | `State_move_base.cpp` (inherits State_Trotting) |
| Headers needing guards | `FSM.h`, `FSM.cpp` |

## Fix Applied

### CMakeLists.txt changes
1. Added `option(UNITREE_ENABLE_TORCH_POLICY ...)` default OFF
2. Made `find_package(Torch)` conditional on the option
3. Moved CUDA compiler and C++17 (torch-only) into conditional
4. Excluded 3 cpp files from SRC_LIST when Torch OFF
5. Added `-DUNITREE_DISABLE_TORCH_POLICY` preprocessor define
6. Made TORCH_LIBRARIES linking conditional
7. Set CXX_STANDARD 17 unconditionally (was hidden by torch override)

### FSM.h changes
- Guarded `#include "FSM/State_Trotting.h"` and `#include "FSM/State_RL_test.h"`
- Guarded struct members `trotting` and `rl`
- Guarded corresponding `delete` calls

### FSM.cpp changes
- Guarded `new State_Trotting(...)` and `new State_RL(...)`
- Guarded `case FSMStateName::TROTTING:` and `case FSMStateName::RL:`

## Build Results

| Test | Result | Notes |
|------|--------|-------|
| unitree_guide build (Torch OFF) | ✅ PASS | junior_ctrl linked successfully |
| ABI=0 in compile flags after | ✅ CLEAN | No _GLIBCXX_USE_CXX11_ABI=0 |
| Full build (Torch OFF) | ⚠️ uav_simulator fails | Pre-existing mockamap issue, unrelated |
| Torch ON build | Not attempted | Requires libtorch + CUDA; deferred |

## Trade-offs
- Torch/RL policy disabled; trotting uses fallback (if available in FSM) or is unavailable
- State_move_base also excluded (requires State_Trotting → torch)
- Restoring Torch: `cmake -DUNITREE_ENABLE_TORCH_POLICY=ON` and rebuild
