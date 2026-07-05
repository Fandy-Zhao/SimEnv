# Issue: Isolate unitree_guide Torch ABI, Restore junior_ctrl Compilation

## Task Goal
Recover `junior_ctrl` compilation by isolating Torch ABI flags so ROS controller source code is not polluted by `_GLIBCXX_USE_CXX11_ABI=0`.

## ABI Diagnosis

| Item | Finding |
|------|---------|
| junior_ctrl target location | `src/unitree_guide/unitree_guide/unitree_guide/CMakeLists.txt:167` |
| Torch injection point | Line 13: `find_package(Torch REQUIRED PATHS ...)` |
| ABI=0 evidence | `CXX_FLAGS = ... -D_GLIBCXX_USE_CXX11_ABI=0 ...` (confirmed in flags.make) |
| CATKIN_IGNORE found | None |
| TORCH_CXX_FLAGS global | Injected via find_package(Torch) into CMAKE_CXX_FLAGS |

## Modification Scope
- `src/unitree_guide/unitree_guide/unitree_guide/CMakeLists.txt` — add UNITREE_ENABLE_TORCH_POLICY option, make Torch optional, filter RL sources

## Non-Scope
- No controller algorithm changes
- No FAST_LIO changes
- No uav_simulator changes
- No push, no merge to main/master
