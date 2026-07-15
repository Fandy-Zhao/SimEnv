# Issue: junior_ctrl segfault on keyboard mode switch when Torch policy is disabled

## Task Goal

Fix a segmentation fault in `junior_ctrl` that occurs when pressing keyboard keys (e.g., `4` for Trotting, `6` for RL) to switch FSM states, when the controller is built without Torch policy support.

## Root Cause

`UNITREE_ENABLE_TORCH_POLICY` defaults to OFF in CMake (line 9 of `unitree_guide/unitree_guide/CMakeLists.txt`). When OFF, `UNITREE_DISABLE_TORCH_POLICY` is defined, which:

1. Excludes `State_Trotting.cpp`, `State_RL_test.cpp`, `State_move_base.cpp` from compilation
2. Prevents `State_Trotting*` and `State_RL*` from being constructed in FSM (pointers remain nullptr)

However, the following code paths are **not guarded** by `#ifndef UNITREE_DISABLE_TORCH_POLICY`:

| Location | Issue |
|----------|-------|
| `KeyBoard::checkCmd()` | Maps key `4` → `UserCommand::START` (Trotting) and `6` → `UserCommand::RL` |
| `main.cpp` ROS callback | Maps `data: 4` → `UserCommand::START` and `data: 6` → `UserCommand::RL` |
| `State_FixedStand::checkChange()` | Returns `FSMStateName::TROTTING` for START, `FSMStateName::RL` for RL |

When these unguarded paths trigger a transition to TROTTING or RL, `FSM::getNextState()` falls through to `default: return _stateList.invalid` (which is `nullptr`). The FSM then dereferences the nullptr → **segfault**.

## Trigger Conditions

- Build without Torch: `./tools/build_with_venv.sh` (default, `UNITREE_ENABLE_TORCH_POLICY=OFF`)
- User presses `4` (Trotting) or `6` (RL) in the terminal running `auto.sh`
- OR: `rostopic pub /fsm/state_cmd std_msgs/Int8 "data: 4"` or `"data: 6"`

## Modification Scope

1. `src/unitree_guide/unitree_guide/unitree_guide/src/interface/KeyBoard.cpp` — guard START/RL cases
2. `src/unitree_guide/unitree_guide/unitree_guide/src/main.cpp` — guard case 4/6 in ROS callback
3. `src/unitree_guide/unitree_guide/unitree_guide/src/FSM/State_FixedStand.cpp` — guard TROTTING/RL returns
4. `src/unitree_guide/unitree_guide/unitree_guide/src/FSM/FSM.cpp` — add nullptr safety check
5. `auto.sh` — update keyboard help text to reflect available states
6. `src/unitree_guide/unitree_guide/unitree_guide/src/FSM/FSM.h` — add `#include <iostream>` for error logging

## Explicit Non-Scope

- Enabling Torch by default (requires CUDA + libtorch, not universally available)
- Modifying state behavior beyond the nullptr guard
- Changes to other state files (FreeStand, BalanceTest, SwingTest, StepTest are already safe)

## Acceptance Criteria

1. `catkin_make` builds successfully with default config (Torch OFF)
2. Pressing `4` or `6` does NOT segfault — either ignored or prints a warning
3. Pressing `2` (FixedStand) still works correctly
4. `rostopic pub /fsm/state_cmd ... "data: 4"` does NOT segfault
5. `auto.sh` help text shows only available keyboard commands
6. When Torch IS enabled (opt-in), '4' and '6' work as before

## Risk Points

- Low risk: changes are additive `#ifndef` guards, no logic changes in the Torch-enabled path
- If `UNITREE_DISABLE_TORCH_POLICY` macro is misspelled in any file, the guard won't work
- The nullptr check in FSM::run() is defense-in-depth — it should never trigger after the guards are in place

## Expected Impacted Modules

- `unitree_guide` (junior_ctrl binary)
- `auto.sh` (help text)