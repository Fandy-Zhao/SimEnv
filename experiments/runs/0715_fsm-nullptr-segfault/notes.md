# Experiment Notes: FSM Nullptr Segfault Fix + Dedicated Terminals

## Summary

1. Fixed a segmentation fault in `junior_ctrl` when pressing `4`/`6` with Torch-disabled builds.
2. Replaced the `/dev/tty`-redirect approach with dedicated terminal windows for `junior_ctrl` and rviz, ensuring reliable keyboard input.

## Root Cause

- `UNITREE_ENABLE_TORCH_POLICY` defaults to OFF → `UNITREE_DISABLE_TORCH_POLICY` defined
- `State_Trotting.cpp` and `State_RL_test.cpp` excluded from compilation
- `_stateList.trotting` and `_stateList.rl` remain nullptr (never constructed)
- Three unguarded code paths triggered transitions to these nullptr states:
  - `KeyBoard::checkCmd()` → `'4'`→START, `'6'`→RL
  - `main.cpp` ROS callback `data:4`→START, `data:6`→RL
  - `State_FixedStand::checkChange()` → START→TROTTING, RL→RL
- `FSM::getNextState()` returned `_stateList.invalid` (nullptr) via default case
- Dereferencing nullptr in `FSM::run()` → segfault

## Fix Applied

1. **KeyBoard.cpp**: Wrapped `case '4'`, `case '5'`, `case '6'` with `#ifndef UNITREE_DISABLE_TORCH_POLICY`
2. **main.cpp**: Wrapped `case 4` and `case 6` in ROS callback with `#ifndef UNITREE_DISABLE_TORCH_POLICY`
3. **State_FixedStand.cpp**: Wrapped TROTTING and RL return paths in `checkChange()` with guard
4. **FSM.cpp**: Added nullptr check after `getNextState()`, and safety check in CHANGE branch
5. **auto.sh**: Updated help text to indicate 4=Trotting and 6=RL require Torch build

### 6. auto.sh terminal refactor
- Added `launch_in_terminal()` function: tries `gnome-terminal`, falls back to `xterm`, then background
- Controller always runs in its own terminal → guaranteed real TTY for keyboard
- RVIZ launched in separate terminal after FAST-LIO2
- Removed `CONTROLLER_FOREGROUND` env var (no longer meaningful)
- Added `ENABLE_RVIZ` env var (default: `true`)
- Replaced end-of-script `wait` with `trap cleanup INT TERM` + `while true; sleep 1; done`

## Build Verification

```bash
source /opt/ros/noetic/setup.bash
catkin_make --only-pkg-with-deps unitree_guide -j
# Result: [100%] Built target junior_ctrl — no errors, no warnings
```

Binary: `devel/lib/unitree_guide/junior_ctrl` (696 KB ELF x86-64)

## Test Checklist

- [x] `catkin_make` builds successfully with default config (Torch OFF)
- [x] Binary `devel/lib/unitree_guide/junior_ctrl` exists and is executable
- [ ] Runtime smoke test: run `auto.sh`, press `2` (FixedStand) → should work
- [ ] Runtime smoke test: run `auto.sh`, press `4` → should NOT segfault (ignored when Torch OFF)
- [ ] Runtime smoke test: run `auto.sh`, press `6` → should NOT segfault (ignored when Torch OFF)
- [ ] Runtime smoke test: `rostopic pub /fsm/state_cmd ... "data: 4"` → should NOT segfault
- [ ] Optional: Torch-enabled build (`UNITREE_ENABLE_TORCH_POLICY=ON`), `4`/`6` should work as before

## Risk Assessment

- **Low risk**: All changes are additive `#ifndef` guards. Torch-enabled path unchanged.
- Verify that all added `#ifndef UNITREE_DISABLE_TORCH_POLICY` macros match the existing ones in FSM.cpp/FSM.h exactly.
- If the macro name is ever changed, all sites must be updated together.