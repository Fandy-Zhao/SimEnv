# ADR-010: FSM Command Source Arbitration

## Status
Accepted (2026-07-18)

## Context
The FSM receives `UserCommand` from three sources:
1. **ROS topic** `/fsm/state_cmd` → `CtrlComponents::pendingStateCmd` latch
2. **Keyboard** `KeyBoard::checkCmd()` → `CmdPanel::userCmd`
3. **Joystick** `/joy` callback → `CmdPanel::setUserCmd()`

These compete for `LowlevelState::userCmd`, which `checkChange()` uses to decide state transitions.

## Decision

### Priority (highest to lowest)
1. **SAFETY** — internal safety overrides (tilt > 60°, checkSafty())
2. **ROS TOPIC** — explicit `/fsm/state_cmd` (programmatic control)
3. **JOYSTICK** — `/joy` topic callback
4. **KEYBOARD** — stdin/keyboard thread
5. **NONE** — no command

### Implementation
In `FSM::run()`:
1. `recvStateOnly()` sets `lowState->userCmd` from keyboard/joystick (CmdPanel)
2. `pendingStateCmd` (from ROS) **overwrites** `lowState->userCmd` if non-NONE
3. `pendingStateCmd` is consumed (set to NONE) after one application

### Command consumption
- ROS commands are **pulse/latch**: consumed on first advancing sim step after receipt
- Keyboard commands are **level**: persist until a different key is pressed
- Joystick commands are **level**: persist until a different button is pressed

### Pause semantics
- During pause (`updateControlTime()` returns false), ROS callbacks still fire
- `pendingStateCmd` remains latched across paused ticks
- On first advancing tick after unpause, latched command is applied
- Pause-duration commands are applied (not discarded)

### Reset semantics
- `controlResetGeneration++` clears all command state
- Old `pendingStateCmd` from pre-reset epoch is NOT applied
- FSM re-enters PASSIVE after reset, requiring explicit command to re-enter states

### Keyboard NONE protection
- `KeyBoard::getUserCmd()` returns the last set `userCmd` value
- If no key was ever pressed, returns `UserCommand::NONE` (initial)
- ROS command overwrites keyboard NONE as designed
- If keyboard was actively commanding a state, ROS can still override it

## Alternatives Considered
1. **Queue-based**: accumulate commands in a queue → **rejected** (unbounded queue risk)
2. **Last-write-wins with timestamp**: compare timestamps → **rejected** (unnecessary complexity)
3. **Keyboard priority over ROS**: → **rejected** (prevents programmatic control during keyboard sessions)

## Consequences
- Single ROS command pulse suffices for state transition
- Continuous ROS publishing (e.g., `-r 10`) keeps the latch filled
- Keyboard and ROS commands don't conflict (ROS wins)
- Must send `data=2` (FixedStand) before `data=4` (Trotting) if in PASSIVE

## Evidence
- Static audit: `main.cpp:168-183`, `FSM.cpp:97-102`, `State_FixedStand.cpp:157-189`
- FSM command diagnostics added in CtrlComponents.h, main.cpp, FSM.cpp
