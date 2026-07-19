# G2 Pre-WAVE Static Control Path Audit

Date: 2026-07-19
Branch: diagnose/0719-g2-pre-wave-block-reason
Baseline: test/0718-g2-trotting-motion-baseline (af99255b)

## 1. FSM Main Loop

```
FSM::run()  [500 Hz tick]
├── recvStateOnly() / sendRecv()
├── updateControlTime()  -- scheduler gate, returns false if not advancing
├── runWaveGen()          -- WaveGenerator::calcContactPhase()
├── estimator->run()
├── checkSafty()          -- cos(tilt) < 0.5 → safety violation
├── _currentState->run()  -- State_Trotting::run() when in TROTTING
├── _currentState->checkChange()  -- userCmd transition check
└── publishCmdOnly() / sendRecv()
```

## 2. State Transitions

```
PASSIVE → FIXEDSTAND (userCmd=L2_A / data=2)
FIXEDSTAND → TROTTING (userCmd=START / data=4)
TROTTING → FIXEDSTAND (userCmd=L2_A)
TROTTING → PASSIVE (userCmd=L2_B)
```

## 3. State_Trotting::enter()

```
1. Capture _pcd = position, _posFeetGlobalGoal = feet positions
2. Reset all readiness/wave flags:
   - _heightTransitionElapsed = 0
   - _readinessStableElapsed = 0
   - _waveContactLossElapsed = 0
   - _heightTransitionComplete = false
   - _waveReady = false
   - _waveStarted = false
   - _waveAbortLatched = false
3. resetCommandState() (vx=vy=wz=0)
4. _yawCmd = current yaw, _Rd = rotz(yaw)
5. setAllStanceNow() → WaveStatus=STANCE_ALL, contact=[1,1,1,1], phase=[0.5,0.5,0.5,0.5]
6. gait->restart()
```

## 4. State_Trotting::run() Per-Tick Execution

```
1. Read state estimate: _posBody, _velBody, _posFeet2BGlobal, _posFeetGlobal, _velFeetGlobal, _B2G_RotMat, _G2B_RotMat, _yaw, _dYaw
2. NUMERICAL GUARD: stateEstimateFinite() → if false: abortWave("non-finite state estimate", true), holdCurrentPose(), return
3. getUserCmd() → resolves vx/vy/wz from /cmd_vel or joystick → stored in _vCmdBody, _dYawCmd
4. updateHeightTransition() → smooth height from entry height → target
5. updateWaveReadiness() → evaluates readiness + hold timer (details below)
6. updateRunningWaveSafety() → checks attitude + contact during wave
7. if _waveAbortLatched: suppressMotionCommand(), holdCurrentPose(), return
8. if !_waveReady: suppressMotionCommand() → _vCmdBody = [0,0,0]
9. calcCmd() → compute _pcd, _vCmdGlobal, _yawCmd, _Rd, _wCmdGlobal
10. NUMERICAL GUARD: commandStateFinite() → if false: abortWave("non-finite command state", true)
11. gait->run() → foot trajectory generation
12. calcTau() → force computation via BalanceCtrl QP
13. calcQQd() → IK: foot positions → joint angles
14. NUMERICAL GUARD: controlOutputFinite() → if false: abortWave("non-finite control output", true)
15. WAVE START DECISION:
    - checkStepOrNot() && _waveReady && !_waveAbortLatched:
      → setStartWave() (WaveStatus=WAVE_ALL), _waveStarted=true
    - else: setAllStance() (WaveStatus=STANCE_ALL)
16. Publish motor commands: tau, qGoal, qdGoal, gains
```

## 5. Readiness Conditions (readinessConditionsMet())

All six must be true simultaneously:

| # | Condition | Threshold | Source |
|---|-----------|-----------|--------|
| R1 | _heightTransitionComplete | 0.75s smooth transition | updateHeightTransition() |
| R2 | expectedAllStance() | contact[i]==1 ∀i | CtrlComponents::contact |
| R3 | hasAllFeetContact(minForce) | footForce[i] ≥ 1.0N ∀i | LowlevelState |
| R4 | linearSpeed < _readyLinearVelocity | < 0.12 m/s | _velBody.norm() |
| R5 | angularSpeed < _readyAngularVelocity | < 0.35 rad/s | getGyroGlobal().norm() |
| R6 | |roll| < _readyTilt && |pitch| < _readyTilt | < 10° | rotMatToRPY(_B2G_RotMat) |

## 6. Readiness Hold Timer

```
updateWaveReadiness():
  if _waveStarted || _waveAbortLatched: return

  if _waveReady:
    if readinessConditionsMet(): return  // stay ready
    else: _waveReady = false, reset timer

  if readinessConditionsMet():
    _readinessStableElapsed += getControlDt()  // accumulate sim time
  else:
    _readinessStableElapsed = 0  // reset on any condition failure

  if _readinessStableElapsed >= 0.20: _waveReady = true
```

**Threading**: All in main control thread (FSM::run). Uses sim-time dt.

## 7. Wave Start Trigger (checkStepOrNot())

Returns true if ANY of:

| Condition | Threshold |
|-----------|-----------|
| \|_vCmdBody(0)\| > 0.03 | vx command > 3 cm/s |
| \|_vCmdBody(1)\| > 0.03 | vy command > 3 cm/s |
| \|posError(0)\| > 0.08 | x position error > 8 cm |
| \|posError(1)\| > 0.08 | y position error > 8 cm |
| \|velError(0)\| > 0.05 | x velocity error > 5 cm/s |
| \|velError(1)\| > 0.05 | y velocity error > 5 cm/s |
| \|dYawCmd\| > 0.20 | yaw rate command > 0.2 rad/s |

**CRITICAL FINDING**: When vx=0 is commanded and robot is stable:
- `_vCmdBody(0) = 0` → not > 0.03
- `posError` bounded by ±0.05 (saturation in calcCmd) → not > 0.08
- `velError ≈ 0` (stable robot) → not > 0.05
- `dYawCmd = 0` → not > 0.20
- **Result**: checkStepOrNot() returns false → wave NEVER starts → robot stays in STANCE_ALL

When vx≥0.10 is commanded:
- `|vCmdBody(0)| = 0.10 > 0.03` → checkStepOrNot() returns true → wave starts after readiness

## 8. Wave Safety (updateRunningWaveSafety())

Only active when `_waveStarted && !_waveAbortLatched`:

| Guard | Threshold | Action |
|-------|-----------|--------|
| Attitude exceed | roll\|≥20° or pitch\|≥20° | abortWave("unsafe body attitude", true) |
| Contact loss | Stance feet without valid contact for >0.08s | abortWave("stance-foot contact loss", true) |

## 9. Wave Cancel (abortWave())

```
abortWave(reason, latchAbort):
  setAllStanceNow() → STANCE_ALL, phase=0.5, reset wave time
  gait->restart()
  _waveStarted = false
  _waveReady = false
  _readinessStableElapsed = 0
  _waveContactLossElapsed = 0
  _waveAbortLatched = latchAbort  // if true, blocks re-entry until state re-entered
  resetCommandState()
```

Cancel reasons:
- "non-finite state estimate" (latched)
- "non-finite command state" (latched)
- "non-finite control output" (latched)
- "unsafe body attitude" (latched)
- "stance-foot contact loss" (latched)
- "control-time reset" (NOT latched)
- "user state change" (via exit())

## 10. Fall Detection

Two layers:

### C++ Safety Check (FSM::checkSafty())
```cpp
_lowState->getRotMat()(2,2) < 0.5  // cos(tilt) < 0.5 → tilt > 60°
```
Returns false when body tilt exceeds ~60° from vertical. Does NOT directly stop the trial.

### Python Fall Predicate (g2_capture_trial.py:302)
```python
min(row["z"] for row in capture.truth_rows) < 0.12  # base height < 12 cm
```
This is the `FALL_DETECTED` reason. Normal FixedStand height ≈ 0.326 m.

## 11. WaveGenerator

```
WaveStatus enum: { STANCE_ALL=0, SWING_ALL=1, WAVE_ALL=2 }

setAllStance()    → _waveStatus = STANCE_ALL (waveGen NOT reset)
setAllStanceNow() → STANCE_ALL + contact=1,1,1,1 + phase=0.5,0.5,0.5,0.5 + waveGen.resetTime()
setStartWave()    → _waveStatus = WAVE_ALL


gaitCycleSequence increments:
  when _waveStatus==WAVE_ALL AND phase[0] crosses 0.8→0.2 (one full gait cycle)
```

## 12. Non-Finite Guard Stages

| Stage | Check | Latch |
|-------|-------|-------|
| 1 | stateEstimateFinite() — body position, velocity, rotation, feet | YES |
| 2 | commandStateFinite() — pcd, vCmdBody, vCmdGlobal, yawCmd, dYawCmd | YES |
| 3 | controlOutputFinite() — foot goals, forces, qGoal, qdGoal, tau | YES |

All three latch `_waveAbortLatched = true`, requiring state exit/re-enter to clear.

## 13. Key Timings

| Parameter | Default | Configurable |
|-----------|---------|--------------|
| Height transition duration | 0.75 s | rosparam: trotting_height_transition_duration |
| Readiness hold duration | 0.20 s | rosparam: trotting_ready_hold_duration |
| Wave contact loss duration | 0.08 s | rosparam: trotting_wave_contact_loss_duration |
| Cmd vel timeout | 0.50 s | rosparam: trotting_cmd_vel_timeout |
| Control dt | 0.002 s (500 Hz) | rosparam: UNITREE_CTRL_DT |

## 14. Call Chain Summary

```
Trial Flow:
  spawn → contact ready → data=2 (FixedStand)
  → wait 1s settle → data=4 (Trotting)
  → FSM transition → State_Trotting::enter()
  → [each 500Hz tick: run() → updateWaveReadiness()]
  → after 0.75s height transition + 0.20s hold stability:
    _waveReady = true
  → if vx≠0: checkStepOrNot() = true → setStartWave()
  → WAVE_ALL → gait cycles begin

Cancel Chain:
  non-finite / tilt / contact-loss / FSM exit
  → abortWave(reason, latch)
  → setAllStanceNow()
  → _waveAbortLatched = true (if latched)
```

## 15. vx=0 vs vx=0.10 vs vx=0.50 Behavioral Prediction

| Phase | vx=0 | vx=0.10 | vx=0.50 |
|-------|------|---------|---------|
| FixedStand stable | Expected | Expected | Expected |
| Trotting enter | Expected | Expected | Expected |
| Height transition | 0.75s | 0.75s | 0.75s |
| Readiness achieved | After 0.20s stability | After 0.20s stability | After 0.20s stability |
| checkStepOrNot() | **FALSE** (no trigger) | TRUE (>0.03) | TRUE (>0.03) |
| WAVE_ALL entered | **NEVER** | After readiness | After readiness |
| First fall risk | Before wave (if unstable) | After wave start | After wave start |
