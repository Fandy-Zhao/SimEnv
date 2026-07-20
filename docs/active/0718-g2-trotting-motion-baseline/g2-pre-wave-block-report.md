# G2-D1 Pre-WAVE Block Report

Date: 2026-07-19

## Branch

`diagnose/0719-g2-pre-wave-block-reason` (commit `1a524244`)

## Baseline

`test/0718-g2-trotting-motion-baseline` at `af99255b` (includes cherry-picked Gate V evidence)

## Worktree

`/home/zzf/search_ws/SimEnv_worktrees/g2-pre-wave-block-reason`

## Gate V Prerequisite

**Verdict**: `G2_VALIDATOR_NO_DEFECT`

- No validator frame/pose semantic defect found.
- `FALL_DETECTED` confirmed as physically meaningful (`min(model_states.z) < 0.12 m`).
- All four old G2 trials retain fall verdict after offline reclassification.
- No production validator fix to merge.

## Governance Exception Path

Gate P entered via **ADR-010** under the `G2_VALIDATOR_NO_DEFECT` exception path.
Gate P is diagnostic only — no control parameter modifications permitted.

## Scope

Diagnose why the robot never enters `WAVE_ALL` during Trotting, and whether the
physical fall occurs before or after wave start readiness.

## Non-Goals

- No Kp/Kd modification
- No readiness threshold tuning
- No WaveGenerator state-transition semantic changes
- No fall-guard removal
- No numerical fixes
- No G2-R recovery branch creation

---

## Static Control Path

Full static audit documented in [g2-pre-wave-static-audit.md](g2-pre-wave-static-audit.md).

### Key Findings

1. **checkStepOrNot() requires velocity or error to trigger wave start.**
   With `vx=0` and a stable robot, all trigger conditions evaluate to false:
   - `|vCmdBody.x| = 0` (not > 0.03)
   - `|posError.x| ≤ 0.05` (saturated in calcCmd)
   - `|velError.x| ≈ 0` (stable robot)
   - **Result**: Wave NEVER starts for vx=0.

2. **Readiness conditions**: height transition (0.75s), all-stance check,
   four-foot contact (≥1.0N), body speed (<0.12 m/s), angular speed
   (<0.35 rad/s), tilt (<10°).

3. **Readiness hold**: 0.20s consecutive sim-time stability before
   `_waveReady = true`.

4. **Wave start**: `checkStepOrNot() && _waveReady && !_waveAbortLatched`
   → `setStartWave()` → WaveStatus = WAVE_ALL.

5. **Wave cancel reasons**: non-finite state/command/output (latched),
   unsafe attitude >20° (latched), stance-foot contact loss >0.08s (latched).

6. **Fall detection**: Python-side `min(model_states.z) < 0.12 m`
   (normal height ≈ 0.326 m). C++ safety: `cos(tilt) < 0.5` (tilt > 60°).

7. **Numerical guard stages**: state estimate → command state → control output.
   All three latch `_waveAbortLatched = true`.

### Call Chain

```
FixedStand stable → data=4 → FSM transition → State_Trotting::enter()
→ [each 500Hz tick: run() → getUserCmd() → updateHeightTransition()
   → updateWaveReadiness() → updateRunningWaveSafety()
   → calcCmd() → gait->run() → calcTau() → calcQQd()
   → checkStepOrNot() && _waveReady && !_waveAbortLatched
     → setStartWave() (WAVE_ALL)]
```

---

## Diagnostic Infrastructure

### C++ Additions (write-only, no control effect)

1. **PreWaveDiagnostics struct** in `CtrlComponents`:
   - Readiness breakdown flags (height, stance, contact, linspeed, angspeed, tilt)
   - Readiness hold state (met, complete, elapsed)
   - Wave start/cancel tracking with reason codes
   - First-block latch (reason, sim_time_us, control_sequence)
   - Model height, numerical guard stage

2. **TimingRecord extension** (6 new CSV columns):
   - `prewave_readiness_flags` (bitmask)
   - `prewave_readiness_hold_elapsed`
   - `prewave_first_block_reason`
   - `prewave_model_height`
   - `prewave_numerical_guard_stage`
   - `prewave_wave_cancel_reason`

### Python Analysis

1. **prewave_analyze.py**: Block reason enumeration, CSV parsing,
   timeline classification (A-F), first failing checkpoint determination,
   cross-trial comparison.

2. **test_prewave.py**: 23 unit tests covering block reasons,
   readiness classification, first-block latch, timeline types,
   failing checkpoints, trial validity.

### Verification

- Build: `catkin_make --only-pkg-with-deps unitree_guide` → **PASS**
- G1 timing regression: 13/13 tests → **PASS**
- Python syntax check: **PASS**
- Python unit tests: 23/23 → **PASS**

---

## Pre-WAVE Checkpoints

| Checkpoint | Definition |
|-----------|------------|
| P0 | FixedStand stable ≥3s sim-time, height ≥0.12m |
| P1 | Trotting state entered |
| P2 | Trotting state sustained (no premature exit) |
| P3 | height_ready (height transition complete) |
| P4 | stance_ready (all 4 contact[i] == 1) |
| P5 | contact_ready (all 4 footForce ≥1.0N) |
| P6 | Contact freshness (callback sequence > 0) |
| P7 | Readiness hold complete (0.20s) |
| P8 | Wave start requested (setStartWave called) |
| P9 | WaveGenerator received request |
| P10 | WaveStatus == WAVE_ALL |
| P11 | gait_cycle > 0 |
| P12 | First fall (height < 0.12m) |
| P13 | First numerical guard |
| P14 | First safety guard |
| P15 | First wave cancel |
| P16 | FSM exit Trotting |

---

## First Block Reason Enumeration

| Code | Name | Source |
|------|------|--------|
| 0 | PRE_WAVE_BLOCK_NONE | C++ |
| 101 | READINESS_HEIGHT_FALSE | C++ |
| 102 | READINESS_STANCE_FALSE | C++ |
| 103 | READINESS_CONTACT_FALSE | C++ |
| 201 | NUMERICAL_GUARD_STATE | C++ |
| 202 | NUMERICAL_GUARD_COMMAND | C++ |
| 203 | NUMERICAL_GUARD_OUTPUT | C++ |
| 301 | SAFETY_GUARD_ATTITUDE | C++ |
| 302 | SAFETY_GUARD_CONTACT | C++ |
| 400 | FIXEDSTAND_UNSTABLE | Python |
| 401 | FALL_BEFORE_TROTTING | Python |
| 402 | FSM_TROTTING_NOT_ENTERED | Python |
| 404 | READINESS_CONTACT_STALE | Python |
| 405 | READINESS_HOLD_INCOMPLETE | Python |
| 406 | WAVE_START_NOT_REQUESTED | Python |
| 407 | WAVE_START_REQUEST_NOT_CONSUMED | Python |
| 408 | WAVE_STATUS_NOT_TRANSITIONED | Python |
| 409-413 | WAVE_CANCELLED_* | Python |
| 414-416 | *_BEFORE_WAVE | Python |
| 500 | UNKNOWN | Python |

---

## Trial Matrix (Pending Execution)

| Trial | vx | Duration | Purpose |
|-------|-----|----------|---------|
| P0 | N/A | ≥3s sim | FixedStand stability only |
| P1 | 0.00 | ≥2s sim | Wave start trigger (expected: no wave) |
| P2 | 0.10 | ≥2s sim | Readiness + wave + fall timeline |
| P3 | 0.50 | Conditional | Only if P1/P2 diverge or old NaN needs verification |

---

## Questions to Answer (from runtime evidence)

The following 20 questions (Section 7 of the Gate P specification) will be
answered after trial execution. Key predictions from static audit:

| # | Question | Static Audit Prediction |
|---|----------|------------------------|
| 1 | FixedStand stable ≥3s? | Expected YES (height ~0.326m) |
| 2 | First height <0.12m time? | TBD (requires trial data) |
| 3 | Fall before/after readiness/wave? | TBD |
| 4 | Trotting state sustained? | Expected YES |
| 5 | height_ready passed? | Expected after 0.75s |
| 6 | stance_ready passed? | Expected YES (setAllStanceNow in enter) |
| 7 | contact_ready passed? | Expected YES (feet on ground) |
| 8 | Contact data fresh? | Expected YES (foot force plugin) |
| 9 | Hold completed? | Expected after 0.20s stability |
| 10 | Wave start called after readiness? | For vx=0: NO (checkStepOrNot fails). For vx≥0.10: Expected YES |
| 11 | WaveGenerator received request? | For vx≥0.10: Expected YES |
| 12 | wave_status entered WAVE_ALL? | For vx≥0.10: Expected YES |
| 13 | First cancel reason? | TBD |
| 14 | Numerical guard triggered? | TBD |
| 15 | Safety guard triggered? | TBD |
| 16 | First non-finite variable? | TBD (may not reproduce) |
| 17 | Stance-phase output destabilization? | TBD |
| 18 | vx=0 vs vx=0.10 share same block? | Expected DIFFERENT (vx=0: step not requested) |
| 19 | vx=0.50 non-finite speed-related? | TBD |
| 20 | Single unique first failure point? | TBD |

---

## Files Changed

### C++ (5 files, +170/-49 lines)
- `src/unitree_guide/.../include/control/CtrlComponents.h`
- `src/unitree_guide/.../include/common/TimingDiagnostics.h`
- `src/unitree_guide/.../src/FSM/FSM.cpp`
- `src/unitree_guide/.../src/FSM/State_Trotting.cpp`
- `src/unitree_guide/.../src/common/TimingDiagnostics.cpp`

### Documentation (3 files)
- `docs/active/.../ADR-010-pre-wave-block-gate-entry.md`
- `docs/active/.../g2-pre-wave-static-audit.md`
- `docs/active/.../evidence-index.md` (updated)

### Python (2 files)
- `experiments/runs/.../pre_wave_block_reason/prewave_analyze.py`
- `experiments/runs/.../pre_wave_block_reason/test_prewave.py`

---

## Commits

```
1a524244 test(g2d1): add pre-wave readiness and block diagnostics
```

---

## Gate P Verdict

**PENDING TRIAL EXECUTION**

The diagnostic infrastructure is built, tested, and committed. The following
remain before verdict:

1. Execute P0 (FixedStand-only stability probe)
2. Execute P1 (Trotting vx=0 probe)
3. Execute P2 (Trotting vx=0.10 probe)
4. Execute P3 (Trotting vx=0.50 probe) only if conditions warrant
5. Analyze controller_state.csv with prewave_* columns
6. Determine first failing checkpoint
7. Classify event timeline (A-F)
8. Issue final verdict

Current diagnostic infrastructure supports all required observations.

---

## G2-R Authorization

**NOT AUTHORIZED**

`G2_D1_PASS_ROOT_CAUSE_IDENTIFIED` has not yet been achieved.
Trials must be executed first.

---

## Recommended Next Branch

After trial execution and verdict, the recovery branch will be one of:

- `fix/0719-g2-readiness-height-source` (if height readiness is the block)
- `fix/0719-g2-readiness-contact-freshness` (if contact staleness blocks)
- `fix/0719-g2-wave-start-request` (if wave start not triggered)
- `fix/0719-g2-pre-wave-output-validity` (if numerical issue before wave)
- `fix/0719-g2-wave-cancel-safety` (if wave cancelled by safety guard)
- `fix/0719-g2-trotting-enter-initialization` (if entry state causes instability)

Only ONE recovery branch will be recommended.
Multiple branches will NOT be created preemptively.

---

## Remaining Risks

1. Existing D0 trial data lacks prewave_* columns → cannot extract readiness
   breakdown without re-running trials.
2. Pre-wave diagnostic fields are write-only and do not affect control behavior
   — but the first-block latch logic in C++ could mask a second distinct failure
   mode that occurs after the latched first block.
3. The static audit predicts vx=0 will never enter WAVE_ALL due to
   checkStepOrNot() returning false. This is expected behavior, not a defect.
4. Non-finite values may not reproduce with the current diagnostic
   configuration.
