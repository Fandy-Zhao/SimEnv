# G2 Trotting Motion Baseline

## Branch

`test/0718-g2-trotting-motion-baseline` at `7723be02`.

## Baseline

G2-B tooling ran one smoke trial at each required command speed:
`0.00`, `0.10`, `0.30`, and `0.50 m/s`. All four trials produced ground-truth,
controller timing, foot-force, joint-state, event, status, and metric files.
All four trials were `INVALID`; therefore no speed point has valid trials for
performance averaging.

## G1 Prerequisite

Inherited status: G1_R_PASS for the normal 2 ms, no-pause/no-reset path.
Inherited risks remain open for the 4 ms effective gait, pause/reset extended
matrix, and RL timing.

## Runtime Configuration

- `FLOOR_COUNT=1`
- `SEED=77`
- `GUI=false`
- `START_CONTROLLER=1`
- `ENABLE_FAST_LIO2=0`
- `ENABLE_FOOT_FORCE_VISUAL=1`
- Private ROS masters: ports `12801`, `12811`, `12831`, `12851`
- Controller timing diagnostics path: per-trial `controller_state.csv`

## Trial Matrix

| Command | Total runs | Valid runs | Invalid runs | Result |
| ------: | ---------: | ---------: | -----------: | ------ |
| 0.00 | 1 | 0 | 1 | INCONCLUSIVE |
| 0.10 | 1 | 0 | 1 | INCONCLUSIVE |
| 0.30 | 1 | 0 | 1 | INCONCLUSIVE |
| 0.50 | 1 | 0 | 1 | INCONCLUSIVE |

## Validity Checks

All four trials entered FixedStand and Trotting, and all four recorded fresh
foot-force callbacks. All four failed the same validity gates:

- `WAVE_ALL_NOT_REACHED`
- `GAIT_NOT_ADVANCING`
- `FALL_DETECTED`

Controller timing showed `wave_status=0` throughout the active command windows
and `gait_cycle_sequence=0` throughout all active command windows.

## Command Path

Nonzero command trials did reach the controller resolved command path:

| Command | Resolved vx range |
| ------: | ----------------: |
| 0.10 | 0.0 to 0.1 |
| 0.30 | 0.0 to 0.3 |
| 0.50 | 0.0 to 0.5 |

This rules out a complete `/cmd_vel` publication failure in these smoke trials.

## Speed Tracking

These values are diagnostic only because every trial is invalid:

| Command | Steady mean vx | Tracking ratio | Tail speed | Invalid reasons |
| ------: | -------------: | -------------: | ---------: | --------------- |
| 0.00 | -0.0044 | n/a | 0.0634 | fall, no WAVE_ALL, no gait |
| 0.10 | -0.0073 | -0.0730 | 0.0582 | fall, no WAVE_ALL, no gait |
| 0.30 | -0.0037 | -0.0122 | 0.0817 | fall, no WAVE_ALL, no gait |
| 0.50 | -0.0188 | -0.0375 | 0.0760 | fall, no WAVE_ALL, no gait |

## Attitude

All four trials reported roll peaks near 180 deg and minimum model height near
0.079 m, which is why the trial validator recorded `FALL_DETECTED`.

## Saturation

LowCmd/joint saturation metrics are not accepted for averaging because no trial
reached a valid WAVE_ALL gait window. The `vx=0.50` controller pane did capture
a stronger pre-gait symptom: Trotting output became non-finite with `q=0`, and
the controller cancelled the wave before gait execution.

## Evidence Paths

- `experiments/runs/0718_g2_trotting_motion_baseline/summary.csv`
- `experiments/runs/0718_g2_trotting_motion_baseline/summary.json`
- `experiments/runs/0718_g2_trotting_motion_baseline/baseline/vx_000/vx_000_run_01/`
- `experiments/runs/0718_g2_trotting_motion_baseline/baseline/vx_010/vx_010_run_01/`
- `experiments/runs/0718_g2_trotting_motion_baseline/baseline/vx_030/vx_030_run_01/`
- `experiments/runs/0718_g2_trotting_motion_baseline/baseline/vx_050/vx_050_run_01/`

## Tests

- `/usr/bin/python3 -m py_compile` for G2 Python tooling: pass.
- `/usr/bin/python3 -m unittest` for metric helpers: 6 tests pass.
- `bash -n` for G2 shell tooling: pass.
- Scoped `git diff --check` for touched G2 paths: pass after formatting fixes.

## G2 Baseline Verdict

`G2_BASELINE_INCONCLUSIVE`

Reason: zero valid trials. The evidence supports a pre-performance validity
failure before WAVE_ALL/gait execution, not a completed speed-tracking baseline.

## Conditions Before Recovery

Do not enter G2-R yet. A recovery branch needs a tighter single-root-cause
investigation for the non-finite Trotting output and fall posture, preferably
with a focused pre-WAVE diagnostic that captures the exact joint target/source
without changing controller behavior.
