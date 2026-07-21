# 0721 RL Fast Validation Report

Verdict: `FAIL_BASE_HEIGHT`

## Scope

This task created governed infrastructure for fast RL state validation:
thin build wrapper, pure metric helpers, F0/F1/F2 FixedStand runner, live
capture, placeholder capture, offline replay scaffold, tests, build logs, and
provenance.

## Non-Scope

No RL policy, FSM, observation/action, estimator, gait, IK, controller gains,
URDF/xacro, spawn pose, Gazebo physics, `earth.world`, or fall threshold was
modified.

## Build

- Requested script: `/home/zzf/search_ws/SimEnv/tools/build_with_venv.sh`
- Effective script: worktree tracked copy `tools/build_with_venv.sh`
- SHA256: `fc25fc0fd38c3f9f652475f45e1926bde8e622db40c11ea38b116ceaf0949d48`
- Fast build: `./tools/build_rl_fast.sh`, PASS.
- Runtime profile: `./tools/build_with_venv.sh`, PASS.
- Full attempt: `SIMENV_CATKIN_WHITELIST="" ./tools/build_with_venv.sh`, PASS.

## Infrastructure

- `tools/build_rl_fast.sh` only sets the Unitree runtime whitelist and delegates
  to `tools/build_with_venv.sh`.
- `rl_fast_metrics.py` provides ROS-free helpers for evaluation windows,
  sustained threshold duration, verdict priority, NaN/Inf detection,
  local-frame displacement, port allocation, artifact provenance, policy hash,
  RTF gate, FixedStand window summaries, and clock/master failure categories.
- `run_rl_fast_smoke.sh` now runs the F0/F1/F2 FixedStand gate on dedicated
  ROS/Gazebo ports, records provenance, retries native controller loading when
  needed, and cleans up owned process groups plus private Gazebo master ports.
- `rl_fast_live_capture.py` performs the live FixedStand capture and writes
  `metrics.json`, `verdict.json`, `timeseries.csv`, `clock_rtf.csv`,
  `joint_states.csv`, and `summary.md`.
- `replay_rl_state.py` validates fixture metadata and policy hash, but does not
  yet reuse C++ observation/action code.

## Tests

- `bash -n`: PASS.
- Python `py_compile`: PASS.
- Unit tests: PASS, 37 tests.
- Placeholder smoke: PASS as `TASK_PARTIAL`.
- Live F0 native FixedStand: executed and classified as `FAIL_BASE_HEIGHT`.
- Offline replay scaffold: PASS as `OFFLINE_REPLAY_SCAFFOLD_ONLY`.

## First-Run Matrix

| Stage | Verdict | Evidence |
| --- | --- | --- |
| F0 native FixedStand | `FAIL_BASE_HEIGHT` | `experiments/runs/0721_rl-fast-validation/raw/runtime/live_fixedstand_f0_controller_retry/F0_native_fixedstand/` |
| F1 competition FixedStand | `NOT_RUN` | gated by F0 failure |
| F2 earth FixedStand | `NOT_RUN` | gated by F0 failure |
| F3 earth RL zero | `NOT_RUN` | RL state diagnostics not implemented |
| F4 earth vx=0.05 | `NOT_RUN` | RL state diagnostics not implemented |
| F5 earth stop | `NOT_RUN` | RL state diagnostics not implemented |

F0 entered FixedStand and captured live runtime data. The model list was
`a1_gazebo`, `ground_plane`, with no Earth high-platform models in the native
run. The evaluation-window minimum base height was 0.0566 m and time below
0.12 m was 0.565 s, so the gate failed as `FAIL_BASE_HEIGHT`. Max evaluation
tilt was 4.57 deg. Median RTF was 0.684, recorded as `LOW_RTF_RISK`.

## Worker Record

`cheap-code-worker` generated the initial scaffold files. Main executor fixed
the metric semantics, verdict taxonomy, runner argument passing, placeholder
verdict, port range, and replay metadata validation, then ran all validation
and builds.

## Risks

- RL first action/history/estimator telemetry and C++ observation/action reuse
  are not implemented yet.
- F0 failure prevents credible F1/F2 comparison until the native FixedStand
  base-height failure is understood.
- Median RTF for F0 was below 0.8, so the F0 result is useful as a failure
  classification but still carries limited-smoke timing risk.

## Next Step

Investigate why native FixedStand briefly sinks below the 0.12 m base-height
threshold in the evaluation window, then rerun F0. Only after F0 passes should
F1/F2 and the stair-policy RL state gates run.
