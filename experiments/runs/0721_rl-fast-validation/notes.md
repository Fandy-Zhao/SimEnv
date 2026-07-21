# 0721 RL Fast Validation Notes

## Scope

Governed infrastructure for RL fast validation experiments. This task creates
the directory structure, pure helpers, runner/capture/replay tools, and a thin
build wrapper without changing controller semantics.

## Scaffolding Decisions

- `rl_fast_metrics.py`: ROS-free pure functions, importable from capture and replay
  scripts, fully unit-testable.
- `run_rl_fast_smoke.sh`: bash runner that executes the F0/F1/F2 FixedStand
  gate on dedicated ROS/Gazebo ports, records provenance, scopes cleanup to
  owned processes and ports, retries native controller loading when needed, and
  defers metrics to Python live capture.
- `rl_fast_capture.py`: Python placeholder that imports `rl_fast_metrics` and writes
  `metrics.json`, `verdict.json`, `timeseries.csv`, and `summary.md` for
  preflight-only or blocked runs.
- `rl_fast_live_capture.py`: live ROS capture that waits for `/clock`,
  `/gazebo/model_states`, the A1 model, and controller timing diagnostics;
  publishes FixedStand, separates transition grace from evaluation, and emits
  `PASS`, `FAIL_FSM_ENTRY`, `FAIL_GAZEBO_SIM_STALL`, `FAIL_BASE_HEIGHT`,
  `FAIL_TILT`, or RTF-gated verdicts.
- `replay_rl_state.py`: offline replay scaffold with fixture parser, metadata
  validator, policy hash verifier, dimension fields, and explicit
  `OFFLINE_REPLAY_SCAFFOLD_ONLY` status.
- `test_rl_fast_metrics.py`: covers evaluation window selection, sustained
  threshold duration with linear crossing interpolation, verdict priority,
  NaN/Inf detection, local-frame displacement, port allocation, path validation,
  policy SHA256 validation, RTF/fixedstand helpers, and clock/master failure
  classification.
- `tools/build_rl_fast.sh`: thin wrapper over `tools/build_with_venv.sh` with
  Unitree RL whitelist and logged output.

## Main Executor Review of Worker Output

The worker created the initial scaffold but did not run validation commands.
Main executor found and fixed:

- duration helpers counted full intervals when only one endpoint crossed a
  threshold; now threshold crossing duration is interpolated;
- verdict names used broad legacy classes; helpers now use the requested
  fine-grained failure taxonomy;
- clock stall classification no longer maps to attitude failure;
- runner heredoc argument passing was invalid;
- placeholder smoke now emits `TASK_PARTIAL` instead of `BLOCKED`;
- default ports now start at `12111`/`12145`;
- replay metadata allows `privileged_dim=0`.

## Build and Provenance

- Root and worktree `tools/build_with_venv.sh` SHA256 match:
  `fc25fc0fd38c3f9f652475f45e1926bde8e622db40c11ea38b116ceaf0949d48`.
- Effective build script is the current worktree tracked copy.
- `./tools/build_rl_fast.sh` passed with Unitree/Torch/Gazebo whitelist.
- `./tools/build_with_venv.sh` default runtime profile passed.
- `SIMENV_CATKIN_WHITELIST="" ./tools/build_with_venv.sh` full attempt passed.
- Artifact provenance is recorded in `hashes/artifact_provenance.txt`.

## Live FixedStand Result

- Command:
  `CASE_FILTER=F0_native_fixedstand CAPTURE_ROOT=experiments/runs/0721_rl-fast-validation/raw/runtime/live_fixedstand_f0_controller_retry EVALUATION_DURATION=3.0 CAPTURE_WALL_TIMEOUT=240 ./experiments/runs/0721_rl-fast-validation/scripts/run_rl_fast_smoke.sh`
- Result: `FAIL_BASE_HEIGHT`.
- Evidence: `raw/runtime/live_fixedstand_f0_controller_retry/F0_native_fixedstand/`.
- `/clock` and `/gazebo/model_states` were captured.
- Model list was `a1_gazebo`, `ground_plane`; no `platform_1`/`platform_2`
  appeared in F0 native.
- FixedStand was entered after 0.534 s wall latency.
- Evaluation-window minimum base height: 0.0566 m.
- Evaluation-window duration below 0.12 m: 0.565 s.
- Steady-state median base height: 0.272 m.
- Evaluation-window max tilt: 4.57 deg.
- Median RTF: 0.684, recorded as `LOW_RTF_RISK`.
- F1/F2 were not run because the matrix gates later cases behind F0 pass.

## Runtime Iteration Notes

- First native attempt failed before Gazebo startup because conda Python 3.13
  contaminated ROS `xacro`; the runner now unsets Python env and prioritizes
  `/usr/bin` before sourcing ROS.
- Second native attempt produced `/clock` only after changing the native launch
  arg from `paused=true` to `paused=false`.
- Third attempt reached `/clock` and model states, but controller feedback was
  not ready because the original controller spawner exited before
  `gazebo_ros_control` was available. The runner now retries the native
  controller spawner before starting `junior_ctrl`.
- Cleanup now kills the process group owning the private Gazebo master port, so
  abnormal exits do not leave a `gzserver` listener on the next run's port.

## Non-Scope

No RL policy, FSM, observation/action, estimator, gait, IK, controller gains,
URDF/xacro, spawn, Gazebo physics, `earth.world`, or fall threshold changes.
No C++ edits. Live FixedStand F0 is claimed as executed and failed; RL state
smoke is not claimed.
