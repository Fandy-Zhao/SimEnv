# 0721 RL Fast Validation Summary

Verdict: `FAIL_BASE_HEIGHT`

## Directories

- `environment/` — env snapshots.
- `raw/build/` — build logs.
- `raw/runtime/` — runtime logs.
- `captures/` — capture artifacts.
- `metrics/` — metric JSON/CSV output.
- `scripts/` — runner, capture, replay, metrics.
- `tests/` — unit tests.
- `hashes/` — artifact provenance hashes.

## Scripts

- `scripts/rl_fast_metrics.py` — pure ROS-free helper functions.
- `scripts/run_rl_fast_smoke.sh` — bash runner for the F0/F1/F2 FixedStand gate.
- `scripts/rl_fast_capture.py` — Python capture placeholder.
- `scripts/rl_fast_live_capture.py` — live ROS/Gazebo FixedStand capture.
- `scripts/replay_rl_state.py` — offline replay scaffold.
- `tests/test_rl_fast_metrics.py` — unit tests for pure helpers.
- `tools/build_rl_fast.sh` — thin build wrapper.

## Build

- Requested build script:
  `/home/zzf/search_ws/SimEnv/tools/build_with_venv.sh`
- Effective build script:
  `/home/zzf/search_ws/SimEnv_worktrees/rl-fast-validation/tools/build_with_venv.sh`
- Script SHA256:
  `fc25fc0fd38c3f9f652475f45e1926bde8e622db40c11ea38b116ceaf0949d48`
- Fast command:
  `./tools/build_rl_fast.sh`
- Fast result: PASS, exit code 0.
- Runtime profile:
  `./tools/build_with_venv.sh`, PASS, exit code 0.
- Full build attempt:
  `SIMENV_CATKIN_WHITELIST="" ./tools/build_with_venv.sh`, PASS, exit code 0.

## Artifact Provenance

- `ROS_PACKAGE_PATH` starts with
  `/home/zzf/search_ws/SimEnv_worktrees/rl-fast-validation/src`.
- `CMAKE_PREFIX_PATH` starts with
  `/home/zzf/search_ws/SimEnv_worktrees/rl-fast-validation/devel`.
- `rospack find unitree_guide`, `unitree_gazebo`, and
  `unitree_legged_control` resolve inside this worktree.
- Key artifact hashes are recorded in `hashes/artifact_provenance.txt`.
- Stair policy path:
  `/home/zzf/search_ws/SimEnv/src/unitree_guide/logs/policy_act_inference_stair.pt`
- Stair policy SHA256:
  `2d5aa72511c0c6609c02f4105845eee6974d3d73431497f8f35306da9588fe14`

## Validation Checks

- `bash -n`: passed on all `.sh` files.
- Python `py_compile`: passed on all `.py` files.
- Python unit tests: passed, 37 tests.
- Placeholder fast smoke: `TASK_PARTIAL`, with policy hash captured.
- Live F0 native FixedStand: `FAIL_BASE_HEIGHT`; FixedStand entered and runtime
  data were captured, but evaluation-window base height stayed below 0.12 m for
  0.565 s. Median evaluation-window base height was 0.272 m; max tilt was
  4.57 deg; median RTF was 0.684, so the result also carries `LOW_RTF_RISK`.
- Offline replay scaffold: `OFFLINE_REPLAY_SCAFFOLD_ONLY`, policy hash matched,
  metadata validator passed.

## Status

The FixedStand live runner is implemented through F0/F1/F2 gating. F0 now
launches Gazebo, confirms `/clock` and model states, starts `junior_ctrl`,
enters FixedStand, splits transition/evaluation windows, and emits fine-grained
metrics. F1/F2 are gated behind F0 and were not run because F0 failed
`FAIL_BASE_HEIGHT`. RL state/action diagnostics and C++ observation/action reuse
remain future work.

## First-Run Matrix

| Stage | Verdict | Sim Duration | RTF | Tilt | Height | Forward | Lateral | Yaw | Notes |
| ----- | ------- | -----------: | --: | ---: | -----: | ------: | ------: | --: | ----- |
| F0 native FixedStand | FAIL_BASE_HEIGHT | 4.0s requested | 0.684 median | 4.57 deg max | 0.0566 m min eval; 0.565s below 0.12m | | | | `a1_gazebo` + `ground_plane`; FixedStand entered |
| F1 competition FixedStand | NOT_RUN | | | | | | | | Gated by F0 failure |
| F2 earth FixedStand | NOT_RUN | | | | | | | | Gated by F0 failure |
| F3 earth RL zero | NOT_RUN | | | | | | | | Gated by F2 not run |
| F4 earth vx=0.05 | NOT_RUN | | | | | | | | Gated by F3 not run |
| F5 earth stop | NOT_RUN | | | | | | | | Gated by F4 not run |

## Cheap-Code-Worker Review

- Worker generated the initial scaffold files under
  `experiments/runs/0721_rl-fast-validation/` and `tools/build_rl_fast.sh`.
- Worker did not run validation commands because command approval was pending.
- Main executor found and fixed:
  - threshold-duration tests and implementation used endpoint inflation;
  - verdict names used old broad `FAIL_ATTITUDE`/`FAIL_CLOCK_STALL` categories;
  - runner heredoc argument passing was broken;
  - placeholder smoke incorrectly emitted `BLOCKED` instead of `TASK_PARTIAL`;
  - default port allocation reused the common 11311 range;
  - replay metadata rejected valid `privileged_dim=0`.
- All task conclusions, build execution, artifact provenance, and final
  `TASK_PARTIAL` verdict are from the main executor.
