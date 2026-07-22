# Notes: Earth RL Deployment Semantics

Date: 2026-07-22
Branch: fix/0722-earth-rl-deployment-semantics
Baseline: e7fbbe639412fbd528a2fc35dc3009aa18c9af83

## Pre-Edit State

- `git status --short`: clean before creating this run directory.
- `git branch --show-current`: `fix/0722-earth-rl-deployment-semantics`
- `git rev-parse HEAD`: `e7fbbe639412fbd528a2fc35dc3009aa18c9af83`

## Commands And Evidence

- Initial full `catkin_make -j` failed during CUDA detection because CMake selected GCC 12 for `nvcc`; retrying with `gcc-11/g++-11` passed CUDA detection.
- Workspace-wide build then failed in unrelated optional packages (`unitree_move_base` missing `move_base_msgs`, UAV packages compiling as pre-C++17 against log4cxx).
- Targeted build command succeeded:
  `source /opt/ros/noetic/setup.bash && CC=/usr/bin/gcc-11 CXX=/usr/bin/g++-11 catkin_make --pkg unitree_guide -j`
- Python syntax check succeeded:
  `python3 -m py_compile experiments/runs/0722_earth_rl_deployment_semantics/transition_capture.py`
- Python TorchScript shape probe could not run because system Python has no `torch` module. The C++ controller loaded `src/unitree_guide/logs/policy_act_inference_stair.pt` successfully during runtime startup.

## Baseline Reproduction

Command:

`WORLD_MODE=earth PHYSICS_PROFILE=normal GUI=False ./auto.sh`

Capture:

`python3 experiments/runs/0722_earth_rl_deployment_semantics/transition_capture.py --output-dir experiments/runs/0722_earth_rl_deployment_semantics`

Artifacts:

- `baseline_transition_zero.csv`
- `baseline_transition_zero_summary.json`

Summary:

- Fresh Earth normal epoch started with ground truth disabled.
- FixedStand was stable before RL entry: last pre-RL sample at sim `116.316`, base height `0.323724 m`, tilt `0.243639 deg`, base vx `0.001839 m/s`.
- RL zero-command fell: minimum post-RL base height `0.093391 m`, maximum post-RL tilt `174.044332 deg`.
- First captured RL sample, at `0.616 s` after switch, already diverged: base height `0.285792 m`, tilt `10.250012 deg`, base vx `0.457334 m/s`, IMU angular velocity `[0.604804, -2.695873, 0.762977] rad/s`.
- Worst height sample, at `2.716 s` after switch: base height `0.093391 m`, tilt `159.820799 deg`.
- First ten captured post-RL motor target absolute max was `3.173030 rad`; the transition had large joint target jumps from FixedStand.

## Semantic Findings

- `WORLD_MODE=earth` defaults `ENABLE_GROUND_TRUTH=0`, so `/ground_truth/base_w` is not published.
- `IOROS` still copied `_base_w_ori` and `_base_w_angular_vel` into the RL policy snapshot and marked the snapshot valid based on low/joint state readiness only.
- `IOInterface` default `_base_w_ori` is `{0.0, 0.0, 0.0, 0.1}`, which is not the identity orientation expected by the policy observation path.
- The RL observation therefore used invalid projected gravity and stale angular velocity immediately at deployment while other state streams appeared valid.
- Joint order mapping was inspected and matches the existing Gazebo-to-policy reindex pattern. No `.pt`, physics, LowCmd scheduling, broad gain, or action-freezing change was made.

## Fix

- Added `base_world_valid` to policy input snapshots and set it only after `/ground_truth/base_w` has been received.
- In RL observation refresh, keep using ground-truth base orientation/angular velocity when valid.
- When ground truth is unavailable, fall back to `/trunk_imu` orientation for projected gravity and IMU gyroscope for body angular velocity.
- Added opt-in RL deployment diagnostics gated by `UNITREE_RL_DIAG_PATH`; it records command, history, observation segments, raw/scaled actions, final joint targets, defaults, and reindex values.

## Post-Fix Runtime Attempt

Command:

`DISPLAY= WAYLAND_DISPLAY= TERMINAL_BACKEND=direct TMUX_SESSION_PREFIX=simenv-b SKIP_GLOBAL_PROCESS_CLEANUP=1 ROS_MASTER_URI=http://localhost:12411 GAZEBO_MASTER_URI=http://localhost:12445 UNITREE_RL_DIAG_PATH=experiments/runs/0722_earth_rl_deployment_semantics/postfix_zero_rl_diag.csv WORLD_MODE=earth PHYSICS_PROFILE=normal GUI=False ./auto.sh`

Result:

- The isolated epoch reached `auto.sh startup complete` and `junior_ctrl` loaded `policy_act_inference_stair.pt`.
- Gazebo then died with exit code `-9` before FixedStand/RL switching could be captured.
- The default ROS/Gazebo epoch from `earth-rl-lowcmd-500hz` remained active on port `11311`; only this worktree's orphaned isolated `auto.sh` process was killed by PID with `SIGKILL` to avoid running `auto.sh` global cleanup.
- No post-fix zero-command or `vx=0.10` runtime stability evidence was captured.

## Status

`RL_DEPLOYMENT_FAIL`: the semantic fix is compiled and root cause evidence supports it, but pass criteria require runtime stability evidence that was not obtained in this shared ROS/Gazebo environment.
