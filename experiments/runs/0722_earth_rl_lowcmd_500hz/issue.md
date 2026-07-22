# Issue: Effective LowCmd 500 Hz Validation

Date: 2026-07-22
Branch: fix/0722-earth-rl-lowcmd-500hz
Baseline: e7fbbe639412fbd528a2fc35dc3009aa18c9af83
Worktree: /home/zzf/search_ws/SimEnv_worktrees/earth-rl-lowcmd-500hz

## Goal

Validate the full LowCmd timing chain and, if supported by evidence, minimally fix the effective applied LowCmd update cadence so Gazebo applies LowCmd updates at approximately 500 Hz per simulation second.

## Scope

- Trace RL/FixedStand action generation through LowCmd construction, ROS publish/send, Gazebo controller receive, joint target/torque update, and physics-step application.
- Distinguish generated, published, received, and applied LowCmd timing.
- Measure duplicate simulation timestamps, repeated old commands, update gaps, deadline misses, and callback backlog symptoms.
- Add bounded diagnostics or a minimal timing fix only where evidence shows the applied command path is below target.
- Validate FixedStand first, then RL zero action.

## Non-Scope

- Do not edit shared governance files: CHANGELOG.md, PROJECT_STATE.md, docs/module_status.md, or experiments/runs/0722_earth_rl_fastcheck/summary.md.
- Do not change policy observation semantics, RL action semantics, or .pt model files.
- Do not modify the physics profile.
- Do not commit large CSV files, rosbags, build/devel artifacts, generated_building output, or full runtime logs.

## Acceptance Criteria

LOWCMD_500HZ_PASS requires:

- Applied LowCmd median rate between 475 and 525 Hz per simulation second.
- Published/applied rate difference <= 3%.
- Duplicate simulation-time update ratio <= 1%.
- No sustained > 10 ms simulation-time control gap.
- No callback burst backlog evidence.
- FixedStand is stable.
- RL zero LowCmd chain also satisfies the same timing metrics.

If any criterion is not met, report LOWCMD_500HZ_FAIL with evidence.

## Risks

- Gazebo/headless runtime may fail due display, ROS master, or environment setup issues.
- Real-time factor is expected to vary and should be recorded but not used as the stop gate.
- Existing generated/build/devel files may change during runtime and must remain uncommitted.

## Expected Impacted Modules

- unitree_guide control loop and I/O bridge.
- unitree_gazebo joint controller LowCmd receive/apply path.
- Experiment notes and compact evidence under this run directory.
