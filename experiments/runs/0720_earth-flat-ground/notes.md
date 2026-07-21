# Notes: `earth.world` Flat-Ground Fix

## Context

- Branch: `fix/0720-earth-flat-ground`
- Base: `2909093d68dbddf135575147198e32cf95bd5f01`
- Worktree: `/home/zzf/search_ws/SimEnv_worktrees/earth-flat-ground`
- Related existing worktree preserved: `/home/zzf/search_ws/SimEnv_worktrees/earth-rl-motion-validation`

## Files Read

- `AGENTS.md`
- `PROJECT_STATE.md`
- `CHANGELOG.md`
- `docs/module_status.md`
- `auto.sh`
- `src/unitree_guide/unitree_guide/unitree_guide/launch/multi_floor_gazeboSim.launch`
- `src/unitree_guide/unitree_ros/unitree_gazebo/launch/normal.launch`
- `src/unitree_guide/unitree_ros/unitree_gazebo/worlds/earth.world`

## World Structure Before Fix

- `earth.world` contained:
  - `model://sun`
  - `model://ground_plane`
  - inline `platform_1` at the robot spawn center with box visual and collision
  - inline `platform_2` forward of spawn with box visual and collision
- `platform_1` covered `x=0 y=0`, matching the `WORLD_MODE=earth` spawn center.

## Spawn Pose

- `auto.sh` earth defaults:
  - `x=0.0`
  - `y=0.0`
  - `z=0.6`
  - `yaw=0.0`
- `roll=0` and `pitch=0` are implicit because `multi_floor_gazeboSim.launch` only passes `-Y`.
- The `z=0.6` value matches the existing Unitree A1 `normal.launch` spawn height.

## Change

- Removed complete inline model blocks:
  - `platform_1`
  - `platform_2`
- Kept physics, scene, `model://sun`, and exactly one `model://ground_plane` include.
- No launch, controller, robot model, RL, FSM, gait, IK, estimator, or physics parameter changes were made.

## Validation

- `python3` XML parse: PASS
- XML content check:
  - no `platform_1`
  - no `platform_2`
  - no platform collision names
  - no inline world models
  - includes are `model://sun` and `model://ground_plane`
  - one ground plane include
  - `WORLD_MODE` default remains `competition`
  - earth mode still resolves to repository `earth.world`
- `gz sdf -k`: PASS
- `git diff --check`: PASS
- `bash -n auto.sh`: PASS
- `gzserver --verbose earth.world` was run under `timeout 15s`; it remained running until timeout and logged only the Gazebo 11.10.2 startup banner, with no parse/load error in the captured tail.

## Runtime Gap

This isolated worktree has no `devel/setup.bash`, so a full A1 spawn plus FixedStand/RL smoke was not claimed here. Re-run in a built overlay before marking the motion benchmark fully recovered.
