# Issue: FALCO A1 Real-Cloud Obstacle Filtering

## Goal

Tune and, if necessary, minimally fix FALCO R3 so real FAST-LIO2 odometry plus
real `/cloud_registered` with `checkObstacle=true` produces a multi-pose path
and finite nonzero raw FALCO `TwistStamped`, while `/navigation/enabled=false`
keeps `/cmd_vel` zero.

## Scope

- Continue in worktree
  `/home/zzf/search_ws/SimEnv_worktrees/falco-dsv-navigation`.
- Continue on branch `feat/0723-falco-dsv-navigation-integration`.
- R3 only: Gazebo, FAST-LIO2, FALCO, and the gated bridge.
- Audit actual A1 geometry from the loaded xacro/URDF/collision model.
- Add switchable, throttled diagnostics if internal FALCO counts are required.
- Land A1-specific FALCO parameters in SimEnv-owned config.

## Non-Scope

- Do not modify `/home/zzf/search_ws/SimEnv`.
- Do not touch `master`, push, or merge.
- Do not modify Gazebo physics, collision models, generated building logic,
  FAST-LIO2 core, Trotting/RL core, or command safety gates.
- Do not permanently disable `checkObstacle`.
- Do not enter R4 Trotting, R5 DSV, or R6 full exploration.

## Acceptance Criteria

- `./tools/build_with_venv.sh` passes.
- A1 geometry audit is recorded.
- FALCO diagnostics identify whether blockage comes from self points, ground
  points, footprint, height filtering, or thresholding.
- Final runtime uses `checkObstacle=true`.
- A front waypoint and slight left/right front waypoints produce path
  `poses.size() > 1`, finite nonzero raw FALCO TwistStamped, and zero gated
  `/cmd_vel` with navigation disabled.
- A wall/blocked waypoint regression does not accept an obvious direct
  through-wall path.

## Risks

- Low RTF can make sim-time windows expensive.
- Python/rospy pointcloud collectors can perturb runtime load.
- Vendor diagnostics must not change FALCO planning behavior.

## Expected Impacted Modules

- `src/navigation/simenv_navigation_bringup/`
- possible minimal diagnostics in `src/navigation/vendor/falco/local_planner/`
- evidence under `experiments/runs/0724_falco_a1_tuning/`
