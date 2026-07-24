# Issue: FALCO R3 Real-Data Path Generation

## Goal

Diagnose and minimally fix FALCO R3 when real FAST-LIO2 input yields only a
single zero pose on `/navigation/path` and zero
`/navigation/falco/cmd_vel_stamped`.

## Scope

- Use the existing worktree and branch:
  `/home/zzf/search_ws/SimEnv_worktrees/falco-dsv-navigation`,
  `feat/0723-falco-dsv-navigation-integration`.
- Re-run only R3 with Gazebo, FAST-LIO2, FALCO, and the gated bridge.
- Validate actual runtime parameters, semantic front-facing waypoints, and
  Case A/B/C planner behavior.
- Store evidence under this experiment directory.

## Non-Scope

- Do not modify `/home/zzf/search_ws/SimEnv`.
- Do not touch `master`, push, or merge.
- Do not enter R4 Trotting, R5 DSV, or R6 full exploration.
- Do not change Gazebo physics, collision geometry, FAST-LIO2 core,
  Trotting/RL core, or permanently disable FALCO obstacle checks.

## Acceptance Criteria

- Real `/navigation/state_estimation` is continuously valid.
- Real `/navigation/registered_scan` is continuously non-empty.
- The waypoint is generated about 0.8 m in front of the robot in the odometry
  frame observed from the live message.
- `/navigation/path` has more than one pose and aligns with the waypoint.
- `/navigation/falco/cmd_vel_stamped` has finite nonzero values.
- With `/navigation/enabled=false`, `/cmd_vel` remains zero.
- No message type conflicts or critical node exits occur.

## Risks

- Low RTF may stretch wall-clock validation windows.
- FALCO may require runtime-only diagnostics to distinguish obstacle filtering
  from waypoint/path selection.
- ROS master pollution from previous runs can invalidate topic ownership and
  timing measurements.

## Expected Impacted Modules

- `src/navigation/simenv_navigation_bringup/`
- possibly `src/navigation/vendor/falco/local_planner/` for minimal,
  switchable diagnostics only if runtime evidence requires it.
