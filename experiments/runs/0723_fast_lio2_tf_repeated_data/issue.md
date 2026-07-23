# FAST-LIO2 TF Repeated Data Fix

## Goal

Diagnose and minimally fix repeated timestamp warnings on dynamic TF edges used by the competition Gazebo + FAST-LIO2 stack.

## Scope

- Audit dynamic/static TF publishers for `map`, `odom`, `base`, `camera_init`, `body`, and `imu_link`.
- Collect static and, where possible, runtime evidence before changing behavior.
- Fix confirmed duplicate timestamp or duplicate owner causes without masking warnings.
- Update governance/status documentation and record validation.

## Non-Scope

- No changes to root worktree state.
- No branch switches, reset, stash, cleanup, or push from the root worktree.
- No large FAST-LIO2 algorithm rewrite.
- No warning suppression or fake timestamp increments.

## Acceptance Criteria

- Dynamic TF ownership is documented.
- `map -> odom` and `odom -> base` do not republish identical simulation timestamps from backlog/pause handling.
- FAST-LIO2 `camera_init -> body` ownership is checked for double publication.
- `/Odometry_gazebo` and its corresponding TF share one callback stamp.
- Pause, zero clock, and clock rollback behavior is explicitly handled.

## Risks

- Full Gazebo runtime validation may be limited by display, ROS environment, or missing built workspace.
- TF authority is not carried directly in raw ROS1 `/tf` message payloads, so attribution may require graph and node-level evidence.

## Expected Impacted Modules

- `src/unitree_guide/unitree_guide/unitree_guide/src/state_from_gazebo.cpp`
- `src/simenv_fast_lio2_integration/scripts/odometry_tf_bridge.py`
- `src/simenv_fast_lio2_integration/scripts/map_to_camera_init_bridge.py`
- `src/simenv_fast_lio2_integration/launch/simenv_fast_lio2_mapping.launch`
- `auto.sh`
