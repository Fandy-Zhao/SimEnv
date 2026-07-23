# Issue: FAST-LIO2 Clean Runtime Validation

## Goal

Continue from commit `30ea47bd` on
`fix/0723-fast-lio2-reproducible-build-pointcloud` and move the FAST-LIO2
runtime result from `REPRO_BUILD_PASS_POINTCLOUD_BLOCKED` to either a clean
runtime validation pass or a confirmed runtime root cause.

## Scope

- Preserve the reproducible external dependency staging result.
- Establish a fresh isolated ROS master and run `./auto.sh` exactly once per
  clean validation attempt.
- Verify `/scan`, `/scan_pointcloud2`, `/trunk_imu`, `/Odometry`,
  `/cloud_registered`, `/clock`, FAST-LIO2 logs, TF connectivity, pointcloud
  effective points, and pause/resume behavior.
- Only modify diagnostic tooling or minimal FAST-LIO2 integration code after a
  clean baseline run proves a specific issue.

## Non-Scope

- Do not modify root workspace branch, HEAD, generated scenes, logs, or results.
- Do not modify stable external FAST_LIO, ikd-Tree, or `livox_ros_driver`.
- Do not change RL, navigation, exploration, Gazebo physics profiles, or scene
  geometry.
- Do not change `state_from_gazebo.cpp` unless a later, explicitly proven
  runtime root cause requires it.

## Acceptance Criteria

- Root workspace is preserved.
- External repositories remain clean and at fixed commits.
- Protected `state_from_gazebo.cpp` SHA256 is unchanged.
- A clean isolated ROS master is used, with no stale publisher registrations.
- Publisher ownership, 30-second wall-clock continuity, effective PointCloud2
  statistics, FAST-LIO2 output, TF/orientation, and pause/resume gates are
  either passed or fail with a confirmed root cause and reproducible evidence.
- Any production fix is minimal and fully regressed through the same clean
  runtime flow.

## Risks

- ROS master stale registrations can make publisher ownership look worse than
  the process table.
- Gazebo startup and controller preflight have variable wall-clock duration.
- Diagnostic subscribers must avoid using simulation time as their wall-clock
  duration source.

## Impacted Modules

- `tools/diagnostics`
- `src/simenv_fast_lio2_integration`
- `auto.sh` only if clean evidence proves startup/cleanup ownership is the root
  cause
- Governance docs if any code or validation status changes
