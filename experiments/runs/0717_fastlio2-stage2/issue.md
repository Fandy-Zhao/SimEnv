# Issue — FAST-LIO2 Stage 2 Navigation Interface

## Goal

Expose stable navigation-facing FAST-LIO2 outputs as `/state_estimation`
(`nav_msgs/Odometry`) and `/registered_scan` (`sensor_msgs/PointCloud2`).

## Scope

- Add transparent ROS relays in `simenv_fast_lio2_integration` while retaining
  the legacy FAST-LIO2 topics.
- Keep the existing `map → camera_init` bridge as the only world-frame bridge.
- Validate topic types, rates, stamps, frames, TF connectivity, and finite data.
- Record a five-minute ROS-time endurance run when the simulator permits it.

## Non-scope

- No changes to external FAST_LIO or FALCO repositories.
- No edits to the untracked original Stage 2 test-plan file in the main worktree.
- No replacement of FAST-LIO2 covariance or coordinate semantics.

## Acceptance criteria

1. Launch exposes both navigation topics through transparent relays while
   `/Odometry` and `/cloud_registered` remain available.
2. Odometry remains `camera_init → body`; registered scans remain in
   `camera_init`; `map → camera_init` remains connected by the existing bridge.
3. Automated contract tests and package build pass.
4. Runtime evidence covers five minutes of ROS time, or records an objective
   environmental blocker and residual risk.

## Risks

- Gazebo may run at low real-time factor or produce unstable physics.
- External FAST_LIO is intentionally not tracked in this repository.
- Relay adds one serialization/publish hop to each navigation-facing topic.

## Impacted module

`src/simenv_fast_lio2_integration`
