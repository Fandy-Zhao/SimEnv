# Issue: FAST-LIO2 LiDAR–IMU axes and odometry frame audit

## Goal

Verify the SimEnv LiDAR-to-IMU extrinsic against the robot model, ensure FAST-LIO2 maps laser points into the body frame in the correct direction, and guard the `map -> camera_init -> body` odometry frame convention against regressions.

## Scope

- `simenv_fast_lio2_integration` configuration, installation metadata, and a read-only extrinsic checker.
- FAST-LIO2 deployment and project-status documentation.

## Non-scope

- Do not modify the system ROS Noetic installation, Gazebo physics, URDF sensor geometry, or external untracked `src/FAST_LIO` sources.
- Do not change the already-correct LiDAR mounting angle.

## Acceptance criteria

1. The configured transform is the inverse of the URDF `imu_link -> laser_livox` pose.
2. FAST-LIO2 uses the body-aligned `/trunk_imu` and maps `laser_livox` points to `imu_link` exactly once.
3. A local checker detects a reversed transform or stale configuration.
4. Runtime TF and `/Odometry` headers agree with the documented frame tree.

## Risks

- Static world alignment is fixed at initialization and inherits any robot tilt at that instant.
- A full moving-robot regression is not attempted while Gazebo is paused and has low real-time factor.
