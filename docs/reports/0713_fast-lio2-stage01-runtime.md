# Task Report

## Branch

`exp/0713-fast-lio2-stage01-runtime` (from `develop`).

## Summary

Completed the requested FAST-LIO2 Stage 1 runtime retest. The repaired launch
now publishes `/Odometry` and `/cloud_registered`, after changing the FAST-LIO2
preprocessor from Livox `CustomMsg` mode to its standard PointCloud2 mode.
The original static-localization failure was later invalidated: it was captured
while the unactuated A1 was falling.  A controlled fixed-stand rerun is stable,
but the strict 60 s ROS-simulation-time window and genuine locomotion tests
remain incomplete.

## Files Changed

- `src/simenv_fast_lio2_integration/config/simenv_mid360.yaml`: use
  `lidar_type: 4` for the adapter's `sensor_msgs/PointCloud2` output.
- `src/FAST_LIO/src/IMU_Processing.hpp` (external source): initialize the
  MARSIM `last_lidar_end_time_` state at construction and reset.
- `docs/slam/fast_lio2_deployment_guide.md`: document the actual input
  contract.
- `experiments/runs/0713_fast-lio2-stage01/`: record runtime issue status,
  measurements, and source CSV captures.
- `PROJECT_STATE.md`, `CHANGELOG.md`, and `docs/module_status.md`: correct the
  prior Stage 1 status and record the blocking quality result.

## Tests

- `roslaunch --files simenv_fast_lio2_integration simenv_fast_lio2_mapping.launch`: pass.
- Runtime: `/scan_pointcloud2` (24,000 points/frame), `/Odometry`, and
  `/cloud_registered`: publish after the fix; values were finite.
- Controlled static capture: 44 odometry samples over 60 s wall time / 4.300
  s ROS time; 0.001967 m positional and 0.066852° orientation change.  Gazebo
  truth moved `8.81e-08 m` / `4.94e-05°`.
- A later simulation-time retry was interrupted at 20.700 s with 0.008262 m /
  0.324734°; it began before fixed stand finished settling (truth moved
  0.014836 m / 0.476989°), so it is diagnostic only. Full 60 s ROS-time
  acceptance is still pending at current RTF.
- Not run: P1 5 m straight path and rectangular loop, because the current
  `junior_ctrl` build excludes the Torch-dependent trot/RL states.

## Documentation Updated

Updated the deployment guide, experiment record, project state, changelog, and
module status.

## Git

Pending commit. No merge or push was performed.

## Risks

- The 3-floor scene has a low real-time factor and one prior `gzserver`
  SIGSEGV was observed.
- `UNITREE_DISABLE_TORCH_POLICY` prevents a genuine locomotion test.  Reusing
  a teleport or kinematic pose update would not validate the requested P1 gait
  path.
- ROS Noetic xacro must run under system Python 3.10, not the IDE's Conda
  Python 3.13.

## Next Step

Complete the fixed-stand 60 s ROS-simulation-time static test, then authorize
and restore the Torch-dependent locomotion stack (or explicitly approve a
separately labelled SLAM-only kinematic alternative) before P1.

## Follow-up root-cause correction — 2026-07-13

The former 325.253 m / 51.607° result is superseded.  `START_CONTROLLER=0`
left the robot unactuated, and direct IMU observation showed 4--5 rad/s
rotation and collision acceleration while it fell.  This is a test-fixture
failure, not evidence of IMU-axis or LiDAR-extrinsic error.

The PointCloud2/MARSIM code path also had genuine undefined behavior:
`ImuProcess::last_lidar_end_time_` was read before initialization.  It is now
set to `-1` in both the constructor and `Reset()`, then rebuilt with
`catkin_make -DCATKIN_WHITELIST_PACKAGES=fast_lio --pkg fast_lio -j2`.

## Extended P0 attempt — interrupted

The latest fixed-stand capture reached `28.900 s` of ROS time before an
independent `/home/zzf/桌面/unitree_ex` launch joined the same
`ROS_MASTER_URI=http://localhost:11311`. It registered duplicate `/gazebo` and
`/robot_state_publisher` names, displaced SimEnv's nodes, and failed because
`hustw_description` was unavailable in SimEnv's package path. This is a
cross-workspace ROS-master collision, not a FAST-LIO2 crash.

Before interruption FAST-LIO2 changed `0.074965 m` / `0.788825°`, while Gazebo
truth changed `0.018344 m` / `0.592970°`. Therefore the controller's fixed
stand is not sufficiently stationary for strict P0 acceptance either. Isolate
the master and establish a stationary support/control mode before a 60 s retry.

## Reduced-duration P0 — RTF diagnostic

Measured RTF was `0.068`; a 10 s ROS-time capture was therefore run under the
separate `p0_stand_sim10_rtf068` tag. FAST-LIO2 changed `0.286359 m` /
`1.216603°`; Gazebo truth changed `0.004874 m` / `0.774496°`. All samples were
finite and `/cloud_registered` remained available, but P0 still fails: fixed
stand itself has too much yaw motion and FAST-LIO2 position change is above the
suggested static threshold. The helper now supports duration/tag environment
variables so diagnostic windows cannot overwrite the formal 60 s evidence.

## Drift-cause diagnostic

A synchronized 10.002 s capture separates estimator motion from physical
motion. FAST-LIO2 changed `0.005980 m` / `0.364035°`; Gazebo truth changed
`0.012825 m` / `0.325467°`. Truth angular speed averaged `0.050412 rad/s`
(maximum `0.831500`), and IMU gyro speed averaged `0.048114 rad/s` (maximum
`0.831491`), demonstrating matching motion rather than an IMU sign/scale
failure. The IMU acceleration norm averaged `12.530 m/s²` and peaked at
`87.112 m/s²`, consistent with contact/stance vibration. The primary P0 cause
is residual physical motion in fixed stand; lack of per-point timestamps is a
secondary risk whenever that motion occurs.
