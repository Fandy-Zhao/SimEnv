# Issue: FAST-LIO2 Stage 0 & Stage 1 Deployment Testing

- **创建日期**: 2026-07-13
- **分支**: exp/0713-fast-lio2-stage01-runtime (from develop)
- **参考文档**: `prompts/fastlio2_tare_dsv_test_plan.md` §3-4, `docs/slam/fast_lio2_deployment_guide.md`
- **状态**: Partially blocked — controlled P0 is stable; P1 lacks an enabled locomotion mode

## Task Goal

按测试计划逐步执行 FAST-LIO2 部署测试的 Stage 0（传感器/时间/TF）和 Stage 1（FAST-LIO2 单独定位建图），验证 L1（安装）和 L2（接口）等级。

## Modification Scope

- Uncomment FAST-LIO2 node in `simenv_fast_lio2_mapping.launch`
- Execute Stage 0 checks: /use_sim_time, /clock, sensor topics, TF tree
- Execute Stage 1 checks: launch FAST-LIO2, verify /Odometry, /cloud_registered
- Static drift test (60s no motion)

## Non-Scope

- Stage 2+ (navigation adaptation, FALCO, TARE, DSV)
- Modifying sensor plugins or URDF models

## Acceptance Criteria

- [ ] Stage 0: All sensor/time/TF checks pass
- [ ] Stage 1 L1: FAST-LIO2 launches without immediate crash
- [x] Stage 1 L2: /Odometry and /cloud_registered publish continuously
- [ ] Stage 1 static test: incomplete — the latest ROS-time capture was interrupted at 28.900 s by a second workspace joining the same ROS master; fixed-stand truth also drifted over that longer window

## Risk Points

- FAST_LIO binary was compiled on 2026-07-06, may have bit-rotted
- livox_ros_driver not compiled — may cause linking issues
- No display available for RViz visualization

## Expected Impacted Modules

- `simenv_fast_lio2_integration/` (launch file)
- `FAST_LIO/` (external source: MARSIM initialization fix)

## 2026-07-13 Runtime Retest Result

- Corrected the input contract: `lidar_type: 1` subscribes to
  `livox_ros_driver/CustomMsg`, while SimEnv's adapter publishes
  `sensor_msgs/PointCloud2`.  The configuration now uses FAST-LIO2's
  `MARSIM`/standard PointCloud2 path (`lidar_type: 4`).
- With the corrected launch, `/Odometry` and `/cloud_registered` publish and
  contain finite values.
- Superseded: the initial static capture used `START_CONTROLLER=0`; live IMU
  data showed the A1 rotating at about 4--5 rad/s and experiencing collision
  accelerations.  It was a falling robot, not a stationary test.

## 2026-07-13 Controlled P0 Follow-up

- Started `junior_ctrl` and selected fixed-stand mode before collecting data.
  Gazebo truth then stayed within `8.9e-08 m` and `4.9e-05 deg` during the
  60 s wall-clock capture.
- FAST-LIO2 moved `0.001967 m` and `0.066852 deg` over the same window
  (`4.300 s` ROS simulation time), meeting the suggested static magnitude
  thresholds.  The latest simulation-time retry was terminated at `20.700 s`;
  it began before fixed-stand had fully settled, so its `0.008262 m` /
  `0.324734 deg` result is diagnostic only, not P0 acceptance evidence.
- The MARSIM/PointCloud2 branch used an uninitialized
  `last_lidar_end_time_` at startup/reset.  It is initialized to `-1` in the
  external FAST_LIO source and the package was rebuilt successfully.
- The complete 60 s ROS-simulation-time P0 run remains pending: the 3-floor
  scene currently advances roughly 4.3 s in 60 s wall time.
- P1 5 m and rectangular-path tests cannot be run as genuine locomotion
  tests: the current `junior_ctrl` build has `UNITREE_DISABLE_TORCH_POLICY`,
  so it contains no `State_Trotting`/RL state.  Enabling it requires an
  explicit Torch ABI/CUDA dependency decision.

## 2026-07-13 P0 ROS-master collision

- The latest capture began at `7.404 s` and reached `36.304 s` ROS time
  (`28.900 s` span). FAST-LIO2 changed `0.074965 m` / `0.788825 deg`; Gazebo
  truth changed `0.018344 m` / `0.592970 deg`, so fixed stand was not yet an
  adequate stationary reference.
- At 22:47:51, `/home/zzf/桌面/unitree_ex` connected to the same
  `ROS_MASTER_URI=http://localhost:11311` and registered `/gazebo` plus
  `/robot_state_publisher`. This displaced SimEnv's same-named nodes; its own
  launch then failed because `hustw_description` was not in SimEnv's package
  path. The shared ROS session ended before 60 s.
- Retrying P0 requires both a dedicated/isolated ROS master and a support or
  controller mode that keeps Gazebo truth stationary for the full window.

## 2026-07-13 Reduced-duration P0

- Current RTF measured `0.068`, so the P0 diagnostic duration was reduced to
  10 s ROS time and written under the independent `p0_stand_sim10_rtf068` tag.
- FAST-LIO2 changed `0.286359 m` / `1.216603 deg`; Gazebo truth changed
  `0.004874 m` / `0.774496 deg`. All values were finite and
  `/cloud_registered` still published, but P0 failed: the fixed stand yaw is
  not stationary and FAST-LIO2 position change exceeds the suggested bound.
- The only non-Torch dynamic alternatives are in-place `StepTest` and bounded
  `BalanceTest`; they cannot validate the requested 5 m/rectangular P1 paths.

## 2026-07-14 P0 drift-cause diagnostic

- A 10.002 s synchronized diagnostic measured FAST-LIO2 at `0.005980 m` /
  `0.364035 deg` and Gazebo truth at `0.012825 m` / `0.325467 deg`.
- Truth angular speed (mean/max `0.050412`/`0.831500 rad/s`) and IMU gyro
  speed (mean/max `0.048114`/`0.831491 rad/s`) agree. The IMU acceleration
  norm averaged `12.530 m/s²` and peaked at `87.112 m/s²`, indicating stance
  contact vibration. Therefore the principal P0 cause is remaining robot
  motion, not an independently diverging FAST-LIO2 pose.
- The xyz-only PointCloud2 input still has no per-point time field and is a
  secondary motion-distortion risk; the diagnostic does not identify the fixed
  LiDAR--IMU extrinsic as the primary fault.
