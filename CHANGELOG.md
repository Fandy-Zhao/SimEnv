# Changelog

## 2026-07-15

### Dedicated Controller & RVIZ Terminals (fix/0715-build-auto-startup)
- Added a tmux-backed terminal mode (default). `junior_ctrl` and RViz run in
  named sessions that survive a GUI-terminal failure and can be reattached with
  `tmux attach-session -t simenv-junior_ctrl` or `simenv-rviz`.
- Run the tmux command from a generated runtime script rather than shell-quoted
  multiline text, preventing `/bin/sh` from rejecting Bash-specific escaping
  and immediately closing the session.
- Corrected the per-session `GAZEBO_MODEL_PATH` export so the generated Bash
  script remains syntactically valid and both named sessions stay alive.
- Validated full startup: both sessions remained live, `/Odometry` published at
  about 10 Hz, and `SIGINT` removed sessions and runtime processes cleanly.
- Fixed terminal creation when `auto.sh` is launched from Snap Code: it now
  starts `gnome-terminal.real` with a clean desktop environment instead of the
  Snap-contaminated D-Bus wrapper, which failed with a GLIBC symbol error.
- Keep a diagnostic interactive shell open after either dedicated command
  exits, so a controller/RViz failure is visible instead of flashing and
  immediately closing its terminal window.
- Run each terminal command in a child shell because `rosrun` ends in `exec`;
  this prevents an RViz launch failure from replacing the outer shell and
  bypassing the terminal-preservation logic.
- `auto.sh` now launches `junior_ctrl` and rviz in dedicated `gnome-terminal`
  windows (fallback: `xterm` → background).  This guarantees a real TTY for
  keyboard state switching (`2`=stand, `4`=trot, `6`=RL), eliminating the
  unreliable `/dev/tty`-redirection approach that caused segfaults and missing
  keyboard input.
- Removed `CONTROLLER_FOREGROUND` variable; the controller always runs in its
  own terminal.  Added `ENABLE_RVIZ` (default: 1).
- Replaced the end-of-script `wait` with a `trap cleanup INT TERM` + infinite
  sleep loop — Ctrl‑C in the main terminal now cleanly stops all ROS processes.

### FSM Nullptr Segfault Fix (fix/0715-build-auto-startup)
- Guarded keyboard `4`/`6` mappings, ROS `/fsm/state_cmd` cases `4`/`6`, and
  `State_FixedStand::checkChange()` TROTTING/RL returns behind `#ifndef
  UNITREE_DISABLE_TORCH_POLICY`.  When Torch is OFF (default), pressing these
  keys or publishing the matching rostopic no longer triggers a transition to a
  nullptr state object → segfault.
- Added a nullptr safety check in `FSM::run()` so that any future unguarded
  path produces a warning instead of a crash.
- Updated `auto.sh` help text to indicate that 4=Trotting and 6=RL require a
  Torch-enabled build.

### Controller Terminal Keyboard Fix (fix/0715-auto-keyboard)
- `auto.sh` now explicitly assigns `/dev/tty` to background `junior_ctrl`.
  This preserves keyboard input when FAST-LIO2 startup requires the controller
  to run before mapping.
- Replaced non-interactive `fg %1` with `wait`, keeping the invoking terminal
  attached to the controller after startup.  Non-TTY launches now warn and
  document ROS-topic control as the supported alternative.

### Build and Controller Startup Guard (fix/0715-build-auto-startup)
- Updated the default `build_with_venv.sh` profile to build every `auto.sh`
  runtime dependency (FAST-LIO2, Mid-360 sensor plugin, `junior_ctrl`, Unitree
  Gazebo plugins) while excluding unrelated locally added packages such as
  legacy `ps3joy`.
- Added an early `junior_ctrl` artifact check in `auto.sh`, preventing scene
  generation and Gazebo startup when the controller has not been built.

### FAST-LIO2 LiDAR–IMU Axis Correction (fix/0715-fast-lio2-axis)
- Corrected `simenv_mid360.yaml` to the direct FAST-LIO2 point transform
  `p_imu = R_L_I * p_lidar + T_L_I`: `Ry(+45°)` and `[0.2, 0, 0.08]`.
  The prior inverse (`Ry(-45°)`, `[-0.085, 0, -0.198]`) conflicted with the
  `laser_livox`-frame points produced by the simulator.
- Added `check_fast_lio2_extrinsics.py`, which derives the expected transform
  from `robot.xacro`, and installed both it and the TF bridge with catkin.
- Updated the deployment guide and ADRs.  A mapper restart is required before
  the corrected parameters affect a running session.

## 2026-07-14

### FAST-LIO2 Frame Correction (zzf/0714-fast-lio2-frame-fix)
- **Root cause**: `map_to_camera_init_bridge.py` applied a spurious **Ry(-45°)** rotation when publishing `map→camera_init`. The 45° LiDAR tilt was already handled by FAST-LIO2's `extrinsic_R`, so this was a duplicate rotation that tilted the entire `camera_init` world frame, causing `/Odometry` body axes to appear incorrect (X pointing downward instead of forward).
- **Fix**: Removed the Ry(-45°) rotation from the bridge. `map→camera_init` is now a direct copy of `map→imu_link` (the trunk IMU pose) without any extra rotation. Removed unused `_rotate_by()` function and `math`/`tf.transformations` imports.
- **Documentation**: Updated stale bridge comment in launch file (was "map→laser_livox", corrected to "map→imu_link"). Added rotation responsibility boundary documentation to YAML config. Created ADR-0714 (frame convention decision record).
- **Verified**: Python syntax, YAML syntax, launch XML, catkin_make build, rotation matrix validity (det≈1, orthogonal). Runtime tests pending (ROS master not available).
- **ADR**: `docs/decisions/ADR-0714-fast-lio2-frame-convention.md`

### FAST-LIO2 Startup Order Fix
- **Controller before FAST-LIO2**: `auto.sh` now starts `junior_ctrl` BEFORE FAST-LIO2 when `ENABLE_FAST_LIO2=1`. Controller is forced to background mode, FixedStand is auto-commanded via `/fsm/state_cmd` (`data: 2`), and the script waits for IMU confirmation (gravity aligned to Z, `linear_acceleration.z >= 9`) before launching FAST-LIO2. Prevents incorrect gravity initialization that caused continuous Z drift in `/cloud_registered`.
- **Duplicate adapter fix**: `simenv_fast_lio2_mapping.launch` now defaults `enable_adapter:=false`. `auto.sh` starts `scan_to_pointcloud2.py` once; the launch file no longer starts a second instance. Pass `enable_adapter:=true` when launching standalone.
- **FSM reference**: Use `rostopic pub /fsm/state_cmd std_msgs/Int8 "data: N"` for programmatic state control (2=FixedStand, 4=Trotting, 6=RL).
- **Controller foreground**: Default `CONTROLLER_FOREGROUND=1`. When `ENABLE_FAST_LIO2=1`, controller starts in background for auto-stabilisation, then `fg` pulls it back after FAST-LIO2 is running. Simple foreground when FAST-LIO2 is disabled.

### FAST-LIO2 Runtime Validation
- **PointCloud2 format fix**: `scan_to_pointcloud2.py` now outputs x,y,z,intensity (xyzi32) instead of xyz32.  FAST-LIO2 `lidar_type=4` requires the intensity field for curvature-based feature extraction; without it every point was rejected as "not effective" and the EKF diverged to 8000+ meters within minutes.
- **TF bridge**: New `map_to_camera_init_bridge.py` publishes a static TF `map→camera_init` by looking up `map→laser_livox` at startup, connecting the previously separate Gazebo and FAST-LIO2 TF trees.
- **RViz config**: Added `config/fast_lio2.rviz` pre-configured for Odometry, Laser_map, cloud_registered, path, and TF displays with Fixed Frame: map.
- **IMU topic fix**: Switched from `/livox/imu` (45° tilted, vibration-corrupted gravity) to `/trunk_imu` (body-aligned, z-up).  Updated LiDAR→IMU extrinsic to `Ry(-45°)`.  Eliminated pure-Z drift of 62 m/s; all axes now within 1 cm after 30 s.
- **Verified convergence**: With robot in FixedStand, FAST-LIO2 converges to ~1 cm from origin within 20 s. All output topics publish stably.

### Locomotion
- **RL diagnosis**: Both `policy_act_inference_plane.pt` and `policy_act_inference_stair.pt` confirmed non-responsive to `/cmd_vel` velocity commands (0.15-1.0 m/s produce <0.2 mm motion).
- **Trotting `/cmd_vel`**: Added `ros::Subscriber` to `State_Trotting` (classical MPC gait). `/fsm/state_cmd` topic (`std_msgs/Int8`) enables programmatic state transitions: 2=FixedStand, 4=Trotting.

### Infrastructure
- **`auto.sh` cleanup**: Replaced 5-line pkill with comprehensive cleanup covering Gazebo core, SimEnv launch, unitree controller, FAST-LIO2, sensor bridges, and stale rostopic processes.

## 2026-07-13

### Documentation
- **New `docs/cli-reference.md`**: comprehensive CLI quick-reference covering all commands: build, auto.sh env vars (6 categories), FAST-LIO2 launch, scene generation, door/elevator control, controller, evaluation, diagnostics, file paths, rosbag.
- **Updated `docs/README.md`**: added FAST-LIO2 deployment guide and CLI reference to the documentation index.
- **Updated `docs/quick-start.md`**: added `ENABLE_FAST_LIO2=1` usage, FAST-LIO2 related env vars, fixed `ROBOT_Y` default from `-2.2` to `2.3`.

### FAST-LIO2 Deployment Test (Stage 0 and initial Stage 1 L1/L2)
- **Stage 0 (PASS)**: Verified Gazebo sensors, simulation time, and TF tree. All checks passed: `/use_sim_time=true`, `/clock`@500Hz, `/scan`@10Hz (`laser_livox` frame), `/livox/imu`@400Hz (`livox_imu_link` frame). TF tree correct: `base→laser_livox` (45° pitch) `→livox_imu_link` (extrinsic matches config).
- **Stage 1 initial L1/L2 check (superseded by runtime retest below)**: FAST-LIO2 node launched without crash and registered all output topics. Discovered and fixed 3 launch bugs:
  - FAST-LIO2 node was commented out in `simenv_fast_lio2_mapping.launch`
  - YAML config loaded in wrong namespace (`ns="laserMapping"`) — FAST_LIO uses public `NodeHandle`, reads from `/common/lid_topic`; fixed to root-level `<rosparam>`
  - Removed redundant `<param>` tags (superseded by YAML rosparam load)

### FAST-LIO2 Runtime Retest
- Corrected FAST-LIO2's input type contract: SimEnv's `/scan_pointcloud2`
  adapter publishes `sensor_msgs/PointCloud2`, so `simenv_mid360.yaml` now
  uses `preprocess.lidar_type: 4` (MARSIM/standard PointCloud2), rather than
  Livox `CustomMsg` mode (`1`).
- Verified the corrected launch publishes finite `/Odometry` and
  `/cloud_registered` messages.
- Marked Stage 1 static localization **failed**: 60 s wall-clock capture
  (4.300 s ROS time) drifted 325.253 m and 51.607° while Gazebo truth moved
  only 0.0476 m over 1.020 s. The 5 m and loop tests are deferred by the
  stage gate.

### FAST-LIO2 Controlled P0 Follow-up
- **Superseded the static-failure interpretation**: the former capture ran
  without `junior_ctrl`, and the A1 was falling/rolling rather than stationary.
  In fixed-stand mode FAST-LIO2 moved 0.001967 m / 0.066852° over 60 s
  wall-clock (4.300 s ROS time), with effectively static Gazebo truth.
- **Fixed FAST-LIO2 MARSIM initialization**: initialized
  `last_lidar_end_time_` in the external FAST_LIO `ImuProcess` constructor and
  `Reset()` to avoid undefined startup/reset behavior in the PointCloud2 path;
  the `fast_lio` package selectively rebuilt successfully.
- **P1 explicitly blocked**: the ABI-isolated `junior_ctrl` build excludes the
  Torch-dependent trot/RL states, so a real 5 m or loop locomotion test needs
  an approved Torch-policy rebuild.  The strict 60 s ROS-time P0 capture also
  remains pending because of low simulation RTF.

### FAST-LIO2 Extended P0 Attempt
- The latest ROS-time fixed-stand capture reached 28.900 s before a second
  workspace (`/home/zzf/桌面/unitree_ex`) joined the same ROS master and
  registered duplicate `/gazebo` and `/robot_state_publisher` names. Its
  missing `hustw_description` package then ended the shared session; this is a
  cross-workspace ROS-master collision, not a FAST-LIO2 crash.
- In that interrupted window Gazebo truth changed 0.018344 m / 0.592970° and
  FAST-LIO2 changed 0.074965 m / 0.788825°. P0 is not accepted: ROS-master
  isolation and a more stationary support/control condition are required.

### FAST-LIO2 Reduced-duration P0
- Measured RTF=0.068 and completed a separate 10 s ROS-time diagnostic window
  to avoid an excessive wall-clock test. FAST-LIO2 changed 0.286359 m /
  1.216603°, versus Gazebo truth 0.004874 m / 0.774496°; all values were
  finite and `/cloud_registered` remained available, but P0 still fails.
- Added duration and output-tag environment options to the P0 capture helper,
  so reduced-duration tests do not overwrite the 60 s evidence.

### P1 Locomotion Analysis & Trotting `/cmd_vel` Fix
- **RL policy diagnosis**: Both `policy_act_inference_plane.pt` and `policy_act_inference_stair.pt` confirmed non-responsive to `/cmd_vel` velocity commands. At 0.3 m/s and 1.0 m/s, robot moves <0.2 mm. RL inference outputs constant standing actions regardless of command.
- **Trotting `/cmd_vel` addition**: Added `ros::Subscriber` for `/cmd_vel` to `State_Trotting` (classical MPC gait controller, no RL dependency). When active, overrides keyboard WASD with `cmd_vel.linear.x/y` and `angular.z`, saturated to robot limits (vx[-0.8,0.8], vy[-0.6,0.6], wz[-1.0,1.0]). Keyboard fallback preserved when no `/cmd_vel` received.
- **State chain**: `Passive → 2(FixedStand) → 4(Trotting)`. Trotting originally keyboard-only (WASD→vx, AD→vy, JL→wz, IK→height).

### FAST-LIO2 P0 Cause Diagnostic
- Added a synchronized truth/IMU/odometry diagnostic and ran it for 10 s.
  FAST-LIO2 tracked the remaining physical truth rotation closely (0.364035°
  versus 0.325467°); truth and IMU gyro rates agreed, while acceleration
  reached 87.112 m/s². This identifies residual fixed-stand contact/rotational
  motion as the primary P0 cause, rather than standalone FAST-LIO2 divergence.

## 2026-07-06

### Build Fix
- Fixed `unitree_guide/junior_ctrl` Torch ABI pollution: `find_package(Torch)` injected `-D_GLIBCXX_USE_CXX11_ABI=0` globally, causing `ros::init` undefined references. Added `UNITREE_ENABLE_TORCH_POLICY` option (default OFF) to isolate Torch flags; excluded 3 torch-dependent source files; guarded transitive header includes in `FSM.h`/`FSM.cpp`. `junior_ctrl` now compiles and links with ROS Noetic (fix/0704-unitree-torch-abi-isolation).

### Documentation
- Added `docs/slam/fast_lio2_deployment_guide.md`: comprehensive FAST-LIO2 deployment guide covering repository layout, sensor topic mapping, parameter reference, pointcloud compatibility, IMU selection, extrinsic calibration, build environment, deployment steps, runtime validation checklist, common failure modes, experiment tracking parameters, and output contract for future navigation (docs/0706-fast-lio2-deploy-guide).
- Added `docs/decisions/ADR-0706-fast-lio2-deploy-guide.md`: architecture decisions for deployment guide.
- Added `docs/reports/0706_fast-lio2-deploy-guide.md`: task report with coverage table.

## 2026-07-04

### Documentation
- Fixed FAST-LIO2 workspace layout documentation: SimEnv is the catkin workspace root, FAST_LIO belongs at `SimEnv/src/FAST_LIO`, not nested under another `catkin_ws/`.
- FAST-LIO2 build environment audit: static checks all pass; catkin_make blocked by missing libtorch (C++ SDK) at hardcoded path in unitree_guide. Logs at `experiments/runs/0704_fast-lio2-build-check/`.
- `tools/build_with_venv.sh`: added auto-detection of pip torch CMake prefix (`torch.utils.cmake_prefix_path`), passes `-DCMAKE_PREFIX_PATH` to catkin_make without overwriting ROS paths.
- `tools/build_with_venv.sh`: auto-selects gcc-11/g++-11 for CUDA 11.8 compatibility; passes CC/CXX/CUDAHOSTCXX + CUDA paths to catkin_make. CUDA host compiler errors eliminated.
- Build fixes: unitree PIE linker (`-no-pie`), FAST_LIO C++14→17, livox_ros_driver C++11→17, PCL shared_ptr serialization, missing `<memory>` includes in Livox-SDK.

### Build Tooling
- Added `tools/build_with_venv.sh`: builds catkin workspace with project `.venv` Python, ensuring consistent interpreter for torch and other Python deps.
- README updated with venv setup and build instructions (torch 2.0.1 pin for Python 3.8 / ROS Noetic).

### Governance & Remote Configuration
- Added GitHub remote (`github` → `git@github.com:zzf/SimEnv.git`), origin retained as Gitee.
- Branch naming policy: maintenance/setup branches now use `zzf/MMDD-short-name`; `chore/` prefix is deprecated for this project.
- Initialized project governance skeleton (AGENTS.md, PROJECT_STATE.md, ROADMAP.md, docs/architecture.md, docs/module_status.md).

### FAST-LIO2 Mapping Integration (feat/0704-fast-lio2-mapping)
- Added `src/simenv_fast_lio2_integration/` ROS package with PointCloud adapter, FAST-LIO2 config, and launch files.
- Added `ENABLE_FAST_LIO2` optional flag in auto.sh.
- FAST-LIO2 operates as external catkin workspace dependency (not vendored).

## Historical (from git log)

### 2025
- `a46d947` — add LICENSE.
- `736ab90` — update README.md.
- `8191cff` — 电梯开关门优化
- `6d2aa9c` — 优化随机生成建筑
- `8ba1867` — 添加危险源及相关算法评估程序
