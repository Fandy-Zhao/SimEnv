# Project State

> **2026-07-24 shared FAST-LIO2 dependency retry**: the task worktree now uses
> `tools/prepare_shared_ros_deps.sh` to validate and link the fixed shared
> public sources under `src/external/`:
> `/home/zzf/search_ws/FAST_LIO` `7cc4175de6f8ba2edf34bab02a42195b141027e9`,
> `ikd-Tree` `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`, and
> `/home/zzf/search_ws/livox_ros_driver`
> `3d240d5666129e1a3052e78ee8487a04b08fdda3`. Shared package discovery through
> `ROS_PACKAGE_PATH=$PWD/src` passes for `fast_lio` and `livox_ros_driver`.
> Formal `./tools/build_with_venv.sh` is blocked before `catkin_make` with exit
> code `20`: the pinned shared `livox_ros_driver` CMake lacks the
> `BUILD_LIVOX_DRIVER_NODE` guard and would try to clone/build Livox-SDK inside
> the shared checkout when no system `liblivox_sdk_static.a` exists. The build
> script now refuses that mutation. First failed gate: `FAST_LIO_BUILD_BLOCKED`.
> FAST-LIO runtime, terrain map, DSV/FALCO data chain, short closed loop, full
> exploration, and return home were not run. Current verdict:
> `FALCO_DSV_EXPLORATION_BLOCKED`.

> **2026-07-24 FALCO + DSV single-floor data path**: branch
> `feat/0724-falco-dsv-single-floor-exploration-0p8` starts from local
> `master` `ce73018c9ecc220bf01351a295a534ca56e67100` in isolated worktree
> `/home/zzf/search_ws/SimEnv_worktrees/falco-dsv-single-floor-0p8`. Root
> `master` dirty scene/log/result files were preserved untouched. Added a
> single-floor navigation launch, runtime terrain-map and boundary adapters,
> FALCO heading-aware raw speed scheduling, and DSV zero-initialization/windowed
> movement fixes. Formal `./tools/build_with_venv.sh` passes. Isolated FALCO
> path follower probes measured straight `0.803999543 m/s`, 30 deg
> `0.600000143 m/s`, 70 deg `0.203999937 m/s`, and max angular
> `0.219911486 rad/s`. Verdict: `FALCO_DSV_DATA_PATH_READY`; S2-S5 `auto.sh`
> Gazebo/Trotting closed-loop validation remains next and is not claimed.

> **2026-07-24 runtime validation update**: the required real entry points were
> started from the same worktree (`FLOOR_COUNT=1 GUI=false ./auto.sh` and
> `roslaunch simenv_navigation_bringup single_floor_exploration.launch`), with
> `/navigation/enabled=false`. The run stopped at the first gate:
> `FAST_LIO_INPUT_BLOCKED`. `auto.sh` reported `fast_lio: NOT_FOUND`;
> `logs/fast_lio2.log` reports `cannot launch node of type
> [fast_lio/fastlio_mapping]`; `rospack find fast_lio` fails after sourcing the
> task worktree. No navigation motion, short loop, full exploration, or return
> home was executed. Current runtime verdict: `FALCO_DSV_EXPLORATION_BLOCKED`.

> **2026-07-24 FALCO A1 real-cloud R3 tuning**: branch
> `feat/0723-falco-dsv-navigation-integration` continued only the R3 FALCO
> local-planner gate from baseline `8f5c89ee`. Added an A1-specific
> `falco_a1.yaml` profile and launch overrides, with opt-in diagnostics for
> point filtering, candidate/free paths, selected path group/rotation, collision
> scores, and output command/path counts. Real FAST-LIO2 `/Odometry` and
> `/cloud_registered` with `checkObstacle=true` now produce repeatable local
> FALCO output: forward regressions command about `0.095-0.100 m/s`, side
> offset regressions turn toward the goal, and the long-front probe shows
> obstacle scoring remains active. `/navigation/enabled=false` still gates
> `/cmd_vel` to zero. Verdict: `FALCO_A1_REAL_PATH_READY`; R4 Trotting, R5 DSV,
> and R6 full exploration were not run.

> **2026-07-24 FALCO R3 real-data diagnosis**: branch
> `feat/0723-falco-dsv-navigation-integration` re-tested only R3 in the
> isolated worktree. Build passes with `./tools/build_with_venv.sh`. Runtime
> diagnosis found two direct blockers: `runtime_real_data.launch` depended on
> intermediate `/state_estimation` and `/registered_scan` relays that were not
> alive in the recovered FAST-LIO2 run, so it now defaults directly to
> FAST-LIO2 `/Odometry` and `/cloud_registered`; with direct sources, Case A
> (`checkObstacle=true`) still publishes the zero one-pose path, while Case B
> (`checkObstacle=false`) publishes a multi-pose path and finite nonzero raw
> FALCO TwistStamped. Motion remains gated: `/navigation/enabled=false` keeps
> `/cmd_vel` zero. Verdict: `FALCO_POINTCLOUD_FILTER_BLOCKED`; R4-R6 were not
> entered.

> **2026-07-23 FALCO + DSV real runtime validation**: branch
> `feat/0723-falco-dsv-navigation-integration` now has staged runtime evidence
> under `experiments/runs/0723_falco_dsv_runtime/`. R2 passed with real Gazebo,
> LiDAR, FAST-LIO2 `/Odometry` and `/cloud_registered`, navigation relays, and
> TF (`camera_init`, `body`, `map`, `base`, `laser_livox`). Added a
> SimEnv-owned `runtime_real_data.launch` to relay global FAST-LIO2 outputs into
> the `/navigation` namespace and expose bridge speed-limit overrides. R3 is
> blocked: FALCO connected to real inputs but produced only a zero one-pose path
> and zero raw velocity under observed low RTF (`~0.062`). R4-R6 were not run.
> Overall verdict: `FALCO_DSV_RUNTIME_TIMING_BLOCKED`.

> **2026-07-23 FALCO + DSV navigation integration**: branch
> `feat/0723-falco-dsv-navigation-integration` starts from local `master`
> `5bc0f6fbfdd8333dccbb44c26f216ecfb2811548` in isolated worktree
> `/home/zzf/search_ws/SimEnv_worktrees/falco-dsv-navigation`. Imported
> FALCO `local_planner` and the minimum DSV-Planner package closure under
> `src/navigation/vendor/`, added SimEnv bridge/bringup packages, and kept
> Gazebo, FAST-LIO2, robot model, controller, RL, and physics code unchanged.
> Validation passed through source package discovery, launch parsing,
> `tools/build_with_venv.sh`, static ROS interface checks, FALCO command-path
> smoke, and DSV service/waypoint smoke. Runtime startup remains intentionally
> decoupled from Gazebo/FAST-LIO2/controller startup until the next integrated
> simulation run.

> **2026-07-23 FAST-LIO2 clean runtime validation**: branch
> `fix/0723-fast-lio2-reproducible-build-pointcloud` now has clean runtime
> evidence on private ROS master `http://127.0.0.1:12732`. The adapter owns
> `/scan_pointcloud2`, FAST-LIO2 `laserMapping` owns `/Odometry` and
> `/cloud_registered`, the topic chain runs around 10 Hz with `/trunk_imu`
> around 340 Hz, and pause/resume returns both pointcloud and odometry output.
> PointCloud2 metadata is `laser_livox`, `24000` points,
> `x/y/z/intensity`, `point_step=16`. FAST-LIO2 logs show only one startup
> `No point, skip this scan!` warning at sim time `0.804`, not a sustained
> empty-pointcloud failure. Verdict: `FAST_LIO2_RUNTIME_VALIDATION_PASS`; no
> merge or push has been performed.

> **2026-07-23 FAST-LIO2 reproducible external build**: branch
> `fix/0723-fast-lio2-reproducible-build-pointcloud` starts from
> `master` `a423bcfd104659bfa05d286ccb79d6a03520b246` and adds a
> SimEnv-owned external dependency staging layer. Fixed source repositories
> remain clean and read-only (`FAST_LIO` `7cc4175`, `ikd-Tree` `e2e3f4e`,
> `livox_ros_driver` `3d240d5`). Staging under `/tmp/simenv-fast-lio2-deps`
> patches only copies: FAST_LIO builds with C++17 and `livox_ros_driver`
> defaults to message-only (`BUILD_LIVOX_DRIVER_NODE=OFF`) without Livox-SDK
> clone. The formal clean-shell `./tools/build_with_venv.sh` runtime whitelist
> build passes. Runtime ownership checks reached Gazebo, controller,
> adapter, and FAST-LIO2 startup with `/scan_pointcloud2` owned by the adapter
> and `/Odometry` plus `/cloud_registered` owned by `laserMapping`; final
> continuity verdict remains blocked by stale ROS master registrations from
> repeated isolated validation attempts, so this branch is not yet a merge
> candidate.

> **2026-07-23 FAST-LIO2 TF repeated timestamp fix**: branch
> `fix/0723-fast-lio2-tf-repeated-data` isolates TF ownership and timestamp
> handling from clean `master`. `state_from_gazebo` now uses one guarded
> callback stamp for `map -> odom`, `odom -> base`, and `/Odometry_gazebo`,
> skips zero/repeated sim-time publications, resets on clock rollback, and
> subscribes to `/gazebo/link_states` with queue depth 1. FAST-LIO2
> `laserMapping` remains the default owner of `camera_init -> body`;
> `odometry_tf_bridge` is opt-in to avoid duplicate dynamic TF owners. Scoped
> `tools/build_with_venv.sh` validation passes for
> `unitree_legged_msgs;simenv_fast_lio2_integration;unitree_guide`
> with `unitree_move_base` blacklisted; `state_from_gazebo` and `junior_ctrl`
> both link successfully. Full runtime reproduction is still not run because
> `auto.sh` performs broad ROS/Gazebo process cleanup on this shared machine.

> **2026-07-23 competition RL sensor-layer root cause**: FAST-LIO2 provenance
> is resolved as an external dependency (`hku-mars/FAST_LIO`
> `7cc4175de6f8ba2edf34bab02a42195b141027e9`, `ikd-Tree`
> `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`, `livox_ros_driver`
> `3d240d5666129e1a3052e78ee8487a04b08fdda3`) and the task worktree rebuilds
> with local validation-only external-source patches. The completed M0-M8
> matrix shows the first RTF collapse at M2: competition sensor data enabled,
> no PointCloud2 converter, no FAST-LIO2, no RL (`mean RTF=0.165125` versus
> M1 `0.989895`). M4/M5 mapping runs are lower (`0.134005`/`0.138520`) but
> secondary; M7/M8 add RL/thread-limit cost after the sensor-layer collapse
> (`0.087573`/`0.058942`). Verdict: `ROOT_CAUSE_IDENTIFIED_SENSOR_LAYER`;
> M3 `NO_CLOCK` flags converter-path startup instability for follow-up.

> **2026-07-23 competition RL partial runtime**: the task worktree now builds
> successfully after fixing only the local venv `empy` version (`3.3.4`).
> M0 earth baseline mean RTF is `0.640749`; M1 competition minimal baseline
> mean RTF is `0.989895`; M6 competition RL-active/no-mapping mean RTF is
> `0.989091` with policy inference at `50.0085 Hz`. Full mapping cases
> M4/M5/M7/M8 are blocked because the external `fast_lio` package is absent
> from the hermetic worktree, so root cause remains partial.

> **2026-07-23 competition RL RTF collapse diagnostics**: branch
> `diagnose/0723-competition-rl-rtf-collapse` creates a governed diagnostic
> issue, static audit, and dry-run-first M0-M8 harness for isolating
> competition scene, LiDAR, PointCloud2, FAST-LIO2, RL load/inference, Torch
> threading, and GUI/RViz contributions to low RTF. Runtime behavior and launch
> defaults are unchanged; causal claims remain pending controlled runtime
> sampling.

> **2026-07-22 RL keyboard command fallback**: branch
> `fix/0722-rl-keyboard-fallback` makes RL mode use fresh `/cmd_vel` when
> available and otherwise derive a command snapshot from keyboard `userValue`
> (`w/s`, `a/d`, `j/l`) with the same mapping convention as Trotting. This
> preserves navigation-facing `/cmd_vel` behavior while restoring interactive
> keyboard driving in RL mode. The default RL policy is now the Earth
> flat-ground recommended `policy_act_inference_plane.pt`; `/rl_policy_path`
> and `RL_POLICY_PATH` still provide runtime overrides.

> **2026-07-22 auto.sh RL policy override visibility**: branch
> `fix/0722-auto-rl-policy-env` makes the existing controller-side
> `RL_POLICY_PATH` support explicit in startup: `auto.sh` now prints whether an
> RL policy override is set and exports it into the dedicated `junior_ctrl`
> terminal environment. The controller-side priority remains `/rl_policy_path`
> -> `RL_POLICY_PATH` -> stair default.

> **2026-07-22 Navigation integration baseline**: branch `master`, HEAD
> `4eeae260b452a296b17f30afaea3b7b2edb7c636`, workspace
> `/home/zzf/search_ws/SimEnv`, status clean, build PASS. Earth RL
> `WORLD_MODE=earth`, `PHYSICS_PROFILE=normal`. Recommended flat-ground policy:
> `policy_act_inference_plane.pt`; runtime policy selection is supported.
> Validated speed range `0.20–0.40 m/s` (`vx=0.10` ineffective). RL zero PASS,
> IMU fallback PASS, LowCmd ≈500 Hz. Preserved interfaces: `/cmd_vel`,
> `/fsm/state_cmd`, `/clock`, robot state/odometry source, IMU input, RL policy
> runtime selection, LowCmd transport/application cadence. Known limitations:
> stair policy has a higher flat-ground start threshold and larger yaw drift,
> obstacle/stair behavior has not been validated by this comparison, and RTF
> fluctuation remains recorded but non-blocking.

> **2026-07-22 Earth RL policy comparison**: branch
> `test/0722-earth-rl-policy-comparison` builds from
> `fix/0722-earth-rl-timebase-fast-validation` and adds a minimal runtime
> `State_RL` policy override (`/rl_policy_path` -> `RL_POLICY_PATH` -> stair
> default) with loader path/SHA/existence/load-success logs. Earth flat-ground
> sweeps used identical launch/control parameters and body-frame metrics for
> stair and plane policies over `vx=0.00..0.40 m/s`. Plane is recommended for
> the master short regression: it tracks `0.20..0.40 m/s` better and keeps yaw
> drift lower, while both policies still have no effective motion at
> `vx=0.10 m/s`. Median LowCmd cadence is 500 Hz in both runs. No control
> parameters were tuned.

> **2026-07-22 Earth RL LowCmd/IMU merge validation**: branch
> `fix/0722-earth-rl-timebase-fast-validation` now includes the validated
> `fix/0722-earth-rl-lowcmd-publisher-stall` merge
> (`39bbb6cfef8fdcccbef4990919e6bf8579414caf`). The merged chain preserves
> the stair policy path and brings in LowCmd queue depth 1, `tcpNoDelay()`,
> persistent callback spinners, simulation-time LowCmd cadence scheduling,
> staged T1-T4 diagnostics, and Earth IMU policy-input fallback. The task
> worktree build passes. Short Earth regression passes for FixedStand and RL
> zero: T0/T1/T2 median cadence is 500 Hz, T3/T4 are 1000 Hz,
> `using_imu_policy_input=1`, policy input/action output finite checks pass,
> and RL zero remains upright for 9 sim-s (`min_base_height=0.311336 m`,
> `max_tilt=3.04993 deg`, median RTF `1.00864`). Remaining active task:
> isolate and minimally fix the `vx=0.10 m/s` command-response issue on
> `fix/0722-earth-rl-command-response`. No merge to `master`.

> **2026-07-22 Earth RL timebase fastcheck**: branch
> `fix/0722-earth-rl-timebase-fast-validation` runs the requested isolated
> Earth `PHYSICS_PROFILE=normal` RL validation from a separate worktree. The
> Unitree runtime build passes after pinning the local worktree venv `empy` to
> ROS-compatible `3.3.4`. `State_RL` now loads the requested stair policy
> (`policy_act_inference_stair.pt`, SHA256
> `2d5aa72511c0c6609c02f4105845eee6974d3d73431497f8f35306da9588fe14`).
> Runtime timing shows policy inference near 48 Hz simulation time, but low-level
> command publication is about 414 Hz against the 500 Hz target with deadline
> misses. Earth normal RTF fails the requested stability gate (`median=0.750945`,
> `p10=0.407949`), and the first RL `vx=0.10 m/s` smoke falls
> (`min_base_height=0.078849 m`). Overall verdict:
> `EARTH_RL_RTF_BLOCKED`; no merge to `master`.

> **2026-07-21 RL fast validation infrastructure**: branch
> `test/0721-rl-fast-validation` adds fast RL-state iteration infrastructure:
> `tools/build_rl_fast.sh`, ROS-free metrics/window helpers, a F0/F1/F2
> FixedStand smoke runner, live capture, offline replay scaffold, tests, and
> build/provenance evidence. The tracked worktree copy of
> `tools/build_with_venv.sh` matches the root script SHA256 and is the effective
> build entry. Fast Unitree build, runtime profile build, and full build attempt
> all return exit code 0. Live F0 native FixedStand now executes and fails as
> `FAIL_BASE_HEIGHT` after entering FixedStand; F1/F2 remain gated by the F0
> failure, and RL state/action diagnostics remain future work.

> **2026-07-21 Unitree runtime rebuild and retest**: branch
> `fix/0721-unitree-runtime-rebuild-and-retest` rebuilt the Unitree/Torch/Gazebo
> runtime chain with GCC/G++ 11 and the project `tools/build_with_venv.sh`
> profile. The original CUDA failure is traced to `/usr/bin/gcc` selecting GCC
> 12 while GCC 12 `cc1plus` is missing; `nvcc -ccbin /usr/bin/g++-11` passes.
> Target artifacts resolve to the current worktree `devel`. Full un-whitelisted
> catkin remains blocked by missing `move_base_msgs` in `unitree_move_base`.
> Runtime validation stops at C0-A native FixedStand: run 01 entered FSM state 2
> but failed the base-height threshold (`min_base_height=0.110049 m`), and run
> 02 stalled `/clock`. C0-B/C0-C/Earth RL were not entered; the requested stair
> policy was only hashed for provenance.

> **2026-07-20 Earth flat-ground runtime validation**: branch
> `test/0720-earth-flat-ground-runtime` cherry-picks `5f5f9045` into the
> runnable `earth-rl-motion` worktree. G0 confirms the platform models are
> absent at runtime (`ground_plane` only, RTF median `0.983`). Validation stops
> at G1: with the controller epoch active but no FixedStand command, A1 settles
> to `min_base_height=0.05044 m` before FixedStand. C0 competition FixedStand
> rerun also fails (`max_tilt_deg=170.68`, final FSM state `2`), so the current
> artifact/entry chain cannot establish the platform root-cause closure. E0 and
> all RL cases are blocked by gate policy; no RL performance conclusion is made.

> **2026-07-20 earth.world flat-ground fix**: branch
> `fix/0720-earth-flat-ground` removes the inline `platform_1` and
> `platform_2` models from `unitree_gazebo/worlds/earth.world`, including
> their box visual and collision geometry. `WORLD_MODE=earth` still spawns A1
> at `x=0.0 y=0.0 z=0.6 yaw=0.0`, matching the existing Unitree A1 earth launch
> height while placing the robot over the Gazebo `ground_plane` include instead
> of a raised platform. XML/SDF/static checks pass; full A1 runtime validation
> remains pending in a worktree with a built `devel/setup.bash`.

> **2026-07-18 FAST-LIO2 runtime point-cloud orientation**: isolated runtime
> validation from `fix/0718-runtime-pointcloud-orientation` confirms the active
> adapter comes from a worktree containing `69ff34e7`, has no rotation params,
> and preserves `/scan` into `/scan_pointcloud2` exactly (`24000/24000` points,
> max error `0`, frame `laser_livox`). TF shows LiDAR local `+X` maps to
> `base` `[+0.707, 0, -0.707]`, so the remaining 45° downward direction is the
> physical mount. FAST-LIO2 `/cloud_registered` ground normal is within
> `0.550°` of `+Z`. See
> `docs/reports/0718_runtime-pointcloud-orientation.md`.

> **2026-07-18 FAST-LIO2 point-cloud frame semantics**: the adapter no longer
> rotates Livox points by default while retaining `laser_livox`; opt-in
> rotations require an explicit output frame. Static/unit validation and the
> LiDAR-to-IMU extrinsic checker pass. Isolated runtime sensor validation is
> blocked by a system-Python `unitree_guide.msg` import failure; no moving-map
> claim is made. See `docs/reports/0718_fast-lio2-pointcloud-frame-semantics.md`.

> **2026-07-17 FAST-LIO2 Stage 2 interface**: branch
> `feat/0717-fastlio2-stage2` adds compatibility-preserving transparent relays
> `/Odometry → /state_estimation` and
> `/cloud_registered → /registered_scan`. Static contract tests and the
> integration-package build pass. Five-minute Gazebo validation remains
> blocked by isolated-worktree runtime setup and a full-build CUDA host
> compiler failure; see `docs/reports/0717_fastlio2-stage2.md`.
> A subsequent isolated retry reached 2.518 s ROS time and verified `/scan`
> plus upright IMU gravity, but the partial worktree devel could not load the
> Unitree joint-controller plugin. No five-minute result is claimed.
> Final validation subsequently passed the revised 150 s target using the
> read-only `trot-rl/devel` overlay: 1500 odometry and 1500 registered scans at
> 10 Hz. A new Odometry TF bridge supplies `camera_init→body`; `map→body`
> succeeded 1197/1501 times including the pre-bridge startup interval.

## Snapshot
- Date: 2026-07-20
- Branch: `test/0720-earth-rl-motion` at `84ff02d7` plus task changes
- Project type: ROS1 Noetic + Gazebo Classic (robotics competition simulation)
- Current focus: isolated `earth.world` launch and RL-motion precondition
  validation

## Active Work
- **2026-07-20 earth flat-ground runtime gate（当前分支
  `test/0720-earth-flat-ground-runtime`）**：已接入 `5f5f9045` 并复用
  `earth-rl-motion` 的可运行 `devel` artifact。G0 通过并证明
  `platform_1/platform_2` 运行时消失；G1 未通过，C0 competition 对照也未复现
  旧的稳定 FixedStand。因此 E0/E1/E2/E3/E5 均按门控阻塞。用户要求的
  stair policy 后续测试已记录，但未在本轮切换或运行。
- **2026-07-20 earth.world flat-ground fix（当前分支
  `fix/0720-earth-flat-ground`）**：删除 `earth.world` 中覆盖出生点的
  `platform_1` 和前方 `platform_2` 完整 model，保留 `sun`、单个
  `ground_plane` include、physics 和 scene。该任务不修改 FSM、RL policy、
  控制器、URDF/xacro、spawn z 或 competition 生成路径。静态检查通过；
  隔离 worktree 缺少 `devel/setup.bash`，因此带 A1 spawn/controller 的
  runtime smoke 待在已构建 overlay 中执行。
- **2026-07-20 Earth RL motion benchmark（当前分支
  `test/0720-earth-rl-motion`）**：新增 `WORLD_MODE=earth` 接入和 tracked
  `earth.world`，competition 默认路径保持不变。world/topic smoke 证明
  `/clock` 与 `/gazebo/model_states` 发布，模型包含 `ground_plane`、
  `platform_1`、`platform_2`、`a1_gazebo`。E0 FixedStand 成功进入 FSM state
  2 并运行 15.174 s 仿真时间，但 `max_roll_deg=91.911513`，判定
  `FAIL_ATTITUDE`；E1+ RL trial 因 E0 失败未运行。Torch-enabled build 仍被
  CUDA `cc1plus` 探测失败阻塞，Torch-off/full workspace build 被
  `move_base_msgs` 和无关 UAV/SDK 示例目标阻塞。当前结论：先修
  earth spawn/contact pose，再进入 RL recovery。
- **2026-07-20 G2 Fast Exit Gate A（当前分支
  `diagnose/0719-g2-pre-wave-block-reason`）**：新增只读/诊断型
  fast-exit probe 和 runner，先执行 P0 FixedStand-only。有效运行
  `p0_fixedstand_run_02` 失败：`CONTACT_NOT_READY`、
  `FIXEDSTAND_NOT_ENTERED`、`FALL_DETECTED`，最终 FSM 仍为 PASSIVE，最低
  model height 为 `0.05698662028992169 m`。Gate A verdict:
  `G2_FAST_EXIT_SHARED_BASE_FAILURE`。P1/P2 未运行，active RL 未授权；只允许
  后续做 RL shadow/static validation。
- **2026-07-19 G2-D1 Gate V validator semantics（完成于分支
  `fix/0719-g2-fall-validator-frame-semantics`）**：已冻结四个旧 G2-B smoke
  trial 的 pose/fall timeline，并用显式 tilt+height 语义离线重判。旧
  validator 的 `FALL_DETECTED` 来源是 `/gazebo/model_states` 的 height-only
  predicate（`min(z)<0.12m`），不是 roll 阈值；D0 FixedStand probe 显示
  正常站立时 model/link/base_w 高度约 0.326m、roll/pitch 近零；四个旧
  trial 在新语义下仍为 FALL。Gate V verdict:
  `G2_VALIDATOR_NO_DEFECT`（针对疑似 frame/pose false positive），未修改
  locomotion/control/physics，G2-R 仍未授权。下一步进入 Gate P：定位
  Pre-WAVE 首个阻塞原因。
- **2026-07-19 G2-B Trotting baseline tooling（in progress）**：已建立
  `docs/active/0718-g2-trotting-motion-baseline/` 的 test plan、acceptance
  criteria、risk register、evidence index、baseline/root-cause/recovery report
  占位和 4 个 ADR。新增
  `experiments/runs/0718_g2_trotting_motion_baseline/` 运行器、ROS trial
  capture、metric helpers、aggregator 和 6 个纯函数单测。已对
  `vx=0.00/0.10/0.30/0.50` 各跑 1 个 smoke trial，4/4 均为 INVALID：
  `WAVE_ALL_NOT_REACHED`、`GAIT_NOT_ADVANCING`、`FALL_DETECTED`。非零速度的
  resolved command 到达 controller，但 wave/gait 未启动；`vx=0.50` controller
  pane 捕获到 Trotting output non-finite (`q=0`) 后 wave cancelled。当前 G2
  verdict 为 `G2_BASELINE_INCONCLUSIVE`，未进入 G2-R。G2-B 阶段未修改
  controller、URDF/SDF 或 Gazebo physics。
- **2026-07-17 单层 Trotting/RL 速度短窗验证（完成）**：在六个全新、独立 ROS
  master epoch 中，对 `0.1/0.5/1.0 m/s` `/cmd_vel` 指令采集了 Gazebo 真值轨迹。
  当前 RTF 为 0.065--0.151，实际速度并不随 RTF 单调变化；RL 的 0.5/1.0 m/s
  停止尾速为 0.288/0.328 m/s，是优先风险。图、CSV、原始轨迹和完整限制见
  `docs/reports/0717_trot-rl-speed-profile.md`。
- **2026-07-17 single-floor Trotting/RL mapping validation (PARTIAL)**: fixed seed 77
  headless trials completed the same bounded `/cmd_vel` route in fresh epochs.
  Trotting/RL moved 0.192487/0.169675 m with finite truth and FAST-LIO2 odometry;
  5,006/5,048 registered-cloud points were saved as PCD and top-down PNG.
  Trotting stopped to 0.007076 m/s mean tail speed; RL retained 0.024517 m/s and
  needs a longer stop regression. The short route only validates local mapping,
  not full-floor coverage; about 1.0%/1.1% of saved points lie at x < -10 m and
  are treated as remote drift/outliers. See `docs/reports/0717_trot-rl-floor-mapping.md`.
- **Gazebo RL timing diagnostics (in progress on `fix/0716-gazebo-rl-time-alignment`)**:
  opt-in buffered CSV tracing now correlates FSM, state, policy, history,
  action, and LowCmd generations. At RTF 0.276 the current policy ran at
  49.25 Hz simulation time with no overtime, while pause advanced policy and
  action on wall overtime; 28 LowCmd copies overlapped action writes. The first
  behavior fix now prevents overtime from being treated as a completed policy
  period. Policy input and output snapshots remove mixed-state observation and
  torn LowCmd updates. Runtime history is deduplicated and simulation-time
  gated while preserving the unverified training-time entry convention;
  Reset cleanup now invalidates pre-reset commands/actions, clears policy and
  history state, and restarts history in the new simulation epoch; the ROS
  microsecond timestamp is now atomic 64-bit. Post-commit review also tags
  policy outputs with the reset generation so an in-flight pre-reset inference
  cannot be applied in the new epoch. Automated regression coverage passes;
  the final report is available at
  `docs/reports/0717_gazebo_rl_time_alignment.md`.
- **Trotting simulation-time synchronization validated**: Gazebo locomotion
  updates now run only when `/clock` advances. Wave phase, estimator `dt`,
  height/readiness timing, and desired body/yaw integration use measured
  simulation deltas. At RTF ~0.10, the 0.75 s transition plus 0.20 s readiness
  required 0.946 simulated seconds rather than 0.10 s. Pause/backward/jump
  paths reset all stance; running Wave now latches off on tilt, scheduled
  contact loss, or non-finite output.
- **Trotting nominal entry validated**: A1 has one foot-space nominal stance;
  FixedStand derives its target by IK. An invalid duplicate parent joint on
  each foot was removed, making nominal FixedStand stationary and restoring
  real contact feedback. Trotting inherits current height/feet, transitions
  height over 0.75 s, and gates wave on velocity, attitude and four fresh foot
  forces. Final `0.3 m/s` forward test averaged `0.273 m/s` in the correct
  direction.
- **FAST-LIO2 startup + TF bridge** (merged to `develop`): Controller starts before FAST-LIO2, auto-commanded FixedStand, IMU upright check. Duplicate adapter eliminated. Controller foreground mode preserved via `fg`. Camera-init TF bridge: `map→camera_init` is direct copy of `map→imu_link` (Ry(-45°) removed — was duplicate of FAST-LIO2 extrinsic_R). Rotation responsibility documented in YAML and ADR-0714.
- **FAST-LIO2 LiDAR–IMU axes corrected**: Source audit established that FAST-LIO2 applies `p_imu = R * p_lidar + T`. The SimEnv point source is local `laser_livox`, so the mapping config now uses the direct runtime TF `imu_link→laser_livox`: `Ry(+45°)`, `[0.2,0,0.08]`. `map→camera_init` remains a no-rotation copy of `map→imu_link`; restart FAST-LIO2 to load the new YAML.
- **Controller terminal keyboard**: `auto.sh` explicitly binds background `junior_ctrl` stdin to `/dev/tty` and then waits for it in foreground mode. This avoids non-interactive Bash replacing stdin with `/dev/null` during FAST-LIO2 startup.
- **FSM nullptr segfault fix**: Guarded keyboard/RPC/checkChange paths against TROTTING/RL states when Torch is disabled. Default build (Torch OFF) no longer crashes on `4`/`6` key press or rostopic. Added FSM-level nullptr safety net.
- **Dedicated controller + rviz terminals**: `auto.sh` defaults to named tmux sessions for `junior_ctrl` and rviz, so their commands survive GUI-terminal failure and can be reattached from any terminal. Every startup first stops the prior two sessions and clears their runtime records; attached GNOME Terminal clients exit with their session rather than becoming orphaned shells. Controlled startup verified both sessions alive, `/Odometry` at ~10 Hz, and Ctrl-C cleanup. The launcher sanitises only its own Snap Code-contaminated environment while preserving ROS inside each session. Removed `CONTROLLER_FOREGROUND`; added `ENABLE_RVIZ` and `TERMINAL_BACKEND`.
- **Build/startup recovery**: the supported build script now compiles the complete `auto.sh` runtime profile, including `livox_laser_simulation` (required to publish `/scan`), while excluding unrelated locally added packages (including legacy PS3 utilities requiring libusb-0.1); `auto.sh` checks for `junior_ctrl` before it mutates the running simulation state.
- **FAST-LIO2 runtime validated**: PointCloud2 intensity fix, TF bridge, RViz config. Converges to ~7 cm within 20 s in FixedStand. All output topics publish stably.
- **Locomotion ready**: Trotting with `/cmd_vel` + `/fsm/state_cmd` programmatic control. RL policies diagnosed as non-functional for velocity tracking. P1 blocked by Gazebo physics stability.
- **`auto.sh` cleanup**: Comprehensive process cleanup on startup.
- **2026-07-13 reduced-P0 update**: measured RTF=0.068, then completed an independent 10 s ROS-time capture. FAST-LIO2 changed 0.286359 m / 1.216603° while truth changed 0.004874 m / 0.774496°; messages remained finite and map output continued, but P0 still fails. P1 remains unavailable because the current non-Torch controller only provides in-place/bounded test states, not a real trajectory gait.
- **2026-07-13 P0 update**: a 60 s ROS-time fixed-stand attempt reached only 28.900 s. During it truth moved 0.018344 m / 0.592970°, and an external `/home/zzf/桌面/unitree_ex` launch attached to the same ROS master with duplicate `/gazebo` and `/robot_state_publisher` node names, ending the SimEnv session. P0 remains blocked pending master isolation and a truly stationary support/control mode.
- FAST-LIO2 Stage 0 & 1 部署测试: Stage 0 (传感器/TF/时间) 全部通过；Stage 1 L1+L2 接口通过。此前的 325.253 m / 51.607°“静止”结果因 `START_CONTROLLER=0` 导致机器人跌倒而无效；固定站立 P0 在 60 s 墙钟/4.3 s 仿真时间内为 0.001967 m / 0.066852°，完整 60 s 仿真时间仍待完成；P1 受已禁用的真实步态控制链路阻塞
- FAST-LIO2 launch 文件修复: 取消注释节点、修复 rosparam namespace (public NodeHandle vs private ns)、移除冗余 param 标签
- 文档完善: 新增 `docs/cli-reference.md` 命令速查, 更新 `docs/README.md` 索引和 `docs/quick-start.md` (FAST-LIO2 用法 + 参数修正)
- FAST-LIO2 集成骨架: `src/simenv_fast_lio2_integration/`
- PointCloud→PointCloud2 适配器
- FAST-LIO2 配置 (simenv_mid360.yaml) 和 launch
- 连接 GitHub remote (`github` → `git@github.com:zzf/SimEnv.git`, 尚未 push)
- venv 构建脚本: `tools/build_with_venv.sh` (zzf/0704-build-with-venv)
- FAST-LIO2 workspace 文档修正: 明确 SimEnv 是 catkin workspace 根目录 (docs/0704-fast-lio2-workspace-docs)
- FAST-LIO2 编译环境审计: 静态检查全部通过，catkin_make 被 libtorch (C++ SDK) 阻塞 (feat/0704-fast-lio2-mapping)
- build_with_venv.sh: 现已支持自动检测 torch CMake prefix，TorchConfig.cmake 路径问题已解决；剩余阻塞: CUDA toolkit + livox_ros_driver
- build_with_venv.sh: 现已强制使用 gcc-11/g++-11 构建，CUDA host compiler 错误已消除；CMakeCache 需清理后重试
- 编译修复: unitree PIE, FAST_LIO C++17, livox shared_ptr/serialization/missing-includes 共6个文件修复 (fix/0704-fast-lio2-build-errors)
- FAST-LIO2 部署指南: 新增 `docs/slam/fast_lio2_deployment_guide.md`，系统整理部署流程、传感器配置映射、参数说明、编译环境、运行验证和排错指南 (docs/0706-fast-lio2-deploy-guide)
- Torch ABI 隔离: `unitree_guide/junior_ctrl` 现已默认不依赖 Torch 编译，新增 `UNITREE_ENABLE_TORCH_POLICY` option (OFF)，`_GLIBCXX_USE_CXX11_ABI=0` 污染已消除 (fix/0704-unitree-torch-abi-isolation)

## Git Remotes
- `origin`: https://gitee.com/guoyulun/SimEnv.git (Gitee, 主远程)
- `github`: git@github.com:zzf/SimEnv.git (GitHub, 新增, 尚未 push)

## Branch Naming Policy (Updated)
- 维护/仓库配置类: `zzf/MMDD-short-name` (项目级覆盖规则)
- 不再使用 `chore/MMDD-short-name`

## Known Risks
- The simulation discontinuity defaults (`0.05 s` maximum forward step and
  `0.5 s` wall pause detector) passed the current 2 ms physics configuration
  but may require tuning when controller scheduling is heavily starved.
- Trotting readiness thresholds are validated only on flat Gazebo terrain;
  slope/stair contact thresholds and lateral/yaw tracking remain to be tuned.
  RL remains unvalidated and unchanged.
- The stable single-parent A1 foot model uses Gazebo fixed-joint lumping. Legacy
  `/ground_truth/*_foot` P3D plugins refer to the child body and may not publish;
  restore those pose topics with a kinematic publisher rather than reintroducing
  the unstable duplicate or independently preserved foot body.
- GitHub 远程仓库可能为空或已有历史，首次 push 前需确认目标分支
- 随机生成的建筑布局可能在某些参数组合下产生不可达房间或源重叠
- Gazebo Classic 已停止维护，长期可能需要迁移到 Ignition/Gazebo Fortress
- FAST-LIO2: SimEnv adapter 输出 `PointCloud2`，必须使用 `preprocess.lidar_type=4`；其 MARSIM 分支的 `last_lidar_end_time_` 未初始化缺陷已在外部源码修复并选择性重编译，仍需完成完整 60 s 仿真时间 P0
- FAST-LIO2 axis correction has offline and live-TF evidence, but its moving mapping regression remains pending a clean mapper restart.
- 当前 3 层场景实时因子很低（60 s 墙钟仅 4.3 s ROS 时间），完整 P0 需要约 10--15 分钟墙钟；`junior_ctrl` 因 Torch ABI 隔离未编译 trot/RL 状态，P1 真实步态测试需重新启用其依赖
- IDE 的 Miniconda Python 3.13 与 Noetic xacro 不兼容；ROS/Gazebo runtime 应使用系统 Python 3.10

## Validation Status
- Build: Torch-enabled `catkin_make -j` passes after Trotting timing/safety
  changes
- Locomotion: nominal FixedStand, gated zero Trotting, forward pair, forced
  RTF ~0.10 timing, pause/reset, tilt abort, and contact-loss abort pass
- Unit tests: `building_generator_core/test/` (3 tests), `building_generator_classic/test/` (2 tests)
- Remote config: `origin` 保持 Gitee, `github` 新增成功
- Governance: 骨架完整, 分支规则已更新

## Next Steps
- 用户确认后执行首次 push: `git push -u github develop`
- 补充 CI/自动化测试流程
- 评估将 UAV simulator 子模块独立或移除
- 完成固定站立的 60 s **ROS 仿真时间**静止测试
- 用户授权后重新启用 `junior_ctrl` Torch 步态策略，或明确批准标记为 SLAM-only 的运动替代方案；随后依序执行 5 m 直线与矩形短闭环
