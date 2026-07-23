# Module Status

> **2026-07-23 `simenv_fast_lio2_integration` external build staging**:
> FAST-LIO2 dependencies are now prepared through a repeatable staging script
> from fixed external source commits. Staged FAST_LIO uses C++17; staged
> `livox_ros_driver` keeps message generation active while skipping the real
> hardware driver node by default. The runtime whitelist build passes through
> `tools/build_with_venv.sh`. The new pointcloud continuity diagnostic is
> unit-tested and uses wall-clock windows plus vectorized PointCloud2 parsing;
> live continuity still needs one clean isolated ROS master run because the
> first validation attempts left stale adapter registrations that polluted
> `/scan_pointcloud2` ownership checks.

> **2026-07-22 `unitree_guide` Earth RL navigation baseline**:
> `master` at `4eeae260`, build PASS, `WORLD_MODE=earth`, `PHYSICS_PROFILE=normal`.
> Plane policy recommended; RL zero and IMU fallback PASS; LowCmd ≈500 Hz;
> validated speed range `0.20–0.40 m/s`; `vx=0.10 m/s` remains ineffective.
> Stair policy has higher flat-ground start threshold and larger yaw drift;
> obstacle/stair behavior is not validated here; RTF fluctuation is recorded but
> non-blocking. **Preserved interfaces**: `/cmd_vel`, `/fsm/state_cmd`,
> `/clock`, robot state / odometry source, IMU input, RL policy runtime
> selection, LowCmd transport/application cadence.

> **2026-07-22 `unitree_guide` Earth RL fastcheck**:
> `fix/0722-earth-rl-timebase-fast-validation` builds the Torch-enabled Unitree
> runtime in an isolated worktree and changes `State_RL` to load the requested
> stair policy by default. Runtime confirms the policy path and SHA256 match.
> The timebase evidence is mixed: policy ACTION/POLICY_WAIT rows remain near
> 48 Hz simulation time, but LOWCMD publication is about 414 Hz versus the
> 500 Hz target with deadline misses. Earth `normal` RTF fails the requested
> p10 gate, and the first `vx=0.10 m/s` RL smoke is unstable, so speed range
> validation remains blocked.

> **2026-07-21 `unitree_guide` RL fast validation infrastructure**:
> `test/0721-rl-fast-validation` adds a build wrapper, pure metric helpers,
> offline replay scaffold, F0/F1/F2 FixedStand runner, and live ROS/Gazebo
> capture for fast RL-state iteration. Fast/runtime/full build entries via
> `tools/build_with_venv.sh` pass in the isolated worktree. F0 native
> FixedStand executes and fails as `FAIL_BASE_HEIGHT` after entering
> FixedStand; F1/F2 are gated by that failure. No controller, policy,
> observation/action, model, spawn, physics, or `earth.world` behavior changed.

> **2026-07-20 `unitree_guide` Earth runtime validation**:
> `test/0720-earth-flat-ground-runtime` verifies that the flattened
> `earth.world` no longer publishes `platform_1` or `platform_2` at runtime.
> G0 passes, but G1 fails before FixedStand because base height drops to
> `0.05044 m`; C0 competition FixedStand rerun also fails with
> `max_tilt_deg=170.68` despite final FSM state `2`. E0 and RL motion gates
> remain blocked. No locomotion/controller/model/policy/physics changes were
> made.

> **2026-07-20 `unitree_guide` earth flat-ground fix**: `earth.world` no
> longer contains inline raised platform models or platform collisions. It now
> keeps only the existing physics, scene, `sun`, and one `ground_plane` include
> for earth-mode motion benchmarking. Launch defaults, spawn pose, controller,
> robot model, RL policy, and competition generation behavior are unchanged.
> XML/SDF/static checks pass; full A1 runtime validation still needs a built
> worktree overlay.

> **2026-07-18 `simenv_fast_lio2_integration` runtime**: added
> `runtime_pointcloud_smoke_check.py` and validated live Gazebo data. The active
> adapter path contains `69ff34e7`, has no rotation params, preserves all
> `/scan` point coordinates into `/scan_pointcloud2`, and FAST-LIO2
> `/cloud_registered` fitted ground normal is within `0.550°` of `+Z`.

> **2026-07-18 `simenv_fast_lio2_integration`**: default adapter behavior now
> preserves source point coordinates and header frame semantics; non-zero
> adapter rotations require `rotated_frame_id`. Python/unit and extrinsic
> checks pass. Isolated live `/scan` validation is pending repair of the
> system-Python `unitree_guide.msg` import path.

> **2026-07-17 Stage 2 navigation topics**: `simenv_fast_lio2_integration`
> now exposes `/state_estimation` and `/registered_scan` via transparent
> `topic_tools` relays without removing FAST-LIO2's legacy output topics or
> changing `camera_init/body` frames. Static tests and package build pass;
> the original five-minute target was later revised to and completed as a
> 150-second isolated runtime
> (`feat/0717-fastlio2-stage2`).
> **2026-07-17 Stage 2 runtime**: revised 150 s isolated target passed with
> `/state_estimation` and `/registered_scan` at 10 Hz. Added the missing
> dynamic `camera_init→body` Odometry TF bridge; `map→body` succeeded for
> 1197/1501 samples including its live-startup interval.
> **2026-07-19 G2-B Trotting baseline tooling**:
> `experiments/runs/0718_g2_trotting_motion_baseline/` now contains isolated
> trial, capture, metric, aggregation, and pure-test helpers for G2-B evidence.
> This is a measurement-only gate; Unitree controller behavior, URDF/SDF, and
> Gazebo physics remain frozen. Four one-run smoke trials covered
> 0.00/0.10/0.30/0.50 m/s and all were invalid before WAVE_ALL/gait execution,
> so the current verdict is `G2_BASELINE_INCONCLUSIVE`.
> **2026-07-19 G2-D1 Gate V validator semantics**:
> The suspected fall-validator frame/pose false positive was not confirmed.
> The old predicate is height-only over `/gazebo/model_states`, while offline
> reclassification with explicit normalized-quaternion tilt+height semantics
> still marks all four trials fallen. D0 FixedStand pose probes remain upright
> and above the threshold. Gate P remains required to locate the first
> Pre-WAVE blocker; no locomotion behavior was changed.
> **2026-07-20 G2 Fast Exit Gate A**:
> Minimal P0 FixedStand-only evidence failed before FixedStand. No foot-contact
> samples were captured, final FSM remained PASSIVE, and model height reached
> `0.05698662028992169 m`. Verdict:
> `G2_FAST_EXIT_SHARED_BASE_FAILURE`; P1/P2 and active RL are blocked.
> **2026-07-20 Earth world benchmark**:
> `auto.sh` now supports isolated `WORLD_MODE=earth` launch against the tracked
> Unitree Gazebo `earth.world`. World/topic smoke passed with `/clock` and
> `/gazebo/model_states`; E0 FixedStand entered FSM state 2 for 15.174 s sim
> time but failed attitude validation (`max_roll_deg=91.911513`). RL runtime
> trials remain blocked until earth spawn/contact pose is repaired. No
> locomotion/control behavior changed.

## Overview

ROS1 Noetic 仿真工作区，含 8 个第一级 ROS 包。2026-07-04 生成。

> **2026-07-04 update**: 仓库远程配置更新 (新增 GitHub remote `zzf/0704-connect-github-remote`)。业务模块代码无变动。
> **2026-07-04 update**: 新增 `tools/build_with_venv.sh` venv 构建脚本 (`zzf/0704-build-with-venv`)。
> **2026-07-04 update**: 修正 FAST-LIO2 文档中 workspace 结构描述 (`docs/0704-fast-lio2-workspace-docs`)。
> **2026-07-04 update**: FAST-LIO2 编译环境审计: 静态检查全通过, catkin_make 被 libtorch 阻塞 (`feat/0704-fast-lio2-mapping`)。
> **2026-07-04 update**: `tools/build_with_venv.sh` 现已自动检测 torch CMake 前缀; TorchConfig.cmake 路径问题已解决。
> **2026-07-04 update**: `tools/build_with_venv.sh` 现已强制 gcc-11/g++-11 构建; CUDA host compiler 错误已消除。
> **2026-07-06 update**: Torch ABI 隔离修复: `unitree_guide/junior_ctrl` 现已默认不依赖 Torch 编译，`UNITREE_ENABLE_TORCH_POLICY=OFF` (fix/0704-unitree-torch-abi-isolation)。
> **2026-07-06 update**: 新增 FAST-LIO2 部署指南 `docs/slam/fast_lio2_deployment_guide.md`，系统整理部署流程、传感器映射、参数说明与排错指南 (docs/0706-fast-lio2-deploy-guide)。
> **2026-07-13 update**: FAST-LIO2 Stage 0 与 Stage 1 初始 L1/L2 部署检查通过; `simenv_fast_lio2_mapping.launch` 修复 3 个 bug; 新增 `docs/cli-reference.md` 命令速查; 更新 `docs/README.md` 和 `docs/quick-start.md` (exp/0713-fast-lio2-stage01).
> **2026-07-13 runtime update**: 修正 SimEnv PointCloud2 adapter 与 FAST-LIO2 `lidar_type` 的消息契约（设为 `4`/MARSIM），并修复该路径的未初始化结束时间。此前 325.253 m / 51.607° 数据来自未启控制器时 A1 跌倒，不能用于静止判定；固定站立下 60 s 墙钟结果为 0.001967 m / 0.066852°。完整 60 s 仿真时间及 P1 真实步态仍待完成（exp/0713-fast-lio2-stage01-runtime）。
> **2026-07-13 extended-P0 update**: 60 s ROS-time 尝试在 28.900 s 中断；Gazebo 真值已移动 0.018344 m / 0.592970°。另一工作区 `/home/zzf/桌面/unitree_ex` 加入同一 ROS master 并抢占 `/gazebo`、`/robot_state_publisher` 名称，故重试前必须隔离 ROS master 并保证真实静止。
> **2026-07-13 reduced-P0 update**: 实测 RTF=0.068，完成独立 10 s ROS-time 窗口。FAST-LIO2 为 0.286359 m / 1.216603°，真值为 0.004874 m / 0.774496°；数据有限值且注册点云仍发布，但 P0 未通过。当前非 Torch 控制器没有真实 5 m 轨迹步态。
> **2026-07-14 FAST-LIO2 validation**: PointCloud2 格式修复 (xyzi32)、TF 桥接 (map→camera_init)、RViz 配置。FixedStand 静止 30s 收敛至 ~7cm。全部输出话题 (/Odometry, /Laser_map, /cloud_registered, /path) 发布稳定。Trotting `/cmd_vel`+`/fsm/state_cmd` 就绪，P1 受 Gazebo 物理稳定性阻塞。
> **2026-07-14 startup-order fix**: `auto.sh` 重构执行顺序: 控制器在 FAST-LIO2 之前启动，自动发送 FixedStand 指令，等待 IMU 确认直立姿态后再初始化 SLAM。修复因机器人未站立导致的 IMU 重力方向估计错误和点云 Z 轴漂移 (`fix/0714-fastlio2-startup-order`)。
> **2026-07-14 TF bridge + foreground**: camera_init 桥接方向迭代: `laser_livox`→`imu_link`→`imu_link+Ry(-45°)`，修复点云 45° 倾斜和 odometry X 轴方向错误。`CONTROLLER_FOREGROUND` 默认 1，FAST-LIO2 初始化后 `fg` 拉回前台。launch 文件 `enable_adapter:=false` 避免重复 scan_to_pointcloud2 节点。
> **2026-07-14 frame correction**: 诊断并修复 Odometry 机体坐标轴异常。根因: `map_to_camera_init_bridge.py` 中 Ry(-45°) 为重复旋转（LiDAR 45°倾斜已由 FAST-LIO2 extrinsic_R 正确处理）。修复: 移除重复旋转，`map→camera_init` 直接复制 `map→imu_link`。新增 ADR-0714 (坐标系约定与旋转责任边界)。静态检查/编译/旋转矩阵验证通过，运行时测试因 ROS master 未运行标记为 NOT RUN (`zzf/0714-fast-lio2-frame-fix`)。
> **2026-07-15 FAST-LIO2 axis correction**: 依据运行时 `imu_link→laser_livox` TF 和 FAST-LIO2 的 `p_imu=R*p_lidar+T` 源码路径，外参改为直接变换 `Ry(+45°), [0.2,0,0.08]`，替换错误逆变换。新增 xacro/YAML 校验脚本；bridge 继续不旋转。运行中的 mapper 尚未重启，运动建图回归待执行 (`fix/0715-fast-lio2-axis`)。
> **2026-07-15 controller keyboard fix**: `auto.sh` 的 FAST-LIO2 启动路径改为显式把 `junior_ctrl` stdin 绑定至 `/dev/tty`，并以 `wait` 取代非交互 shell 不可靠的 `fg`。交互终端可继续使用键盘；无 TTY 时提示改用 ROS 控制话题 (`fix/0715-auto-keyboard`)。
> **2026-07-15 build/startup guard**: `build_with_venv.sh` 默认编译 `auto.sh` 的完整运行时包集，包含发布 `/scan` 所需的 `livox_laser_simulation`，并避开工作区内不属于仓库的可选包（如依赖 libusb-0.1 的 `ps3joy`）；`auto.sh` 在清理/生成场景前检查 `junior_ctrl` 是否已生成，避免控制器缺失导致启动后才退出 (`fix/0715-build-auto-startup`)。

> **2026-07-15 FSM nullptr segfault fix**: 当 Torch 策略默认禁用时，键盘 `4`/`6` 键和 ROS callback `data:4`/`data:6` 不再触发 TROTTING/RL 状态转换（对应状态指针为 nullptr）。新增 FSM 级 nullptr 安全检查作为防御层。`auto.sh` 帮助文本已更新，标明 `4`/`6` 需要 Torch 构建 (`fix/0715-build-auto-startup`).
> **2026-07-15 locomotion runtime validation**: Torch-enabled headless tests confirmed both FSM transitions and `/cmd_vel` subscription, but neither gait passes. Trotting produces Gazebo/IMU NaN under zero Twist; uninitialized `_dYawCmdPast` is the leading propagation source. RL stays finite but failed a paired `linear.x=0.3` test (`4.686 s`, `dx=-0.000315 m`, `dy=-0.036406 m`, yaw about `-9.6 deg`). The likely RL contract mismatch is inconsistent joint reindexing versus default-pose ordering; policy training metadata is absent. See `docs/reports/0715_trotting-rl-cmd-test.md` (`exp/0715-trotting-rl-cmd-validation`).
> **2026-07-15 Trotting repair**: field-level guards showed IK position was the first non-finite output. Gazebo 11 returned the folded calf through an equivalent positive angle, corrupting PD/kinematics, while the spawn pose sat near the folded-knee singularity. Revolute feedback normalization, a non-singular local FixedStand spawn pose, initialized yaw filtering, command timeout/clamping, and finite motor guards now keep zero/forward Trotting finite. Forward direction passes; `0.3 m/s` averaged about `0.121 m/s`, so calibration remains pending (`fix/0715-trotting-safety`).
> **2026-07-15 dedicated terminals**: `auto.sh` 默认在独立 tmux 会话中启动 `junior_ctrl` 和 rviz（`simenv-junior_ctrl`、`simenv-rviz`）；即使 GNOME Terminal 闪退，会话可用 `tmux attach-session -t ...` 重连。每次新启动会先停止旧的两个会话并删除其运行记录；attach 客户端随会话结束而退出，不再遗留交互 shell。受控完整启动已验证两个会话存活、`/Odometry` 约 10 Hz，且 `Ctrl-C` 会清理 ROS 进程和 tmux 会话。GNOME Terminal 仅作为 attach 客户端，并会清除 Snap Code 注入的库路径以避免 GLIBC 符号错误。用户桌面终端的实际 `2`/`4`/`6` 键盘交互仍待手工确认。新增 `TERMINAL_BACKEND`（设为 `direct` 可回退旧行为）(`fix/0715-build-auto-startup`).
> **2026-07-16 A1 nominal stance gate**: FixedStand 改为从 A1 单一足端名义站姿做 IK；删除 xacro 中每只 foot 的第二个非法父关节，消除 Gazebo 重复 foot/collision 和直腿站姿振荡。Trotting 继承当前高度/足端，0.75 s 平滑过渡，并在低线/角速度、直立、四足真实力反馈连续满足 0.20 s 后才允许 wave。最终零速稳定，`linear.x=0.3` 的 6.920 s 配对测试移动 1.891 m（0.273 m/s，方向正确）(`fix/0715-trotting-safety`).
>
> **2026-07-16 Gazebo sim-time control**: Wave、Estimator 和 Trotting 目标只在 `/clock` 前进时更新，全部积分使用实际仿真 `dt`；暂停、回退和前跳重置为 all-stance。RTF≈0.10 时入口门控实测 0.946 仿真秒/9.459 墙钟秒。Wave 运行期倾角和连续 0.080 仿真秒接触丢失均会锁存取消，不再继续 gait/IK (`fix/0715-trotting-safety`).

## Modules

| Module | Purpose | Status | Tests / Checks | Notes |
|--------|---------|--------|----------------|-------|
| `building_obstacles` | Scene generation, danger spawning, evaluation | stable | Smoke: `generate_competition_scene.py` + `evaluate_danger.py` CLI | 核心比赛逻辑; 无单元测试目录 |
| `building_generator_core` | Python core: layout, constraints, generation | stable | `nosetests` via catkin (3 tests) | 纯 Python 库，被 building_obstacles 依赖 |
| `building_generator_classic` | Gazebo export + door/elevator control runtime | stable | `nosetests` via catkin (2 tests) | 门/电梯控制的主要入口 |
| `building_generator_interfaces` | ROS message/service definitions | stable | 编译时类型检查 | `.msg` / `.srv` 定义，无运行时逻辑 |
| `unitree_guide` | A1 robot controller + RL locomotion | Earth flat-ground runtime gate blocked at G1/C0; Trotting validated earlier; G2-B currently inconclusive; RL partial; referee odom TF timestamp guard updated 2026-07-23 | Torch build + headless nominal FixedStand + low-RTF timing + pause/tilt/contact abort + zero/forward/invalid Twist; 2026-07-17 short-window truth profiles; G2 metric helpers unit-tested; 2026-07-19 G2 smoke trials invalid; G2-D1 Gate V validator semantics tests pass; 2026-07-20 flat-world XML/SDF/static checks pass; 2026-07-20 G0 runtime platform absence passes but G1/C0 fail; 2026-07-23 `state_from_gazebo.cpp` `g++ -fsyntax-only` PASS and scoped `tools/build_with_venv.sh` PASS | `earth.world` platform collisions were removed for benchmark use only and are absent at runtime. Controller, RL, URDF/xacro, spawn z, and competition scene generation remain unchanged. Current runtime artifact does not reproduce stable C0 FixedStand, so Earth E0/RL are gated off. Current G2 smoke evidence shows commands reach `resolved_vx`, but WAVE_ALL/gait do not start and a non-finite Trotting output (`q=0`) was captured. Gate V did not confirm a fall-validator false positive; Gate P must find the first Pre-WAVE blocker before any controller or physics fix. `state_from_gazebo` now owns `map -> odom` and `odom -> base` with one guarded callback stamp and `/gazebo/link_states` queue depth 1 |
| `Mid360_imu_sim` | Livox Mid-360 LiDAR plugin | stable | 编译检查 + 话题发布检查 | Gazebo plugin，依赖 Gazebo 开发头文件 |
| `simenv_fast_lio2_integration` | FAST-LIO2 bridge: adapter, config, launch, TF bridge | partial validation; odometry TF bridge default off 2026-07-23 | extrinsic checker, `roslaunch --files`, runtime `/Odometry` + `/cloud_registered`, controlled P0; 2026-07-23 XML parse + Stage 2 static contract tests PASS + scoped `tools/build_with_venv.sh` PASS | 2026-07-15: LiDAR→IMU external parameter corrected to direct `Ry(+45°), [0.2,0,0.08]`; bridge does not rotate world frames. Restart and moving regression still required. `laserMapping` is the default `camera_init -> body` dynamic TF owner; `odometry_tf_bridge` is opt-in to avoid duplicate authority |
| `docs/slam/` | FAST-LIO2 deployment guide & SLAM docs | new | markdown lint (manual) | 部署指南覆盖 15 个章节，含参数映射、编译环境、排错流程 |
| `uav_simulator` (5 sub-pkgs) | UAV local sensing, mapping, SO3 control | experimental | 编译检查 (含部分测试) | 与地面机器人大赛解耦，为独立实验功能 |
| `tools/` | 构建和仓库治理工具脚本 | new | `bash -n` 语法检查 + 权限检查 | `build_with_venv.sh` 提供 venv 版 catkin 构建 |
| SimEnv workspace | 整体集成 | stable | `catkin_make` + `./auto.sh` 冒烟测试 | 比赛入口 |

## Risks

- `uav_simulator` 子模块与比赛目标解耦，维护成本高且无明确 owner；建议移出到独立仓库或归档
- `Mid360_imu_sim` 仅提供编译检查，无运行时自动化验证；依赖硬件特定的点云格式
- 所有 Python 包的测试覆盖率偏低（仅 core 和 classic 有少量单元测试）
- A1 为保持稳定接触动力学已删除 foot 的第二父关节并使用 fixed-joint lump；旧 `/ground_truth/*_foot` P3D body 名可能失效，需另行用运动学发布器恢复，不能重新引入双父关节
