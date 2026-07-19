# Project State

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
- Date: 2026-07-19
- Branch: `test/0718-g2-trotting-motion-baseline` at `a2e00509`
- Project type: ROS1 Noetic + Gazebo Classic (robotics competition simulation)
- Current focus: G2-B Trotting motion baseline evidence with frozen controller,
  model, and Gazebo physics configuration

## Active Work
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
