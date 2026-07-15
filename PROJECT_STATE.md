# Project State

## Snapshot
- Date: 2026-07-15
- Branch: fix/0715-build-auto-startup
- Project type: ROS1 Noetic + Gazebo Classic (robotics competition simulation)
- Current focus: validate user-terminal keyboard interaction through the
  tmux-backed controller session

## Active Work
- **FAST-LIO2 startup + TF bridge** (merged to `develop`): Controller starts before FAST-LIO2, auto-commanded FixedStand, IMU upright check. Duplicate adapter eliminated. Controller foreground mode preserved via `fg`. Camera-init TF bridge: `map→camera_init` is direct copy of `map→imu_link` (Ry(-45°) removed — was duplicate of FAST-LIO2 extrinsic_R). Rotation responsibility documented in YAML and ADR-0714.
- **FAST-LIO2 LiDAR–IMU axes corrected**: Source audit established that FAST-LIO2 applies `p_imu = R * p_lidar + T`. The SimEnv point source is local `laser_livox`, so the mapping config now uses the direct runtime TF `imu_link→laser_livox`: `Ry(+45°)`, `[0.2,0,0.08]`. `map→camera_init` remains a no-rotation copy of `map→imu_link`; restart FAST-LIO2 to load the new YAML.
- **Controller terminal keyboard**: `auto.sh` explicitly binds background `junior_ctrl` stdin to `/dev/tty` and then waits for it in foreground mode. This avoids non-interactive Bash replacing stdin with `/dev/null` during FAST-LIO2 startup.
- **FSM nullptr segfault fix**: Guarded keyboard/RPC/checkChange paths against TROTTING/RL states when Torch is disabled. Default build (Torch OFF) no longer crashes on `4`/`6` key press or rostopic. Added FSM-level nullptr safety net.
- **Dedicated controller + rviz terminals**: `auto.sh` defaults to named tmux sessions for `junior_ctrl` and rviz, so their commands survive GUI-terminal failure and can be reattached from any terminal. GNOME Terminal only attaches to those sessions. Controlled startup verified both sessions alive, `/Odometry` at ~10 Hz, and Ctrl-C cleanup. The launcher sanitises only its own Snap Code-contaminated environment while preserving ROS inside each session. Removed `CONTROLLER_FOREGROUND`; added `ENABLE_RVIZ` and `TERMINAL_BACKEND`.
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
- GitHub 远程仓库可能为空或已有历史，首次 push 前需确认目标分支
- 随机生成的建筑布局可能在某些参数组合下产生不可达房间或源重叠
- Gazebo Classic 已停止维护，长期可能需要迁移到 Ignition/Gazebo Fortress
- FAST-LIO2: SimEnv adapter 输出 `PointCloud2`，必须使用 `preprocess.lidar_type=4`；其 MARSIM 分支的 `last_lidar_end_time_` 未初始化缺陷已在外部源码修复并选择性重编译，仍需完成完整 60 s 仿真时间 P0
- FAST-LIO2 axis correction has offline and live-TF evidence, but its moving mapping regression remains pending a clean mapper restart.
- 当前 3 层场景实时因子很低（60 s 墙钟仅 4.3 s ROS 时间），完整 P0 需要约 10--15 分钟墙钟；`junior_ctrl` 因 Torch ABI 隔离未编译 trot/RL 状态，P1 真实步态测试需重新启用其依赖
- IDE 的 Miniconda Python 3.13 与 Noetic xacro 不兼容；ROS/Gazebo runtime 应使用系统 Python 3.10

## Validation Status
- Build: catkin_make 编译通过（最近提交已验证）
- Unit tests: `building_generator_core/test/` (3 tests), `building_generator_classic/test/` (2 tests)
- Remote config: `origin` 保持 Gitee, `github` 新增成功
- Governance: 骨架完整, 分支规则已更新

## Next Steps
- 用户确认后执行首次 push: `git push -u github develop`
- 补充 CI/自动化测试流程
- 评估将 UAV simulator 子模块独立或移除
- 完成固定站立的 60 s **ROS 仿真时间**静止测试
- 用户授权后重新启用 `junior_ctrl` Torch 步态策略，或明确批准标记为 SLAM-only 的运动替代方案；随后依序执行 5 m 直线与矩形短闭环
