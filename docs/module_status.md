# Module Status

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

## Modules

| Module | Purpose | Status | Tests / Checks | Notes |
|--------|---------|--------|----------------|-------|
| `building_obstacles` | Scene generation, danger spawning, evaluation | stable | Smoke: `generate_competition_scene.py` + `evaluate_danger.py` CLI | 核心比赛逻辑; 无单元测试目录 |
| `building_generator_core` | Python core: layout, constraints, generation | stable | `nosetests` via catkin (3 tests) | 纯 Python 库，被 building_obstacles 依赖 |
| `building_generator_classic` | Gazebo export + door/elevator control runtime | stable | `nosetests` via catkin (2 tests) | 门/电梯控制的主要入口 |
| `building_generator_interfaces` | ROS message/service definitions | stable | 编译时类型检查 | `.msg` / `.srv` 定义，无运行时逻辑 |
| `unitree_guide` | A1 robot controller + RL locomotion | stable | 编译检查 + 启动冒烟测试 | `junior_ctrl` 为核心二进制 |
| `Mid360_imu_sim` | Livox Mid-360 LiDAR plugin | stable | 编译检查 + 话题发布检查 | Gazebo plugin，依赖 Gazebo 开发头文件 |
| `simenv_fast_lio2_integration` | FAST-LIO2 bridge: adapter, config, launch, TF bridge | partial validation | extrinsic checker, `roslaunch --files`, runtime `/Odometry` + `/cloud_registered`, controlled P0 | 2026-07-15: LiDAR→IMU external parameter corrected to direct `Ry(+45°), [0.2,0,0.08]`; bridge does not rotate world frames. Restart and moving regression still required |
| `docs/slam/` | FAST-LIO2 deployment guide & SLAM docs | new | markdown lint (manual) | 部署指南覆盖 15 个章节，含参数映射、编译环境、排错流程 |
| `uav_simulator` (5 sub-pkgs) | UAV local sensing, mapping, SO3 control | experimental | 编译检查 (含部分测试) | 与地面机器人大赛解耦，为独立实验功能 |
| `tools/` | 构建和仓库治理工具脚本 | new | `bash -n` 语法检查 + 权限检查 | `build_with_venv.sh` 提供 venv 版 catkin 构建 |
| SimEnv workspace | 整体集成 | stable | `catkin_make` + `./auto.sh` 冒烟测试 | 比赛入口 |

## Risks

- `uav_simulator` 子模块与比赛目标解耦，维护成本高且无明确 owner；建议移出到独立仓库或归档
- `Mid360_imu_sim` 仅提供编译检查，无运行时自动化验证；依赖硬件特定的点云格式
- 所有 Python 包的测试覆盖率偏低（仅 core 和 classic 有少量单元测试）
