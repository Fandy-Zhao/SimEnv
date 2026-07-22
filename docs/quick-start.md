# 快速启动

本文面向第一次运行比赛仿真环境的选手。默认工作目录为：

```bash
cd /home/ros/Guoyulun/Competition/SimEnv
```

## 运行要求

- Ubuntu 20.04 或兼容环境
- ROS Noetic，建议安装 `ros-noetic-desktop-full`
- Gazebo Classic
- Python >= 3.8
- `python3-yaml`
- `numpy`，用于评估脚本
- CUDA >= 11.7
- libtorch C++ 版本，用于 Unitree A1 控制器

libtorch 和 CUDA 路径在 `src/unitree_guide/unitree_guide/unitree_guide/CMakeLists.txt` 中配置。当前工程默认指向 `/home/ros/Guoyulun/Download/libtorch` 和 `/usr/local/cuda/bin/nvcc`。如部署路径不同，需要按实际机器调整。

## 编译

```bash
source /opt/ros/noetic/setup.bash
./tools/build_with_venv.sh
source ./devel/setup.bash
```

`build_with_venv.sh` 默认只编译 `auto.sh` 的运行时依赖（FAST-LIO2、
Unitree 控制器及 Gazebo 插件），避免工作区内的可选第三方包阻塞启动构建。
如确实要编译全部已发现包，可显式运行
`SIMENV_CATKIN_WHITELIST="" ./tools/build_with_venv.sh`；该模式要求所有本地
额外源码的依赖均已满足。

## 一键启动

```bash
./auto.sh
```

`auto.sh` 会执行以下流程：

1. 清理旧的 Gazebo、roslaunch、`junior_ctrl`、门/电梯控制服务和可选虚拟手柄进程。
2. 生成随机楼栋、危险源、干扰源和真值文件。
3. 写入 `generated_building/competition_scene.world`。
4. 启动 Gazebo、Unitree A1 模型、传感器、状态话题和控制器接口。
5. 启动 `building_generator_classic` 门/电梯控制服务。
6. 启动 `devel/lib/unitree_guide/junior_ctrl`。

Livox 点云插件启动时会读取扫描模式 CSV 文件，启动后前十几秒可能出现 `rostopic hz /scan` 暂时显示 `no new messages`。请等待 `auto.sh` 完成并再等待数秒后检查传感器话题。

## 常用启动方式

固定随机种子，便于复现实验：

```bash
SEED=77 ./auto.sh
```

无 GUI 启动，适合远程服务器或性能较弱机器：

```bash
GUI=false ./auto.sh
```

调大场景规模：

```bash
SEED=20260507 FLOOR_COUNT=4 ROOMS_PER_FLOOR=5 DANGER_COUNT=5 DISTRACTOR_COUNT=8 ./auto.sh
```

不启动控制器，只启动环境：

```bash
START_CONTROLLER=0 ./auto.sh
```

启动 Earth 平地模式并使用推荐的 plane RL policy：

```bash
RL_POLICY_PATH=/home/zzf/search_ws/SimEnv/src/unitree_guide/logs/policy_act_inference_plane.pt \
WORLD_MODE=earth \
PHYSICS_PROFILE=normal \
GUI=False \
./auto.sh
```

`WORLD_MODE=earth` 会跳过比赛场景随机生成，使用 `earth.world` 平地世界，
并默认关闭楼栋控制、FAST-LIO2、RViz、传感器数据、裁判里程计和 ground truth
话题，适合做 Unitree A1/RL 控制链路的短回归。

## 启动参数

| 环境变量 | 默认值 | 说明 |
|----------|--------|------|
| `WORLD_MODE` | `competition` | 启动模式。`competition` 生成比赛楼栋；`earth` 使用平地 `earth.world` |
| `PHYSICS_PROFILE` | `normal` | Gazebo 物理预设。支持 `normal`、`fidelity`；competition 未显式配置时保留 legacy `0.002/500/40` |
| `SEED` | 空 | 场景随机种子。为空时自动生成随机种子并写入 manifest |
| `FLOOR_COUNT` | `1` | 楼层数，支持单值 |
| `ROOMS_PER_FLOOR` | `4` | 每层房间数，支持单值 |
| `BUILDING_WIDTH` | `20.0` | 楼栋宽度，单位 m |
| `BUILDING_LENGTH` | `36.0` | 楼栋长度，单位 m |
| `DANGER_COUNT` | `3:6` | 危险源数量，支持 `min:max` |
| `DISTRACTOR_COUNT` | `4:8` | 干扰源数量，支持 `min:max` |
| `GUI` | `false` | 是否启动 Gazebo GUI |
| `PAUSED` | `true` | Gazebo 启动后是否暂停 |
| `AUTO_UNPAUSE` | `1` | 是否在启动后自动调用 `/gazebo/unpause_physics` |
| `AUTO_UNPAUSE_DELAY` | `6` | 自动解除暂停前等待时间，单位 s |
| `START_CONTROLLER` | `1` | 是否在独立终端启动 `junior_ctrl` |
| `ENABLE_RVIZ` | `1` | 是否在独立终端启动 rviz |
| `START_BUILDING_CONTROL` | `1` | 是否启动楼栋门/电梯控制服务 |
| `UNITREE_CTRL_DT` | `0.002` | `junior_ctrl` 控制周期，单位 s |
| `RL_POLICY_PATH` | 空 | RL TorchScript policy 覆盖路径；传入 `junior_ctrl`，优先级低于 ROS 参数 `/rl_policy_path` |
| `START_VIRTUAL_JOY` | `0` | 是否启动虚拟手柄，通常需要 `uinput` 权限 |
| `ROBOT_X` | `0.0` | 机器人出生点 x |
| `ROBOT_Y` | `2.3` | 机器人出生点 y |
| `ROBOT_Z` | `0.6` | 机器人出生点 z |
| `ROBOT_YAW` | `1.5708` | 机器人出生点 yaw |
| `ENABLE_FAST_LIO2` | `1` | 是否启动 FAST-LIO2 建图（需编译 FAST_LIO）；`WORLD_MODE=earth` 默认改为 `0` |
| `ENABLE_POINTCLOUD_CONVERTER` | `1` | 是否启动 odom 系点云转换（FAST-LIO2 建图时建议设为 `0`） |
| `ENABLE_SENSORS` | `1` | 是否启用传感器数据（LiDAR、IMU、RealSense） |
| `ENABLE_SENSOR_DATA` | 同 `ENABLE_SENSORS` | 是否启用传感器数据；优先级高于兼容变量 `ENABLE_SENSORS` |
| `ENABLE_REFEREE_ODOM` | `1` | 是否发布裁判真值里程计 |
| `ENABLE_GROUND_TRUTH` | `1` | 是否发布 ground truth 话题 |
| `ENABLE_FOOT_FORCE_VISUAL` | `0` | 是否启用足端接触力可视化 |
| `ENABLE_JOY_NODE` | `0` | 是否启动 joystick ROS 节点 |
| `POINTCLOUD_USE_GROUND_TRUTH_ODOM` | `1` | 点云转换节点是否使用 ground-truth odom |
| `WRITE_GENERATED_TRUTH_COPY` | `1` | 是否写 `danger_truth.json` 到 `generated_building/` |
| `TERMINAL_BACKEND` | `tmux` | 独立控制器/RViz 终端后端。可设为 `direct` 使用旧式终端启动 |
| `TMUX_SESSION_PREFIX` | `simenv` | tmux 会话名前缀，默认会话为 `simenv-junior_ctrl`、`simenv-rviz` |
| `SKIP_GLOBAL_PROCESS_CLEANUP` | `0` | 是否跳过启动前全局清理旧 Gazebo/ROS/controller 进程 |
| `GAZEBO_PHYSICS_MAX_STEP_SIZE` | 随 profile | 覆盖 Gazebo `max_step_size` |
| `GAZEBO_PHYSICS_REAL_TIME_UPDATE_RATE` | 随 profile | 覆盖 Gazebo `real_time_update_rate` |
| `GAZEBO_PHYSICS_ODE_ITERS` | 随 profile | 覆盖 ODE solver iteration 数 |
| `GAZEBO_PHYSICS_CONTACT_MAX_CORRECTING_VEL` | `5.0` | 覆盖 Gazebo contact max correcting velocity |

`WORLD_MODE=earth` 会覆盖一组默认值：`START_BUILDING_CONTROL=0`、
`ENABLE_FAST_LIO2=0`、`ENABLE_RVIZ=0`、`ENABLE_SENSOR_DATA=0`、
`ENABLE_POINTCLOUD_CONVERTER=0`、`ENABLE_REFEREE_ODOM=0`、
`ENABLE_GROUND_TRUTH=0`、`WRITE_GENERATED_TRUTH_COPY=0`，并将机器人出生姿态
设为 `x=0.0 y=0.0 z=0.6 yaw=0.0`。显式设置的环境变量仍会覆盖这些 earth
模式默认值。

RL policy 的运行时选择优先级为：

1. ROS 参数 `/rl_policy_path`
2. 环境变量 `RL_POLICY_PATH`
3. 控制器默认 policy：`src/unitree_guide/logs/policy_act_inference_stair.pt`

Earth 平地推荐使用：

```bash
RL_POLICY_PATH=/home/zzf/search_ws/SimEnv/src/unitree_guide/logs/policy_act_inference_plane.pt
```

启动摘要会打印 `RL policy override`，`junior_ctrl` 日志还会打印实际加载的
policy path、realpath、SHA256 和 `load success`。可用以下命令确认：

```bash
tmux capture-pane -J -t simenv-junior_ctrl -p -S -300 | rg "RL-POLICY|SHA256|load success"
```

性能较弱时建议优先使用：

```bash
GUI=false ./auto.sh
```

如只需要测试感知链路，可暂时不启动控制器：

```bash
START_CONTROLLER=0 ./auto.sh
```

启动 FAST-LIO2 LiDAR-Inertial SLAM 建图（需要先编译 FAST_LIO）：

```bash
ENABLE_FAST_LIO2=1 GUI=false ./auto.sh
```

`auto.sh` 会自动在独立终端中启动 `junior_ctrl` 和 rviz。控制器终端支持键盘输入：
`2` 站立、`4` 小跑、`6` RL（`4`/`6` 需要 Torch 构建）。也可通过
`/fsm/state_cmd` 与 `/cmd_vel` 以编程方式控制。

从 Snap 版 VS Code 的集成终端启动时，脚本会使用干净的桌面环境直接打开
GNOME Terminal，避免 Snap 动态库污染阻止控制器和 RViz 窗口创建。
如果控制器或 RViz 进程退出，对应终端会保留诊断信息；输入 `exit` 才会关闭。
这也适用于 RViz 启动失败的情况。

默认使用 tmux 托管两个会话，因此图形终端闪退不会停止控制器或 RViz：

```bash
tmux attach-session -t simenv-junior_ctrl
tmux attach-session -t simenv-rviz
```

如需旧的直接终端方式，可设置 `TERMINAL_BACKEND=direct ./auto.sh`。

若 `auto.sh` 在场景生成前报告 `junior_ctrl is not built`，请先完成
Unitree 控制器构建。该预检会保护当前生成场景，避免控制器缺失时启动一半
Gazebo 后终端因等待失败而返回。

> FAST-LIO2 部署详情参见 [FAST-LIO2 部署指南](slam/fast_lio2_deployment_guide.md) 和 [集成包 README](../src/simenv_fast_lio2_integration/README.md)。
>
> `ENABLE_POINTCLOUD_CONVERTER=0` 可关闭 odom 系点云转换，使 FAST-LIO2 适配器获得 sensor-frame 原始点云。

## 单独生成场景

只生成比赛场景，不启动 Gazebo：

```bash
source ./devel/setup.bash
rosrun building_obstacles generate_competition_scene.py \
  --seed 77 \
  --floor-count 3 \
  --rooms-per-floor 4 \
  --width 20 \
  --length 36 \
  --danger-count 4 \
  --distractor-count 6 \
  --output-dir ./generated_building \
  --results-dir ./results
```

兼容旧命令：

```bash
rosrun building_obstacles generate_multi_floor_building.py ./generated_building 3 4
```

旧入口会转调新的比赛场景生成器。

## 默认场景规模

默认楼栋尺寸按 Unitree A1 室内探索做了收敛：走廊约 2.2 m，单层默认 4 个房间，建筑占地约 20 m x 36 m。该尺寸保留进门、转向和传感器观测余量，同时避免探索时间主要消耗在长距离行走上。需要提高难度时，可逐步增大 `BUILDING_WIDTH`、`BUILDING_LENGTH` 和 `ROOMS_PER_FLOOR`。
