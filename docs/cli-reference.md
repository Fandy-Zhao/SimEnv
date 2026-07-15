# 命令速查

本文汇总 SimEnv 仿真环境中所有常用命令行操作，包括编译、启动、建图、诊断和评估。适合已读过 [快速启动](quick-start.md) 的选手日常参考。

---

## 1. 编译

```bash
cd ~/SimEnv
source /opt/ros/noetic/setup.bash
catkin_make -j
source ./devel/setup.bash
```

使用 venv 构建（推荐，含 CUDA/torch 支持）：

```bash
source .venv/bin/activate
./tools/build_with_venv.sh
source ./devel/setup.bash
```

---

## 2. 一键启动

### 基本启动

```bash
./auto.sh
```

### 常用启动模式

```bash
GUI=false ./auto.sh                                      # 无 GUI，适合服务器
START_CONTROLLER=0 ./auto.sh                             # 只启动环境，不启控制器
SEED=77 ./auto.sh                                        # 固定随机种子
ENABLE_FAST_LIO2=1 GUI=false ./auto.sh                    # 启动 FAST-LIO2 建图
ENABLE_POINTCLOUD_CONVERTER=0 GUI=false ./auto.sh         # 关闭 odom 系点云（FAST-LIO2 需要）
```

### 调大场景规模

```bash
SEED=20260507 FLOOR_COUNT=4 ROOMS_PER_FLOOR=5 \
  DANGER_COUNT=5 DISTRACTOR_COUNT=8 ./auto.sh
```

### 完整环境变量参考

#### 场景参数

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `SEED` | 空（随机） | 场景随机种子 |
| `FLOOR_COUNT` | `3` | 楼层数 |
| `ROOMS_PER_FLOOR` | `4` | 每层房间数 |
| `BUILDING_WIDTH` | `20.0` | 楼栋宽度（m） |
| `BUILDING_LENGTH` | `36.0` | 楼栋长度（m） |
| `DANGER_COUNT` | `3:6` | 危险源数量，`min:max` |
| `DISTRACTOR_COUNT` | `4:8` | 干扰源数量，`min:max` |

#### 仿真控制

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `GUI` | `true` | 是否启动 Gazebo GUI |
| `PAUSED` | `true` | Gazebo 启动后是否暂停 |
| `AUTO_UNPAUSE` | `1` | 是否自动取消暂停 |
| `AUTO_UNPAUSE_DELAY` | `6` | 自动取消暂停延迟（秒） |

#### 控制器

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `START_CONTROLLER` | `1` | 是否启动 `junior_ctrl`（独立终端） |
| `ENABLE_RVIZ` | `1` | 是否在独立终端启动 rviz（需要 FAST-LIO2 rviz 配置文件） |
| `UNITREE_CTRL_DT` | `0.004` | 控制周期（秒） |
| `START_VIRTUAL_JOY` | `0` | 是否启动虚拟手柄 |

#### 机器人出生点

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `ROBOT_X` | `0.0` | 出生点 x（m） |
| `ROBOT_Y` | `2.3` | 出生点 y（m） |
| `ROBOT_Z` | `0.6` | 出生点 z（m） |
| `ROBOT_YAW` | `1.5708` | 出生点 yaw（rad） |

#### 传感器与建图

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `ENABLE_SENSORS` | `1` | 启用传感器数据 |
| `ENABLE_POINTCLOUD_CONVERTER` | `1` | 启用 odom 系点云转换 |
| `ENABLE_REFEREE_ODOM` | `1` | 启用裁判真值里程计 |
| `ENABLE_GROUND_TRUTH` | `1` | 启用 ground truth 话题 |
| `ENABLE_FOOT_FORCE_VISUAL` | `0` | 启用足端力可视化 |
| `ENABLE_JOY_NODE` | `0` | 启用手柄节点 |
| `WRITE_GENERATED_TRUTH_COPY` | `1` | 写真值副本到 `generated_building/` |

#### FAST-LIO2

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `ENABLE_FAST_LIO2` | `0` | 是否启动 FAST-LIO2 建图 |
| `POINTCLOUD_USE_GROUND_TRUTH_ODOM` | `1` | 点云转换是否使用真值里程计 |

#### Gazebo 物理

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `GAZEBO_PHYSICS_MAX_STEP_SIZE` | `0.002` | 物理最大步长（秒） |
| `GAZEBO_PHYSICS_REAL_TIME_UPDATE_RATE` | `500` | 物理更新频率（Hz） |
| `GAZEBO_PHYSICS_ODE_ITERS` | `40` | ODE 求解器迭代次数 |
| `GAZEBO_PHYSICS_CONTACT_MAX_CORRECTING_VEL` | `5.0` | 接触修正最大速度（m/s） |

---

## 3. FAST-LIO2 建图

### 前置条件

```bash
# 确保 FAST_LIO 已克隆到 src/
ls src/FAST_LIO/CMakeLists.txt
# 确保已编译
source ./devel/setup.bash
rospack find fast_lio
```

### 集成启动（推荐）

```bash
ENABLE_FAST_LIO2=1 GUI=false ./auto.sh
```

### 分步启动（调试）

终端 1 — 启动仿真：

```bash
ENABLE_POINTCLOUD_CONVERTER=0 GUI=false START_CONTROLLER=0 ./auto.sh
```

终端 2 — 启动 FAST-LIO2：

```bash
source ./devel/setup.bash
roslaunch simenv_fast_lio2_integration simenv_fast_lio2_mapping.launch
```

### 验证 FAST-LIO2 输出

```bash
rostopic hz /Odometry
rostopic hz /cloud_registered
rostopic echo -n 1 /Odometry | grep -E "position|orientation"
rosrun simenv_fast_lio2_integration pointcloud_fields_check.py _topic:=/scan_pointcloud2
```

### FAST-LIO2 常用话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/Odometry` | `nav_msgs/Odometry` | LiDAR-inertial 里程计 |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | 配准后点云 |
| `/Laser_map` | `sensor_msgs/PointCloud2` | 全局地图 |
| `/path` | `nav_msgs/Path` | 机器人轨迹 |

---

## 4. 单独生成场景

### 比赛场景生成

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

### 兼容旧命令

```bash
rosrun building_obstacles generate_multi_floor_building.py ./generated_building 3 4
```

### CLI 生成器（精细控制）

```bash
rosrun building_generator_core building_generator_cli generate \
  --seed 41 \
  --floor-count 2:4 \
  --rooms-per-floor 4:8 \
  --width 30 \
  --length 58 \
  --target gazebo_classic \
  --output-dir /tmp/building_gen_case
```

### 批量生成

```bash
rosrun building_generator_core building_generator_cli batch \
  --seed-list 11,12,13 \
  --floor-count 2:5 \
  --rooms-per-floor 4:10 \
  --width 36 \
  --length 72 \
  --target gazebo_classic \
  --output-dir /tmp/building_gen_batch
```

---

## 5. 门与电梯控制

### 手动启动控制服务

```bash
source ./devel/setup.bash
rosrun building_generator_classic building_generator_classic_control \
  --door-config ./generated_building/door_config.yaml \
  --elevator-config ./generated_building/elevator_config.yaml
```

### 服务调用

```bash
# 呼叫电梯到 3 楼（楼层索引 2）
rosservice call /call_elevator "{elevator_id: 'elevator_main', target_floor: 2, open_doors: true}"

# 关闭主入口门
rosservice call /set_door_state "{door_id: 'main_entrance', open: false}"

# 打开主入口门
rosservice call /set_door_state "{door_id: 'main_entrance', open: true}"
```

楼层索引从 `0` 开始：`0`=1楼, `1`=2楼, `2`=3楼。

---

## 6. 控制器操作

在 `junior_ctrl` 的独立终端中输入（`auto.sh` 自动打开）：

| 按键 | 功能 |
|------|------|
| `2` | 站立 |
| `6` | 切换到 RL 模式（接收 `/cmd_vel`） |

RL 模式下发布速度指令：

```bash
# 发布 Twist 消息（线速度 + 角速度）
rostopic pub -1 /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

## 7. 评估

```bash
python3 ./src/building_obstacles/scripts/evaluate_danger.py \
  --truth-file ./results/danger_truth.json \
  --detected-file ./results/detected_danger.json \
  --output-file ./results/evaluation_result.json
```

可选参数：

```bash
--use-scene-ratio                       # 使用场景尺度百分比阈值
--match-threshold 1.5                   # 自定义匹配阈值（m），默认 1.0
```

---

## 8. 诊断与调试

### 进程管理

```bash
# 查看 ROS 节点
rosnode list
rosnode info /laserMapping

# 查看话题列表
rostopic list
rostopic list | grep -E "scan|imu|odom|cloud|cmd"

# 检查话题频率
rostopic hz /scan
rostopic hz /livox/imu
rostopic hz /Odometry

# 检查话题消息
rostopic echo -n 1 /scan/header
rostopic echo -n 1 /Odometry
rostopic echo -n 1 /clock

# 查看话题类型和发布者
rostopic info /cmd_vel
rostopic info /Odometry
```

### TF 诊断

```bash
# 查看 TF 树
rosrun tf2_tools view_frames.py

# 查看两个坐标系间变换
rosrun tf tf_echo base laser_livox
rosrun tf tf_echo base livox_imu_link

# 查看静态 TF
rostopic echo -n 1 /tf_static
```

### 参数检查

```bash
# 仿真时间
rosparam get /use_sim_time

# FAST-LIO2 参数
rosparam get /common/lid_topic
rosparam get /common/imu_topic
rosparam get /preprocess/lidar_type

# 列出某节点下所有参数
rosparam list /laserMapping
```

### 服务检查

```bash
# 列出所有服务
rosservice list

# Gazebo 物理控制
rosservice call /gazebo/pause_physics
rosservice call /gazebo/unpause_physics
```

### 日志查看

```bash
tail -f logs/competition_gazebo.log
tail -f logs/building_control.log
tail -f logs/fast_lio2.log        # FAST-LIO2 启用时
tail -f logs/junior_ctrl.log      # 后台控制器启用时
```

### 进程清理

```bash
pkill -f "gzserver|gzclient|gazebo"
pkill -f "junior_ctrl"
pkill -f "fastlio_mapping"
pkill -f "building_generator_classic_control"
```

---

## 9. 文件路径速查

| 路径 | 说明 |
|------|------|
| `generated_building/competition_scene.world` | 当前 Gazebo 世界 |
| `generated_building/team_scene_info.json` | 公开场景信息（算法可读） |
| `generated_building/scene_manifest.json` | 场景 manifest |
| `generated_building/door_config.yaml` | 动态门配置 |
| `generated_building/elevator_config.yaml` | 电梯配置 |
| `results/danger_truth.json` | 真值文件（算法不可读） |
| `results/detected_danger.json` | 算法输出文件 |
| `results/evaluation_result.json` | 评估结果 |
| `logs/competition_gazebo.log` | Gazebo/launch 日志 |
| `logs/building_control.log` | 门/电梯控制日志 |
| `logs/fast_lio2.log` | FAST-LIO2 日志 |
| `devel/lib/fast_lio/fastlio_mapping` | FAST-LIO2 可执行文件 |

---

## 10. rosbag 录制

```bash
rosbag record \
  /clock /tf /tf_static \
  /Odometry /cloud_registered \
  /scan /livox/imu \
  /cmd_vel \
  /way_point \
  /gazebo/model_states \
  -O experiment_001.bag
```
