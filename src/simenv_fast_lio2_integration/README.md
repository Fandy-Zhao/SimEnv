# simenv_fast_lio2_integration

SimEnv 与 FAST-LIO2 的集成桥接包。提供点云格式适配、配置文件和启动入口。

## 依赖

- ROS1 Noetic
- SimEnv 仿真环境 (提供 `/scan` 和 `/livox/imu`)
- **外部依赖**: [FAST_LIO](https://github.com/hku-mars/FAST_LIO) 必须单独克隆到同一 catkin workspace

## 安装 FAST_LIO

```bash
cd <catkin_ws>/src
git clone https://github.com/hku-mars/FAST_LIO.git
cd FAST_LIO
git submodule update --init
cd <catkin_ws>
catkin_make
```

> ⚠️ 不要将 FAST_LIO 源码放入 SimEnv 仓库。本包仅提供桥接配置和适配脚本。

## 文件结构

```
src/simenv_fast_lio2_integration/
├── package.xml
├── CMakeLists.txt
├── README.md                                  (本文件)
├── launch/
│   └── simenv_fast_lio2_mapping.launch        启动建图
├── config/
│   └── simenv_mid360.yaml                     FAST-LIO2 参数
└── scripts/
    ├── scan_to_pointcloud2.py                  /scan → /scan_pointcloud2 适配器
    └── pointcloud_fields_check.py              点云字段诊断工具
```

## 使用方法

### 1. 启动 SimEnv 仿真

```bash
cd /home/ros/Guoyulun/Competition/SimEnv
source /opt/ros/noetic/setup.bash
source ./devel/setup.bash

# 启动仿真 (FAST-LIO2 需要 sensor-frame 点云，建议关闭 odom 转换)
ENABLE_POINTCLOUD_CONVERTER=0 ./auto.sh
```

### 2. 启动 FAST-LIO2 建图

在另一个终端中:

```bash
source /opt/ros/noetic/setup.bash
source ./devel/setup.bash
roslaunch simenv_fast_lio2_integration simenv_fast_lio2_mapping.launch
```

### 3. 可选: 使用 auto.sh 集成启动

```bash
ENABLE_FAST_LIO2=1 ./auto.sh
```

此模式会自动在后台启动 scan_to_pointcloud2 适配器和 FAST-LIO2。

### 4. 诊断点云字段

```bash
rosrun simenv_fast_lio2_integration pointcloud_fields_check.py _topic:=/scan_pointcloud2
```

## FAST-LIO2 输入/输出契约

### 输入
| Topic | 类型 | 帧 | 说明 |
|-------|------|-----|------|
| `/scan_pointcloud2` | `sensor_msgs/PointCloud2` | `laser_livox` | LiDAR 点云 (x,y,z) |
| `/livox/imu` | `sensor_msgs/Imu` | `livox_imu_link` | LiDAR 内置 IMU (1000 Hz) |

### 输出 (FAST-LIO2 发布, 需运行时验证)
| Topic | 类型 | 说明 |
|-------|------|------|
| `/Odometry` | `nav_msgs/Odometry` | LiDAR-inertial 里程计 |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | 配准后点云 (world 帧) |
| `/Laser_map` | `sensor_msgs/PointCloud2` | 全局地图点云 |
| `/path` | `nav_msgs/Path` | 机器人路径 |
| `/aft_mapped_to_init` | `geometry_msgs/TransformStamped` | 地图到初始帧变换 |
| TF: `camera_init` → `body` | | SLAM 坐标系 |

> 注: 实际 topic 名称以 FAST-LIO2 launch/config 和运行结果为准。本文件列出的为常见默认值。

## 已知限制

1. **无 per-point 时间戳**: Gazebo 仿真 LiDAR 插件当前仅输出 x, y, z。FAST-LIO2 配置中 `timestamp_unit=0` 禁用了 per-point 运动补偿。建图精度在高速运动时会受影响。

2. **无扫描线/ring 信息**: `line=0` (硬编码)。FAST-LIO2 使用 `scan_line=4` 参数作为近似。

3. **无 intensity**: 反射率信息不可用。

4. **LiDAR 45° 俯仰安装**: A1 上 Livox Mid-360 以 45° pitch 安装。FAST-LIO2 外参已从 URDF 中提取，但可能需要实际运行调优。

## 后续改进

- 修改 Gazebo LiDAR 插件输出 per-point 时间戳和 ring 信息
- 添加自定义 LiDAR 消息类型支持
- 添加 RViz 可视化配置文件
