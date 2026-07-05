# FAST-LIO2 Deployment Guide for SimEnv

## 1. Purpose

本文档的目标是在 SimEnv 仿真环境中部署 FAST-LIO2 进行 LiDAR-Inertial SLAM 建图。

**范围**：
- 只关注 SLAM 建图（mapping only）
- 说明传感器配置、参数映射、编译环境、运行验证和排错流程

**不包含**：
- 导航探索算法
- 控制器修复（unitree_guide）
- 危险源识别
- UAV simulator

## 2. Repository Layout

SimEnv 仓库本身就是 catkin workspace 根目录：

```text
SimEnv/                          # ← catkin workspace root
├── .catkin_workspace
├── src/
│   ├── FAST_LIO/                # 外部源码，需 git clone
│   ├── livox_ros_driver/        # 外部源码，需 git clone
│   ├── simenv_fast_lio2_integration/  # SimEnv 适配包（本文档所属包）
│   ├── building_obstacles/
│   ├── building_generator_core/
│   ├── building_generator_classic/
│   ├── building_generator_interfaces/
│   ├── unitree_guide/
│   ├── uav_simulator/
│   ├── Mid360_imu_sim/
│   └── move_base_msgs/
├── build/
├── devel/
├── tools/
│   └── build_with_venv.sh       # 推荐构建脚本
└── .venv/                       # Python venv
```

**关键说明**：
- SimEnv 是 catkin workspace 根目录，不是 `SimEnv/catkin_ws/`。
- FAST_LIO 应放在 `SimEnv/src/FAST_LIO`，由用户自行 `git clone`。
- livox_ros_driver (ROS1 版本) 应放在 `SimEnv/src/livox_ros_driver`，或通过外部 workspace source。
- `simenv_fast_lio2_integration` 是适配包，**不包含** FAST_LIO 源码（不 vendor）。仅提供 SimEnv 适配配置、launch、adapter、检查脚本和文档。

## 3. Required Packages

### 3.1 基础依赖

| 依赖 | 版本/说明 |
|------|----------|
| ROS | Noetic (Ubuntu 20.04) |
| catkin | catkin_make 或 catkin tools |
| PCL | ROS Noetic 自带 |
| Eigen | ROS Noetic 自带 |
| Python venv | 推荐使用 `.venv` |

### 3.2 外部 ROS Package

| Package | 安装位置 | 获取方式 |
|---------|---------|---------|
| FAST_LIO | `SimEnv/src/FAST_LIO` | `git clone https://github.com/hku-mars/FAST_LIO.git` |
| livox_ros_driver (ROS1) | `SimEnv/src/livox_ros_driver` | `git clone https://github.com/Livox-SDK/livox_ros_driver.git` |

FAST_LIO 依赖 `livox_ros_driver`（在其 CMakeLists.txt 中声明），因此两者都需要存在于 workspace 中。

### 3.3 可选：CUDA + torch（仅 unitree_guide 需要）

如果参与编译的包中包含 `unitree_guide`（默认 catkin_make 会编译所有包）：

| 组件 | 版本/说明 |
|------|----------|
| CUDA Toolkit | 11.8 |
| gcc/g++ | 11 (CUDA 11.8 要求 host compiler ≤ gcc-12) |
| torch (pip) | 2.0.1+cu118 |
| torchvision | 0.15.2+cu118 |

> **如果只是验证 FAST-LIO2 mapping**，可以临时跳过 `unitree_guide` 和 `uav_simulator`（通过 `CATKIN_IGNORE` 或选择性编译），但需记录影响：跳过 unitree_guide 将无法使用 `junior_ctrl` 控制器驱动机器人，只能靠外力或手动控制移动。

## 4. FAST-LIO2 Input Mapping

以下表格列出 FAST-LIO2 配置参数与 SimEnv 中实际传感器/话题的对应关系：

| FAST-LIO2 参数 | SimEnv 对应项 | 当前值 | 来源文件 | 风险 |
|---------------|-------------|--------|---------|------|
| `lid_topic` | `/scan_pointcloud2` | `"/scan_pointcloud2"` | `simenv_mid360.yaml` | adapter 必须运行 |
| `imu_topic` | `/livox/imu` | `"/livox/imu"` | `simenv_mid360.yaml` | 需确认 IMU 坐标系方向 |
| `lidar_type` | Livox Mid-360 仿真 | `1` (Livox serials) | `simenv_mid360.yaml` | 正确 |
| `scan_line` | Mid-360 = 4 线 | `4` | `simenv_mid360.yaml` | 仿真无 ring 信息，为近似值 |
| `timestamp_unit` | 仿真无 per-point time | `0.0` (禁用) | `simenv_mid360.yaml` | **关键风险**：无运动补偿 |
| `time_scale` | 同上 | `0.0` | `simenv_mid360.yaml` | 同上 |
| `blind` | LiDAR 盲区 | `0.5` m | `simenv_mid360.yaml` | 匹配 Gazebo min_range=0.1 |
| `extrinsic_T` | LiDAR → IMU 平移 | `[-0.011, -0.02329, 0.04412]` | `simenv_mid360.yaml`, `robot.xacro` | 需运行时验证 |
| `extrinsic_R` | LiDAR → IMU 旋转 | `[1,0,0; 0,1,0; 0,0,1]` | `simenv_mid360.yaml`, `robot.xacro` | 单位矩阵，需确认方向 |
| `extrinsic_est_en` | 在线外参估计 | `false` | 未显式设置（默认 false） | 保守策略 |
| `acc_cov` | IMU 加速度噪声 | `0.1` | `simenv_mid360.yaml` | 仿真 IMU 噪声为 0，需调优 |
| `gyr_cov` | IMU 陀螺仪噪声 | `0.1` | `simenv_mid360.yaml` | 同上 |
| `det_range` | LiDAR 最大探测距离 | `40.0` m | `simenv_mid360.yaml` | 匹配 Gazebo max_range=40 |
| `fov_degree` | LiDAR 水平 FOV | `180` | `simenv_mid360.yaml` | 实际为 360°，180 为前向限制 |
| `filter_size_surf` | 表面点下采样 | `0.5` m | `simenv_mid360.yaml` | 默认值，需实验调优 |
| `filter_size_map` | 地图点下采样 | `0.5` m | `simenv_mid360.yaml` | 默认值，需实验调优 |
| `cube_side_length` | 地图立方体边长 | `1000.0` m | `simenv_mid360.yaml` | 大型场景可用 |
| `publish.path_en` | 发布路径 | `true` | `simenv_mid360.yaml` | |
| `publish.scan_publish_en` | 发布配准扫描 | `true` | `simenv_mid360.yaml` | |
| `publish.dense_publish_en` | 发布稠密点云 | `true` | `simenv_mid360.yaml` | |
| `publish.scan_bodyframe_pub_en` | 发布 body 帧扫描 | `true` | `simenv_mid360.yaml` | |
| `pcd_save.pcd_save_en` | 保存 PCD | `true` | `simenv_mid360.yaml` | |
| `pcd_save.interval` | 保存间隔 | `-1` (结束时保存) | `simenv_mid360.yaml` | 大量点云可能内存溢出 |
| `map_file_path` | 地图保存路径 | 待确认 (使用默认 PCD 目录) | — | 需确认磁盘空间 |

> **注**: 上述 "待确认" 字段表示在 `simenv_mid360.yaml` 中未显式配置，沿用 FAST_LIO 内部默认值。

## 5. SimEnv Sensor Topics

### 5.1 LiDAR 话题

| Topic | Type | Frame | Frequency | Used by FAST-LIO2 | Notes |
|-------|------|-------|-----------|-------------------|-------|
| `/scan` | `sensor_msgs/PointCloud` | 10 Hz | `laser_livox` | 间接（经 adapter） | Gazebo 插件直接发布，~24000 点/帧 |
| `/scan_pointcloud2` | `sensor_msgs/PointCloud2` | 10 Hz | `laser_livox` | **是** (lid_topic) | adapter 输出，仅 x, y, z |
| `/livox/Pointcloud2` | `sensor_msgs/PointCloud2` | ~10 Hz | odom 帧 | 否（odom 帧不适合 SLAM） | 由 `pointcloud2livox` 节点发布 |

**LiDAR 规格**（来自 `gazebo.xacro`）：
- 水平 FOV: 360°
- 垂直 FOV: -5.22° 到 57.22°
- 测距: 0.1 m – 40 m
- 分辨率: 0.01 m
- 噪声: 高斯噪声 σ=0.005
- 每帧点数: ~24000

### 5.2 IMU 话题

| Topic | Type | Frame | Frequency | Used by FAST-LIO2 | Notes |
|-------|------|-------|-----------|-------------------|-------|
| `/livox/imu` | `sensor_msgs/Imu` | 1000 Hz | `livox_imu_link` | **是** (默认 imu_topic) | Livox 内置 IMU，与 LiDAR 共位 |
| `/trunk_imu` | `sensor_msgs/Imu` | 1000 Hz | `imu_link` | 备选 | 躯干 IMU，位于质心 |

### 5.3 FAST-LIO2 输出话题（预期，需运行时验证）

| Topic | Type | Frame | Notes |
|-------|------|-------|-------|
| `/Odometry` | `nav_msgs/Odometry` | `camera_init` → `body` | LiDAR-inertial 里程计 |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | world 帧 | 配准后的点云 |
| `/Laser_map` | `sensor_msgs/PointCloud2` | world 帧 | 全局地图点云 |
| `/path` | `nav_msgs/Path` | — | 机器人路径（若 `path_en: true`） |
| TF | `camera_init` → `body` | — | SLAM 坐标系变换 |

### 5.4 真值话题（仅调试，不作为 SLAM 输入）

| Topic | Type | Notes |
|-------|------|-------|
| `/Odometry_gazebo` | `nav_msgs/Odometry` | Gazebo 真值里程计 |
| `/ground_truth/base_w` | `nav_msgs/Odometry` | base 在 world 中的真值位姿 |
| `/ground_truth/base_trunk` | `nav_msgs/Odometry` | base 在 trunk 中的真值位姿 |

> ⚠️ **禁止将真值话题作为 SLAM 算法输入。** 这些话题仅用于本地调试和精度评估。

## 6. PointCloud Compatibility

### 6.1 当前点云字段

`/scan` (PointCloud) 由 Gazebo LiDAR 插件发布，包含 x, y, z。`scan_to_pointcloud2.py` adapter 将其转换为 PointCloud2 (xyz32 格式)，仅保留 x, y, z。

| 字段 | 状态 | 影响 |
|------|------|------|
| `x` | ✅ 有 | — |
| `y` | ✅ 有 | — |
| `z` | ✅ 有 | — |
| `intensity` | ❌ 无 | 影响反射率相关算法和可视化 |
| `time` / `timestamp` / `offset_time` | ❌ 无 | **关键缺失**：无法进行 per-point 运动补偿（去畸变） |
| `ring` / `line` | ❌ 无 | 无法按扫描线进行结构化处理 |

### 6.2 影响评估

- **per-point time 缺失**：
  - FAST-LIO2 配置 `timestamp_unit=0.0`, `time_scale=0.0` 禁用时间补偿。
  - 在低速运动（< 0.5 m/s）时影响较小，是仿真环境的可接受近似。
  - 高速运动或快速旋转时，未去畸变的点云会导致建图精度下降。
- **intensity 缺失**：不影响里程计估计，但影响可视化质量和部分基于反射率的特征提取。
- **ring/line 缺失**：`scan_line=4` 作为近似值（Mid-360 实际为 4 线），不影响核心算法但可能影响点云结构化处理。

### 6.3 诊断命令

```bash
# 检查 adapter 输出的点云字段
rosrun simenv_fast_lio2_integration pointcloud_fields_check.py _topic:=/scan_pointcloud2

# 检查原始 /scan 点云（PointCloud 类型）
rostopic echo -n 1 /scan | head -20
```

## 7. IMU Selection

### 7.1 两个 IMU 对比

| 属性 | Livox IMU | Trunk IMU |
|------|-----------|-----------|
| Topic | `/livox/imu` | `/trunk_imu` |
| Frame | `livox_imu_link` | `imu_link` |
| 父坐标系 | `laser_livox` → `base` | `trunk` → `base` |
| 相对 LiDAR 位置 | 固定偏移 `(-0.011, -0.02329, 0.04412)` | LiDAR 在 base 前上方 (0.2, 0, 0.08)，IMU 在质心处 |
| 外参确定性 | **高**：物理共位，直接从 URDF 提取 | **低**：需经过 base→laser_livox 和 base→trunk→imu_link 两级变换 |
| 推荐用途 | **默认** | 备选（通过 `use_trunk_imu:=true` 切换） |

### 7.2 决策依据

- **第一版默认使用 `/livox/imu`**：Livox IMU 与 LiDAR 物理共位，`extrinsic_T` 为 URDF 中 `laser_livox → livox_imu_link` 的固定平移，`extrinsic_R` 为单位矩阵（两坐标系对齐）。
- 如果切换为 `/trunk_imu`，必须重新计算外参（从 trunk→imu_link→base→laser_livox 的完整变换链），并更新 `extrinsic_T` 和 `extrinsic_R`。
- 仿真中两个 IMU 均为理想传感器（`gaussianNoise=0.0`），实际建图效果差异需要通过实验验证。

### 7.3 坐标系方向验证

需要确认 IMU 消息的角速度和线加速度方向与 FAST-LIO2 期望一致（ROS 标准：x 前、y 左、z 上）。如果方向不一致（如 z 轴反向），会导致重力方向估计错误，建图失败。

## 8. Extrinsic Calibration

### 8.1 TF 树（来源：URDF/Xacro）

```
world
  └── base                              # 机器人基坐标系
        ├── trunk                       # 浮动基座（质心），floating_base joint: (0,0,0)
        │     └── imu_link              # 躯干 IMU，imu_joint: (0,0,0)
        ├── laser_livox                 # LiDAR，laser_livox_joint: xyz=(0.2, 0, 0.08), rpy=(0, 0.785, 0)
        │     └── livox_imu_link        # Livox IMU，livox_imu_joint: xyz=(-0.011, -0.02329, 0.04412), rpy=(0,0,0)
        └── real_sense                  # 深度相机，real_sense_joint: xyz=(0.28, 0, 0.043), rpy=(0,0,0)
```

### 8.2 FAST-LIO2 外参配置

FAST-LIO2 中 `extrinsic_T` 和 `extrinsic_R` 表示 **LiDAR → IMU** 的变换（IMU 在 LiDAR 坐标系中的位置）：

- `extrinsic_T: [-0.011, -0.02329, 0.04412]` — 来自 URDF `laser_livox → livox_imu_link`
- `extrinsic_R: [1, 0, 0; 0, 1, 0; 0, 0, 1]` — URDF 中该 joint 无旋转，两坐标系对齐

### 8.3 LiDAR 45° Pitch 安装风险

`laser_livox` 相对于 `base` 有 `rpy=(0, 0.785, 0)`（即绕 Y 轴旋转 45° / 0.785 rad）。这意味着 LiDAR 坐标系本身已经倾斜。FAST-LIO2 通过 `extrinsic_T` + `extrinsic_R` + IMU 重力方向对齐来处理这一问题。然而：

- 如果 IMU 噪声模型不准确，重力方向估计可能偏移。
- `fov_degree=180`（仅使用前向 180°）可能是为了减少后向地面点云的干扰。
- **需要运行时通过 TF 和外参验证命令来检查实际效果**。

### 8.4 验证命令

```bash
# 查看完整 TF 树
rosrun tf view_frames

# 检查 LiDAR 与 IMU 之间的变换
rosrun tf tf_echo laser_livox livox_imu_link

# 检查 base 与 LiDAR 之间的变换
rosrun tf tf_echo base laser_livox

# 查看一帧点云的 frame_id
rostopic echo -n 1 /scan_pointcloud2 | grep frame_id

# 查看 IMU 数据的 frame_id
rostopic echo -n 1 /livox/imu | grep frame_id
```

## 9. Build Environment Notes

### 9.1 推荐构建流程

```bash
cd SimEnv
source /opt/ros/noetic/setup.bash
source .venv/bin/activate
./tools/build_with_venv.sh
source devel/setup.bash
```

`build_with_venv.sh` 自动处理：
- 检测 pip torch 的 CMake prefix path (`torch.utils.cmake_prefix_path`)
- 选择 gcc-11/g++-11 作为编译器
- 设置 CUDA 相关路径

### 9.2 工具链要求

| 问题 | 原因 | 解决方案 |
|------|------|---------|
| `TorchConfig.cmake not found` | torch 未安装或 CMake 找不到 | `python -c "import torch; print(torch.utils.cmake_prefix_path)"` 确认路径 |
| `Caffe2 version uses CUDA but I cannot find CUDA libraries` | CUDA Toolkit 未安装 | 安装 CUDA 11.8 Toolkit |
| `nvcc fatal: Failed to preprocess host compiler properties` | gcc 版本过高（>12） | 使用 gcc-11/g++-11 |
| `gcc: fatal error: cannot execute cc1plus` | g++ 未安装 | `sudo apt install g++-11` |

### 9.3 编译注意事项

- **不修改系统默认 gcc/g++ alternatives**。通过 `build_with_venv.sh` 中的 CC/CXX 环境变量指定。
- 如果 CMakeCache 绑定了旧编译器，需要清理 `build/` 和 `devel/`（**需用户确认**）：
  ```bash
  rm -rf build devel
  ```
- `livox_ros_driver` 需要 C++17（已在 `fix/0704-fast-lio2-build-errors` 分支中修复）。
- `unitree_guide` 的 PIE 链接问题需要 `-no-pie` 标志（已在 `fix/0704-fast-lio2-build-errors` 分支中修复）。
- 如果只是验证 FAST-LIO2 mapping，可跳过 `unitree_guide` 和 `uav_simulator`。跳过方法：在对应包目录下创建 `CATKIN_IGNORE` 文件，或使用 `catkin_make -DCATKIN_BLACKLIST_PACKAGES="unitree_guide;uav_simulator"`。

### 9.4 编译顺序

1. 确认 FAST_LIO 存在于 `src/FAST_LIO`
2. 确认 livox_ros_driver 存在于 `src/livox_ros_driver`
3. 运行 `./tools/build_with_venv.sh`
4. 如果构建失败，检查 `experiments/runs/0704_fast-lio2-build-errors/` 中的历史错误日志

## 10. Deployment Steps

### 完整部署流程

```bash
# === Step 1: 进入 SimEnv workspace ===
cd SimEnv
source /opt/ros/noetic/setup.bash

# === Step 2: 激活 venv ===
source .venv/bin/activate

# === Step 3: 确认 FAST_LIO 和 livox_ros_driver 已安装 ===
ls src/FAST_LIO/CMakeLists.txt && echo "OK: FAST_LIO" || echo "MISSING: clone FAST_LIO first"
ls src/livox_ros_driver/CMakeLists.txt 2>/dev/null && echo "OK: livox_ros_driver" || echo "MISSING: clone livox_ros_driver first"

# === Step 4: 构建 ===
./tools/build_with_venv.sh
source devel/setup.bash

# === Step 5: 确认包可被发现 ===
rospack find fast_lio
rospack find livox_ros_driver
rospack find simenv_fast_lio2_integration

# === Step 6: 启动 SimEnv 仿真（关闭 odom 转换，保留传感器帧点云） ===
ENABLE_POINTCLOUD_CONVERTER=0 ./auto.sh

# === Step 7: 在另一个终端，检查传感器话题 ===
source devel/setup.bash
rostopic list | grep -E "scan|livox|imu"
rostopic hz /scan
rostopic hz /livox/imu

# === Step 8: 运行点云字段检查 ===
rosrun simenv_fast_lio2_integration pointcloud_fields_check.py _topic:=/scan_pointcloud2

# === Step 9: 启动 FAST-LIO2 mapping ===
roslaunch simenv_fast_lio2_integration simenv_fast_lio2_mapping.launch

# === Step 10: 检查输出 ===
rostopic list | grep -E "Odometry|cloud_registered|Laser_map"
rostopic hz /Odometry
rostopic hz /cloud_registered

# === Step 11: 保存地图（需在配置中启用 pcd_save_en: true） ===
# PCD 默认保存在 FAST_LIO/PCD/ 目录下
```

### 简化启动（使用 auto.sh 集成）

```bash
ENABLE_FAST_LIO2=1 ./auto.sh
```

此模式自动在后台启动 `scan_to_pointcloud2` adapter 和 FAST-LIO2 节点。

> **注意**：当前 `simenv_fast_lio2_mapping.launch` 中的 FAST-LIO2 node 块为注释状态。需在 FAST_LIO 成功编译后取消注释，或确认集成启动逻辑已启用。

## 11. Runtime Validation Checklist

启动后逐项检查：

- [ ] ROS master running (`rostopic list`)
- [ ] Gazebo running（仿真世界可见或 headless 模式正常）
- [ ] `/scan` exists (`rostopic hz /scan` > 0)
- [ ] `/scan_pointcloud2` exists（adapter 必须运行）
- [ ] `/livox/imu` exists (`rostopic hz /livox/imu` ≈ 1000)
- [ ] FAST-LIO2 node alive (`rosnode list | grep laserMapping`)
- [ ] `/Odometry` publishing (`rostopic hz /Odometry` > 0)
- [ ] `/cloud_registered` publishing (`rostopic hz /cloud_registered` > 0)
- [ ] `/Laser_map` publishing（可能频率较低，正常）
- [ ] TF connected (`rosrun tf view_frames` 确认 `camera_init → body` 存在)
- [ ] RViz map visible（可选，启动 `rviz:=true` 参数）
- [ ] PCD saved（若启用，检查 `FAST_LIO/PCD/` 目录）

## 12. Common Failure Modes

| Error | Likely Cause | Action |
|-------|-------------|--------|
| `TorchConfig.cmake not found` | torch 未安装或 CMake prefix 路径错误 | 确认 venv 已激活: `source .venv/bin/activate`；运行 `python -c "import torch; print(torch.utils.cmake_prefix_path)"` |
| `Caffe2 version uses CUDA but I cannot find CUDA libraries` | CUDA 11.8 未安装 | 安装 CUDA Toolkit 11.8 或跳过 unitree_guide |
| `nvcc fatal: Failed to preprocess host compiler properties` | gcc 版本与 CUDA 不兼容 | 使用 `./tools/build_with_venv.sh`（自动选择 gcc-11） |
| `gcc: fatal error: cannot execute cc1plus` | g++ 未安装 | `sudo apt install g++-11` |
| `livox_ros_driverConfig.cmake not found` | livox_ros_driver 未 clone | `cd src && git clone https://github.com/Livox-SDK/livox_ros_driver.git` |
| `undefined reference to ros::init` | PIE 链接问题（unitree_guide） | 添加 `-no-pie` 链接标志或跳过 unitree_guide |
| `std::shared_mutex only available from C++17` | livox_ros_driver C++ 标准过低 | 设置 `-std=c++17`（已在 fix 分支修复） |
| `deque does not name a type` | 缺少 `<deque>` include | 添加 `#include <deque>`（已在 fix 分支修复） |
| `rospack find fast_lio` failed | FAST_LIO 未 clone 或未 source | `cd src && git clone https://github.com/hku-mars/FAST_LIO.git && cd FAST_LIO && git submodule update --init` |
| `/scan_pointcloud2` not publishing | adapter 未启动 | 检查 `ENABLE_POINTCLOUD_CONVERTER=0` 是否生效，adapter 是否运行 |
| FAST-LIO2 publishes no map | 传感器数据异常、外参错误、或无运动 | 检查 `/scan_pointcloud2` 是否有数据，机器人是否在移动 |
| PCD save fails (OOM) | 点数过多 + `interval: -1` | 设置 `interval: 100` 分帧保存 |
| IMU 消息方向错误 | 坐标系约定不一致 | 检查 `/livox/imu` 的 `angular_velocity.z` 方向是否与 ROS 标准一致 |

## 13. Parameters That Need Experiment Tracking

以下参数需要在运行时实验中系统调优。建议实验记录放在：

```text
experiments/runs/MMDD_fast-lio2-param-test/
```

| 参数 | 当前值 | 调优方向 | 优先级 |
|------|--------|---------|--------|
| `scan_line` | 4 | 验证 4 是否合理（仿真无 ring），尝试 1 或更大值 | 中 |
| `timestamp_unit` | 0.0 | 如果后续 Gazebo 插件支持 per-point time，改为非零 | 低（依赖插件改动） |
| `blind` | 0.5 | 根据实际点云噪声调整 | 低 |
| `filter_size_surf` | 0.5 | 室内场景尝试 0.2–1.0 | 高 |
| `filter_size_map` | 0.5 | 同上 | 高 |
| `cube_side_length` | 1000.0 | 多楼层场景可能需要更大值 | 中 |
| `det_range` | 40.0 | 匹配 LiDAR 实际有效距离 | 低 |
| `acc_cov` / `gyr_cov` | 0.1 | 仿真 IMU 噪声为 0，当前值为保守占位；需实验 | 高 |
| `extrinsic_T` | `[-0.011, -0.02329, 0.04412]` | 运行时验证精度 | 高 |
| `extrinsic_R` | identity | 确认方向正确 | 高 |
| IMU choice | `/livox/imu` | 对比 `/trunk_imu` 建图效果 | 中 |
| PointCloud adapter | xyz32 only | 评估是否需要补 intensity/time | 中 |
| PCD save interval | -1 | 实验大型场景的内存使用 | 低 |

## 14. Output Contract for Future Navigation

以下表格定义 FAST-LIO2 建图完成后，后续导航探索算法需要确认的输出契约。**当前仅做文档定义，不实现代码。**

| Output | Topic / TF | Consumer | Status |
|--------|-----------|----------|--------|
| Odometry | `/Odometry` | navigation / exploration | 待运行验证 |
| Registered cloud | `/cloud_registered` | map builder / visualization | 待运行验证 |
| Global map | `/Laser_map` | exploration / costmap | 待运行验证 |
| Path | `/path` | visualization / debug | 待运行验证 |
| TF | `camera_init` → `body` | navigation stack | 待确认 frame 命名 |
| PCD | `FAST_LIO/PCD/` | offline map debug | 待确认路径 |

### 14.1 导航集成待确认事项

- FAST-LIO2 输出 frame（`camera_init` → `body`）是否适合后续导航？标准导航 TF 树通常为 `map → odom → base_link`。可能需要插入一个 transform 重映射节点。
- `/Odometry` 的协方差是否正确填充？导航包（如 `move_base`）依赖协方差进行定位融合。
- `/Laser_map` 的发布频率和点数是否满足 costmap 更新需求？
- 是否需要将 FAST-LIO2 输出从 `camera_init` 帧重新发布到标准 `map` 帧？

## 15. Open Questions

| # | 问题 | 当前状态 | 验证方法 |
|---|------|---------|---------|
| 1 | `/scan_pointcloud2` 是否有 per-point time？ | 已知无（xyz32 adapter） | `pointcloud_fields_check.py` |
| 2 | adapter 是否应该补 intensity？ | 当前不补 | 评估是否需要修改 adapter |
| 3 | `scan_line=4` 是否合理？ | 当前为近似值 | 对比 scan_line=1 和 scan_line=4 的建图效果 |
| 4 | LiDAR 45° pitch 外参方向是否已验证？ | 外参从 URDF 提取，未运行时验证 | `tf_echo` + FAST-LIO2 运行测试 |
| 5 | `/livox/imu` vs `/trunk_imu` 哪个建图效果更稳定？ | 未对比 | A/B 对照实验 |
| 6 | FAST-LIO2 输出 frame `camera_init` 是否适合后续导航？ | 待确认 | 检查 nav stack 兼容性 |
| 7 | 仿真 IMU 噪声为 0，`acc_cov=0.1` 是否合适？ | 当前为保守占位值 | 参数扫描实验 |
| 8 | 是否需要 TF 重映射 `camera_init → map`？ | 待确认 | 检查导航包 frame 要求 |
| 9 | `fov_degree=180` 是否为合理的前向限制？ | 当前限制前向 180° | 对比 360° 建图效果 |
| 10 | FAST_LIO launch 中的 node 块当前为注释状态，实际运行时需取消注释 | 代码层面已准备好 | 确认 FAST_LIO 编译后取消注释 |
