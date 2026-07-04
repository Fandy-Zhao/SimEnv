# ADR-0704-fast-lio2-mapping: FAST-LIO2 Integration Architecture

## Status
Proposed

## Context
SimEnv 仿真环境需要 SLAM 建图能力，FAST-LIO2 是 LiDAR-Inertial SLAM 的标准方案。当前环境发布 `sensor_msgs/PointCloud` 点云和两个 IMU 话题，需要做出多项架构决策。

## Decision

### 1. 不 vendor FAST_LIO 源码
**决定**: 不 vendor FAST_LIO。FAST_LIO 作为外部源码 ROS package，放在 `SimEnv/src/FAST_LIO`（SimEnv 仓库本身就是 catkin workspace 根目录）。用户通过 `git clone` 自行安装。
**理由**: 避免维护外部代码的负担、许可证兼容性风险、以及仓库膨胀。用户通过 README 中的 `git clone` 指令自行安装。

### 2. 新增 PointCloud2 适配器
**决定**: 创建 `scan_to_pointcloud2.py`，将 `/scan` (PointCloud) 转换为 `/scan_pointcloud2` (PointCloud2, laser_livox 帧)。
**理由**: `/scan` 是 PointCloud 类型，FAST-LIO2 期望 PointCloud2。现有的 `pointcloud2livox.py` 将点云转换到 odom 帧 (使用真值里程计)，不适合 SLAM。新适配器保留传感器帧。
**替代方案**: 修改 Gazebo LiDAR 插件直接发布 PointCloud2。该方案更优但风险更高，推迟到后续阶段。

### 3. 禁用 per-point 运动补偿
**决定**: FAST-LIO2 配置 `timestamp_unit=0`, `time_scale=0.0`。
**理由**: 仿真 LiDAR 插件不输出 per-point 时间戳。无此信息时强制启用会导致错误。

### 4. 使用 `/livox/imu` 而非 `/trunk_imu`
**决定**: 默认使用 `/livox/imu` (帧 `livox_imu_link`)。
**理由**: Livox IMU 与 LiDAR 物理上共位，外参更精确 (直接从 URDF 提取)。躯干 IMU 位于 CoM 处，距 LiDAR 较远，外参误差更大。
**替代方案**: 通过 `use_trunk_imu:=true` 参数可选切换。

### 5. 第一阶段只做建图
**决定**: 本任务仅完成 FAST-LIO2 集成骨架，不实现导航/探索。
**理由**: 分阶段开发降低风险。先验证 SLAM 建图可用性，再在此基础上构建导航。

## Alternatives Considered
1. **Vendor FAST_LIO**: 被拒绝——带来许可证、维护和仓库膨胀问题。
2. **直接修改 pointcloud2livox.py**: 被拒绝——该脚本负责 `/livox/Pointcloud2`，修改可能破坏现有使用方。新适配器独立运行。
3. **修改 Gazebo 插件添加 per-point 时间**: 被推迟——需要重新编译插件，且注释代码已有 `PublishPointCloud2XYZRTL` 实现，可作为后续参考。

## Consequences
- 用户需手动安装 FAST_LIO: 将 FAST_LIO clone 到 `SimEnv/src/FAST_LIO`，然后使用 `./tools/build_with_venv.sh` 构建
- 建图精度受限：缺少 per-point 运动补偿和强度信息
- 后续导航探索模块需依赖 FAST-LIO2 输出的 `/Odometry`、`/cloud_registered`、TF `camera_init → body`

## Validation
- 静态检查: py_compile, roslaunch --files 通过
- 构建检查: catkin_make 通过
- 运行时验证: 需在完整 Gazebo 仿真环境中启动并检查 FAST-LIO2 输出话题 (待 Gazebo 环境可用)
