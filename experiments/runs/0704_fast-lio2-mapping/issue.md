# Issue: 接入 FAST-LIO2 作为第一阶段 SLAM 建图能力

## 任务目标
在 SimEnv 中接入 FAST-LIO2，使 Unitree A1 仿真环境启动后可以进行 LiDAR-Inertial SLAM 建图。只完成"可编译、可启动、可验证的集成骨架"，不实现导航探索。

## 修改范围
- governance 文档更新 (已在上一轮初始化完成)
- 新增 `src/simenv_fast_lio2_integration/` ROS 集成包
- FAST-LIO2 配置文件 + launch + 适配脚本
- `auto.sh` 可选启动入口 (`ENABLE_FAST_LIO2`)
- 诊断工具 `pointcloud_fields_check.py`

## 非范围
- 不做探索/导航/路径规划
- 不做危险源识别
- 不读取真值文件
- 不改控制器 `junior_ctrl`
- 不 push
- 不 merge main/master
- 不 vendor FAST_LIO 源码

## 验收标准
- [x] 原有 `./auto.sh` 默认流程不被破坏
- [x] 新增独立 FAST-LIO2 mapping launch
- [x] 配置中明确 LiDAR topic (/scan_pointcloud2), IMU topic (/livox/imu), frame, 外参
- [x] 点云 time 字段缺失已文档化
- [x] 可执行静态检查和构建检查
- [x] 任务分支为 `feat/0704-fast-lio2-mapping`

## 风险点
- FAST-LIO2 对 per-point time 依赖 → 已禁用时间补偿
- Livox Mid-360 仿真点云仅 x,y,z
- IMU/LiDAR 外参需运行时验证
- FAST_LIO 外部依赖未自动化安装
