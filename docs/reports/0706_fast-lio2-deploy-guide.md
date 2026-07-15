# Task Report — 0706 Fast-LIO2 Deployment Guide

## Branch
- 工作分支: `docs/0706-fast-lio2-deploy-guide`
- 分支类型: 文档 (docs)
- 是否 merge 回 dev: 是
- 是否 push: 否

## Summary
- 新增 `docs/slam/fast_lio2_deployment_guide.md`，系统整理了 FAST-LIO2 在 SimEnv 中的部署流程、传感器配置映射、参数说明、编译环境、运行验证和排错指南。
- 新增 ADR `docs/decisions/ADR-0706-fast-lio2-deploy-guide.md`，记录了部署相关的架构决策。
- 更新了 `PROJECT_STATE.md`、`CHANGELOG.md`、`docs/module_status.md`。

## Key Findings

### LiDAR 配置
- `/scan` (PointCloud, 10 Hz, frame: `laser_livox`) 由 Gazebo LiDAR 插件发布
- `/scan_pointcloud2` (PointCloud2, xyz32) 由 `scan_to_pointcloud2.py` adapter 生成
- LiDAR 挂载: base → laser_livox: (0.2, 0, 0.08), rpy=(0, 0.785, 0) [45° pitch]
- 规格: 360° H-FOV, -5.22°~57.22° V-FOV, 0.1-40m, 24000 点/帧

### IMU 配置
- `/livox/imu`: 1000 Hz, frame: `livox_imu_link` (LiDAR 内置 IMU, 默认)
- `/trunk_imu`: 1000 Hz, frame: `imu_link` (躯干 IMU, 备选)
- Livox IMU 与 LiDAR 物理共位, 外参确定性强

### 点云字段风险
- x, y, z ✅
- intensity ❌
- per-point time ❌ (timestamp_unit=0 禁用)
- ring/line ❌ (scan_line=4 作为近似)

### 外参风险
- extrinsic_T: [-0.011, -0.02329, 0.04412] (LiDAR→Livox IMU, 来自 URDF)
- extrinsic_R: identity (两坐标系对齐)
- LiDAR 45° pitch 安装可能影响建图效果

### 编译环境风险
- torch 2.0.1+cu118 + CUDA 11.8 + gcc-11 工具链
- livox_ros_driver 需要 C++17
- unitree_guide PIE 问题
- 推荐 `tools/build_with_venv.sh` 统一构建

### 后续导航接口风险
- FAST-LIO2 输出 `camera_init→body` TF, 可能需要重映射为 `map→odom→base_link`
- `/Odometry` 协方差需验证
- `/Laser_map` 频率需确认

## Documentation Coverage

| Topic | Covered | Location |
|-------|---------|----------|
| workspace layout | ✅ | §2 |
| FAST_LIO path | ✅ | §2 |
| livox_ros_driver | ✅ | §3 |
| LiDAR topic mapping | ✅ | §4, §5.1 |
| IMU topic mapping | ✅ | §4, §5.2, §7 |
| pointcloud compatibility | ✅ | §6 |
| extrinsic calibration | ✅ | §8 |
| build environment | ✅ | §9 |
| deployment steps | ✅ | §10 |
| runtime validation | ✅ | §11 |
| common failures | ✅ | §12 |
| parameters for experiments | ✅ | §13 |
| output contract | ✅ | §14 |
| open questions | ✅ | §15 |

## Tests
| Check | Result | Notes |
|-------|--------|-------|
| markdown file exists | ✅ PASS | `docs/slam/fast_lio2_deployment_guide.md` |
| grep coverage check | ✅ PASS | Key terms verified across docs |
| check_repo_clean.py | ✅ PASS | No governance issues |
| git status | ✅ PASS | Clean working tree (only untracked) |

## Risks
- 文档中仍有 10 个开放问题待运行时验证
- FAST-LIO2 参数（acc_cov, gyr_cov, filter_size_*）仍需实验调优
- 编译环境依赖 CUDA 11.8 + gcc-11 工具链，不适用于所有环境
- 后续导航集成前需先确认 output contract
