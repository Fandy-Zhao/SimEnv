# Task Report — 0704_fast-lio2-mapping

## Branch
- 工作分支: `feat/0704-fast-lio2-mapping`
- 分支类型: 功能开发 (feat)
- 是否遵守 `zzf/` 维护分支规则: N/A (本任务为功能分支)
- 是否 merge 回 dev: 待测试后 merge
- 是否 push: 否

## Summary
- 完成了 FAST-LIO2 集成骨架: 新增 `simenv_fast_lio2_integration` ROS 包
- 未完成: 运行时验证 (需要完整的 Gazebo + FAST_LIO 环境)
- 原因: catkin_make 编译环境需 ROS 依赖, Gazebo 显示环境需 X server

## FAST-LIO2 Integration
- FAST-LIO2 依赖方式: 不 vendor。FAST_LIO 作为外部源码放在 `SimEnv/src/FAST_LIO` 构建。
- 是否 vendor FAST_LIO: 否
- SimEnv LiDAR topic: `/scan` → `/scan_pointcloud2` (via adapter)
- SimEnv IMU topic: `/livox/imu` (frame: livox_imu_link)
- 可选 IMU topic: `/trunk_imu`
- FAST-LIO2 launch: `simenv_fast_lio2_mapping.launch`
- 配置文件: `config/simenv_mid360.yaml`
- 输出 topic / TF: `/Odometry`, `/cloud_registered`, `/Laser_map`, TF `camera_init → body`
- PCD 保存: `pcd_save_en: true` (interval: -1, 结束时保存)

## PointCloud / IMU Compatibility
- `/scan` 字段: x, y, z only (sensor_msgs/PointCloud)
- 是否包含 `x,y,z,intensity`: x,y,z ✅, intensity ❌
- 是否包含 per-point time 字段: ❌ (timestamp_unit=0 禁用)
- 是否包含 ring/line 字段: ❌ (scan_line=4 作为近似)
- `/livox/imu` 可用性: ✅ (1000 Hz, frame: livox_imu_link)
- `/trunk_imu` 可用性: ✅ (1000 Hz, frame: imu_link)
- 外参来源: URDF (`robot.xacro` + `gazebo.xacro`)
- 坐标系风险: LiDAR 45° pitch 安装, IMU 与 LiDAR 共位

## Tests
| 测试 | 结果 | 说明 |
|------|------|------|
| `python3 -m py_compile` (2 scripts) | ✅ PASS | system + venv Python 3.10 both OK |
| `roslaunch --files` | ✅ PASS | launch file syntax valid |
| `rospack find simenv_fast_lio2_integration` | ✅ PASS | package discovered correctly |
| `rospack find fast_lio` | ✅ PASS | FAST_LIO at `src/FAST_LIO` |
| `check_repo_clean.py` | ✅ PASS | no governance issues |
| `catkin_make` / `build_with_venv.sh` | ⚠️ PROGRESS | Torch CMake path detected; blocked by CUDA (cu118) + livox_ros_driver C++11 |

## Documentation Updated
- 治理文档: PROJECT_STATE.md, CHANGELOG.md, docs/module_status.md, docs/architecture.md
- ADR: docs/decisions/ADR-0704-fast-lio2-mapping.md
- 报告: docs/reports/0704_fast-lio2-mapping.md (本文件)
- 实验记录: experiments/runs/0704_fast-lio2-mapping/ (issue.md, notes.md)

## Git
- 分支: feat/0704-fast-lio2-mapping
- remote: origin (Gitee) + github (GitHub)
- (pending commit)

## Risks
- 剩余风险: 运行时验证无法在当前环境完成
- 需要用户确认: FAST_LIO 已存在于 `SimEnv/src/FAST_LIO`；当前阻塞点是 CUDA toolkit (torch cu118 需要) + livox_ros_driver C++11 编译问题
- 运行时验证限制: 需要 Gazebo + A1 模型 + FAST_LIO 编译

## Next Step
建议下一阶段: `feat/0704-mapping-output-contract` — 定义 FAST-LIO2 输出契约, 为导航探索输入接口设计。
