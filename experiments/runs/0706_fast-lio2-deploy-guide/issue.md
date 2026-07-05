# Issue: FAST-LIO2 Deployment Guide for SimEnv

## Task Goal

整理 SimEnv 中部署 FAST-LIO2 的完整流程、参数映射、传感器配置和重点风险，生成一份可指导用户从零部署的技术文档。

## Modification Scope

- FAST-LIO2 部署文档 (新增 `docs/slam/fast_lio2_deployment_guide.md`)
- 参数说明文档
- 已有 mapping 报告补充
- 治理文档更新 (PROJECT_STATE.md, CHANGELOG.md, docs/module_status.md)
- 实验记录 (experiments/runs/0706_fast-lio2-deploy-guide/)
- 按需: ADR, docs/architecture.md, simenv_fast_lio2_integration README

## Explicit Non-Scope

- 不修改核心代码
- 不修编译问题
- 不改 FAST_LIO 源码
- 不改 unitree_guide
- 不改 uav_simulator
- 不做导航探索
- 不 push
- 不 merge main/master

## Acceptance Criteria

1. 文档能指导用户从 SimEnv workspace 根目录部署 FAST-LIO2
2. 明确 FAST_LIO 应位于 `SimEnv/src/FAST_LIO`
3. 明确 livox_ros_driver 应位于 `SimEnv/src/livox_ros_driver` 或已 source
4. 明确 `/scan`, `/scan_pointcloud2`, `/livox/imu`, `/trunk_imu` 的关系
5. 明确 FAST-LIO2 配置参数与 SimEnv 传感器设置的映射
6. 明确点云缺少 per-point time, intensity, ring/line 的风险
7. 明确外参从 URDF/Xacro/SDF 核对
8. 明确编译环境注意事项
9. 明确后续导航探索前需要确认的输出契约

## Risk Points

- FAST-LIO2 对 per-point time 依赖较强
- SimEnv 点云字段可能不完整
- LiDAR 45° pitch 安装可能影响外参
- `/livox/imu` 与 `/trunk_imu` 的坐标系、噪声和同步差异会影响建图
- torch/cu118/CUDA/gcc 工具链可能影响编译
- 跳过 unitree_guide 或 uav_simulator 会影响控制/仿真完整性，但不一定影响 mapping 编译验证

## Expected Impacted Modules

- `src/simenv_fast_lio2_integration/` (reference only)
- `src/FAST_LIO/` (reference only)
- `docs/` (new document)
- Governance files (update)
