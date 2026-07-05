# ADR-0706-fast-lio2-deploy-guide: FAST-LIO2 Deployment Guide Decisions

## Status
Proposed

## Context
整理 FAST-LIO2 在 SimEnv 中的部署文档时，需要明确多项架构和配置决策，确保文档能够指导用户从零部署 FAST-LIO2 建图。

## Decision

### 1. 文档独立于集成包
**决定**: 部署指南放在 `docs/slam/fast_lio2_deployment_guide.md`，不放在 `simenv_fast_lio2_integration/README.md`。
**理由**: 部署指南涉及编译环境、传感器配置、TF 树、URDF 外参等跨模块内容，不适合局限在单个 package 的 README 中。`simenv_fast_lio2_integration/README.md` 仍作为包级快速参考。

### 2. 确认 FAST_LIO 不 vendor
**决定**: 重申 FAST_LIO 作为外部源码放在 `SimEnv/src/FAST_LIO`，不 vendor 到 SimEnv 仓库或 `simenv_fast_lio2_integration` 内。
**理由**: 避免维护外部代码负担、许可证兼容性风险、仓库膨胀。已经在 ADR-0704-fast-lio2-mapping 中确立，本次文档进一步明确。

### 3. 默认使用 `/livox/imu`
**决定**: 部署指南默认推荐 `/livox/imu` 作为 IMU 输入，`/trunk_imu` 作为备选。
**理由**: Livox IMU 与 LiDAR 物理共位，外参确定性强（URDF 中可直接提取）。躯干 IMU 位于质心，与 LiDAR 距离较远且需要多级 TF 变换，外参不确定性更大。

### 4. 不使用 `/Odometry_gazebo` 作为 SLAM 输入
**决定**: 文档明确禁止将 Gazebo 真值话题作为 SLAM 算法输入。
**理由**: 比赛场景下不存在真值里程计，使用真值会掩盖 SLAM 算法本身的问题。真值仅用于调试和精度评估。

### 5. PointCloud time 字段是关键风险
**决定**: 文档重点说明 per-point time 缺失的影响和 `timestamp_unit=0` 的后果。
**理由**: FAST-LIO2 的核心优势之一是 per-point 运动补偿（去畸变）。仿真环境下禁用此功能，建图精度在高速运动时会下降。这是后续需要解决的已知限制。

### 6. 后续导航前需先定义 output contract
**决定**: 在部署指南中提前列出 FAST-LIO2 输出契约表格，但不实现导航集成。
**理由**: 为后续导航探索算法开发提供明确的接口预期，避免后期发现接口不匹配。

## Alternatives Considered
1. **将部署指南放在 `simenv_fast_lio2_integration/README.md`**: 拒绝——跨模块内容不适合单包 README。
2. **默认使用 `/trunk_imu`**: 拒绝——外参不确定性更高。
3. **在文档中修复 `simenv_fast_lio2_mapping.launch` 中的注释**: 拒绝——代码修改不在本文档范围内，已在 launch 文件中注释说明原因。

## Consequences
- 用户可通过 `docs/slam/fast_lio2_deployment_guide.md` 获得完整的部署指导
- 10 个开放问题需要在后续运行时实验中逐一验证
- FAST-LIO2 参数调优需要专门的实验分支
- 后续导航探索开发前需要先验证 output contract

## Validation
- 文档覆盖 15 个必需章节
- 所有参数映射均基于实际代码和 URDF 验证
- 编译环境说明基于历史 build error 记录
- check_repo_clean.py 通过
