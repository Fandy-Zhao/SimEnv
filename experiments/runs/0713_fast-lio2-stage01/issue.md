# Issue: FAST-LIO2 Stage 0 & Stage 1 Deployment Testing

- **创建日期**: 2026-07-13
- **分支**: exp/0713-fast-lio2-stage01 (from develop)
- **参考文档**: `prompts/fastlio2_tare_dsv_test_plan.md` §3-4, `docs/slam/fast_lio2_deployment_guide.md`
- **状态**: In Progress

## Task Goal

按测试计划逐步执行 FAST-LIO2 部署测试的 Stage 0（传感器/时间/TF）和 Stage 1（FAST-LIO2 单独定位建图），验证 L1（安装）和 L2（接口）等级。

## Modification Scope

- Uncomment FAST-LIO2 node in `simenv_fast_lio2_mapping.launch`
- Execute Stage 0 checks: /use_sim_time, /clock, sensor topics, TF tree
- Execute Stage 1 checks: launch FAST-LIO2, verify /Odometry, /cloud_registered
- Static drift test (60s no motion)

## Non-Scope

- Stage 2+ (navigation adaptation, FALCO, TARE, DSV)
- Modifying FAST_LIO source code
- Modifying sensor plugins or URDF models

## Acceptance Criteria

- [ ] Stage 0: All sensor/time/TF checks pass
- [ ] Stage 1 L1: FAST-LIO2 launches without immediate crash
- [ ] Stage 1 L2: /Odometry and /cloud_registered publish continuously
- [ ] Stage 1 static test: drift within acceptable bounds

## Risk Points

- FAST_LIO binary was compiled on 2026-07-06, may have bit-rotted
- livox_ros_driver not compiled — may cause linking issues
- No display available for RViz visualization

## Expected Impacted Modules

- `simenv_fast_lio2_integration/` (launch file)
- `FAST_LIO/` (external, read-only)
