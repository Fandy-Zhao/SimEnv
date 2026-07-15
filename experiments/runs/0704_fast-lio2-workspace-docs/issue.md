# Issue: 修正 FAST-LIO2 集成文档中的 workspace 结构说明

## Task Goal

修正 FAST-LIO2 集成文档中关于 workspace 结构的错误或模糊描述。明确 SimEnv 仓库本身就是 catkin workspace 根目录，FAST_LIO 应 clone 到 `SimEnv/src/FAST_LIO`，而不是嵌套在另一个 `catkin_ws` 下。

## Modification Scope

- `src/simenv_fast_lio2_integration/README.md` — 修正安装指令
- `src/simenv_fast_lio2_integration/config/simenv_mid360.yaml` — 修正注释
- `src/simenv_fast_lio2_integration/launch/simenv_fast_lio2_mapping.launch` — 修正注释
- `docs/decisions/ADR-0704-fast-lio2-mapping.md` — 修正架构描述
- `docs/architecture.md` — 修正依赖标注
- `docs/reports/0704_fast-lio2-mapping.md` — 同步修正
- 治理文档更新 (PROJECT_STATE.md, CHANGELOG.md, module_status.md)
- 任务记录

## Non-Scope

- 不修改业务代码
- 不 clone FAST_LIO
- 不运行完整 Gazebo
- 不 push
- 不 merge main/master
- 不删除文件

## Acceptance Criteria

1. 文档不再建议把 SimEnv 放在另一个 catkin workspace 的 `src/` 下
2. 文档明确 FAST_LIO 应位于 `SimEnv/src/FAST_LIO`
3. 文档明确 `simenv_fast_lio2_integration` 只是适配包，不是 FAST_LIO 本身
4. 文档明确 FAST_LIO 不 vendor、不作为 submodule
5. 更新治理文档

## Risk Points

- 用户可能已按旧文档 clone 到错误位置，文档需说明如何检查和迁移
