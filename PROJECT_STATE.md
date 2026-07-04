# Project State

## Snapshot
- Date: 2026-07-04
- Branch: feat/0704-fast-lio2-mapping (from develop)
- Project type: ROS1 Noetic + Gazebo Classic (robotics competition simulation)
- Current focus: FAST-LIO2 SLAM 建图集成 (Phase 1 — mapping only)

## Active Work
- FAST-LIO2 集成骨架: `src/simenv_fast_lio2_integration/`
- PointCloud→PointCloud2 适配器
- FAST-LIO2 配置 (simenv_mid360.yaml) 和 launch
- auto.sh 可选启动入口 (`ENABLE_FAST_LIO2`)
- 连接 GitHub remote (`github` → `git@github.com:zzf/SimEnv.git`, 尚未 push)
- venv 构建脚本: `tools/build_with_venv.sh` (zzf/0704-build-with-venv)
- FAST-LIO2 workspace 文档修正: 明确 SimEnv 是 catkin workspace 根目录 (docs/0704-fast-lio2-workspace-docs)
- FAST-LIO2 编译环境审计: 静态检查全部通过，catkin_make 被 libtorch (C++ SDK) 阻塞 (feat/0704-fast-lio2-mapping)

## Git Remotes
- `origin`: https://gitee.com/guoyulun/SimEnv.git (Gitee, 主远程)
- `github`: git@github.com:zzf/SimEnv.git (GitHub, 新增, 尚未 push)

## Branch Naming Policy (Updated)
- 维护/仓库配置类: `zzf/MMDD-short-name` (项目级覆盖规则)
- 不再使用 `chore/MMDD-short-name`

## Known Risks
- GitHub 远程仓库可能为空或已有历史，首次 push 前需确认目标分支
- 随机生成的建筑布局可能在某些参数组合下产生不可达房间或源重叠
- Gazebo Classic 已停止维护，长期可能需要迁移到 Ignition/Gazebo Fortress

## Validation Status
- Build: catkin_make 编译通过（最近提交已验证）
- Unit tests: `building_generator_core/test/` (3 tests), `building_generator_classic/test/` (2 tests)
- Remote config: `origin` 保持 Gitee, `github` 新增成功
- Governance: 骨架完整, 分支规则已更新

## Next Steps
- 用户确认后执行首次 push: `git push -u github develop`
- 补充 CI/自动化测试流程
- 评估将 UAV simulator 子模块独立或移除
