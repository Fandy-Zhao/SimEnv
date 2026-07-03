# Project State

## Snapshot
- Date: 2026-07-04
- Branch: zzf/0704-connect-github-remote (from develop)
- Project type: ROS1 Noetic + Gazebo Classic (robotics competition simulation)
- Current focus: 仓库远程配置 — 连接 GitHub remote

## Active Work
- 连接 GitHub remote (`github` → `git@github.com:zzf/SimEnv.git`)
- 维护类分支命名规则从 `chore/` 迁移到 `zzf/`
- 治理文档初始化 (已在上一轮完成)

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
