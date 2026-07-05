# Project State

## Snapshot
- Date: 2026-07-06
- Branch: docs/0706-fast-lio2-deploy-guide (from develop)
- Project type: ROS1 Noetic + Gazebo Classic (robotics competition simulation)
- Current focus: FAST-LIO2 SLAM 建图集成 (Phase 1 — mapping only), 部署文档完善

## Active Work
- FAST-LIO2 集成骨架: `src/simenv_fast_lio2_integration/`
- PointCloud→PointCloud2 适配器
- FAST-LIO2 配置 (simenv_mid360.yaml) 和 launch
- auto.sh 可选启动入口 (`ENABLE_FAST_LIO2`)
- 连接 GitHub remote (`github` → `git@github.com:zzf/SimEnv.git`, 尚未 push)
- venv 构建脚本: `tools/build_with_venv.sh` (zzf/0704-build-with-venv)
- FAST-LIO2 workspace 文档修正: 明确 SimEnv 是 catkin workspace 根目录 (docs/0704-fast-lio2-workspace-docs)
- FAST-LIO2 编译环境审计: 静态检查全部通过，catkin_make 被 libtorch (C++ SDK) 阻塞 (feat/0704-fast-lio2-mapping)
- build_with_venv.sh: 现已支持自动检测 torch CMake prefix，TorchConfig.cmake 路径问题已解决；剩余阻塞: CUDA toolkit + livox_ros_driver
- build_with_venv.sh: 现已强制使用 gcc-11/g++-11 构建，CUDA host compiler 错误已消除；CMakeCache 需清理后重试
- 编译修复: unitree PIE, FAST_LIO C++17, livox shared_ptr/serialization/missing-includes 共6个文件修复 (fix/0704-fast-lio2-build-errors)
- FAST-LIO2 部署指南: 新增 `docs/slam/fast_lio2_deployment_guide.md`，系统整理部署流程、传感器配置映射、参数说明、编译环境、运行验证和排错指南 (docs/0706-fast-lio2-deploy-guide)

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
