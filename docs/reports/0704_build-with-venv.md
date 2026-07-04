# Task Report: 0704 build-with-venv

## Branch

- 工作分支: `zzf/0704-build-with-venv`
- 分支类型: 维护/构建工具
- 是否遵守 `zzf/` 维护分支规则: 是
- 是否 merge 回 dev: 待执行

## Summary

新增 `tools/build_with_venv.sh`，统一使用项目 `.venv` 构建 catkin workspace。脚本自动检查 ROS Noetic 和 `.venv` 可用性，使用 `-DPYTHON_EXECUTABLE` 确保 CMake 使用一致的 Python 解释器。

README 已更新，包含 venv 创建步骤、torch 固定版本安装说明、以及构建后手动 source 的提示。

## Script Details

- 路径: `tools/build_with_venv.sh`
- ROS setup: `/opt/ros/noetic/setup.bash`
- venv Python: `$REPO_ROOT/.venv/bin/python`
- catkin_make 参数: `-DPYTHON_EXECUTABLE="$PWD/.venv/bin/python"`
- 额外参数透传: 支持 (`"$@"`)

## Files Changed

| File | Action | Reason |
|------|--------|--------|
| `tools/build_with_venv.sh` | Added | Core deliverable: venv-based catkin build script |
| `README.md` | Updated | Added "使用 venv 构建" section with setup and usage instructions |
| `PROJECT_STATE.md` | Updated | Recorded active work item |
| `CHANGELOG.md` | Updated | Added build tooling entry |
| `docs/module_status.md` | Updated | Added tools/ module row and update note |
| `experiments/runs/0704_build-with-venv/issue.md` | Added | Task issue record |
| `experiments/runs/0704_build-with-venv/notes.md` | Added | Experiment notes |

## Tests

| Test | Result | Notes |
|------|--------|-------|
| `bash -n tools/build_with_venv.sh` | PASS | Syntax valid |
| `test -x tools/build_with_venv.sh` | PASS | Executable |
| `shellcheck` | SKIPPED | Not installed; residual risk is low |

## Risks

- `.venv` 已存在但可能缺少 torch; 脚本会因业务依赖（如 torch）失败，但脚本本身的错误检查逻辑正确
- `shellcheck` 未安装，未执行静态分析
- 构建成功与否取决于业务包依赖是否满足; 脚本只保证环境一致性，不修复业务依赖问题

## Next Steps

- 在 `.venv` 中安装 torch 固定版本: `python -m pip install torch==2.0.1`
- 使用 `./tools/build_with_venv.sh` 执行完整构建
- 继续 FAST-LIO2 mapping 集成任务
