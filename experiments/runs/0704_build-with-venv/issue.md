# Issue: 新增 venv 版 catkin 构建脚本

## Task Goal

新增 `tools/build_with_venv.sh`，统一使用项目 `.venv` 构建 catkin workspace，解决 `catkin_make` 构建过程中需要 `torch` 等 Python 依赖时解释器不一致的问题。

## Modification Scope

- `tools/build_with_venv.sh` — 新增构建脚本
- `README.md` — 补充 venv 构建说明
- `PROJECT_STATE.md` — 更新当前工作状态
- `CHANGELOG.md` — 记录变更
- `docs/module_status.md` — 更新工具模块状态
- `experiments/runs/0704_build-with-venv/` — 任务记录

## Non-Scope

- 不安装 torch
- 不创建 `.venv`
- 不修改业务代码
- 不修改 ROS package
- 不修改 FAST-LIO2 集成代码
- 不 push
- 不 merge main/master
- 不删除文件

## Acceptance Criteria

1. `tools/build_with_venv.sh` 存在且可执行
2. 脚本从仓库根目录运行
3. 检查 `/opt/ros/noetic/setup.bash` 是否存在
4. 检查 `.venv/bin/python` 是否存在
5. source ROS Noetic
6. source `.venv/bin/activate`
7. 运行 `catkin_make -DPYTHON_EXECUTABLE="$PWD/.venv/bin/python"`
8. 构建完成后 source `devel/setup.bash`（如果存在）
9. 不在根目录生成临时文件
10. 文档说明如何创建 `.venv --system-site-packages`
11. 缺失 ROS 或 `.venv` 时有清晰错误提示

## Risk Points

- 用户可能没有创建 `.venv`
- 用户可能没有安装 ROS Noetic
- 用户可能没有安装 torch
- `catkin_make` 可能因为业务依赖失败
- source `devel/setup.bash` 只影响脚本子进程，不会改变父 shell 环境

## Expected Impacted Modules

- SimEnv workspace（构建工具链）
