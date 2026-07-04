# Module Status

## Overview

ROS1 Noetic 仿真工作区，含 8 个第一级 ROS 包。2026-07-04 生成。

> **2026-07-04 update**: 仓库远程配置更新 (新增 GitHub remote `zzf/0704-connect-github-remote`)。业务模块代码无变动。
> **2026-07-04 update**: 新增 `tools/build_with_venv.sh` venv 构建脚本 (`zzf/0704-build-with-venv`)。
> **2026-07-04 update**: 修正 FAST-LIO2 文档中 workspace 结构描述 (`docs/0704-fast-lio2-workspace-docs`)。

## Modules

| Module | Purpose | Status | Tests / Checks | Notes |
|--------|---------|--------|----------------|-------|
| `building_obstacles` | Scene generation, danger spawning, evaluation | stable | Smoke: `generate_competition_scene.py` + `evaluate_danger.py` CLI | 核心比赛逻辑; 无单元测试目录 |
| `building_generator_core` | Python core: layout, constraints, generation | stable | `nosetests` via catkin (3 tests) | 纯 Python 库，被 building_obstacles 依赖 |
| `building_generator_classic` | Gazebo export + door/elevator control runtime | stable | `nosetests` via catkin (2 tests) | 门/电梯控制的主要入口 |
| `building_generator_interfaces` | ROS message/service definitions | stable | 编译时类型检查 | `.msg` / `.srv` 定义，无运行时逻辑 |
| `unitree_guide` | A1 robot controller + RL locomotion | stable | 编译检查 + 启动冒烟测试 | `junior_ctrl` 为核心二进制 |
| `Mid360_imu_sim` | Livox Mid-360 LiDAR plugin | stable | 编译检查 + 话题发布检查 | Gazebo plugin，依赖 Gazebo 开发头文件 |
| `simenv_fast_lio2_integration` | FAST-LIO2 bridge: adapter, config, launch | new | py_compile, roslaunch --files | External dep: FAST_LIO must be cloned separately |
| `uav_simulator` (5 sub-pkgs) | UAV local sensing, mapping, SO3 control | experimental | 编译检查 (含部分测试) | 与地面机器人大赛解耦，为独立实验功能 |
| `tools/` | 构建和仓库治理工具脚本 | new | `bash -n` 语法检查 + 权限检查 | `build_with_venv.sh` 提供 venv 版 catkin 构建 |
| SimEnv workspace | 整体集成 | stable | `catkin_make` + `./auto.sh` 冒烟测试 | 比赛入口 |

## Risks

- `uav_simulator` 子模块与比赛目标解耦，维护成本高且无明确 owner；建议移出到独立仓库或归档
- `Mid360_imu_sim` 仅提供编译检查，无运行时自动化验证；依赖硬件特定的点云格式
- 所有 Python 包的测试覆盖率偏低（仅 core 和 classic 有少量单元测试）
