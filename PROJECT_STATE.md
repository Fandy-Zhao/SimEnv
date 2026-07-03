# Project State

## Snapshot
- Date: 2026-07-04
- Branch: master
- Project type: ROS1 Noetic + Gazebo Classic (robotics competition simulation)
- Current focus: 比赛仿真环境稳定运行 + 项目治理初始化

## Active Work
- 项目治理骨架初始化 (首次使用 project-governance skill)
- 随机场景生成稳定性
- 电梯开关门优化 (最近提交 `8191cff`)

## Known Risks
- 随机生成的建筑布局可能在某些参数组合下产生不可达房间或源重叠
- Gazebo Classic 已停止维护，长期可能需要迁移到 Ignition/Gazebo Fortress
- `junior_ctrl` 以前台方式运行时的键盘交互依赖终端输入，自动化 CI 中不易验证

## Validation Status
- Build: catkin_make 编译通过（最近提交已验证）
- Unit tests: `building_generator_core/test/` (3 tests), `building_generator_classic/test/` (2 tests)
- Smoke test: `./auto.sh` 可完成场景生成、Gazebo 启动和传感器初始化
- Evaluation: `evaluate_danger.py` 评分脚本可用

## Next Steps
- 补充 CI/自动化测试流程
- 评估将 UAV simulator 子模块独立或移除
- 文档迁移至 docs/ 子目录 (architecture.md, module_status.md 等)
