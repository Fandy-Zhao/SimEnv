# Issue: Gazebo—unitree_guide—RL 时序对齐验证与修复

## Goal

以运行证据验证并最小化修复 Gazebo 仿真模式下 RL policy、history、状态快照、action/LowCmd 与仿真时间的对齐问题，同时保留实机模式合理的墙钟调度。

## Scope

- `src/unitree_guide/unitree_guide/unitree_guide/` 中 FSM、RL state、ROS I/O、时间类型与相关测试。
- `unitree_guide` Gazebo launch 的 `/use_sim_time` 配置。
- 诊断 CSV、RTF/pause/reset 实验与回归检查。
- `PROJECT_STATE.md`、`CHANGELOG.md`、`docs/module_status.md` 和任务报告。

## Non-scope

- 模型权重、训练参数、奖励函数、观测维度/排列、关节映射、action scale、Kp/Kd。
- Gazebo 场景内容、建筑生成物、FAST-LIO2 算法行为。
- 实机墙钟控制策略的重构。
- 推送远程、创建 PR、合并 `master`。

## Acceptance Criteria

1. 诊断能区分 policy wait 的仿真周期到达、墙钟超时、shutdown 和仿真时间回退。
2. 正常 RTF 0.15--0.55 下，未发生 wall overtime 时 policy 约为 50 Hz simulation time。
3. pause/stall 的 wall overtime 不推进 policy、history 或 action。
4. policy observation 来自可追踪的一致状态快照，action 通过完整快照发布，推理线程不直接逐关节写共享 `_lowCmd`。
5. history 不在相同 state sequence/sim timestamp 下重复追加，并与已验证的训练端初始化语义一致。
6. 仿真时间使用 64 位微秒，reset 后旧 history/action 不再生效。
7. 新增回归测试覆盖 wait、pause、reset、去重和 action snapshot；每阶段通过 `catkin_make`。
8. 记录运行实验的 RTF、policy/LowCmd/overtime/history 指标；无法运行的验收项明确报告原因与剩余风险。

## Risks

- Torch/ROS/Gazebo 依赖可能限制 TSan 或无显示运行测试。
- 当前工作区包含大量与本任务无关的用户修改和未跟踪依赖；必须显式暂存，避免混入提交。
- 状态快照改变并发边界，需确保不改变 policy 数学输入输出语义。
- reset 清理过多状态可能改变进入 RL 的训练语义；只清理跨时间纪元不应保留的状态。

## Expected Modules

- `unitree_guide` controller and ROS I/O
- Gazebo launch configuration
- timing diagnostics and regression tests
- project governance/status documentation
