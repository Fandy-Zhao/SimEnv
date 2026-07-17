# Issue: 单层 Trotting/RL 定速运动性能剖析

## Goal

在可重复的单层 Gazebo 场景中，分别测试 Trotting（FSM `4`）和 RL
（FSM `6`）对 `0.1`、`0.5`、`1.0 m/s` `/cmd_vel.linear.x` 指令的实测运动能力。
每个样本从全新仿真启动，使用 Gazebo 真值和 `/clock` 同时记录轨迹、仿真时间、
墙钟时间及停止表现。

## Scope

- 固定 `FLOOR_COUNT=1`、`SEED=77`、出生位姿 `(0, 2.3, 0.6, 1.5708)`。
- 六个独立样本：两种状态 × 三个前进速度；每次包含 FixedStand、定速直线段及零速段。
- 以 `sim_elapsed / wall_elapsed` 定义 real-time factor（RTF），以真值轨迹长度/仿真时间
  定义实际平均水平速度。
- 保存原始真值 CSV、单次 JSON、汇总 JSON/CSV、可复现绘图脚本和两张 PNG 图。

## Non-goals

- 不调参、重训 RL 策略或为了追求 1 m/s 修改安全限幅。
- 不启动 FAST-LIO2；本任务测量控制与 Gazebo 的当前性能，避免把建图算力混入 RTF。
- 不提交 `generated_building/`、`logs/`、`results/` 或任何外部嵌套仓库。

## Acceptance criteria

1. 六组均由新 Gazebo epoch 完成，且用 `/fsm/state_cmd` 和 `/cmd_vel` 实际下发。
2. 每组有有限 Gazebo 真值、仿真/墙钟区间、RTF、实际速度、跟踪比与停止速度。
3. 报告包含六轨迹平面图及 RTF/移动能力关系图，明确安全限制、异常和解释边界。

## Risks

- RTF 会随主机负载、GPU/驱动和启用节点改变，结果仅代表记录的运行配置。
- 1.0 m/s 是安全性能探测，不等同于控制器承诺的可跟踪速度。
- 需要 Torch-enabled `junior_ctrl`；本 worktree 以未跟踪、只读的已验证 `devel` 覆盖层运行。
