# Issue: Trotting/RL 单层运动建图验证

## Goal

在固定单层场景中分别以 Trotting（FSM 4）和 RL（FSM 6）接收相同
`/cmd_vel` 路线，验证机器人运动、停止安全与 FAST-LIO2 建图链路，并保存可复现证据。

## Scope

- 固定 `FLOOR_COUNT=1`、`SEED=77` 和机器人出生位姿。
- 每种状态使用全新仿真，执行 FixedStand、直行、左转、直行、停止。
- 采集 Gazebo 真值、FAST-LIO2 里程计、点云话题状态及地图点云/平面投影。
- 若发现明确的控制缺陷，只提交可通过回归验证的最小修复。

## Non-scope

- 不修改 FAST-LIO2 Stage 2 导航接口。
- 不调优 FAST-LIO2 算法参数或 RL 模型。
- 不修改或提交外部嵌套仓库，不操作主工作区脏内容。

## Acceptance criteria

1. `/fsm/state_cmd` 可进入 Trotting 和 RL；两者均收到同一路线的 `/cmd_vel`。
2. 真值位姿有限且机器人未翻倒；指令结束后发布零速度并验证停止趋势。
3. `/Odometry`、`/cloud_registered` 持续可用，优先保存 `/Laser_map` PCD。
4. 为两个状态生成地图证据和指标，报告明确 PASS/FAIL 与残余风险。

## Risks

- Gazebo 实时率低，ROS 仿真时间测试可能耗费较长墙钟时间。
- Trotting/RL 需要 Torch-enabled controller 构建。
- headless 环境无法使用 RViz 时需以点云俯视投影替代截图。

## Expected modules

- `unitree_guide`（只在发现并验证控制缺陷时修改）
- `simenv_fast_lio2_integration`（只读运行依赖）
- `experiments/runs/0717_trot-rl-floor-mapping/`
- `docs/reports/0717_trot-rl-floor-mapping.md`
- `PROJECT_STATE.md`
