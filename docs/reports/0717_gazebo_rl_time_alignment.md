# Gazebo—unitree_guide—RL 时序对齐验证与修复报告

**日期**：2026-07-17

**分支**：`fix/0716-gazebo-rl-time-alignment`

**基线**：`master@0a459c81`

**范围**：Gazebo 仿真时钟、FSM、RL policy/history、状态/命令快照、LowCmd、pause/reset

## 1. 结论

本任务确认并修复了三类实际问题：Gazebo pause 时 policy 因墙钟超时继续推进；推理线程与 FSM 共享状态和 LowCmd 时可能产生混合 observation/torn action；reset simulation 后旧 history/action 可能跨 epoch 使用。修复后，policy 只有在仿真时间累计达到 20 ms 时生成新 action，pause 期间不推进，history 正常窗口约 80 ms，LowCmd 总是读取完整 action generation，时间回退会清理状态并拒绝仍在飞行的旧 epoch 推理结果。

现有证据覆盖两个实际低 RTF 区间：RTF 0.276 的 10.355 s 基线，以及平均 RTF 约 0.098 的 1,377.340 s 扩展回归。两者的 policy 仿真频率分别为 49.25 Hz 和 50.000 Hz。没有得到四个独立受控的 0.15/0.20/0.30/0.50 RTF 样本，因此本报告不宣称完成四档横向对比。

## 2. 根因与修复

| 问题 | 运行证据 | 修复结果 |
|---|---|---|
| `rosAbsoluteWait()` 在 pause 后因墙钟 OVERTIME 返回，旧循环把它当作下一 policy 周期 | 基线 pause：policy +46、action +45、history duplicate +44 | OVERTIME 仅用于诊断/退出检查，继续等待同一 sim-time 起点；5 s pause 中 policy/action 增量均为 0 |
| 推理线程直接读 callback 数据并写 12 路 `_lowCmd` | 基线 10.355 s 中检测到 28 次 LowCmd copy 与 action write 重叠 | 输入、command、输出均使用完整快照；只有 FSM 主线程写 LowCmd；后续 6,776,908 次 LowCmd torn=0 |
| history 可在相同状态或 pause 中重复追加 | 基线运行期和 pause 均出现重复 | 追加必须满足新 state sequence、递增 sim timestamp、间隔至少 20 ms；稳定窗口 80–84 ms，运行期 duplicate=0 |
| `uint32_t` 微秒时钟约 71.6 min 回绕，reset 后旧 history/action 残留 | 基线 reset 78.315 s→1.048 s 后保留旧状态 | 时钟改为 atomic `uint64_t`；回退使 snapshot/action/history 失效并在新 epoch 重建 |
| reset 与正在执行的 Torch 推理之间仍有极窄竞态 | post-commit 调用链审查发现 | 输出携带 `reset_generation`；FSM 拒绝与当前 epoch 不同的 action |
| `/use_sim_time` 传递不明确 | launch 静态检查 | 两个 Gazebo launch 显式传递 `use_sim_time` |

## 3. 关键测量

### 3.1 正常低 RTF

| 指标 | RTF 0.276 基线 | RTF 约 0.098 扩展回归 |
|---|---:|---:|
| action-source 仿真跨度 | 10.355 s | 1,377.340 s |
| policy/action 数 | 510 | 68,868 |
| policy 仿真频率 | 49.25 Hz | 50.000 Hz |
| WALL_OVERTIME | 0 | 5,047 |
| LowCmd 数 | 17,641 | 6,776,908 |
| torn action | 修复前 28 | 修复后 0 |
| history timestamp span | 80–84 ms | 稳态 80 ms |

扩展回归中的 5,047 次 OVERTIME 只记录“墙钟等待达到诊断阈值”，没有触发额外 action。它们对应低 RTF 或停顿期间反复返回检查，而下一次 policy 仍需满足 20 ms 仿真时间条件。

### 3.2 Pause、unpause 与 reset

- 修复后固定 sim time 13.277 s 的 pause 中出现 14 次 OVERTIME，但 policy、action、history 增量全部为 0。
- 后续 5 s 墙钟 pause 在 sim time 17.907 s 上，policy/action 保持 616/472，无新 action。
- reset 回归检测到一次 `SIM_TIME_RESET`；首个新 action 为 sequence 1，来源为新 epoch state/time 21/21,000 us。history 从 20 ms 预热到 80 ms，没有保留 reset 前 timestamp。
- epoch 补丁进一步保证：即使 reset 前已经开始的 Torch forward 在 reset 后才返回，也不能被 FSM 应用。

### 3.3 History 初始化与官方参考

本地官方参考 `/home/zzf/search_ws/unitree_rl/src/unitree_guide/unitree_guide/src/FSM/State_RL.cpp` 在 `enter()` 中同样先建立零 buffer，再连续调用 observation buffer update 填满历史。因此当前 `HISTORY_LEN=5` 的入口会产生 4 个同 timestamp duplicate，这是有意保持部署约定；它与运行期重复分开统计。运行后 history 只接受不同 state generation，5 个 timestamp 的“最新减最旧”验收值约为 80 ms。

## 4. 八个核心问题

1. **RTF 0.15–0.55 时 policy 仿真频率是多少？** 已实测 RTF 0.276 为 49.25 Hz、RTF 约 0.098 为 50.000 Hz，说明频率按仿真时间稳定。0.15/0.20/0.30/0.50 四档没有独立数据，不能外推成四个测量结果。
2. **OVERTIME 在什么条件发生？** 20 ms 仿真周期尚未达到，但 `rosAbsoluteWait()` 的墙钟诊断预算耗尽时发生；`usleep(50)` 的实际调度误差使名义 200 ms 常约为 400 ms。它现在不授权下一次推理。
3. **Pause 时 policy/history/action 是否推进？** 不推进。OVERTIME 计数可以增加，但三个 sequence 保持不变。
4. **LowCmd 数据竞争是否确认？** 已确认。基线观测到 28 次 copy/write 重叠；结构上也存在推理线程逐关节写、FSM 同时读的问题。
5. **是否观察到 torn action？** 修复前观察到探针重叠；修复后的快照链路中 6,776,908 次 LowCmd torn=0。
6. **History 初始化是否与训练/官方部署一致？** 与本地官方 `unitree_rl` 部署实现一致，均重复当前 observation 填充入口 history。由于没有训练环境源码，严格的训练端 episode wrapper 仍不可直接核验。
7. **Reset 前有哪些状态未清理？** 基线中的 history tensor/timestamp、action、policy scheduler 起点会跨 reset；现已清理/失效，并增加 reset generation 防止旧在途推理落入新 epoch。
8. **LowCmd 在 Gazebo step 间发送相同还是不同内容？** 主要重复相同完整 action generation。扩展回归 6,776,908 次发送中，6,708,040 次重复当前 generation，68,868 次切换到新 generation；无 sequence 回退或部分 action。

## 5. 验证结果

- `catkin_make -j`：通过，Torch policy 启用，`junior_ctrl` 构建成功。
- `catkin_make run_tests_unitree_guide_gtest_timing_alignment_test`：5/5 通过。
- `catkin_test_results build/test_results`：10 tests，0 errors，0 failures，0 skipped。
- 单元覆盖：正常推进、无推进/OVERTIME、shutdown、时间回退、跨原 uint32 边界、history 去重、并发 action 完整性、reset generation。
- 无 GUI 集成覆盖：正常低 RTF、pause/unpause、reset simulation、history warm-up、action/LowCmd generation 追踪。

## 6. 文件与提交

主要修改包括：

- `FSMState.cpp`、`FSM.cpp`：等待结果分类、仿真时间门控、pause/reset 处理。
- `State_RL_test.cpp/.h`：输入/command/action 快照、history gate、reset 清理和 epoch 校验。
- `IOROS.cpp/.h`、`IOInterface.h`：一致状态快照、atomic 64-bit 仿真时钟、LowCmd generation 诊断。
- `TimingDiagnostics.*`、`TimingAlignment.h`、`PolicySnapshots.h`：诊断与可测试的同步组件。
- Gazebo launch：显式 `use_sim_time`。
- `timing_alignment_test.cpp`：回归测试。
- `experiments/runs/0716_gazebo-rl-time-alignment/`：issue、notes、聚合 metrics；原始 CSV 保持本地未提交。

任务提交从 `3fadb3a3` 到 `d4c8907f`，按诊断、pause、快照、history、reset、测试和审查补丁分阶段完成。

## 7. 风险与后续

- 当前 `0.5 s` 墙钟 pause 判定在诊断 I/O 很重或 `/clock` 突发推进时可能误报；RL 的 pause hook 不改变 action/history，但 gait/stance reset 的阈值仍应参数化并基于实际负载调优。
- 状态快照保证一次拷贝中的字段一致，但各 ROS topic 本身没有统一传感器 header timestamp；若未来需要严格多传感器同步，应引入消息时间戳和同步策略。
- `final_rtf_timing.csv` 约 1.5 GiB，末行被进程结束截断；报告只使用 13,696,598 条完整记录。原始 CSV 不应提交 Git，建议后续压缩归档或只保留聚合指标。
- 若验收必须覆盖四档 RTF，应建立可控 physics update-rate 基准，分别采集至少 10 s 仿真时间；当前机器负载可能无法达到实际 RTF 0.50，届时应记录目标值与实测值。
- 本任务不修改 policy 权重、observation 定义/scale、动作映射、控制增益或 Gazebo 场景；RL 速度跟踪能力属于另一个功能验证任务。
