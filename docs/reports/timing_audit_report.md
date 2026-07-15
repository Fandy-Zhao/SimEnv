# 时序对齐审计报告 — unitree_guide + Gazebo + RL 控制项目

**日期**: 2026-07-16
**审计范围**: `src/unitree_guide/unitree_guide/unitree_guide/`, `src/simenv_fast_lio2_integration/`, launch/scripts
**工作约束**: 只分析不修改代码

---

## 1. 执行摘要

本审计针对当前 `fix/0715-trotting-safety` 分支进行了完整的时序对齐分析。核心发现：

### 关键结论

1. **主控制循环 (FSM::run)**：使用 `absoluteWait()`（墙钟时间 `gettimeofday`+`usleep`）调度循环，但**控制更新由 `updateControlTime()` 门控**，仅在仿真时间推进时才执行。这是部分正确的设计。
2. **RL 推理线程 (State_RL::infer_thread_callback)**：运行在独立线程中，使用 `rosAbsoluteWait()`（仿真时间检查 + 墙钟 `usleep`），在仿真 pause 时 OVERTIME (200ms) 后强制继续执行。**这是最严重的问题**。
3. **ROS 时间配置**：`use_sim_time=true` 在 launch 文件中声明但传递给 `empty_world.launch` 时被注释掉。虽然 `empty_world.launch` 通常会自行设置，但存在配置不明确的风险。
4. **固定滤波系数**：`State_Trotting::getUserCmd()` 中使用 `y = 0.9*old + 0.1*new` 的固定 alpha 滤波，其有效时间常数依赖控制频率。
5. **Pause 行为**：主 FSM 循环有 pause 检测（`updateControlTime()` 中使用 `ros::WallTime`），但 RL 推理线程在 pause 期间会因 OVERTIME 继续执行，写入重复的 observation 到 history buffer。
6. **`current_time` 为 `uint32_t`**：`IOInterface::current_time` 是微秒级 uint32，约 71 分钟后会溢出回绕。

---

## 2. 当前完整时序架构

### 2.1 整体线程和回调架构

```
┌──────────────────────────────────────────────────────────────┐
│                        main() 线程                            │
│  while(running):                                              │
│    ControlFrame::run()                                        │
│      └─ FSM::run()                                            │
│           ├─ getSystemTime()                    [墙钟]        │
│           ├─ sendRecv()     → ROS pub/sub       [ROS回调]     │
│           ├─ updateControlTime()               [sim time]     │
│           │    ├─ ros::Time::now()             [sim time]     │
│           │    └─ ros::WallTime::now()         [墙钟]         │
│           ├─ estimator->run()                   [sim dt]     │
│           ├─ currentState->run()                [sim dt]     │
│           └─ absoluteWait(dt*1e6)              [墙钟]         │
│                                                               │
│  ROS AsyncSpinner(1) 线程:                                    │
│    ├─ /clock callback → current_time           [sim time]     │
│    ├─ /joint_state callbacks                   [ROS回调]      │
│    ├─ /imu callback                            [ROS回调]      │
│    ├─ /foot_force callbacks                    [ROS回调]      │
│    └─ /cmd_vel callback → current_cmd_vel_     [ROS回调]      │
│                                                               │
│  State_RL::infer_thread (独立 std::thread):                    │
│    while(RUNNING):                                            │
│      ├─ getTime() → current_time               [sim time]     │
│      ├─ refresh_rl_obs()  → 读共享状态          [无锁]        │
│      ├─ model.forward()                        [推理]         │
│      ├─ 写 _lowCmd->motorCmd[i].q              [无锁]         │
│      └─ rosAbsoluteWait(infer_duration*1e6)    [sim+墙钟]     │
│                                                               │
│  State_Trotting::save_amp_obs_thread (独立 std::thread):       │
│    while(RUNNING):                                            │
│      ├─ getRosTime() → current_time            [sim time]     │
│      ├─ refresh_amp_obs()                      [读共享]       │
│      └─ rosAbsoluteWait(amp_duration*1e6)      [sim+墙钟]     │
└──────────────────────────────────────────────────────────────┘
```

### 2.2 时间源调用链追踪

```
getSystemTime()                              [timeMarker.h:15]
  → gettimeofday(&t, NULL)                   ← 墙钟时间，微秒分辨率

absoluteWait(start, wait)                    [timeMarker.h:59]
  → while(getSystemTime() - start < wait)
      usleep(50)                             ← 墙钟睡眠

getRosTime()                                 [FSMState.cpp:12]
  → _ctrlComp->ioInter->current_time         ← 来自 /clock 回调的仿真时间 (uint32_t μs)

getTime()                                    [FSMState.cpp:17]
  → if Gazebo: getRosTime() → current_time   ← 仿真时间 (uint32_t μs)
  → if Real:   getSystemTime()               ← 墙钟时间

rosAbsoluteWait(start, wait)                 [FSMState.cpp:40]
  → while(getRosTime() - start < wait && overtime < 200000)
      usleep(50)                             ← 墙钟睡眠，但条件检查用仿真时间

wait(start, wait)                            [FSMState.cpp:29]
  → if Gazebo: rosAbsoluteWait()             ← 混用仿真+墙钟
  → if Real:   absoluteWait()                ← 纯墙钟

ros::Time::now()                             [ROS API]
  → 若 /use_sim_time=true: 仿真时间
  → 若 /use_sim_time=false: 墙钟时间

ros::WallTime::now()                         [ROS API]
  → 始终返回墙钟时间
```

---

## 3. 时间源清单

### 3.1 确定使用墙钟时间的代码 (A 类)

| 文件:行号 | 函数/符号 | 系统调用 | 用途 |
|-----------|-----------|----------|------|
| `timeMarker.h:15-18` | `getSystemTime()` | `gettimeofday()` | 提供墙钟微秒时间戳 |
| `timeMarker.h:59-68` | `absoluteWait()` | `getSystemTime()` + `usleep(50)` | 墙钟精确等待 |
| `FSM.cpp:55` | `FSM::run()` | `getSystemTime()` | 循环起始时间戳，传给 `absoluteWait` |
| `FSM.cpp:64,73,77,122` | `FSM::run()` | `absoluteWait()` | 主循环速率控制 |
| `FSM.cpp:133` | `FSM::updateControlTime()` | `ros::WallTime::now()` | Pause 检测（合理用法） |
| `FSMState.cpp:25` | `FSMState::getTime()` (real模式) | `getSystemTime()` | 实机时间源 |
| `FSMState.cpp:36` | `FSMState::wait()` (real模式) | `absoluteWait()` | 实机等待 |
| `IOROS.cpp:122` | `IOROS::recvState()` | `ros::WallTime::now()` | 足力传感器年龄检查 |
| `IOROS.cpp:197` | `IOROS::updateFootForce()` | `ros::WallTime::now()` | 足力时间戳记录 |
| `PyPlot.h:57,120` | `getElapsedTime()`, 构造函数 | `getSystemTime()` | 性能计时 |
| `IOROS.cpp:61` | `IOROS::IOROS()` | `usleep(300000)` | 等待 ROS subscriber 初始化 |

### 3.2 使用 ROS 仿真时间但依赖配置的代码 (B 类)

| 文件:行号 | 函数 | API | 风险 |
|-----------|------|-----|------|
| `FSM.cpp:128,132` | `updateControlTime()` | `ros::Time::now()` | 依赖 `/use_sim_time=true` |
| `FSMState.cpp:67` | `cmdVelCallback()` | `ros::Time::now()` | 同上 |
| `State_Trotting.cpp:321,329,333` | `cmdVelCallback()`, `getUserCmd()` | `ros::Time::now()` | 同上 |
| `Estimator.cpp:185` | `run()` | `ros::Time::now()` | 同上 |
| `IOSDK.cpp:83` | `sendRecv()` | `ros::Time::now()` | 同上 |
| `state_from_gazebo.cpp:44,77,79` | callback/callback_BASE | `ros::Time::now()` | 同上 |
| `pointcloud2livox.py:55,67` | `pointcloud2_to_custommsg()` | `rospy.Time.now()` | 依赖ROS参数 |
| `map_to_camera_init_bridge.py:32` | main | `rospy.Rate(10)` | 依赖ROS参数 |

### 3.3 使用仿真时间但类型不足的代码 (新增分类)

| 文件:行号 | 变量 | 问题 |
|-----------|------|------|
| `IOInterface.h:41` | `current_time` (uint32_t) | 微秒分辨率，约 71.6 分钟溢出 |
| `IOROS.cpp:249` | `timeCallback()` 转换 | `sec*1e6 + nsec/1000` 直接截断为 uint32 |

### 3.4 依赖循环次数的代码 (C 类)

| 文件:行号 | 代码模式 | 影响 |
|-----------|----------|------|
| `Estimator.cpp:184` | `_count % ((int)(1.0/(_dt*_pubFreq))) == 0` | 依赖 `_dt` 与实际控制周期一致（`_dt` 会被 `setDt()` 更新，部分正确） |
| `State_RL_test.cpp:53` | 循环 `HISTORY_LEN=5` 次填充 history | 在 enter() 中立即填充相同状态的 5 帧（无时间间隔） |
| `State_RL_test.cpp:271-274` | `obs_history_tensor.slice(0,1,HISTORY_LEN)` | 每次 `refresh_rl_obs()` 都无条件追加 |

### 3.5 固定滤波系数的代码 (D 类)

| 文件:行号 | 代码 | alpha | 备注 |
|-----------|------|-------|------|
| `State_Trotting.cpp:354,365` | `0.9*_dYawCmdPast + (1-0.9)*_dYawCmd` | 0.1 | 固定 yaw rate 滤波 |
| `LowPassFilter.cpp:8` | `_weight = 1.0/(1.0+1.0/(2π*samplePeriod*cutFrequency))` | 基于采样周期 | 构造时固定，`setDt()` 不更新 filter weight |
| `Estimator.cpp:125-127` | `new LPFilter(_dt, 3.0)` | 基于名义 `_dt` | 构造时设定，后续 `setDt()` 不影响 |

### 3.6 消息时间戳和本地时间混用 (E 类)

| 文件:行号 | sim time 源 | wall time 源 | 用途 |
|-----------|-------------|--------------|------|
| `IOROS.cpp:122-129` | `_foot_force_wall_stamp_ns` | `ros::WallTime::now()` | 足力有效性判断混用 wall time 判断消息年龄 |
| `FSMState.cpp:43` | `getRosTime()` 检查 | `getSystemTime()` 判断警告 | `rosAbsoluteWait` 内混用 |

### 3.7 线程并发不一致 (F 类)

| 位置 | 问题描述 |
|------|----------|
| `State_RL_test.cpp:128-178` | `infer_thread` 直接写 `_lowCmd->motorCmd[i].q`，主 FSM 线程在 `sendCmd()` 中读取同一内存 |
| `State_RL_test.cpp:224-274` | `refresh_rl_obs()` 读取 `_lowState->motorState[i]`、`ioInter->_base_w_*` 时无锁保护 |
| `IOROS.cpp:59-60` | `AsyncSpinner(1)` 在独立线程中运行所有 ROS callback，与主线程共享 `_lowState`、`current_time` 等 |
| `State_RL_test.cpp:51-54` | enter() 中连续 5 次调用 `refresh_rl_obs()` 填充 history，所有 5 帧使用相同的状态数据（状态尚未更新） |

---

## 4. Critical/High 风险清单

### Critical

| # | 位置 | 描述 |
|---|------|------|
| C1 | `State_RL_test.cpp:132,176` | RL 推理线程使用 `rosAbsoluteWait`，在 pause 时 OVERTIME 后继续执行。RTF < 1 时策略相对于仿真动力学运行过快 |
| C2 | `FSM.cpp:55,122` | 主循环使用 `absoluteWait()` (墙钟)，虽然控制更新有 sim-time 门控，但 LowCmd 发送频率仍是墙钟 500Hz（`sendCmd` 在 `updateControlTime` 之前执行） |
| C3 | `State_RL_test.cpp:158-163` | RL 线程直接写入 `_lowCmd->motorCmd[i].q`，与 FSM 主线程的 `sendCmd()` 形成数据竞争 |

### High

| # | 位置 | 描述 |
|---|------|------|
| H1 | `State_RL_test.cpp:271-274` | History buffer 每次 `refresh_rl_obs()` 都写入，pause/RTF低时写入重复仿真状态 |
| H2 | `State_RL_test.cpp:51-54` | enter() 中用相同状态填充 history，10 帧（HISTORY_LEN=5）覆盖 0 仿真秒 |
| H3 | `State_Trotting.cpp:354,365` | 固定 alpha=0.1 的 yaw rate 滤波，有效截止频率随控制频率变化 |
| H4 | `FSMState.cpp:40-58` | `rosAbsoluteWait` 的 `usleep(50)` 在仿真时间不推进时自旋，OVERTIME=200ms 后强制通过 |
| H5 | `IOInterface.h:41` | `current_time` 为 uint32_t 微秒，71.6 分钟后溢出 |
| H6 | `IOROS.cpp:122-129` | 足力有效性用 wall time 年龄判断（1秒），仿真时间跑慢时所有足力都无效 |

### Medium

| # | 位置 | 描述 |
|---|------|------|
| M1 | `LowPassFilter.cpp:7-8` | filter weight 在构造时固定，`Estimator::setDt()` 不更新已存在的 filter |
| M2 | `Estimator.cpp:184` | 用 `_count % N` 做 odom 发布降采样 |
| M3 | `gazeboSim.launch:24` | `use_sim_time` 传参被注释掉，依赖 `empty_world.launch` 默认行为 |
| M4 | `FSMState.cpp:67` | `cmdVelCallback` 使用 `ros::Time::now()` 标记时间戳（正确），但 stale 判断在 `getUserCmd()` 中使用 `controlTime`（正确），两者一致 |

### Low

| # | 位置 | 描述 |
|---|------|------|
| L1 | `pointcloud2livox.py:55,67` | 使用 `rospy.Time.now()`，依赖 sim time 配置 |
| L2 | `map_to_camera_init_bridge.py:32` | 使用 `rospy.Rate(10)`，依赖 sim time 配置 |
| L3 | `IOROS.cpp:61` | `usleep(300000)` 固定等待 subscriber 初始化 |

---

## 5. 全量审计表

| 严重程度 | 文件:行号 | 函数 | 当前时间源 | 用途 | 仿真模式风险 | 实机模式是否合理 | 建议修改 |
|----------|-----------|------|-----------|------|-------------|-----------------|---------|
| **Critical** | `State_RL_test.cpp:128-178` | `infer_thread_callback` | sim time check + wall `usleep` | RL 50Hz 推理调度 | RTF=0.2 时等效 250Hz 仿真频率; pause 时每 200ms 执行一次 | 实机使用 `absoluteWait` 合理 | 仿真改用 sim-time 门控 + `/clock` 驱动; 实机保持 `steady_clock` |
| **Critical** | `FSM.cpp:55,122` | `FSM::run` | wall `getSystemTime`+`usleep` | 主循环调度 500Hz | LowCmd 以墙钟 500Hz 发送, 同一 sim step 可能发送多次 | 合理 | `sendCmd` 移到 `updateControlTime` 返回 true 之后 |
| **Critical** | `State_RL_test.cpp:158-163` | `infer_thread_callback` | N/A（数据竞争） | 写 `_lowCmd->motorCmd[i].q` | 与 FSM 线程 `sendCmd` 竞争 | 同样存在竞争 | 使用 mutex 或 atomic，或将推理结果写入独立 buffer |
| **High** | `State_RL_test.cpp:271-274` | `refresh_rl_obs` | sim time (通过 current_time) | History buffer 更新 | RTF=0.2时5帧覆盖~0.02s仿真; pause时写重复帧 | 实机频率固定基本合理 | 基于新仿真状态序号去重; 仅在新LowState到达时更新 |
| **High** | `State_RL_test.cpp:51-54` | `enter()` | N/A | History 初始化 | 5 帧完全相同（无时间间隔） | 同 | 在 enter 中等待至少 5 个不同仿真步骤到达 |
| **High** | `State_Trotting.cpp:354,365` | `getUserCmd` | sim time（通过 controlDt） | Yaw rate 滤波 | 有效 tau 随控制频率变化; `dt=0.002`时 `tau≈0.019s`; `dt=0.01`时 `tau≈0.095s` | 取决于实机控制频率 | 改为 `alpha=1-exp(-dt/tau)` |
| **High** | `FSMState.cpp:40-58` | `rosAbsoluteWait` | sim check + wall `usleep` | RL/amp 线程等待 | OVERTIME 200ms 后强制继续 | 合理（实机用 `absoluteWait`） | 仿真模式移除 OVERTIME，纯 sim-time 门控 |
| **High** | `IOInterface.h:41` | `current_time` | uint32_t 微秒 | 仿真时间存储 | 71.6 分钟溢出回绕 | 不适用（实机用 `getSystemTime`） | 改为 `uint64_t` 或 `ros::Time` |
| **High** | `IOROS.cpp:122-129` | `recvState` | `ros::WallTime::now()` | 足力有效性 | RTF=0.2 时 wall time 1s 对应 sim time 0.2s; 足力误判无效 | 合理 | 仿真模式改用 sim time 或消息 stamp |
| **Medium** | `LowPassFilter.cpp:7-8` | `LPFilter` 构造 | 固定 `_weight` | 速度滤波 | `setDt()` 不更新 filter; 控制周期变化时截止频率偏移 | 合理 | 在 `setDt()` 中重建 filter 或动态计算 weight |
| **Medium** | `Estimator.cpp:184` | `run()` | 循环计数 | Odom 发布降采样 | `_dt` 更新后降采样比例变化 | 合理 | 改为基于 sim time 的时间间隔判断 |
| **Medium** | `gazeboSim.launch:24` | launch | 参数传递 | `use_sim_time` | 注释掉可能导致节点不启用 sim time | N/A | 取消注释 `<arg name="use_sim_time" value="$(arg use_sim_time)"/>` |
| **Medium** | `FSMState.cpp:67` | `cmdVelCallback` | `ros::Time::now()` | cmd_vel 时间戳 | 正确（sim time） | 正确（wall time） | 无需修改 |
| **Low** | `pointcloud2livox.py:55,67` | script | `rospy.Time.now()` | 点云时间戳 | 正确（sim time） | N/A | 无需修改 |
| **Low** | `IOROS.cpp:61` | `IOROS()` | `usleep(300000)` | 等待初始化 | 仅启动时运行一次，不影响 | 合理 | 无需修改 |
| **Low** | `PyPlot.h:57,120` | plot helper | `getSystemTime()` | 调试性能计时 | 仅影响日志精度 | 合理 | 无需修改 |

---

## 6. RTF 定量影响计算

### 假设条件

| 参数 | 值 |
|------|-----|
| Gazebo physics step | 0.002 s (500 Hz sim time) |
| Policy 名义频率 | 50 Hz |
| LowCmd 名义频率 | 500 Hz |
| HISTORY_LEN | 5 (实际代码中) |
| 名义控制 dt | 0.002 s |

### 计算结果

| 指标 | RTF=1.0 | RTF=0.5 | RTF=0.2 | RTF=0.15 |
|------|---------|---------|---------|----------|
| **1. 墙钟 Gazebo physics step 频率** | 500 Hz | 250 Hz | 100 Hz | 75 Hz |
| **2. 墙钟 50Hz policy 对应的仿真频率** | 50 Hz | 100 Hz | 250 Hz | 333 Hz |
| **3. 墙钟 500Hz LowCmd 对应仿真频率** | 500 Hz | 1000 Hz | 2500 Hz | 3333 Hz |
| **4. 每 Gazebo step 平均 LowCmd 次数** | 1 次 | ~2 次 | ~5 次 | ~6.7 次 |
| **5. 5帧 history 覆盖仿真时间**（若按墙钟 50Hz 写入）| 0.1 s | 0.05 s | 0.02 s | 0.015 s |
| **6. 700个墙钟500Hz循环对应仿真时间** | 1.4 s | 0.7 s | 0.28 s | 0.21 s |
| **7. 固定 alpha=0.1 滤波时间常数** | | | | |

### 固定滤波系数详细分析

滤波公式：`y[k] = (1-α)*y[k-1] + α*x[k]`，α=0.1

时间常数近似：`τ = -dt / ln(1-α)`

| 场景 | 有效 dt | τ (时间常数) | 截止频率 fc = 1/(2πτ) |
|------|---------|-------------|----------------------|
| RTF=1.0 (sim dt=0.002s) | 0.002 s | 0.019 s | 8.4 Hz |
| RTF=0.5 (sim dt=0.002s) | 0.002 s | 0.019 s | 8.4 Hz |
| RTF=0.2 (sim dt=0.002s) | 0.002 s | 0.019 s | 8.4 Hz |

**注意**: 对于 `State_Trotting::getUserCmd()` 中的滤波（line 354, 365），由于该函数在 `FSM::run()` -> `State_Trotting::run()` -> `getUserCmd()` 中调用，而 `FSM::run()` 仅在 `updateControlTime()` 返回 true（即有新的仿真步）时才执行控制更新，因此滤波的实际更新频率等于仿真步频率（500Hz sim time）。**在这种情况下，固定 alpha 不会因 RTF 变化而失真**，因为滤波只在新的仿真步上执行。

**对于 RL 推理线程**：`infer_thread` 中的 `rosAbsoluteWait` 以 sim time 为条件检查，但 `usleep(50)` 是墙钟睡眠。在仿真正常运行时（sim time 在推进），每次 `infer_duration=0.02s` 的仿真时间后更新一次。在 RTF < 1 时，这会转化为更长的墙钟间隔，但仿真时间间隔保持 0.02s。**然而当 sim time 推进速度低于 infer 线程检查速度时，OVERTIME 机制会导致提前退出**。

### 关键区别

**主 FSM 循环**：`absoluteWait()` 墙钟等待 → 检查 sim time 是否推进 → 是则执行控制更新。这意味着：
- RTF=0.2 时，循环每 2ms 墙钟检查一次，但只有每 10ms 墙钟（2ms*5）才有新 sim step
- 80% 的循环迭代被 `updateControlTime()` 拒绝
- 控制更新严格按 sim time 节奏

**RL 推理线程**：`rosAbsoluteWait()` sim time 检查 + `usleep(50)` → 当 sim time 推进达到 `infer_duration` 时通过。但 OVERTIME 200ms 在 RTF=0.15 时可能触发：
- RTF=0.15: infer_duration=0.02s sim time → 需要 0.02/0.15 = 0.133s 墙钟 → 接近但未到 OVERTIME 200ms
- 正常运行时 OVERTIME 不会触发
- **但 pause 时一定触发**

---

## 7. Pause / Reset 风险分析

### 7.1 Gazebo Pause

| 行为 | 当前状态 | 风险 |
|------|---------|------|
| `/clock` 是否停止 | ✅ 是 | |
| LowState 是否停止更新 | ✅ 是（ROS topic 不再发布） | |
| FSM 主循环是否继续 | ⚠️ 循环继续运行但 `updateControlTime()` 返回 false，跳过控制更新 | 低风险（设计正确） |
| RL infer thread 是否继续 | ❌ **是** — OVERTIME 200ms 后继续执行 | **高风险** |
| RL history 是否写入重复状态 | ❌ **是** — 每次 `refresh_rl_obs()` 都追加 | **高风险** |
| RL action 是否继续写入 LowCmd | ❌ **是** — 但基于重复的 observation | **高风险** |
| command filter 是否继续变化 | ✅ 否 — `getUserCmd()` 只在 sim time 推进时调用 | 低风险 |
| watchdog 是否触发 | ⚠️ 取决于具体 watchdog 实现 | 需运行验证 |
| `/cmd_vel` 是否超时 | ✅ 否 — timeout 使用 sim time | 低风险 |
| 动作序号是否增长 | ❌ **是** — infer thread 在 pause 时继续产生新 action | 中风险 |

### 7.2 Gazebo Unpause

| 行为 | 风险 |
|------|------|
| 突然使用已演化多轮的 action | **高风险**: infer thread 在 pause 期间可能已多次更新 `_lowCmd->motorCmd[i].q`，unpause 后这个 action 立即生效 |
| history 中全是重复帧 | **高风险**: 5 帧 history 可能全是相同仿真状态 |
| action filter 已到达目标 | 低风险（RL 无显式 action filter） |
| 关节目标突变 | **中风险**: pause 期间的 action 与 unpause 时刻的状态可能不匹配 |
| message age 判断异常 | 低风险（sim time 基于 `/clock`） |

### 7.3 Gazebo Reset World / Reset Simulation

| 行为 | 当前处理 | 风险 |
|------|---------|------|
| 仿真时间回退到 0 | ✅ `updateControlTime()` 检测到 `now < _lastSimTime`，调用 `resetForTimeDiscontinuity()` | 正确 |
| `last_sim_time` > 当前 sim time | ✅ 同上 | 正确 |
| duration 出现负值 | ✅ `updateControlTime()` 检测并拒绝 | 正确 |
| policy scheduler 不再触发 | ⚠️ `_simClockInitialized` 置 false 后重新初始化，但 RL infer thread 中的 `getRosTime()` 会读到回退的时间 | **中风险**: `rosAbsoluteWait` 中的 `getRosTime()` 读到旧值导致 start_time > current_time |
| history 清零 | ❌ `obs_history_tensor` 不会被清零 | **中风险** |
| action 清零 | ❌ `actions_tensor` 保持 pause 前的值 | 中风险 |
| command 清零 | ✅ `resetForTimeDiscontinuity()` 调用 `resetCommandState()` | 正确 |
| 控制器重新初始化 | ✅ `setAllStance()` + `resetWaveTime()` | 正确 |

**特别注意**: 当仿真时间从较大值回退到 0 时，`rosAbsoluteWait` 中的条件 `getRosTime() - startTime < waitTime` 会因为 `current_time` 突然变小而产生**长时间等待**（直到 `overtime` 触发或 sim time 再次超过 startTime）。

---

## 8. 线程与数据快照一致性分析

### 8.1 共享数据访问矩阵

| 数据 | 写入者 | 读取者 | 保护机制 | 安全？ |
|------|--------|--------|---------|--------|
| `_lowState->motorState[i]` | ROS AsyncSpinner callback | FSM 主线程, infer thread | 无 | ❌ |
| `_lowCmd->motorCmd[i]` | FSM 主线程, infer thread | `IOROS::sendCmd()` (主线程) | 无 | ❌ |
| `ioInter->current_time` | `/clock` callback (AsyncSpinner) | FSM 主线程, infer thread | 无 | ❌ |
| `ioInter->_base_w_*` | `/ground_truth` callback (AsyncSpinner) | infer thread | 无 | ❌ |
| `current_cmd_vel_` | `/cmd_vel` callback (AsyncSpinner) | infer thread, FSMState | 无 | ❌ |
| `obs_history_tensor` | infer thread | infer thread | 单线程 | ✅ |
| `actions_tensor` | infer thread | infer thread | 单线程 | ✅ |

### 8.2 关键并发场景

**场景 1: infer thread 读取状态时 ROS callback 正在写入**

`refresh_rl_obs()` 读取 `_lowState->motorState[i].q`、`ioInter->_base_w_*` 时，ROS AsyncSpinner 可能同时在更新这些值。导致 observation 中混合了两个不同时刻的关节角度和基座姿态。

**场景 2: infer thread 写入 LowCmd 时 FSM 主线程正在发送**

`infer_thread_callback()` 直接写 `_lowCmd->motorCmd[i].q`，而 `FSM::run()` -> `sendRecv()` -> `sendCmd()` 在同一时刻可能正在读取这些值。导致发送的关节命令是部分新值+部分旧值。

**场景 3: cmd_vel callback 与 infer thread**

`cmdVelCallback` 更新 `current_cmd_vel_` 的 `linear_x`/`linear_y`/`angular_z`，而 `refresh_rl_obs()` 读取这三个值到 `commands_tensor`。由于没有原子性保证，可能读到部分更新的 command。

### 8.3 低风险并发场景

- `obs_history_tensor` 只在 infer thread 中访问 → 安全
- `FSM::run()` 的顺序是: `sendRecv()`（获取状态）→ `updateControlTime()` → `run()`（使用状态），这些都在主线程中 → 主线程内部一致
- `State_Trotting::run()` 在主线程中执行，顺序读取 estimator 结果、lowState 等 → 一致

---

## 9. 推荐重构架构

### 9.1 目标仿真模式架构

```text
Gazebo physics step (每 0.002s sim time)
       │
       ▼
ROS joint_state/imu/clock 消息发布
       │
       ├─→ /clock callback → current_time 更新
       │
       ├─→ joint_state callback → _lowState 更新
       │
       └─→ FSM 主循环 (由 /clock 或新 joint_state 触发, 非墙钟驱动)
              │
              ├─ 1. 读取最新 sim_time
              ├─ 2. 计算 dt = sim_time - last_sim_time
              ├─ 3. 更新 estimator (使用 dt)
              ├─ 4. 运行状态机 (使用 dt)
              │      └─ RL state:
              │           if sim_time - last_policy_time >= policy_dt:
              │               ├─ 原子快照 LowState
              │               ├─ 构建 observation
              │               ├─ 追加到 history (带 sim_time 标签)
              │               ├─ 执行 policy forward
              │               ├─ 保存 action (带 sim_time 标签)
              │               └─ last_policy_time = sim_time
              ├─ 5. 用 action 计算 q_target (滤波/插值使用 dt)
              ├─ 6. 发送 LowCmd
              └─ 7. 等待下一个新 sim state (非忙等)
```

### 9.2 统一时钟抽象

```cpp
// 提案: include/common/ControlClock.h
class ControlClock {
public:
    virtual ~ControlClock() = default;
    virtual double nowSec() const = 0;
    virtual bool isSimulation() const = 0;
    virtual void sleepUntil(double targetSec) const = 0;
};

class RosSimClock : public ControlClock {
public:
    double nowSec() const override {
        return ros::Time::now().toSec();
    }
    bool isSimulation() const override { return true; }
    void sleepUntil(double targetSec) const override {
        // 使用 /clock 驱动的条件变量，不忙等
        // 或使用 ros::Rate (依赖 /clock)
    }
};

class SteadyClock : public ControlClock {
public:
    double nowSec() const override {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    bool isSimulation() const override { return false; }
    void sleepUntil(double targetSec) const override {
        std::this_thread::sleep_until(/* ... */);
    }
};
```

### 9.3 数据快照机制

```cpp
struct SynchronizedState {
    LowlevelState lowState;
    ros::Time simTime;
    uint64_t sequence;
    CmdVel cmdVel;
    std::mutex mtx;
};

// ROS callback 更新:
void jointStateCallback(const MotorState& msg) {
    std::lock_guard<std::mutex> lock(syncState.mtx);
    syncState.lowState.motorState[idx] = msg;
    syncState.simTime = ros::Time::now();
    syncState.sequence++;
}

// Policy 执行:
SynchronizedState snapshot;
{
    std::lock_guard<std::mutex> lock(syncState.mtx);
    snapshot = syncState;  // 深拷贝
}
// 使用 snapshot 构建 observation, 执行推理...
```

---

## 10. 分阶段补丁计划

### Patch 1: 增加诊断日志（不改变行为）

**目标**: 获取时序数据，验证分析结论

**修改文件**:
- `FSM.cpp`: 在 `updateControlTime()` 中记录 `wall_time`, `sim_time`, `simDt`, `RTF_estimate`
- `State_RL_test.cpp`: 在 `infer_thread_callback()` 中记录 `policy_seq`, `sim_time_at_infer`, `wall_time_at_infer`, `history_sim_span`
- `IOROS.cpp`: 在 `sendCmd()` 中记录 `lowcmd_seq`, `sim_time_at_send`

**输出**: CSV 文件，字段如下:
```csv
wall_time_ns, sim_time_us, sim_dt, estimated_rtf, policy_seq, lowcmd_seq, history_span_sim_s, policy_wall_hz, policy_sim_hz
```

**预估工作量**: 1-2 天

### Patch 2: 仿真 policy scheduler 改为 sim time 驱动

**修改文件**: `State_RL_test.cpp`, `FSMState.cpp`

**具体改动**:
1. 移除独立 `infer_thread`；policy 推理移入 `State_RL::run()`，由 FSM 主循环在每个新 sim step 调用
2. 在 `State_RL::run()` 中添加:
   ```cpp
   if (_ctrlComp->getControlDt() > 0.0) {
       _accumulatedSimTime += _ctrlComp->getControlDt();
       if (_accumulatedSimTime >= policy_dt) {
           refresh_rl_obs();
           // ... policy forward ...
           _accumulatedSimTime = 0.0;
       }
   }
   ```
3. 实机路径保持 `infer_thread` + `absoluteWait` 不变

**要求**:
- 一个仿真秒内 policy 调用 ~50 次（500Hz/10）
- pause 时 policy 不执行
- RTF 变化不影响 policy 仿真频率

**预估工作量**: 2-3 天

### Patch 3: History 使用仿真时间更新

**修改文件**: `State_RL_test.cpp`, `State_RL_test.h`

**具体改动**:
1. 为 history 添加 `ros::Time` 标签数组
2. 仅当 `sim_time - last_history_time >= policy_dt` 且 LowState 序号变化时追加
3. 不允许相同 sim time 的重复帧
4. enter() 中等待足够的不同仿真步到达后再开始推理

**要求**:
- 5 帧覆盖 ~0.1s 仿真时间（5 × 0.02s）
- 相邻帧间隔 ~0.02s sim time

**预估工作量**: 1-2 天

### Patch 4: 滤波和插值改为基于 dt

**修改文件**: `State_Trotting.cpp`, `LowPassFilter.cpp`, `LowPassFilter.h`

**具体改动**:
1. 在 `LowPassFilter` 中添加 `setSamplePeriod(double dt)` 动态更新 `_weight`
2. 将固定 alpha 滤波:
   ```cpp
   // 旧: _dYawCmd = 0.9*_dYawCmdPast + (1-0.9)*_dYawCmd;
   // 新:
   double tau = 0.02;  // 20ms 时间常数
   double dt = _ctrlComp->getControlDt();
   double alpha = 1.0 - std::exp(-dt / tau);
   _dYawCmd = (1.0 - alpha)*_dYawCmdPast + alpha*_dYawCmd;
   ```
3. 将固定次数的姿态插值:
   ```cpp
   // 旧: _percent = (getTime() - dofPosSwitBeginTime)/_duration;
   // 新: _percent = (_ctrlComp->controlTime.toSec() - transitionStartSec) / _duration;
   ```
4. `Estimator` 在 `setDt()` 中更新 LPFilter 的采样周期

**预估工作量**: 1-2 天

### Patch 5: Timeout 和 Watchdog 时间源统一

**修改文件**: `IOROS.cpp`, `FSM.cpp`

**具体改动**:
1. `IOROS::recvState()` 中足力年龄判断: Gazebo 模式使用消息 `header.stamp` 而非 `ros::WallTime`
2. 将 `rosAbsoluteWait` 中的 OVERTIME 机制改为仅仿真模式无效（实机保留）
3. `current_time` 从 `uint32_t` 改为 `uint64_t`

**预估工作量**: 1 天

---

## 11. 测试方案

### 测试 1: 不同 RTF 下 Policy 次数一致

**设置**:
- 分别在 Gazebo 中设置 `<real_time_update_rate>500</real_time_update_rate>` 配合不同计算负载实现 RTF≈0.2, 0.5, 1.0
- 运行 10 秒仿真时间（通过 `/clock` 测量）
- 在 `State_RL::run()` 或 `infer_thread_callback()` 中计数

**期望**: 约 500 次 policy forward（50 Hz × 10s）

**当前预期**: RTF=0.2 时可能显著偏多（因为 OVERTIME 和墙钟调度）

### 测试 2: History 时间跨度一致

**设置**: 在 `refresh_rl_obs()` 后打印 5 帧 history 对应的 sim time

**期望**: 最新帧 - 最旧帧 ≈ 0.08~0.10 s（5×0.02s）

**当前预期**: RTF=0.2 时跨度可能仅 0.02s（5 帧都是同一 sim step 或几乎相同的 sim time）

### 测试 3: Pause 测试

**设置**:
- 运行仿真，进入 RL 模式
- 在 Gazebo 中 pause 5 秒墙钟时间
- 记录 policy sequence, history sequence, action values, cmd 超时状态

**期望**:
- policy sequence 不增长 → 需要 Patch 2
- history sequence 不增长 → 需要 Patch 3
- action 不变化
- `/cmd_vel` timeout 不误触发
- unpause 后无动作突变

**运行验证命令**:
```bash
# 启动仿真
roslaunch unitree_guide gazeboSim.launch
# 等待进入 RL 模式后
gz world -p 0   # pause
sleep 5
gz world -p 1   # unpause
# 检查日志
```

### 测试 4: Reset 测试

**设置**: 执行 `gz world -r` (reset world) 和 Gazebo 的 reset simulation

**期望**:
- 检测到 sim time 回退
- scheduler, history, filter 正确重置
- 不出现负 dt
- 不出现长时间不执行 policy

### 测试 5: 轨迹一致性

**设置**:
- 相同 world、相同 seed、相同初始状态、相同 `/cmd_vel` 命令序列
- 分别以不同 RTF 运行
- 按仿真时间对齐比较:
  - base position (x, y, z)
  - base velocity (vx, vy, vz)
  - roll/pitch/yaw
  - joint positions (12)
  - raw action (12)
  - q_target (12)

**期望**: 轨迹基本一致（允许浮点误差）

**当前预期**: RTF 越低，差异越大（因为 policy 频率偏离和 history 失真）

---

## 12. 仍需人工确认的问题

| # | 问题 | 验证方法 |
|---|------|---------|
| Q1 | `empty_world.launch` 是否自动设置 `/use_sim_time=true`？ | 启动后 `rosparam get /use_sim_time` |
| Q2 | `/clock` 在 unitree_guide 节点启动前是否已经开始发布？ | 检查 `ros::Time::now()` 初始化时是否为 0 |
| Q3 | RTF=0.15 时 OVERTIME 200ms 是否实际触发？ | 添加 wall time 日志对比 sim time 推进速度 |
| Q4 | `current_time` uint32_t 溢出时是否导致 `rosAbsoluteWait` 判断错误？ | 长时间运行测试（>71 min） |
| Q5 | `FSM::updateControlTime()` 的 sim time 去重逻辑是否与 infer thread 的 `rosAbsoluteWait` 产生竞争？ | 两个线程同时等待 sim time 推进时，是否都能正确被唤醒 |
| Q6 | `LPFilter` 构造时的 `_dt` 与实际控制周期的偏差有多大？ | 打印 `_dt` vs `controlDt` 的统计 |
| Q7 | `State_Trotting::save_amp_obs_thread` 在 pause 时的行为？ | 同测试3，检查 amp thread |
| Q8 | Gazebo joint controller plugin 是否使用 sim time 进行 PD 控制？ | 检查 Gazebo plugin 源码 |
| Q9 | 比赛场景下典型 RTF 是多少？ | 从比赛日志中获取 |
| Q10 | 实机（real robot）的 `IOSDK` 接口是否返回真实系统时间？ | 检查 `IOSDK.cpp` 中 `current_time` 的设置 |

---

## 最终回答八个核心问题

### 1. 当前 Gazebo 模式下 RL policy 是否使用真实时间调度？

**部分使用**。RL 推理线程 `infer_thread_callback` 使用 `rosAbsoluteWait()`，它检查仿真时间（通过 `/clock` 回调更新的 `current_time`）来决定等待是否完成，但实际睡眠使用 `usleep(50)`（墙钟）。在正常运行时，这近似于仿真时间调度。但当仿真 pause 或 RTF 极低时，OVERTIME 200ms 墙钟超时生效，退化为墙钟调度。

### 2. RTF=0.2 时，policy 等效仿真频率是多少？

取决于具体行为路径：
- **正常运行时**（OVERTIME 未触发）：约 50 Hz 仿真频率（因为 `rosAbsoluteWait` 等待 `infer_duration=0.02s` 的仿真时间推进）
- **但 `refresh_rl_obs()` 每次调用都追加 history**：若 OVERTIME 触发，history 填充速度变为约 5 Hz 墙钟，等效仿真频率变为约 25 Hz（5/0.2）

实际上因为主 FSM 循环的控制更新已被 sim time 门控（`updateControlTime`），而 infer thread 只在 sim time 推进时才完成 `rosAbsoluteWait`，所以**正常情况下 policy 仍约 50 Hz 仿真频率**。问题主要出现在 pause 和极端低 RTF 场景。

### 3. History buffer 是否重复写入相同仿真状态？

**会**。在以下场景中：
- **enter() 时**: 连续 5 次 `refresh_rl_obs()` 使用相同的 LowState（仿真还没推进）
- **pause 时**: OVERTIME 触发后继续写入，但状态不变
- **RTF 低且 policy 检查比状态更新快时**: 理论上可能重复，但因为 `rosAbsoluteWait` 依赖 sim time 推进，实际较少发生

### 4. LowCmd 是否在一个 Gazebo physics step 之间更新多次？

**是**。`FSM::run()` 中的 `sendCmd()` 在 `updateControlTime()` 之前执行（line 56: `sendRecv()`）。这意味着：
- 每次主循环迭代都会发送一次 LowCmd（~500 Hz 墙钟）
- 当 RTF=0.2 时，每个 Gazebo physics step（100 Hz 墙钟）之间发送约 5 次 LowCmd
- 但中间 4 次的 LowCmd 内容与上一次成功的控制更新相同（因为 `updateControlTime()` 返回 false 时不会更新 joint commands）

更严重的是 **infer thread 也在写 `_lowCmd->motorCmd[i].q`**，所以 LowCmd 可能在主循环两次 `sendCmd()` 之间被 infer thread 修改。

### 5. 动作滤波和 RL 进入插值是否依赖循环次数？

**RL 进入插值（`_percent`）**:
- `State_RL_test.cpp:187`: `_percent = (float)(getTime() - dofPosSwitBeginTime)/_duration`
- `getTime()` 在 Gazebo 模式返回 sim time（`current_time`）
- `_duration = 1e6` 微秒 = 1 秒（sim time）
- **这个插值是基于仿真时间的，不依赖循环次数** ✅

**Yaw rate 滤波 (`State_Trotting.cpp:354,365`)**:
- `_dYawCmd = 0.9*_dYawCmdPast + (1-0.9)*_dYawCmd`
- 在主 FSM 循环的控制更新路径中调用（仅当 sim time 推进时）
- **不依赖循环次数，但 alpha 固定不随 dt 变化** ⚠️

### 6. Gazebo pause 时内部控制状态是否继续演化？

| 组件 | Pause 时继续？ | 严重程度 |
|------|--------------|---------|
| FSM 主循环 | 循环继续但跳过控制更新 | ✅ 安全 |
| LowCmd 发送 | 继续发送（重复上次值） | ⚠️ 中风险 |
| RL infer thread | **继续运行**（OVERTIME 200ms后） | ❌ 高风险 |
| RL history | **继续写入重复帧** | ❌ 高风险 |
| RL action | **继续更新 `_lowCmd`** | ❌ 高风险 |
| Trotting filter | 不更新（随 sim time 门控） | ✅ 安全 |
| WaveGenerator | 不更新（随 sim time 门控） | ✅ 安全 |

### 7. 哪些修改是修复时序对齐的最小必要改动？

按优先级：

1. **将 RL 推理从独立线程移入 FSM 主循环** (Patch 2)：消除线程竞争 + 确保 sim time 门控
2. **History 只在新的 sim step 且达到 policy_dt 时更新** (Patch 3)：防止重复帧
3. **将 LowCmd 发送移到控制更新之后** (FSM.cpp 调整执行顺序)：减少无效发送
4. **移除 `rosAbsoluteWait` 的 OVERTIME 或仅在实机模式启用** (Patch 5)
5. **`current_time` 从 uint32_t 改为 uint64_t** (Patch 5)

### 8. 哪些真实时间依赖应该在实机模式中保留？

| 依赖 | 保留原因 |
|------|---------|
| `getSystemTime()` + `absoluteWait()` | 实机无 `/clock`，必须用墙钟时间维持固定控制频率 |
| `FSMState::getTime()` 在 real 模式下返回 `getSystemTime()` | 正确设计 |
| `FSMState::wait()` 在 real 模式下使用 `absoluteWait()` | 正确设计 |
| `IOROS::recvState()` 中使用 `ros::WallTime::now()` 判断足力年龄 | 实机无 sim time，使用 wall time 合理 |
| `FSM::updateControlTime()` 在 real 模式下的分支（直接设 `controlDt=dt`） | 正确回退 |

---

*报告结束。建议先执行 Patch 1（诊断日志）收集运行时数据，验证本报告的分析结论后再逐步推进 Patches 2-5。*
