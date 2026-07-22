# LowCmd 500 Hz Timing Notes

Date: 2026-07-22
Worktree: /home/zzf/search_ws/SimEnv_worktrees/earth-rl-lowcmd-500hz
Branch: fix/0722-earth-rl-lowcmd-500hz
Baseline: e7fbbe639412fbd528a2fc35dc3009aa18c9af83

## Static Trace

Required search:

```bash
rg -n "LowCmd|lowCmd|sendRecv|sendCmd|publish|absoluteWait|usleep|sleep|Rate|Timer|update|robot_state|lowState|control_dt" src/unitree_guide src/unitree_gazebo src/unitree_legged_control
```

The top-level `src/unitree_gazebo` and `src/unitree_legged_control` paths do
not exist in this worktree. The actual controller paths are:

- LowCmd construction/action states: `src/unitree_guide/unitree_guide/unitree_guide/src/FSM/State_FixedStand.cpp`, `src/unitree_guide/unitree_guide/unitree_guide/src/FSM/State_RL_test.cpp`.
- FSM scheduling and publish gate: `src/unitree_guide/unitree_guide/unitree_guide/src/FSM/FSM.cpp`.
- ROS publish bridge: `src/unitree_guide/unitree_guide/unitree_guide/src/interface/IOROS.cpp`.
- Gazebo receive/apply controller: `src/unitree_guide/unitree_ros/unitree_legged_control/src/joint_controller.cpp`.

Trace chain:

1. FixedStand/RL writes `LowlevelCmd`.
2. `FSM::run()` accepts simulation-time control steps through `ControlTimeScheduler`.
3. `IOROS::sendCmd()` publishes 12 `MotorCmd` topics and records `LOWCMD`.
4. `UnitreeJointController::setCommandCB()` receives one joint command.
5. `UnitreeJointController::update()` reads the latest command buffer and applies torque to the joint during Gazebo controller update.

## Change

Evidence showed ROS publish timing alone was insufficient: FixedStand publisher
median was 500 Hz, while controller receive/apply initially measured about
333 Hz with command sequence jumps. The minimal receive-path fix was:

- Set the Gazebo joint command subscriber queue depth to 1.
- Enable `tcpNoDelay()` on the high-rate command subscriber.
- Add opt-in diagnostics that distinguish `CMD_RECEIVE` and newly applied `CMD_APPLY` events for one representative joint.

No physics profile, policy observation/action semantics, `.pt` files, or shared
governance documents were edited.

## Commands

Build attempts:

```bash
./tools/build_with_venv.sh
./tools/build_with_venv.sh -j1
SIMENV_CATKIN_WHITELIST='unitree_legged_msgs;unitree_legged_control;unitree_guide' ./tools/build_with_venv.sh -j1
SIMENV_CATKIN_WHITELIST='unitree_legged_msgs;unitree_legged_control;unitree_guide' ./tools/build_with_venv.sh -DUNITREE_ENABLE_TORCH_POLICY=OFF -j1
SIMENV_CATKIN_WHITELIST='unitree_legged_msgs;unitree_legged_control' ./tools/build_with_venv.sh -j1
SIMENV_CATKIN_WHITELIST='unitree_legged_msgs;unitree_legged_control;unitree_guide' ./tools/build_with_venv.sh -DUNITREE_ENABLE_TORCH_POLICY=ON -j1
```

Runtime launches used the required mode plus diagnostic environment variables:

```bash
LOWCMD_APPLY_DIAGNOSTICS_ENABLED=1 \
LOWCMD_APPLY_DIAGNOSTICS_PATH=experiments/runs/0722_earth_rl_lowcmd_500hz/<scenario>_apply.csv \
LOWCMD_APPLY_DIAGNOSTICS_JOINT=FR_hip_joint \
WORLD_MODE=earth PHYSICS_PROFILE=normal GUI=False ./auto.sh
```

Publisher diagnostics were set before `junior_ctrl` startup:

```bash
rosparam set /timing_diagnostics_enabled true
rosparam set /timing_diagnostics_path experiments/runs/0722_earth_rl_lowcmd_500hz/<scenario>_publisher.csv
```

Analysis:

```bash
python3 experiments/runs/0722_earth_rl_lowcmd_500hz/analyze_lowcmd_timing.py \
  --publisher-csv experiments/runs/0722_earth_rl_lowcmd_500hz/<scenario>_publisher.csv \
  --apply-csv experiments/runs/0722_earth_rl_lowcmd_500hz/<scenario>_apply.csv \
  --output-json experiments/runs/0722_earth_rl_lowcmd_500hz/<scenario>_metrics.json
```

## Metrics

FixedStand before transport fix (`fixedstand_before_transport_metrics.json`):

- Published median: 500.0 Hz.
- Received median: 333.33 Hz.
- Applied-new median: 333.33 Hz.
- Published/applied median-rate difference ratio: 33.33%.
- Duplicate applied timestamp ratio: 0.0%.
- Max applied gap: 11 ms.
- Callback burst sequence jumps: 4186, max jump 5.

FixedStand after transport fix (`fixedstand_after_transport_metrics.json`):

- Published median: 500.0 Hz.
- Received median: 500.0 Hz.
- Applied-new median: 500.0 Hz.
- Published/applied median-rate difference ratio: 0.0%.
- Duplicate applied timestamp ratio: 0.0%.
- Max applied gap: 5 ms.
- Callback burst sequence jumps: 2, max jump 2.
- Tail FSM state: `2` (`FIXEDSTAND`).

RL zero whole run (`rlzero_metrics.json`):

- Published median: 500.0 Hz.
- Received median: 333.33 Hz.
- Applied-new median: 333.33 Hz.
- Published/applied median-rate difference ratio: 33.33%.
- Max applied gap: 57 ms.
- Callback burst sequence jumps: 2822, max jump 3.

RL zero isolated RL-state window after first `fsm_state=8` plus 2 s settling
(`rlzero_metrics_rl_window.json`):

- Published median: 333.33 Hz.
- Received median: 250.0 Hz.
- Applied-new median: 250.0 Hz.
- Published/applied median-rate difference ratio: 25.0%.
- Max applied gap: 12 ms.
- Callback burst sequence jumps: 3962, max jump 3.
- Tail FSM state: `8` (`RL`).

## Result

LOWCMD_500HZ_FAIL

FixedStand satisfies the 500 Hz applied LowCmd criteria after the minimal
transport/backlog fix. RL zero does not: in the isolated RL-state window the
generated/published chain itself falls below 500 Hz and the controller applies
new commands at about 250 Hz median with sustained gaps above 10 ms.

Root cause: there were two layers. First, the Gazebo controller command
subscriber accumulated high-rate TCPROS backlog/stale commands, fixed for
FixedStand by queue depth 1 and `tcpNoDelay()`. Second, the Torch RL runtime
still misses simulation-time 2 ms deadlines and feeds the controller below
500 Hz; this remains unresolved and needs a separate RL scheduling/load
investigation.
