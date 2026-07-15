# Gazebo RL Timing Alignment Notes

## Baseline

- Branch: `fix/0716-gazebo-rl-time-alignment`
- Base: `master@0a459c81`
- Build: `catkin_make --force-cmake -j` passed with Torch policy enabled.
- Runtime: headless `unitree_guide/gazeboSim.launch`, `Building.world`, 1 ms physics step.
- ROS time: `/use_sim_time=true`; `/clock` advanced and stopped during pause.
- Diagnostics: `/timing_diagnostics_enabled=true`, CSV written to `baseline_timing.csv` (kept as an uncommitted local artifact because it is 27 MiB).

## Static Findings

- Normal callbacks are not serviced by a persistent `AsyncSpinner`: the spinner in `IOROS::IOROS()` is local and stops when the constructor returns. Runtime callbacks are primarily serviced by `ros::spinOnce()` in `IOROS::sendCmd()` on the FSM thread.
- The independent RL thread still races with the FSM thread while reading state/base/command/time and writing `_lowCmd`.
- No training source or reset/history wrapper exists in this repository. The TorchScript archive contains the actor network but not episode-reset history initialization, so deployment `enter()` history initialization must not be changed without external training evidence.

## Runtime Baseline Results

### Normal low RTF

- Simulation interval: 48.019--58.374 s (`10.355 s`).
- Wall interval: `37.567 s`.
- Estimated RTF: `0.276`.
- Policy waits: 510 `SIM_PERIOD_REACHED`, 0 `WALL_OVERTIME`.
- Policy simulation rate: `510 / 10.355 = 49.25 Hz`.
- LowCmd sends: 17,641 (`~1703.6` sends/simulation-second); these are mostly repeated command generations.
- Torn-action probe: 28 LowCmd copies overlapped an RL action write.
- History timestamp span after warm-up: average 81,286 us, min 80,000 us, max 84,000 us.
- History duplicate counter: 4, all introduced by the five same-timestamp `enter()` refreshes.

### Pause/stall

- Simulation time stayed at 70.993 s.
- Observed 46 `WALL_OVERTIME` exits during the extended pause.
- Policy sequence advanced by 46, action sequence by 45, history duplicate count by 44.
- Actual wall duration per overtime was about 0.40 s because `overtime += 50` assumes `usleep(50)` sleeps exactly 50 us.

### Reset simulation

- Simulation time moved backward from 78.315 s to 1.048 s.
- One wait returned `SIM_TIME_RESET` with `policy_wait_sim_elapsed_us=-78,501,000`.
- The policy/action had already run from pre-reset state before the wait detected the reset.
- The first post-reset history retained pre-reset timestamps/tensor content; the FSM called the base no-op `State_RL::onControlTimeReset()`.

## Evidence-based Patch Triggers

- P1 pause/OVERTIME: confirmed.
- P2 action data race/torn action: confirmed.
- P3 coherent policy input snapshot: static race confirmed; generation trace required in the fix.
- P4 history time alignment: pause duplicates confirmed; `enter()` semantics intentionally unchanged pending training evidence.
- P5 reset/time type: reset carry-over and 32-bit declaration confirmed.
