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

## Patch 2

- `WALL_OVERTIME` now returns control for diagnostics/shutdown checks but the
  policy thread continues waiting from the same simulation-time origin.
- `SIM_TIME_RESET` establishes a new wait origin and does not execute policy.
- Only `SIM_PERIOD_REACHED` permits the next observation/history/action cycle.
- Real-robot `absoluteWait()` behavior is unchanged.
- Headless pause regression: 14 `WALL_OVERTIME` events at constant simulation
  time 13.277 s reused policy/action sequence 87; policy delta, action delta,
  and history duplicate delta were all zero. `catkin_make -j` passed.

## Patch 3/4: State and Action Snapshots

- `IOROS::recvState()` now publishes LowState, base quaternion/angular velocity,
  simulation timestamp, and state sequence as one mutex-protected snapshot.
- RL command input has its own mutex, sequence, and ROS timestamp.
- Torch inference reads local copies without holding either mutex.
- Inference publishes a complete 12-joint action/target snapshot; only
  `State_RL::run()` on the FSM thread applies it to LowCmd.
- The inference running flags are atomic and are set before thread creation.
- Headless interval 9.220--14.622 s: 265 policy/action generations, 7,241
  LowCmd sends, zero torn actions, zero source-trace mismatches, and zero
  LowCmd action-sequence regressions. `catkin_make --force-cmake -j` passed.

## Patch 5: History Alignment

- Normal history append now requires a different state sequence, a strictly
  increasing simulation timestamp, and at least 20,000 us since the previous
  policy/history update.
- The five repeated-current-observation rows in `enter()` are preserved because
  no training reset/history implementation is available in this repository or
  the exported TorchScript actor archive.
- Headless steady-state interval 10.222--14.464 s: 209 policy waits, history
  duplicate counter remained at the four entry-initialization duplicates,
  timestamp span averaged 81,325 us (80,000--84,000 us), and oldest history
  timestamps were strictly increasing. `catkin_make -j` passed.
