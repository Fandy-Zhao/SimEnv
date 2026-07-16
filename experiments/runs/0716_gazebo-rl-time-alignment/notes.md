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

## Patch 6: Reset and 64-bit Time

- `IOInterface::current_time` is now an atomic `uint64_t`; the `/clock`
  conversion performs 64-bit multiplication before addition and invalidates
  the coherent input snapshot when time moves backward.
- Pause keeps the last complete action and policy/history state unchanged.
  Backward or excessive-forward time discontinuities invalidate command/action
  snapshots, publish a measured-position hold, clear policy tensors and
  history timestamps, and restart the RL entry transition in the new epoch.
- `rosAbsoluteWait()` detects backward time inside the wait loop and returns
  `SIM_TIME_RESET` immediately instead of waiting for wall overtime.
- Both Gazebo launch files now explicitly pass `use_sim_time` to
  `empty_world.launch`; runtime confirmed `/use_sim_time=true`.
- Pause regression at fixed simulation time 17.907 s: over five wall seconds,
  policy/action stayed at 616/472 with zero delta.
- Reset regression: time moved from 18.487 s to the new epoch; one
  `SIM_TIME_RESET` was recorded. The old action was invalidated, the first new
  action used source state/time 21/21,000 us and action sequence 1, and history
  warmed from a 20,000 us span to the nominal 80,000 us span without retaining
  pre-reset timestamps.
- Failed history-gate polling no longer increments `policy_sequence`; the
  counter now represents completed inference calls. `catkin_make -j` passed.
- The opt-in diagnostics load caused very low/bursty RTF and the existing
  0.5 s wall pause detector emitted false pause warnings between `/clock`
  bursts. The RL pause hook is non-mutating, but pause detection based only on
  wall silence remains a documented tuning risk.

## Regression Tests

- Extracted the production wait-result classifier, history gate, and complete
  policy-output buffer into small testable components; State_RL/FSMState use
  those same components rather than parallel test-only implementations.
- `catkin_make -j` passed after integration.
- `catkin_make run_tests_unitree_guide_gtest_timing_alignment_test` passed all
  4 timing-alignment tests: normal advance, no-advance wall overtime, shutdown,
  time rollback, former uint32 boundary crossing, repeated/early state
  rejection, reset acceptance, and 10,000-generation concurrent action reads.
- `catkin_test_results build/test_results` reported 8 tests total, 0 errors,
  0 failures, and 0 skipped (including existing workspace results).

## Post-commit review (2026-07-17)

- Compared entry-history initialization with the local official reference at
  `/home/zzf/search_ws/unitree_rl`: its `State_RL::enter()` also starts from a
  zero buffer and repeatedly appends the current observation. The four
  same-timestamp startup duplicates are therefore intentional deployment
  parity, while runtime duplicates remain forbidden.
- Found a narrow reset race: an inference already running on a pre-reset
  snapshot could publish after the main thread invalidated the action buffer.
  Policy outputs now carry the reset generation, and the FSM rejects every
  output whose generation does not match the current simulation epoch.
- Streamed the uncommitted `final_rtf_timing.csv` rather than loading its
  1.5 GiB contents into memory. Of 13,696,599 data lines, the last line is a
  truncated process-ending record and was excluded. Across 1,377.340 s of
  action-source simulation time and 14,056.005 s wall time (average RTF
  approximately 0.098), 68,868 actions were produced at exactly 50.000 Hz
  simulation time. There were 5,047 diagnostic wall-overtime returns, but no
  extra action sequence, no sequence regression, and no torn LowCmd.
- The same extended run sent 6,776,908 LowCmd generations: 6,708,040 repeated
  the current action generation and 68,868 carried a changed generation. This
  confirms that wall-scheduled sends repeat the same complete command between
  simulation updates; they do not create partial or independently evolving
  policy actions.
- The extended file does not constitute four independently controlled RTF
  points. Together with the earlier RTF 0.276 interval it validates two actual
  low-RTF regimes; 0.15/0.20/0.30/0.50 must not be claimed as measured.
- Post-review validation: `catkin_make -j` passed; the timing target passed 5/5
  tests and `catkin_test_results build/test_results` reported 10 tests total,
  0 errors and 0 failures.
