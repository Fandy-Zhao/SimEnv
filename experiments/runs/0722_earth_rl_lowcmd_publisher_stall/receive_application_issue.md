# Issue: LowCmd Receive/Application Cadence

Date: 2026-07-22
Branch: fix/0722-earth-rl-lowcmd-publisher-stall
Baseline: 28d0d649 on top of candidate integration 94188231

## Goal

Resolve or precisely classify the LowCmd receive-to-Gazebo-joint-application
cadence drop where RL zero publishes about 500 Hz in simulation time but the
joint controller receive/apply diagnostics previously reported about 333.33 Hz.

## Scope

- Instrument the Gazebo joint command path for T1 callback entry, T2 latest
  command buffer write, T3 controller update read, and T4 effective joint
  command application.
- Keep diagnostics low overhead and distinguish control-cycle application rate
  from new-payload rate.
- Run FixedStand 5 sim-s and RL zero 8 sim-s using earth normal physics.

## Non-Scope

- No policy path, Torch model, observation, IMU fallback, action scale, history,
  Kp/Kd, or physics profile changes.
- No publisher rate increase, queue expansion, backlog replay, or master merge.

## Acceptance Criteria

- FixedStand and RL zero T1-T4 rates are 475-525 Hz by simulation time.
- T0 publisher remains 475-525 Hz where applicable.
- T1-T4 sequence loss is <= 3%, out-of-order is 0, and max effective apply gap
  is <= 10 ms.
- RL zero remains stable for at least 8 sim-s with IMU policy input selected.

## Risk Points

- Existing per-event CSV writes may perturb the measured receive/apply cadence.
- MotorCmd has no header or sequence field, so end-to-end correlation must use
  local sequence plus simulation time and payload hash rather than message
  protocol changes.
