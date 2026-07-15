# ADR-0716: Gazebo locomotion control advances only with simulation time

## Status

Accepted (2026-07-16).

## Context

The controller process is scheduled at a nominal wall-clock rate, while Gazebo
can run much slower than real time. At approximately `0.10` real-time factor,
Trotting previously completed a configured `0.75 s` height transition and
`0.20 s` readiness hold in only `0.10 s` of Gazebo time. `WaveGenerator` used
`getSystemTime()`, so its `0.45 s` period became roughly `0.045 s` in physics
time. Estimator propagation and desired-position integration had the same
fixed-`dt` mismatch.

## Decision

For the Gazebo platform, `/clock` is the locomotion control clock:

- the FSM runs wave generation, estimator propagation, and state control only
  when `ros::Time::now()` advances;
- the measured positive simulation delta is passed to the estimator and used
  for height, readiness, body-position, and yaw integration;
- `WaveGenerator` derives phase directly from ROS simulation timestamps;
- a pause lasting `0.5` wall seconds, a backward timestamp, or a forward delta
  above `0.05 s` resets gait time and commands all stance;
- a running Trotting wave latches an all-stance hold on unsafe roll/pitch,
  sustained loss of scheduled stance-foot contact, or non-finite output.

The wall clock remains appropriate only for detecting that an unchanged
simulation timestamp has stayed paused long enough to require a one-time safe
reset. Real-robot control retains its configured nominal period.

## Validation

At a forced Gazebo real-time factor of about `0.10`, Trotting entered at
`5.788 s` and became ready at `6.734 s`: `0.946 s` of simulation time and
`9.459 s` of wall time. A forward command then ran upright from approximately
`10.81 s` to `17.21 s` and moved about `2.08 m` in body-forward/world-+Y.

Pausing at `18.032 s` produced one gait-time reset and Wave cancellation. After
resume, readiness again required `0.946 s` of simulation time. A separate
force-injection test removed all four contacts; after exactly `0.080` simulated
seconds the controller cancelled Wave and stayed in the latched all-stance
path without continuing gait/IK calculation.

## Consequences

- Gait behavior is independent of Gazebo real-time factor and does not advance
  while physics is paused.
- State commands received during a pause remain pending until time advances.
- A discontinuity intentionally discards phase continuity and requires
  Trotting readiness to be re-established.
- The `0.05 s` forward-jump threshold and Trotting safety thresholds are ROS
  parameters and may need tuning for heavily loaded or terrain-rich scenes.
