# unitree_legged_control

Gazebo effort controllers for Unitree joints.

High-rate command subscriptions use `tcpNoDelay()` and queue depth 1 so each
joint controller prefers the newest available MotorCmd and avoids accumulating
stale callback backlog.

## LowCmd Apply Diagnostics

`UnitreeJointController` supports an opt-in CSV trace for validating effective
LowCmd timing at the controller application point. Set these ROS parameters or
equivalent environment variables before loading the controllers:

- `/lowcmd_apply_diagnostics_enabled` or
  `LOWCMD_APPLY_DIAGNOSTICS_ENABLED`: `true`/`1` to enable the trace.
- `/lowcmd_apply_diagnostics_path` or `LOWCMD_APPLY_DIAGNOSTICS_PATH`: output
  CSV path.
- `/lowcmd_apply_diagnostics_joint` or `LOWCMD_APPLY_DIAGNOSTICS_JOINT`: joint
  name or controller namespace to log; defaults to `FR_hip_joint`.

The trace records `CMD_RECEIVE` callback events and newly applied `CMD_APPLY`
controller update events for one representative joint so generated, published,
received, and applied LowCmd timing can be compared without logging all 12
joints or every repeated physics-step hold.
