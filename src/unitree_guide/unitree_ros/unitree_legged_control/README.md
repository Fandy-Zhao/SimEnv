# unitree_legged_control

Gazebo effort controllers for Unitree joints.

High-rate command subscriptions use `tcpNoDelay()` and queue depth 1 so each
joint controller prefers the newest available MotorCmd and avoids accumulating
stale callback backlog.

Command subscriptions run on a shared dedicated callback queue with an
`AsyncSpinner` so command receipt is not tied to other Gazebo or controller ROS
callbacks. The default command spinner thread count is `4`; override it with
`/lowcmd_command_spinner_threads` or `LOWCMD_COMMAND_SPINNER_THREADS` when
isolating transport behavior.

The ROS simulation control interface also keeps its subscriber callback spinner
alive for the lifetime of `IOROS`; state, clock, joystick, and FSM callbacks are
not pumped from the 500 Hz command publish loop. The default interface spinner
thread count is `2`; override it with `/unitree_ros_callback_spinner_threads`
or `UNITREE_ROS_CALLBACK_SPINNER_THREADS`.

In Gazebo mode the FSM loop uses a short poll wait (`unitree_gazebo_poll_wait_us`,
default `100 us`) and lets the sim-time control scheduler choose each 2 ms
command deadline. This avoids wall-clock sleeps causing missed sim-time LowCmd
publish slots when headless Gazebo runs ahead of wall time.

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

The trace records staged `LOWCMD_TRACE` rows for one representative joint:
`T1_CALLBACK_ENTRY`, `T2_BUFFER_WRITE`, `T3_CONTROLLER_READ`, and
`T4_JOINT_APPLY`. This keeps receive, realtime-buffer, controller-read, and
effective joint-application cadence separate without logging all 12 joints.
Repeated payloads are still counted at `T4_JOINT_APPLY` because the controller
can effectively apply the same latest command on multiple control cycles.
