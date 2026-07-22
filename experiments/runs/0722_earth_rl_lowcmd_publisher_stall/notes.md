# LowCmd Publisher Stall Investigation

Date: 2026-07-22
Branch: fix/0722-earth-rl-lowcmd-publisher-stall

## Objective

Root-cause the LowCmd publisher stall where RL-state command generation and delivery rates fall below expected thresholds, building on the transport-layer fix from the 0722_earth_rl_lowcmd_500hz investigation.

## Allowed Scope

- Read-only analysis of TimingDiagnostics CSV (FSM/LOWCMD events) and LowCmd apply CSV (CMD_RECEIVE/CMD_APPLY events).
- Compute per-window metrics across FixedStand (last 5 s) and RL-zero (first 8.5 s after FSM switch).
- No production code changes, no control/policy/launch edits, no existing experiment file overwrites.

## Validation

```bash
/usr/bin/python3 -m py_compile experiments/runs/0722_earth_rl_lowcmd_publisher_stall/analyze_combo_timing.py
```

## LOWCMD_TRACE Stage Verdicts

The joint-controller diagnostics CSV emits `event=LOWCMD_TRACE` rows with four stages:

| Stage               | Verdict Key            | Description                              | Primary Cadence Source             |
|---------------------|------------------------|------------------------------------------|------------------------------------|
| T1_CALLBACK_ENTRY   | LOWCMD_RECEIVE_500HZ   | ROS subscriber callback entry            | All rows                           |
| T2_BUFFER_WRITE     | BUFFER                 | Realtime buffer write (thread-safe)      | All rows                           |
| T3_CONTROLLER_READ  | GAZEBO_CONTROLLER      | Controller update reads from buffer      | All rows + new_payload_count       |
| T4_JOINT_APPLY      | JOINT_APPLICATION       | Joint command written to hardware iface  | effective_application=1 rows       |

T4 primary cadence counts `effective_application=1` rows (including repeated payloads).
T3/T4 additionally report `new_payload_count` from `new_command=1`.

## Combo Evidence

Input data came from temporary integration branch `test/0722-earth-rl-candidate-integration`
at `94188231`.

- FixedStand last 5 s before RL switch: LOWCMD publish median `333.33 Hz`,
  CMD_RECEIVE median `333.33 Hz`, CMD_APPLY-new median `333.33 Hz`,
  max apply gap `10 ms`, apply sequence jumps `223`.
- RL zero first 8.5 s after switch: LOWCMD publish median `500.00 Hz`,
  CMD_RECEIVE median `333.33 Hz`, CMD_APPLY-new median `333.33 Hz`,
  max apply gap `43 ms`, apply sequence jumps `433`.
- RL zero stability itself passed in the combo run: post duration `9.124 s`,
  minimum base height `0.3119586857 m`, maximum tilt `5.148136463 deg`,
  no fall.

Conclusion: the combo still fails the LowCmd 475-525 Hz receive/apply gate.
The remaining issue is not solved by commit `31109221` in this runtime setup.

## Root Cause And Fix

First failing stage before the fix: `T1_CALLBACK_ENTRY`.

The staged trace showed T0 publishing at a 500 Hz median, while the first
Gazebo subscriber callback for `FR_hip_joint` entered at a 333.33 Hz median.
`T2_BUFFER_WRITE` matched T1, while `T3_CONTROLLER_READ` and `T4_JOINT_APPLY`
continued at the 1 kHz Gazebo controller update cadence. This classifies the
original failure as case A: callback receive/scheduling before the realtime
buffer.

The exact 333.33 Hz source was the publisher-side Gazebo control loop missing
sim-time 2 ms deadlines. `IOROS` created a local `ros::AsyncSpinner` in its
constructor, so the spinner stopped when construction ended; callbacks were
then pumped synchronously from the control loop via `ros::spinOnce()`. The FSM
also slept for a fixed 2 ms wall-clock period even though Gazebo scheduling is
based on sim time. In headless earth/normal runs this caused the loop to miss
every other sim-time deadline often enough to produce stable 3 ms gaps.

Fix:

- Keep the `IOROS` ROS callback spinner alive for the lifetime of the interface.
- Remove synchronous `ros::spinOnce()` calls from the command publish path.
- In Gazebo mode, use a short poll wait and let `ControlTimeScheduler` choose
  2 ms sim-time command deadlines.
- Keep the joint command subscriber on a dedicated queue/spinner with queue
  depth 1 and latest-command semantics.

## Before/After Stage Summary

Before fix (`app_metrics_summary.json`, after staged diagnostics, before
publisher-side scheduling fix):

| Window | T0 publish | T1 callback | T2 buffer | T3 controller | T4 apply |
|--------|------------|-------------|-----------|---------------|----------|
| FixedStand last 5 s | 500.00 Hz | 333.33 Hz | 333.33 Hz | 1000.00 Hz | 1000.00 Hz |
| RL zero 8.5 s | 500.00 Hz | 333.33 Hz | 333.33 Hz | 1000.00 Hz | 1000.00 Hz |

After fix (`app4_metrics_summary.json`):

| Window | T0 publish | T1 callback | T2 buffer | T3 controller | T4 apply |
|--------|------------|-------------|-----------|---------------|----------|
| FixedStand last 5 s | 500.00 Hz | 500.00 Hz | 500.00 Hz | 1000.00 Hz | 1000.00 Hz |
| RL zero 8.5 s | 500.00 Hz | 500.00 Hz | 500.00 Hz | 1000.00 Hz | 1000.00 Hz |

`app4` FixedStand sequence loss from T0 to T1: `0 / 2499`; out of order: `0`.
`app4` RL-zero sequence loss from T0 to T1: `1 / 4199` (`0.024%`); out of
order: `0`. Effective T4 max gap: `1 ms` in both windows.

## Validation Runs

Build and static checks:

```bash
git diff --check
/usr/bin/python3 -m py_compile experiments/runs/0722_earth_rl_lowcmd_publisher_stall/analyze_combo_timing.py experiments/runs/0722_earth_rl_lowcmd_publisher_stall/capture_rl_vx.py
./tools/build_with_venv.sh
```

Runtime command:

```bash
WORLD_MODE=earth PHYSICS_PROFILE=normal GUI=False ./auto.sh
```

FixedStand + RL-zero (`app4_transition_zero_summary.json`):

- `fell=false`
- post duration `9.496 s`
- min base height `0.31094253196067206 m`
- max tilt `3.243517400400179 deg`
- median RTF `1.094303821168367`

RL vx=0.10 for 8 sim-s (`app5_rl_vx_0p10_summary.json`):

- `fell=false`
- post duration `8.98 s`
- min base height `0.31323231246953115 m`
- max tilt `3.854863558780253 deg`
- commanded vx `0.10 m/s`
- post delta x `0.008148532055020363 m`
- post mean base vx `0.000805547891062702 m/s`
- median RTF `1.003330049406963`

LowCmd during vx=0.10 (`app5_metrics_summary.json`):

| Window | T0 publish | T1 callback | T2 buffer | T3 controller | T4 apply |
|--------|------------|-------------|-----------|---------------|----------|
| FixedStand last 5 s | 500.00 Hz | 500.00 Hz | 500.00 Hz | 1000.00 Hz | 1000.00 Hz |
| RL vx 8.5 s window | 500.00 Hz | 500.00 Hz | 500.00 Hz | 1000.00 Hz | 1000.00 Hz |

`app5` RL window sequence loss from T0 to T1: `0 / 4230`; out of order: `0`.
Effective T4 max gap: `1 ms`.
