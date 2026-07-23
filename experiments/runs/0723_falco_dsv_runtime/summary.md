# FALCO + DSV Real Runtime Summary

Date: 2026-07-23

## Verdicts

| Gate | Verdict |
|---|---|
| Governance | `GOVERNANCE_PASS` |
| R0 interface audit | `R0_INTERFACE_AUDIT_PASS` |
| R1 build | `R1_BUILD_PASS` |
| R2 FAST-LIO2 real data | `R2_FAST_LIO_REAL_DATA_PASS` |
| R2 TF | `R2_TF_PASS` |
| R3 FALCO real input | `R3_FALCO_REAL_INPUT_FAIL` |
| R3 motion gate | `R3_MOTION_GATE_PASS` |
| R4 FALCO + Trotting closed loop | `NOT_RUN` |
| R5 DSV real map | `NOT_RUN` |
| R5 DSV to FALCO | `NOT_RUN` |
| R6 DSV + FALCO + Trotting | `R6_DSV_FALCO_TROTTING_NOT_RUN` |
| Navigation disable stop | `NOT_RUN` |
| Command timeout stop | `NOT_RUN` |
| Trotting state gate | `NOT_RUN` |
| Scope audit | `SCOPE_AUDIT_PASS` |

Overall verdict: `FALCO_DSV_RUNTIME_TIMING_BLOCKED`

## Key Results

- Initial R2 attempt failed because `fast_lio` was not available in this
  worktree's ROS package path.
- Running `tools/external_deps/prepare_fast_lio2_deps.sh --prepare` restored
  ignored `src/FAST_LIO` and `src/livox_ros_driver` symlinks from fixed,
  clean external sources.
- Formal rebuild after dependency preparation passed and built
  `devel/lib/fast_lio/fastlio_mapping`.
- R2 retry passed: `/laserMapping` published `/Odometry` and
  `/cloud_registered`; relays published `/state_estimation` and
  `/registered_scan` at about 10 Hz.
- R2 message validity passed: odometry pose finite, quaternion norm `1.0`,
  stamps increased, registered cloud nonempty with `23148` points.
- Actual frames: odometry `header.frame_id=camera_init`,
  `child_frame_id=body`; registered cloud `header.frame_id=camera_init`.
- Actual TF uses `base`, not `base_link`; `map -> base`,
  `camera_init -> base`, `base -> laser_livox`, and `map -> camera_init`
  were available.
- R3 required a launch fix because the real FAST-LIO2 relays publish global
  `/state_estimation` and `/registered_scan`, while validation/DSV expect
  `/navigation/state_estimation` and `/navigation/registered_scan`.
- R3 retry connected real inputs to FALCO, but FALCO produced only a zero
  one-pose path and zero raw velocity for the tested manual waypoint.
- Gazebo RTF during R3 was about `0.062`; FAST-LIO2 output continuity was
  intermittent during startup, so motion tests were blocked by runtime timing
  and real-input planner behavior.

## Runtime Commands

Formal build:

```bash
./tools/build_with_venv.sh
```

Prepare ignored FAST-LIO2 source links when needed:

```bash
./tools/external_deps/prepare_fast_lio2_deps.sh --prepare
```

Unified simulation startup used for R2/R3:

```bash
FLOOR_COUNT=1 GUI=false ENABLE_RVIZ=0 START_VIRTUAL_JOY=0 \
START_CONTROLLER=1 ENABLE_FAST_LIO2=1 ENABLE_SENSOR_DATA=1 \
ENABLE_POINTCLOUD_CONVERTER=1 START_BUILDING_CONTROL=0 \
AUTO_UNPAUSE=1 AUTO_UNPAUSE_DELAY=6 FAST_LIO2_DELAY=5 \
TMUX_SESSION_PREFIX=simenv-r3 TERMINAL_BACKEND=tmux ./auto.sh
```

FALCO real-data launch:

```bash
NAV_MAX_LINEAR_X=0.10 NAV_MAX_ANGULAR_Z=0.20 \
  roslaunch simenv_navigation_bringup runtime_real_data.launch start_dsv:=false
```

Emergency stop:

```bash
rostopic pub -1 /navigation/enabled std_msgs/Bool "data: false"
rostopic pub -1 /navigation/stop_exploring std_msgs/Bool "data: true"
rostopic pub -1 /cmd_vel geometry_msgs/Twist "{}"
```

## Blockers

- `RTF/timing blocker`: R3 sampled `real_time_factor=0.062393412`.
- `FALCO configuration/runtime blocker`: with real registered cloud and manual
  waypoint, `/navigation/path` was not a useful nonzero path.
- `Trotting execution blocker`: R4 was not entered because R3 did not pass.
- `DSV configuration/runtime blocker`: R5/R6 were not entered because R3/R4
  prerequisites did not pass.
