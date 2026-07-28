# Static Audit: `auto.sh` event/state startup and optional RTF load

## Baseline

- Date: 2026-07-28 (Asia/Shanghai)
- Local `master`: `e2e0e4ec241f100b7873f318a0b4e28944d7e65e`
- Task branch: `refactor/0728-auto-event-driven-rtf-cleanup`
- Task worktree: `/home/zzf/search_ws/SimEnv_worktrees/auto-event-driven-rtf-cleanup`
- Root workspace was dirty only in user-owned generated scene/log/result artifacts plus an unrelated untracked report; none are used as task edits.
- Attachment path correction: FAST-LIO2 integration is `src/simenv_fast_lio2_integration/`, not `src/mapping/simenv_fast_lio2_integration/`.

## Current Startup Stages

1. Parse environment/defaults and reject unsupported world/physics values.
2. Check `junior_ctrl` executable before any scene mutation.
3. Broadly kill prior ROS/Gazebo/navigation processes, then fixed `sleep 2`.
4. Validate Python environment; source ROS and worktree overlay; validate external FAST-LIO packages.
5. Generate the competition scene (or select Earth world), export Gazebo paths, print configuration.
6. Optionally start virtual joystick.
7. Launch `multi_floor_gazeboSim.launch`; fixed `sleep 25`; accept readiness if the roslaunch PID is alive.
8. Optionally start building control without service confirmation.
9. Launch `junior_ctrl`; fixed `sleep 2`; schedule delayed unpause.
10. Fixed `sleep 3`, optional `FAST_LIO2_DELAY` (default 5), poll `/fsm/state_cmd` subscriber, then publish FixedStand directly once.
11. For FAST-LIO2 only, sample upright IMU up to six times; failure is warning-only.
12. Start scan adapter; fixed `sleep 2`; start FAST-LIO2; do not validate adapter/mapping output before optional RViz.
13. For navigation, validate incrementing stamps on `/Odometry` and `/cloud_registered`; launch supervisor and navigation; fixed `sleep 6`; publish safe defaults once; optional trotting/enable/exploration requests are separated by fixed `sleep 2` and are not confirmed.
14. Optionally start recorder without checking its mode-specific inputs.
15. Call final unpause and sample `/clock`; ignore final-unpause failure; remain alive until signal.

## Dependencies by Stage

| Stage | Existing dependency | Functional dependency required |
|---|---|---|
| environment | files, Python, packages | commands/files/overlay and writable outputs |
| ROS/Gazebo | roslaunch PID | ROS master, `/clock`, advancing clock, `/gazebo/get_world_properties`, model `a1_gazebo` |
| robot/control interface | implicit launch delay | model, `/a1_gazebo/controller_manager/list_controllers`, all joint controllers running, `/a1_gazebo/lowState/state`, `/trunk_imu` |
| `junior_ctrl` | terminal/tmux creation | node `/unitree_gazebo_servo`, live process/session, `/fsm/state_cmd` subscriber |
| FixedStand | one direct publish | supervisor request ownership when available, valid IMU, accepted `/fsm/state_cmd=2`, stable output window |
| sensors | implicit Gazebo delay | `/scan` publisher + fresh non-empty cloud; `/trunk_imu` publisher + finite fresh samples |
| adapter | fixed delay | `/scan_pointcloud2` publisher, non-empty XYZI cloud, `laser_livox`, advancing stamps |
| FAST-LIO2 | launch process only | `/laserMapping`, finite advancing `/Odometry`, non-empty advancing `/cloud_registered`, no sustained fatal mapping symptom |
| supervisor | process launch only | `/nav_state_supervisor`, subscribers on all request topics, publishers on outputs, safe state false/false/2 |
| navigation | roslaunch process only | valid navigation relays/TF; FALCO nodes + closed bridge gate; DSV nodes/service/terrain in `dsv_falco` mode |
| auto state | fire-and-forget requests | confirmed ordered 2 -> 4 -> enabled -> exploring transitions |
| recorder | navigation boolean only | writable directory, valid odometry/cloud, mode-specific optional DSV inputs |

All readiness timeouts must use wall time because `/clock` can advance slowly or stall at low RTF.

## Fixed Sleeps and Purposes

- `schedule_unpause_physics`: `AUTO_UNPAUSE_DELAY` (default 6) before service polling; arbitrary delay.
- `schedule_unpause_physics`: 0.25 s polling interval; bounded backoff and acceptable if incorporated into a wall-time wait.
- `gazebo_final_unpause`: 1.0/0.5/1.0 s service retry and clock sampling; short observation windows but currently not one unified timeout.
- `wait_for_topic`: 0.5 s backoff; acceptable bounded polling.
- global cleanup: 2 s OS reclamation; arbitrary and replaceable with port/process state checks.
- after Gazebo launch: 25 s; arbitrary and currently the principal readiness gate.
- after controller launch: 2 s; arbitrary.
- before subscriber search: 3 s; arbitrary.
- `FAST_LIO2_DELAY`: default 5 s; arbitrary compatibility delay.
- `/fsm/state_cmd` polling: 1 s; loop has 15 iterations but continues even when none appears.
- IMU loop: 0.5 s between six samples; bounded but checks only integer `z >= 9` and warning-only failure.
- after scan adapter: 2 s; arbitrary.
- after navigation launch: 6 s; arbitrary.
- before each auto trotting/enable/exploring request: 2 s each; arbitrary and no result check.
- cleanup navigation disable: 0.5 s; short best-effort delivery window, but no state/zero confirmation.
- runtime loop: 1 s; intentional keepalive, not startup readiness.

## One-shot Publishes

- Direct `/fsm/state_cmd=2` before the supervisor exists.
- Safe defaults through `/navigation/request_enabled=false`, `/navigation/request_exploring=false`, `/navigation/request_fsm_state=2` after supervisor launch.
- Optional `/navigation/request_fsm_state=4`, `/navigation/request_enabled=true`, and `/navigation/request_exploring=true`.
- Cleanup only requests `/navigation/request_enabled=false`; it does not request exploring false or FSM 2 and does not publish/verify zero `/cmd_vel`.

All use `rostopic pub ... -1`; there is no acknowledgement or stable-state confirmation.

## PID/Process-only Checks

- Gazebo readiness is reduced to `kill -0 $LAUNCH_PID` after 25 s.
- Building control, adapter, FAST-LIO2, supervisor, navigation, recorder, controller terminal/tmux, and RViz are launched without functional readiness confirmation.
- Navigation input topics are checked before launch, but the launched navigation nodes/services/output topics are not.

## Defaults and Propagation

| Setting | Competition default in `auto.sh` | Propagation |
|---|---:|---|
| `ENABLE_SENSOR_DATA` | `true` (`ENABLE_SENSOR_DATA` -> legacy `ENABLE_SENSORS` -> 1) | launch `enable_sensor_data` -> xacro `ENABLE_SENSOR_DATA`; gates LiDAR, Livox IMU, and depth camera together; also gates converter launch group |
| `ENABLE_POINTCLOUD_CONVERTER` | `true` | launch `enable_pointcloud_converter` -> conditional `pointcloud2livox.py` node inside sensor group |
| `ENABLE_FAST_LIO2` | `true` | controls adapter, mapping launch, and RViz eligibility; navigation requires true |
| `ENABLE_NAVIGATION` | `false` | controls supervisor, DSV/FALCO/bridge bringup, state automation, recorder eligibility |
| `ENABLE_RVIZ` | `true` | controls dedicated RViz terminal after mapping launch |

Earth mode already forces all five off unless explicitly overridden. Existing auto-state names are `NAV_AUTO_TROTTING`, `NAV_AUTO_ENABLE`, and `NAV_AUTO_START_EXPLORATION`; compatibility aliases requested by the task (`AUTO_COMMAND_TROTTING`, `AUTO_ENABLE_NAVIGATION`, `AUTO_START_EXPLORATION`) are not currently recognized.

## Depth-camera Frozen Boundary

- Links/joints and optical frame: `robot.xacro`, `real_sense`, `real_sense_joint`, `real_sense_optical_frame`.
- Plugin: `gazebo.xacro` lines 386-422, under the existing `ENABLE_SENSOR_DATA` group, using `libgazebo_ros_openni_kinect.so`.
- Frozen behavior: sensor `update_rate=20`, plugin `updateRate=10`, 640x480 RGB/depth, FOV, clip 0.05-8.0, all image/depth/pointcloud/camera-info names, optical frame, distortion, baseline, and point-cloud cutoff.
- Baseline full-file SHA256: `robot.xacro` `5170c543...c1d302`; `gazebo.xacro` `c4e429fc...f8588f`.
- No readiness gate may add a depth topic, and `ENABLE_SENSOR_DATA` must keep controlling this block exactly as before.

## Topic Producers and Consumers

| Topic | Producer | Current consumers |
|---|---|---|
| `/scan` | A1 `liblivox_laser_simulation.so` ray plugin | `scan_to_pointcloud2.py`; optional legacy `pointcloud2livox.py` |
| `/scan_pointcloud2` | SimEnv FAST-LIO2 adapter | FAST-LIO2 (`common/lid_topic`) |
| `/trunk_imu` | A1 trunk `libgazebo_ros_imu_sensor.so` | Unitree servo/controller and FAST-LIO2 (`common/imu_topic`) |
| `/livox/imu` | tilted LiDAR IMU plugin | no current FAST-LIO2/navigation consumer; launch XML's unused `imu_topic` default says `/livox/imu`, but loaded YAML overrides it with `/trunk_imu` |
| `/Odometry` | FAST-LIO2 `laserMapping` | FAST-LIO relays; navigation odometry-to-map; recorder/diagnostics |
| `/cloud_registered` | FAST-LIO2 `laserMapping` | FAST-LIO relay; navigation registered-scan relay; recorder/diagnostics |
| `/navigation/state_estimation` | `odometry_to_map.py` from `/Odometry` | DSV, FALCO, terrain builder, graph planner |
| `/navigation/registered_scan` | topic relay from `/cloud_registered` | FALCO, terrain builder, DSV octomap |
| `/navigation/terrain_map` | `registered_cloud_to_terrain_map.py` | DSV, graph planner, FALCO |
| `/navigation/way_point` | DSV graph/exploration chain | FALCO `localPlanner` |
| `/navigation/path` | FALCO `localPlanner` | FALCO `pathFollower`, recorder |
| `/navigation/falco/cmd_vel_stamped` | FALCO `pathFollower` | `cmd_vel_bridge.py` |
| `/navigation/enabled` | `nav_state_supervisor.py` | cmd_vel bridge and recorder/monitors |
| `/navigation/start_exploring` | supervisor | DSV exploration and recorder/monitors |
| `/fsm/state_cmd` | supervisor (latched/periodic) | `junior_ctrl` and cmd_vel bridge |
| `/cmd_vel` | gated cmd_vel bridge or manual operator | Unitree Trotting/RL controllers and recorder |

## Legacy Converter Conclusion

`pointcloud2livox.py` is not in the FAST-LIO2 + DSV + FALCO chain. It performs an additional per-point Python conversion/filter/optional odometry transform and publishes `/livox/lidar2` plus `/livox/Pointcloud2`. Repository search found these outputs in documentation, an optional field-check utility, the converter itself, and `ALLOWED_TEAM_TOPICS`; the scene generator's list is an allowlist, not a required-topic assertion. No test, judge, navigation, or mapping runtime consumer requires them. Default-off with explicit opt-in is therefore compatible.

## LiDAR Visualization and Livox IMU

- Active A1 LiDAR configuration is `gazebo.xacro`: ray sensor `<visualize>true</visualize>` and plugin child `<visualize>true</visualize>`.
- The active plugin source has the SDF `visualize` read and Gazebo transport scan publishing logic commented out; the plugin-local field is not active in this source. The ray sensor field is the effective Gazebo sensor visualization switch. Setting both false documents a consistent default without changing ray count, frequency, range, noise, or `/scan`.
- `/livox/imu` is redundant for the audited main chain, but it shares the same `ENABLE_SENSOR_DATA` xacro block with the frozen depth camera. Per task constraints it will be recorded and left unchanged; no attempt will split that block.

## High-frequency Logging

- `cmd_vel_bridge.py` already logs gate edges and throttles rejection diagnostics to 5 s.
- Trotting/RL non-finite and timeout warnings use ROS throttling.
- No clearly removable per-frame pointcloud or TF logging was found in the SimEnv-owned required chain. No log-frequency code change is justified.

## Static Audit Verdict

`STATIC_AUDIT_PASS`: the required paths, defaults, state ownership, producer/consumer chain, forbidden boundary, and safe optional-load candidates are identified before implementation.
