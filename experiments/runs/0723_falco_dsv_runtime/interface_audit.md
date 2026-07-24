# R0 Interface Audit

Date: 2026-07-23

Verdict: `R0_INTERFACE_AUDIT_PASS`

## Evidence

- Raw grep evidence: `r0_grep_audit.txt`
- Worker mechanical inventory: `worker_interface_inventory.md`
- Manual source review:
  - `src/simenv_fast_lio2_integration/launch/simenv_fast_lio2_mapping.launch`
  - `src/simenv_fast_lio2_integration/README.md`
  - `src/navigation/simenv_navigation_bringup/launch/falco_only.launch`
  - `src/navigation/simenv_navigation_bringup/launch/dsv_only.launch`
  - `src/navigation/simenv_navigation_bringup/config/dsv_simenv.yaml`
  - `src/navigation/simenv_navigation_bringup/config/falco_simenv.yaml`
  - `src/navigation/simenv_navigation_bringup/config/navigation_safety.yaml`
  - `src/navigation/simenv_navigation_bridge/scripts/cmd_vel_bridge.py`
  - `src/unitree_guide/unitree_guide/unitree_guide/src/FSM/State_Trotting.cpp`
  - `src/unitree_guide/unitree_guide/unitree_guide/src/main.cpp`

## Actual Interface Contract

| Producer | Topic | Type | Frame | Consumer | Remap / Bridge |
|---|---|---|---|---|---|
| FAST-LIO2 `laserMapping` | `/Odometry` | `nav_msgs/Odometry` | Runtime message header, expected `camera_init`; child expected `body` | `topic_tools/state_estimation_relay` | Relay to `/state_estimation` |
| FAST-LIO2 `laserMapping` | `/cloud_registered` | `sensor_msgs/PointCloud2` | Runtime message header, expected FAST-LIO map frame | `topic_tools/registered_scan_relay` | Relay to `/registered_scan` |
| `state_estimation_relay` | `/state_estimation` | `nav_msgs/Odometry` | Preserved from `/Odometry` | Navigation launch remap input | FALCO remaps to `/navigation/state_estimation`; DSV config directly consumes `/navigation/state_estimation` |
| `registered_scan_relay` | `/registered_scan` | `sensor_msgs/PointCloud2` | Preserved from `/cloud_registered` | Navigation launch remap input | FALCO remaps to `/navigation/registered_scan`; DSV octomap config consumes `/navigation/registered_scan` |
| FALCO `localPlanner` | `/navigation/path` | `nav_msgs/Path` | Runtime path header | FALCO `pathFollower` | `/path` remapped to `/navigation/path` |
| FALCO `pathFollower` | `/navigation/falco/cmd_vel_stamped` | `geometry_msgs/TwistStamped` | Hardcoded `vehicle` | `cmd_vel_bridge.py` | `/cmd_vel` remapped away from global topic |
| `cmd_vel_bridge.py` | `/cmd_vel` | `geometry_msgs/Twist` | N/A | Unitree Trotting | Converts `TwistStamped.twist` to `Twist` only when safety gates pass |
| Unitree controller ROS callback | `/fsm/state_cmd` | `std_msgs/Int8` | N/A | Controller FSM and bridge gate | Value `2` maps to FixedStand; value `4` maps to `UserCommand::START`, which enters `FSMStateName::TROTTING` when Torch policy build supports it |
| DSV `exploration` | `/navigation/way_point` | `geometry_msgs/PointStamped` | `interface/tfFrame`, configured `map` | FALCO `localPlanner`; DSV exploration also self-subscribes | DSV config `interface/waypointTopic`; FALCO `/way_point` remap |
| DSV `drrtPlanner` | `/navigation/drrtPlannerSrv` | `dsvplanner/dsvplanner_srv` | N/A | DSV `exploration` via service call | Config `planner/plannerServiceName` |
| DSV `drrtPlanner` | `/navigation/cleanFrontierSrv` | `dsvplanner/clean_frontier_srv` | N/A | DSV `exploration` | Config `planner/cleanFrontierServiceName` |
| DSV `graph_planner` | `/navigation/way_point` | `geometry_msgs/PointStamped` | `world_frame_id`, configured `map` | FALCO `localPlanner` | Config `pub_waypoint_topic` |
| DSV `grid` | `/navigation/dsv/occupancy_grid_map` | `sensor_msgs/PointCloud2` | `grid/world_frame_id`, configured `map` | Diagnostics/visualization | Config `grid/pubGridPointsTopic` |

## Bridge Defaults

| Parameter | Value |
|---|---|
| `input_topic` | `/navigation/falco/cmd_vel_stamped` |
| `output_topic` | `/cmd_vel` |
| `enabled_topic` | `/navigation/enabled` |
| `stop_topic` | `/navigation/stop_exploring` |
| `state_cmd_topic` | `/fsm/state_cmd` |
| `max_linear_x` | `0.20` |
| `max_linear_y` | `0.00` |
| `max_angular_z` | `0.30` |
| `command_timeout` | `0.5` |
| `require_navigation_enabled` | `true` |
| `require_trotting_state_cmd` | `true` |
| `trotting_state_value` | `4` |
| `publish_zero_when_disabled` | `true` |

## Notes

- `topic_tools/relay` preserves the incoming message headers and frames.
- FALCO's upstream `pathFollower` publishes `geometry_msgs/TwistStamped` on
  `/cmd_vel`, but `falco_only.launch` remaps it to
  `/navigation/falco/cmd_vel_stamped`, preventing a global `/cmd_vel` type
  conflict.
- Trotting subscribes to global `/cmd_vel` as `geometry_msgs::Twist`.
- `/navigation/enabled` defaults effectively disabled because the bridge starts
  with `_navigation_enabled=false` when `require_navigation_enabled=true`.
