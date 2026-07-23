## ROS Interface Inventory — FALCO + DSV Navigation Runtime

### Topics (pub → sub)

| Topic | Type (visible) | Producer | Consumer | Remap Chain |
|---|---|---|---|---|
| `/state_estimation` | `nav_msgs/Odometry` * | `topic_tools/relay` (/Odometry→) | — | `falco_only`: → `/navigation/state_estimation`; `dsv_only`: config → `/navigation/state_estimation` |
| `/registered_scan` | `sensor_msgs/PointCloud2` * | `topic_tools/relay` (/cloud_registered→) | — | `falco_only`: → `/navigation/registered_scan`; `dsv_simenv.yaml`: → `/navigation/registered_scan` |
| `/navigation/falco/cmd_vel_stamped` | `geometry_msgs/TwistStamped` | FALCO `pathFollower` (remapped from `/cmd_vel`) | `cmd_vel_bridge` (input) | `falco_only`: `/cmd_vel` → `/navigation/falco/cmd_vel_stamped` |
| `/cmd_vel` | `geometry_msgs/Twist` | `cmd_vel_bridge` (output) | SimEnv Trotting | — |
| `/navigation/way_point` | * | FALCO `localPlanner`, DSV `exploration` | — | `falco_only`: `/way_point` →; `dsv_simenv.yaml` config → |
| `/navigation/path` | * | FALCO `localPlanner` | FALCO `pathFollower` | `falco_only`: `/path` → |
| `/navigation/terrain_map` | * | FALCO `localPlanner` | DSV (planner/grid/graph_planner) | `falco_only`: `/terrain_map` → |
| `/navigation/boundary` | * | — | FALCO `localPlanner`, DSV `navigationBoundary` | `falco_only`: `/navigation_boundary` →; `dsv_only`: `/navigation_boundary` → |
| `/navigation/stop_exploring` | `std_msgs/Bool` | — | DSV planner, `cmd_vel_bridge` | `dsv_simenv.yaml`: `shutDownTopic`, `stopSignalTopic` |
| `/navigation/start_exploring` | * | — | DSV interface | `dsv_simenv.yaml`: `beginSignalTopic` |
| `/fsm/state_cmd` | `std_msgs/Int8` | — | `cmd_vel_bridge` (gate) | — |
| `/navigation/enabled` | `std_msgs/Bool` | — | `cmd_vel_bridge` (gate) | — |
| `/navigation/graph_planner_command` | * | DSV interface | DSV `graph_planner` | `dsv_simenv.yaml` top-level |
| `/navigation/graph_planner_status` | * | DSV `graph_planner` | DSV interface, DSV graph | `dsv_simenv.yaml` top-level |
| `/navigation/graph_planner_path` | * | DSV `graph_planner` | DSV graph | `dsv_simenv.yaml`: `pub_path_topic` |
| `/navigation/local_graph` | * | DSV graph | DSV `graph_planner` | `dsv_simenv.yaml` |
| `/navigation/global_graph` | * | DSV graph | — | `dsv_simenv.yaml` |
| `/navigation/global_points` | * | DSV graph | DSV frontier | `dsv_simenv.yaml` |
| `/navigation/global_frontier` | * | DSV frontier | — | `dsv_simenv.yaml` |
| `/navigation/local_frontier` | * | DSV frontier | — | `dsv_simenv.yaml` |
| `/navigation/dsv/new_tree_path` | * | DSV planner | — | `dsv_simenv.yaml` |
| `/navigation/dsv/remaining_tree_path` | * | DSV planner | — | `dsv_simenv.yaml` |
| `/navigation/dsv/planning_horizon` | * | DSV planner | — | `dsv_simenv.yaml` |
| `/navigation/dsv/next_goal` | * | DSV planner | — | `dsv_simenv.yaml` |
| `/navigation/dsv/sampled_points` | * | DSV planner | — | `dsv_simenv.yaml` |
| `/navigation/dsv/occupancy_grid_map` | * | DSV grid | — | `dsv_simenv.yaml` |
| `/navigation/dsv/runtime` | * | DSV interface | — | `dsv_simenv.yaml` |
| `/navigation/dsv/total_time` | * | DSV interface | — | `dsv_simenv.yaml` |
| `/navigation/dsv/plan_time` | * | DSV planner | — | `dsv_simenv.yaml` |
| `/navigation/falco/free_paths` | * | FALCO `localPlanner` | — | `falco_only`: `/free_paths` → |
| `/navigation/octomap_unknown` | * | DSV frontier | — | `dsv_simenv.yaml` |

### Services (visible in config)

| Service | Visible In |
|---|---|
| `/navigation/drrtPlannerSrv` | `dsv_simenv.yaml` planner |
| `/navigation/cleanFrontierSrv` | `dsv_simenv.yaml` planner |

\* Type not directly visible in the allowed evidence files; see source vendor code.
