# Third-Party Navigation Sources

## Baseline

- SimEnv baseline: local `master` at `5bc0f6fbfdd8333dccbb44c26f216ecfb2811548`
- Task branch/worktree: `feat/0723-falco-dsv-navigation-integration`,
  `/home/zzf/search_ws/SimEnv_worktrees/falco-dsv-navigation`
- Root workspace dirty state at task start: generated scene files, logs,
  `results/danger_truth.json`, and an unrelated report were dirty/untracked;
  they were not used for development.

## FALCO

- Local path: `/home/zzf/search_ws/ground_based_autonomy_basic`
- Upstream URL: `https://github.com/jizhang-cmu/ground_based_autonomy_basic.git`
- Branch: `noetic`
- Source commit: `7ae94b72206430a399ae012f49715cf51fadb0e0`
- Local status: clean
- ROS distribution: Noetic branch
- License declared by copied package: BSD
- Copied packages: `local_planner`
- Excluded packages: `terrain_analysis` for the first stage, because
  `local_planner` can consume `/registered_scan` directly with
  `useTerrainAnalysis:=false`; `vehicle_simulator`, `loam_interface`,
  `joystick_drivers`, `joy`, `ps3joy`, `waypoint_example`, and
  `velodyne_simulator` packages are excluded.
- Vendor patch: `local_planner/package.xml` and `CMakeLists.txt` now declare
  direct dependencies already used by source includes (`geometry_msgs`,
  `nav_msgs`, and `tf`). Topic names and planner behavior are unchanged.

## DSV-Planner

- Local path: `/home/zzf/search_ws/dsv_planner`
- Upstream URL: `https://github.com/HongbiaoZ/dsv_planner.git`
- Branch: `noetic`
- Source commit: `5ad721d243545cd0e6383ba2f3a4b9f218959ab4`
- Local status: clean
- ROS distribution: Noetic branch
- License declared by copied packages: BSD, except `kdtree` declares `TBD`
  in `package.xml`
- Copied packages:
  `catkin_simple`, `kdtree`, `minkindr`, `minkindr_conversions`,
  `volumetric_msgs`, `volumetric_map_base`, `octomap_world`, `misc_utils`,
  `graph_utils`, `graph_planner`, `dsvplanner`, and `dsvp_launch`
- Excluded packages: no upstream Gazebo world, robot model, joystick,
  Velodyne simulator, LOAM, or bag-record/runtime simulator package was copied.
- Vendor patch: `catkin_simple/test/scenarios/hello_world/*/package.xml`
  demonstration manifests were omitted so SimEnv's package discovery reports
  only imported runtime packages.

## Octomap ROS Dependencies

The local apt source on this host does not provide
`ros-noetic-octomap-msgs` or `ros-noetic-octomap-ros`, and `rospack` found no
installed ROS1 octomap packages. To keep the system ROS installation unchanged,
the ROS1 melodic source branches were imported under
`src/navigation/vendor/dsv/deps/`.

- `octomap_msgs`
  - Upstream URL: `https://github.com/OctoMap/octomap_msgs.git`
  - Branch: `melodic-devel`
  - Source commit: `dcaaf62bd071db0fcd806a98a9101a2e470f7f6d`
  - License declared: BSD
- `octomap_ros`
  - Upstream URL: `https://github.com/OctoMap/octomap_ros.git`
  - Branch: `melodic-devel`
  - Source commit: `8816e50bf00411c41b7891842effd41bfbc2e2df`
  - License declared: BSD

## Actual SimEnv Interfaces Found

- FAST-LIO2 launch relays `/Odometry` to `/state_estimation` and
  `/cloud_registered` to `/registered_scan`; launch arguments can override
  these names.
- FAST-LIO2 output frames are preserved by relay; current docs identify
  odometry as FAST-LIO2 `camera_init`/`body` semantics and registered cloud as
  FAST-LIO2 native output.
- `map_to_camera_init_bridge.py` connects `map -> camera_init`; FAST-LIO2
  owns dynamic `camera_init -> body` by default.
- Trotting subscribes to global `/cmd_vel` as `geometry_msgs/Twist`.
- FALCO `pathFollower` publishes `geometry_msgs/TwistStamped` to `/cmd_vel`
  upstream; SimEnv remaps that to `/navigation/falco/cmd_vel_stamped`.
- FSM command interface is `/fsm/state_cmd` as `std_msgs/Int8`; Trotting is
  enum value `4`. No dedicated current-state ROS topic was found, so
  `cmd_vel_bridge` gates on explicit navigation enable plus observed
  Trotting state command without modifying controller core.

## Imported Package List

```text
local_planner
catkin_simple
kdtree
minkindr
minkindr_conversions
volumetric_msgs
volumetric_map_base
octomap_world
octomap_msgs
octomap_ros
misc_utils
graph_utils
graph_planner
dsvplanner
dsvp_launch
simenv_navigation_bridge
simenv_navigation_bringup
```
