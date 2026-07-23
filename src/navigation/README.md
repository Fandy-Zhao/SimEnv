# SimEnv Navigation

This directory contains the minimum source import and SimEnv-specific bridge
for FALCO and DSV-Planner.

## Layout

- `vendor/falco/local_planner`: FALCO local planner and path follower.
- `vendor/dsv`: DSV-Planner and its required ROS package closure.
- `simenv_navigation_bridge`: safety bridge from FALCO
  `geometry_msgs/TwistStamped` to SimEnv Trotting `geometry_msgs/Twist`.
- `simenv_navigation_bringup`: launch and config for FALCO-only, DSV-only,
  and combined DSV + FALCO runs.

## Topic Contract

- FAST-LIO2 odometry: `/state_estimation` -> `/navigation/state_estimation`
- FAST-LIO2 registered scan: `/registered_scan` -> `/navigation/registered_scan`
- DSV waypoint output: `/navigation/way_point`
- FALCO path output: `/navigation/path`
- FALCO speed output: `/navigation/falco/cmd_vel_stamped`
- Trotting command input: `/cmd_vel`
- Navigation enable gate: `/navigation/enabled`
- DSV start/stop: `/navigation/start_exploring`,
  `/navigation/stop_exploring`

Navigation is disabled by default at the bridge. Publish
`std_msgs/Bool(data=True)` on `/navigation/enabled` and explicitly command
Trotting with `/fsm/state_cmd` value `4` before non-zero FALCO velocity can
reach `/cmd_vel`.

The bridge launch runs the Python node through `/usr/bin/python3` so ROS
Noetic uses the system Python runtime even when the build venv was created by
a newer shell Python.
