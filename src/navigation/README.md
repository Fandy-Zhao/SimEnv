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

- FAST-LIO2 odometry: `/Odometry` -> `/navigation/state_estimation`
- FAST-LIO2 registered scan: `/cloud_registered` -> `/navigation/registered_scan`
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

For runtime validation against an already running SimEnv + FAST-LIO2 stack:

```bash
NAV_MAX_LINEAR_X=0.10 NAV_MAX_ANGULAR_Z=0.20 \
  roslaunch simenv_navigation_bringup runtime_real_data.launch
```

This launch relays global FAST-LIO2 `/Odometry` and `/cloud_registered`
into `/navigation/state_estimation` and `/navigation/registered_scan` before
starting FALCO, optional DSV, and the gated command bridge.

## A1 FALCO Profile

`falco_only.launch` and `runtime_real_data.launch` default to
`simenv_navigation_bringup/config/falco_a1.yaml`. This profile is tuned for the
SimEnv Unitree A1 model and real FAST-LIO2 `/cloud_registered` input with
`checkObstacle=true`.

Selected R3 parameters are:

- `minRelZ=-0.25`, `maxRelZ=0.25`
- `vehicleLength=0.56`, `vehicleWidth=0.43`
- `pointPerPathThre=2`
- `adjacentRange=3.5`, `pathScale=1.0`, `minPathScale=0.75`,
  `minPathRange=1.0`, `goalClearRange=0.5`

Launch-time overrides are available for these values, plus opt-in diagnostics:

```bash
roslaunch simenv_navigation_bringup runtime_real_data.launch \
  start_dsv:=false start_falco:=true start_bridge:=true \
  enable_diagnostics:=true diagnostic_throttle_sec:=0.5
```

R3 tuning evidence is recorded under
`experiments/runs/0724_falco_a1_tuning/`. This evidence validates local FALCO
path readiness only; Trotting motion, DSV exploration, and full building
navigation remain separate gates.
