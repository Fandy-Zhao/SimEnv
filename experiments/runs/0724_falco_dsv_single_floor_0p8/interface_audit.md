# Interface Audit: Single-Floor FALCO + DSV 0.8 m/s

## Verdict

`SCOPE_AUDIT_PASS`

## FALCO Speed Chain

- Config: `falco_a1.yaml` sets `maxSpeed=0.80`, `autonomySpeed=0.80`, heading schedule enabled.
- Path follower: raw `/navigation/falco/cmd_vel_stamped` now uses heading-aware speed scheduling before bridge limits.
- Bridge: `navigation_safety.yaml` and launch defaults set `max_linear_x=0.80`, `max_angular_z=0.22`.
- Synthetic path follower probes:
  - Straight: raw max `0.803999543 m/s`, angular max `0.0 rad/s`.
  - 30 deg: raw max `0.600000143 m/s`, angular max `0.219911486 rad/s`.
  - 70 deg: raw max `0.203999937 m/s`, angular max `0.219911486 rad/s`.

## DSV

- Initialization no longer depends on moving away from home when initial offset is zero.
- New defaults: `skipInitialMotion=true`, `initializationTimeout=5.0`, `initializationDistanceTolerance=0.15`.
- Movement detection uses a 2.0 s odometry window and `0.08 m` displacement threshold.
- Stuck detection uses `stuckTimeout=15.0` and two replan attempts before frontier cleaning.
- Planner and clean-frontier service names are parameterized.
- Single-floor goal Z is clamped around initial odometry with `max_goal_z_deviation=0.20`.

## Terrain Map

- New `registered_cloud_to_terrain_map.py` subscribes `/navigation/registered_scan` and `/navigation/state_estimation`.
- It transforms cloud points to `map` when TF is available, filters robot body, ground, ceiling/high points, and local radius, then voxel downsamples.
- Output: `/navigation/terrain_map`, default 8 Hz.

## Boundary

- New `simenv_navigation_boundary.py` publishes `/navigation/boundary` and `/navigation/boundary_marker`.
- Defaults: `20.0 x 36.0 m`, centered from public env/params, shrunk by `0.4 m`.
- Launch parse shows the boundary node is part of `single_floor_exploration.launch`.

## Launch

- `single_floor_exploration.launch` starts relays, terrain map, runtime boundary, DSV, FALCO, and bridge.
- It does not start Gazebo, FAST-LIO2, robot model, junior_ctrl, RViz, or rosbag.
- `roslaunch --nodes` and `roslaunch --files` passed; see `launch_check.txt`.

## Not Modified

- Gazebo physics, collision geometry, robot controller, RL controller, FAST-LIO2 core, sensor extrinsics, scene generator, and ground truth were not modified.
