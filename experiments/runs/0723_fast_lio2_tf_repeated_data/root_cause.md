# Root Cause

## PRIMARY_ROOT_CAUSE

`SINGLE_OWNER_DUPLICATE_TIMESTAMP_FOUND` for `map -> odom` and `odom -> base`
in `state_from_gazebo`.

Static evidence:

- `state_from_gazebo` publishes both edges from `/gazebo/link_states`.
- The callback uses `ros::Time::now()` independently for `map -> odom`,
  `odom -> base`, and `/Odometry_gazebo.header.stamp`.
- `/gazebo/link_states` is subscribed with queue depth 10 even though it represents current state.
- `gazebo_msgs/LinkStates` has no header stamp, so the broadcaster uses `/clock` via
  `ros::Time::now()` under simulated time.

This permits multiple queued link-state callbacks to be processed while `/clock` is unchanged,
especially under low RTF, pause, or callback backlog conditions.

## SECONDARY_CONTRIBUTORS

- `enable_referee_odom` defaults true in competition `auto.sh`, so `state_from_gazebo`
  is normally active.
- `simenv_fast_lio2_mapping.launch` defaulted `enable_odometry_tf_bridge=true` even though
  project ADR/docs identify FAST-LIO2 `laserMapping` as the owner of `camera_init -> body`.

## NON_CAUSES

- No active Gazebo/Livox plugin TF broadcaster for `map -> odom` or `odom -> base` was found.
- `robot_state_publisher` owns URDF joint/frame edges such as `base -> imu_link`, not
  `map -> odom`.
- `map_to_camera_init_bridge.py` publishes one static `map -> camera_init` edge, not the
  repeated dynamic edges in the warning.
- FAST-LIO2 itself is not the primary cause for the reported `map -> odom` and `odom -> base`
  repeated timestamps based on available static evidence.
