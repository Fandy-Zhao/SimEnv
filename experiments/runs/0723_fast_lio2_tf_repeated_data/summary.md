# Summary

## Result

`FAST_LIO2_TF_REPEATED_DATA_FIX_PASS` for static ownership and available
compile/static validation, including scoped `tools/build_with_venv.sh`.

## Changes

- `state_from_gazebo` no longer publishes multiple dynamic TF/odometry messages at
  identical simulation timestamps from callback backlog or paused `/clock`.
- `/Odometry_gazebo.header.stamp`, `map -> odom`, and `odom -> base` now share one
  callback stamp.
- `/clock == 0` is skipped, duplicate stamps are skipped with throttled debug logs,
  and time rollback resets the guard.
- `/gazebo/link_states` queue depth is reduced to 1 because the callback consumes current
  state, not an event stream.
- FAST-LIO2 `laserMapping` is the default owner of `camera_init -> body`; the local
  odometry TF bridge is opt-in.

## Residual Risk

Runtime TF warning disappearance still needs to be verified in a dedicated runtime session.
The build is ready, but `auto.sh` was not launched here because its cleanup step can stop
unrelated ROS/Gazebo processes on the shared host.
