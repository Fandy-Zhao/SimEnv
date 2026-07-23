# Change Plan

1. In `state_from_gazebo.cpp`, read `ros::Time::now()` once per callback and use that stamp
   for both TF messages and `/Odometry_gazebo`.
2. Skip dynamic TF/odometry publication when `/clock` is zero.
3. Skip duplicate callback publications at the same simulation stamp with throttled debug logging.
4. Detect clock rollback, reset the guard, and allow the new epoch to publish.
5. Change `/gazebo/link_states` subscription queue depth from 10 to 1.
6. Make `enable_odometry_tf_bridge` default false so `laserMapping` retains sole default
   ownership of `camera_init -> body`.
7. Update static tests and module documentation.
