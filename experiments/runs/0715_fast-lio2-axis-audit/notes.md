# FAST-LIO2 axis audit notes

## Commands

```bash
python3 src/simenv_fast_lio2_integration/scripts/check_fast_lio2_extrinsics.py \
  --config src/simenv_fast_lio2_integration/config/simenv_mid360.yaml \
  --robot-xacro src/unitree_guide/unitree_ros/robots/a1_description/xacro/robot.xacro
timeout 3 rosrun tf tf_echo imu_link laser_livox
timeout 12 rostopic echo -n 1 /Odometry
```

## Results

- Runtime TF: `imu_link -> laser_livox` = translation `[0.2, 0, 0.08]`,
  pitch `+44.977°`.
- FAST-LIO2 source transforms points as
  `p_imu = offset_R_L_I * p_lidar + offset_T_L_I`.
- The former inverse parameter (`Ry(-45°)`, `[-0.085, 0, -0.198]`) was thus
  incorrect for the local `laser_livox` points emitted by the Livox plugin.
- Updated YAML and checker agree on `Ry(+45°)`, `[0.2, 0, 0.08]`.
- The active `fastlio_mapping` node was intentionally not restarted; it keeps
  the old startup parameters until the next launch.
- `catkin_make --pkg simenv_fast_lio2_integration` could not run because the
  existing workspace cache whitelists only `unitree_guide;unitree_legged_msgs`.
