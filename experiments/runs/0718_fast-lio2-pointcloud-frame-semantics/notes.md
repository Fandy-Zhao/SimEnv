# FAST-LIO2 point-cloud frame semantics notes

## Baseline commands

- `git status --short` — isolated task worktree clean at creation.
- `git branch --show-current` — `fix/0718-fast-lio2-pointcloud-frame-semantics`.
- `git rev-parse --short HEAD` — `a2e00509`.
- `rosmaster` was unavailable in the originating environment, so no active
  user simulation was inspected or restarted.

## Planned evidence

1. Derive the adapter's exact column-vector rotation order using unit vectors.
2. Compare the plugin's local point construction with the adapter output frame.
3. Verify FAST-LIO2's extrinsic multiplication in source before approving any edit.

## Static decision evidence

- The Livox plugin builds each local ray and output point from `(1, 0, 0)`;
  `/scan` is therefore in the sensor's local `+X` convention.
- `laser_livox_joint` is `base -> laser_livox` at `(0.2, 0, 0.08)` and
  `Ry(+0.785)`, with a zero Gazebo sensor pose.
- The former adapter applied `Ry(-90)` then `Rx(+180)`: `(1, 0, 0)` became
  approximately `(0, 0, -1)` but retained the `laser_livox` frame name.
- FAST-LIO2 source applies `p_imu = extrinsic_R * p_lidar + extrinsic_T`.
  The checked configuration matches the xacro's direct `Ry(+45 deg)` and
  translation, so it is not changed.
- No launch file overrides either adapter rotation parameter.

## Validation results

- `python3 -m py_compile .../scan_to_pointcloud2.py`: PASS.
- `python3 -m unittest discover -s src/simenv_fast_lio2_integration/test`: PASS
  (7 tests), including unit-vector and Y-then-X regression coverage.
- `check_fast_lio2_extrinsics.py`: PASS; configured LiDAR-to-IMU transform
  matches `robot.xacro`.
- Isolated ROS/Gazebo retry used ROS master `11331` and Gazebo master `11361`.
  Gazebo, `/clock`, TF, and robot state publisher started; no `/scan` appeared
  because the root devel wrapper for `pointcloud2livox.py` could not import
  `unitree_guide.msg` under system Python. All exact isolated process PIDs were
  stopped. No live point-cloud numerical comparison is claimed.
