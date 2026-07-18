# FAST-LIO2 point-cloud frame semantics repair

## Goal

Correct the layer that makes SimEnv's FAST-LIO2 input point cloud appear behind
the A1 without compensating through URDF or FAST-LIO2 extrinsics.

## Baseline

- Date: 2026-07-18
- Baseline commit: `a2e00509`
- Task branch: `fix/0718-fast-lio2-pointcloud-frame-semantics`
- The originating worktree has pre-existing generated-scene, log, result, and
  untracked changes. This isolated worktree starts clean and must not import
  or overwrite them.
- No ROS master was available at task creation; runtime evidence must be
  collected only from an isolated instance if it can be launched safely.

## Known evidence and hypotheses

- The A1 mechanical forward axis is `base`/`trunk +X`.
- `base -> laser_livox` is expected to be `(0.2, 0, 0.08), Ry(+45 deg)`.
- The Livox Gazebo plugin constructs local rays from `+X` and publishes `/scan`
  in `laser_livox`.
- The adapter's defaults rotate every point by `Ry(-90 deg)` and then
  `Rx(180 deg)` while retaining `laser_livox` as the output frame.
- FAST-LIO2 applies `p_imu = extrinsic_R * p_lidar + extrinsic_T`; its current
  SimEnv extrinsic represents the physical LiDAR-to-IMU transform.

Primary hypothesis: the adapter violates the declared `laser_livox` frame
semantics. Alternatives to exclude: plugin axis/pose, URDF mount, FAST-LIO2
extrinsic, map-to-camera bridge, and RViz-only interpretation.

## Allowed modification scope

- `src/simenv_fast_lio2_integration/scripts/scan_to_pointcloud2.py`
- Targeted adapter tests, if needed
- Relevant FAST-LIO2 integration documentation and this task record

## Explicit non-scope

- `src/FAST_LIO/**`, its extrinsic configuration, and algorithm
- A1 URDF/Xacro, Gazebo sensor pose, controller, navigation, scene generation,
  generated files, logs, maps, and unrelated packages

## Acceptance criteria

1. `/scan` and `/scan_pointcloud2` retain equal point coordinates, timestamp,
   and frame semantics in the default SimEnv path.
2. Unit-vector tests demonstrate that no implicit point rotation occurs by
   default.
3. FAST-LIO2 extrinsics and A1 mounting remain unchanged.
4. Python syntax and targeted tests pass; an isolated runtime check is run if
   the existing launcher can start without affecting user processes.
5. The final report distinguishes static proof from any unavailable runtime
   evidence.

## Test plan

- Static matrix/unit-vector check and adapter tests.
- Python compilation and package-level test/build checks.
- Isolated ROS/Gazebo runtime only if a separate master and Gazebo port can be
  established without starting or stopping the user's instance.

## Rollback

Revert the task commit on this branch; no generated or user-owned artifacts
are part of the task.
