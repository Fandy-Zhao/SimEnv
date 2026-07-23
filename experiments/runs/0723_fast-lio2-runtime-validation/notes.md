# FAST-LIO2 Runtime Validation Notes

Date: 2026-07-23
Branch: `fix/0723-fast-lio2-reproducible-build-pointcloud`
Baseline: `master` `a423bcfd104659bfa05d286ccb79d6a03520b246`
Previous commit: `30ea47bd820e042f0e19ea4a2282af0a49ccdc46`

## Verdict

`FAST_LIO2_RUNTIME_VALIDATION_PASS`

The clean isolated runtime publishes `/scan_pointcloud2` from the current
adapter process and FAST-LIO2 continuously publishes `/Odometry` and
`/cloud_registered`. The prior "pointcloud blocked" state is not reproduced
after using the external dependency staging layer and a clean ROS master.

## Governance

- Root workspace `/home/zzf/search_ws/SimEnv` was inspected only; its
  pre-existing dirty generated/log/result files were not modified.
- Task worktree:
  `/home/zzf/search_ws/SimEnv_worktrees/fast-lio2-repro-pointcloud`.
- Protected file
  `src/unitree_guide/unitree_guide/unitree_guide/src/state_from_gazebo.cpp`
  stayed unchanged; SHA256:
  `5d5ffd2bf0e01f284e39f9071547b875d306332eb2d74016fae4dbb7a6b89ad0`.
- Runtime was launched on private ROS master
  `http://127.0.0.1:12732`.
- Launch used `SKIP_GLOBAL_PROCESS_CLEANUP=true`, `TERMINAL_BACKEND=direct`,
  `GUI=False`, and `ENABLE_RVIZ=false`.

## External Dependency Provenance

- FAST_LIO: `/home/zzf/search_ws/FAST_LIO`
  `7cc4175de6f8ba2edf34bab02a42195b141027e9`.
- ikd-Tree submodule:
  `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`.
- livox_ros_driver: `/home/zzf/search_ws/livox_ros_driver`
  `3d240d5666129e1a3052e78ee8487a04b08fdda3`.
- Staging check and prepare both passed with ignored symlinks under `src/`.

## Publisher Ownership

Captured in `publisher_ownership_raw.txt`.

- `/scan` publisher: `/gazebo`.
- `/scan_pointcloud2` publisher:
  `/scan_to_pointcloud2_392581_1784801372018`, PID `392581`, command path
  `src/simenv_fast_lio2_integration/scripts/scan_to_pointcloud2.py`.
- `/trunk_imu` publisher: `/gazebo`.
- `/Odometry` publisher: `/laserMapping`, PID `392832`, command path
  `devel/lib/fast_lio/fastlio_mapping`.
- `/cloud_registered` publisher: `/laserMapping`.
- `/clock` publisher: `/gazebo`.

## Topic Continuity

Captured with `rostopic hz` because continuous Python subscription to large
PointCloud/PointCloud2 messages overloaded the diagnostic process before it
could write structured metrics.

- `/scan`: about `9.8-10.0 Hz`.
- `/scan_pointcloud2`: about `10.08 Hz`.
- `/trunk_imu`: about `342-348 Hz`.
- `/Odometry`: about `10.0 Hz`.
- `/cloud_registered`: about `10.0 Hz`.
- `/clock`: about `480-492 Hz`.

## PointCloud2 Contract

Captured in `scan_pointcloud2_header.txt`, `scan_pointcloud2_fields.txt`,
`scan_pointcloud2_width.txt`, and `scan_pointcloud2_point_step.txt`.

- `frame_id`: `laser_livox`.
- `width`: `24000`.
- `fields`: `x`, `y`, `z`, `intensity`.
- `point_step`: `16`.
- No per-point timestamp field is present; this matches
  `simenv_mid360.yaml` where `timestamp_unit` and `time_scale` are disabled.

## FAST-LIO2 Runtime

Captured in `laserMapping_node_info.txt`, topic-rate files, and
`fast_lio2_log_findings.txt`.

- `/laserMapping` subscribes to `/scan_pointcloud2`, `/trunk_imu`, and
  `/clock`.
- `/laserMapping` publishes `/Odometry`, `/cloud_registered`,
  `/cloud_registered_body`, `/cloud_effected`, `/Laser_map`, `/path`, and
  `/tf`.
- Log scan found one startup-only warning:
  `No point, skip this scan!` at sim time `0.804`.
- No repeated `No point`, `No Effective Points`, or runtime FAST-LIO2 error
  pattern was found in the captured log scan.

## TF Checks

Captured in `tf_*.txt`.

- `map -> odom`: identity and available.
- `camera_init -> base`: available.
- `map -> base`: available.
- `base -> laser_livox`: translation `[0.200, 0.000, 0.080]`, pitch about
  `44.977 deg`.
- `body` frame is not present in the active TF tree. The live tree uses
  `base`; this is a documentation/terminology mismatch with older integration
  comments, not a runtime pointcloud blockage.

## Pause/Resume

Captured in `pause_resume_clock.txt`,
`post_resume_scan_pointcloud2_hz.txt`, and `post_resume_odometry_hz.txt`.

- `/gazebo/pause_physics` and `/gazebo/unpause_physics` returned.
- After resume, `/scan_pointcloud2` returned at about `9.87-9.90 Hz`.
- After resume, `/Odometry` returned at about `9.90-10.14 Hz`.

## Diagnostic Tooling Note

Continuous Python subscribers for large `/scan` and `/scan_pointcloud2`
messages did not exit reliably under this workload. Evidence collection used
ROS CLI frequency probes plus one-message metadata probes instead. This is a
diagnostic-tooling limitation, not a publisher or FAST-LIO2 runtime failure.
