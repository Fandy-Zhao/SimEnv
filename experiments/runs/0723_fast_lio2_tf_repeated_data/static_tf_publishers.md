# TF Publishers — Mechanical Scan (fast-lio2-tf-fix)

**Branch**: `fix/0723-fast-lio2-tf-repeated-data`
**Scan date**: 2026-07-23
**Auto.sh scenario**: `multi_floor_gazeboSim.launch` + `simenv_fast_lio2_mapping.launch`

## Candidate TF Edges

| Edge | Publisher | Node | File:line | Dyn/Static | Timestamp Src | Launch Condition |
|---|---|---|---|---|---|---|
| `map → odom` | TransformBroadcaster | `state_from_gazebo` | `state_from_gazebo.cpp:44` | Dyn | pre-fix: independent `ros::Time::now()` calls; fixed: one guarded callback stamp | `enable_referee_odom`; competition `auto.sh` default **true**, earth mode default **false** |
| `odom → base` | TransformBroadcaster | `state_from_gazebo` | `state_from_gazebo.cpp:77` | Dyn | pre-fix: independent `ros::Time::now()` calls; fixed: one guarded callback stamp | `enable_referee_odom`; competition `auto.sh` default **true**, earth mode default **false** |
| `map → camera_init` | StaticTransformBroadcaster | `map_to_camera_init_bridge` | `map_to_camera_init_bridge.py:64` | Static (one-shot) | `rospy.Time.now()` | Always |
| `camera_init → body` | TransformBroadcaster | `laserMapping` | external FAST_LIO, documented in ADR-0714 and integration README | Dyn | FAST-LIO2 odometry state | `simenv_fast_lio2_mapping.launch` starts `laserMapping` |
| `camera_init → body` | TransformBroadcaster | `odometry_tf_bridge` | `odometry_tf_bridge.py:37` | Dyn | Odometry header stamp | pre-fix default **true**; fixed default **false**, opt-in only |
| `base → imu_link` (URDF chain) | robot_state_publisher | `robot_state_publisher` | `multi_floor_gazeboSim.launch:67` | Dyn | `/joint_states` stamps | Always |
| `world → base` (+ lasers) | TransformBroadcaster | `odom_visualization` | `odom_visualization.cpp:379-382` | Dyn | odom msg stamp | **needs Codex review** — uav_simulator; likely not in auto.sh |
| `body→world`, `intermediate→world` | TransformBroadcaster | `tf_assist` | `tf_assist.py:80-90` | Dyn | odom msg stamp | **needs Codex review** — uav_simulator; likely not in auto.sh |

## Notes

- `livox_points_plugin` (Mid360_imu_sim) has a **commented-out** TransformBroadcaster — inactive.
- `pcl_render_node` (uav_simulator/local_sensing) has a **commented-out** TF publish — inactive.
- `scan_to_pointcloud2.py` optionally re-writes `frame_id` on PointCloud2 (param `~rotated_frame_id`).

## Duplicate / Overlap Concerns

1. **SINGLE_OWNER_DUPLICATE_TIMESTAMP_FOUND**: static code review shows `state_from_gazebo` is the only active owner found for `map → odom` and `odom → base` in the competition launch path, but it reused simulated `ros::Time::now()` without guarding repeated callback stamps.
2. **MULTIPLE_TF_OWNER_FOUND before fix**: project docs/ADR identify FAST-LIO2 `laserMapping` as the `camera_init → body` owner, while `odometry_tf_bridge` also broadcast the relayed Odometry edge by default. The fix makes the bridge opt-in.
3. `odom_visualization` and `tf_assist` are not launched by `auto.sh` or the reviewed competition FAST-LIO2 launch path.
