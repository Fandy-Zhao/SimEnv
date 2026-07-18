# Runtime Inventory — FAST-LIO2 Pointcloud Orientation
**Date:** 2026-07-18  **Branch:** `fix/0718-runtime-pointcloud-orientation`

## Key Files
| File | Role | Rotation Params? |
|---|---|---|
| `src/simenv_fast_lio2_integration/scripts/scan_to_pointcloud2.py` | PointCloud→PointCloud2 adapter; reads `~rotation_y_deg` (default 0), `~rotation_x_deg` (default 0), `~rotated_frame_id` | Source definition only (lines 102–122) |
| `src/simenv_fast_lio2_integration/launch/simenv_fast_lio2_mapping.launch` | Launches adapter + FAST-LIO2; line 71: "No extra rotation is applied" | **Does NOT set** rotation_y_deg or rotation_x_deg |
| `src/simenv_fast_lio2_integration/config/simenv_mid360.yaml` | FAST-LIO2 config; `lid_topic: /scan_pointcloud2`; docs `laser_livox` joint | N/A |
| `src/unitree_guide/.../xacro/robot.xacro` | Defines `laser_livox` link + `laser_livox_joint` (fixed, 45° tilt) | N/A |
| `auto.sh` | Orchestration script; sets `ROS_PACKAGE_PATH`, `CMAKE_PREFIX_PATH`; `ENABLE_FAST_LIO2` flag | **Does NOT set** |
| `experiments/runs/0717_fastlio2-stage2/run_overlay_runtime.sh` | Runtime overlay; runs `scan_to_pointcloud2.py` + mapping launch | **Does NOT set** |

## Rotation Param Status
- **`rotation_y_deg`**: default 0 in source; **no launch/shell/YAML sets `-90`**.
- **`rotation_x_deg`**: default 0 in source; **no launch/shell/YAML sets `180`**.
- **`rotated_frame_id`**: unset at runtime (only required when rotations non-zero).

## Conclusion
At runtime, `scan_to_pointcloud2.py` runs with both rotations at their defaults (0,0) — no orientation override is applied. The LiDAR 45° tilt is handled by the URDF `laser_livox_joint`, not by the adapter node.
