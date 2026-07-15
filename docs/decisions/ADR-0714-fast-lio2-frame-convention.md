# ADR-0714: FAST-LIO2 Frame Convention & Rotation Responsibility

## Status
Superseded in part by ADR-0715 (2026-07-15). The bridge's no-extra-rotation
decision remains valid; its stated FAST-LIO2 `extrinsic_R` direction does not.

## Context

FAST-LIO2 mapping produces correct point clouds but `/Odometry` body axes appear wrong in RViz: X axis points downward instead of forward. The LiDAR sensor is mounted with a 45° forward tilt (Ry(+45°) in URDF `laser_livox_joint`), while the trunk IMU (`imu_link`) is horizontal and body-aligned.

The project uses two coordinate frame trees:
1. **Gazebo/URDF tree**: `map → odom → base → trunk → imu_link` (from `robot_state_publisher`)
2. **FAST-LIO2 tree**: `camera_init → body` (from `laserMapping`)

A bridge node (`map_to_camera_init_bridge.py`) connects them by publishing `map → camera_init` as a static TF.

## Problem

The bridge was applying `Ry(-45°)` to the `map → imu_link` lookup before publishing `map → camera_init`. This tilted the entire FAST-LIO2 world frame, causing Odometry body axes to appear incorrect in the `map` frame. The rotation was a **duplicate** — the 45° LiDAR tilt is already handled by FAST-LIO2's `extrinsic_R`.

Root cause: `Ry(-45°)` in `map_to_camera_init_bridge.py` line 76 (removed).

## Decision

### Frame Definitions

| Frame | Physical Meaning | Orientation | Publisher |
|-------|-----------------|-------------|-----------|
| `base` | A1 robot chassis root | +X fwd, +Y left, +Z up | `robot_state_publisher` |
| `imu_link` | Trunk IMU (body-aligned) | Same as `base` (identity joint) | `robot_state_publisher` |
| `laser_livox` | LiDAR sensor frame | Tilted Ry(+45°) rel. to `base` | `robot_state_publisher` |
| `livox_imu_link` | LiDAR built-in IMU | Tilted Ry(+45°) rel. to `base` | `robot_state_publisher` |
| `camera_init` | FAST-LIO2 world frame | Aligned with initial `body` pose | Bridge (static TF) |
| `body` | FAST-LIO2 state frame (= IMU frame) | Same as `imu_link` | FAST-LIO2 (`laserMapping`) |

### Rotation Responsibility Boundary

```
URDF (laser_livox_joint)
  → Defines 45° LiDAR tilt (sensor geometry)
  → NO change to data

scan_to_pointcloud2.py (adapter)
  → Message type conversion only (PointCloud → PointCloud2)
  → frame_id = "laser_livox" (unchanged)
  → NO rotation

FAST-LIO2 (extrinsic_R = Ry(-45°))
  → Transforms LiDAR data from laser_livox frame → IMU (body) frame
  → THE ONE AND ONLY place where 45° tilt is compensated

map_to_camera_init_bridge.py
  → Publishes map → camera_init = map → imu_link (direct copy)
  → NO rotation — camera_init aligns with the initial IMU pose
```

### TF Tree (After Fix)

```
map
  ├── odom → base → trunk → imu_link ───────── (Gazebo + robot_state_publisher)
  │              ├── laser_livox → livox_imu_link
  │              └── ...
  │
  └── camera_init → body ────────────────────── (bridge + FAST-LIO2)
       (map → camera_init = map → imu_link, no rotation)
       (body ≡ physical IMU frame)
```

## Why Not Other Approaches

### Why not change extrinsic_R?
The extrinsic_R = Ry(-45°) is mathematically correct: it computes the LiDAR→IMU relative transform from URDF joint definitions. Changing it would break point cloud registration.

### Why not tilt IMU data before FAST-LIO2?
This would require also transforming angular velocity, covariance, and maintaining temporal alignment — high risk of introducing errors.

### Why not make `body = base_link` directly?
FAST-LIO2's `body` is defined by the IMU frame_id of incoming data. The trunk IMU publishes in `imu_link`, which is at identity relative to `base`. So `body` and `base` share the same orientation. Renaming them would require modifying FAST-LIO2 source code (out of scope).

### Why not add `body → base` static TF?
This would create a TF ownership conflict: FAST-LIO2 publishes `camera_init → body` dynamically, and a static publisher would fight with it. Not safe.

## Consequences

### Positive
- Odometry body axes now correctly show +X forward, +Y left, +Z up
- No duplicate rotations in the sensor→SLAM pipeline
- Clear rotation responsibility boundary documented
- Point cloud mapping quality unchanged (verified by mathematical analysis)

### Negative / Risks
- If the `imu_link` frame is not horizontal at FAST-LIO2 initialization time (e.g., robot not yet in FixedStand), the `camera_init` orientation will match that initial tilt. This is a pre-existing startup-order concern, not introduced by this fix.
- Navigation modules that directly reference `body` frame may need adjustment (they should use `base` as configured in costmap params).

### Rollback
To revert: re-add the `Ry(-45°)` rotation and `_rotate_by` function in `map_to_camera_init_bridge.py`. The YAML and launch comment changes are cosmetic and do not affect behavior.
