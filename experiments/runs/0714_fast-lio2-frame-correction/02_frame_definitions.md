# Coordinate Frame Definitions — SimEnv + FAST-LIO2
## Based on code evidence, not assumptions

---

## Frame Table

| Frame | Physical Object | +X | +Y | +Z | Parent Frame | Defined In | Publisher |
|-------|----------------|----|----|----|-------------|------------|-----------|
| `world` | Gazebo world origin | East | North | Up | (root) | Gazebo | Gazebo |
| `map` | ROS world (Gz→ROS bridge) | East | North | Up | `world` (identity) | Gazebo ROS | `gazebo_ros_api` |
| `odom` | Gazebo odometry | East | North | Up | `map` | Gazebo ROS | `gazebo_ros_api` |
| `base` | A1 robot chassis root | **Forward** | **Left** | **Up** | `odom` | URDF (spawned) | `robot_state_publisher` |
| `trunk` | A1 trunk body (visual + collision) | Forward | Left | Up | `base` (identity) | URDF | `robot_state_publisher` |
| `imu_link` | Trunk IMU (body-aligned) | Forward | Left | Up | `trunk` (identity) | URDF | `robot_state_publisher` |
| `laser_livox` | LiDAR Mid-360 sensor | **Tilted 45° forward** | Left | **Tilted 45° up** | `base` (Ry+45°) | URDF | `robot_state_publisher` |
| `livox_imu_link` | LiDAR built-in IMU | Tilted 45° forward | Left | Tilted 45° up | `laser_livox` (identity rot) | URDF | `robot_state_publisher` |
| `camera_init` | FAST-LIO2 world frame (first LiDAR pose) | **TBD** | **TBD** | **TBD** | `map` (via bridge, with Ry(-45°)) | Bridge | `map_to_camera_init_bridge` |
| `body` | FAST-LIO2 body frame (= IMU frame) | Forward | Left | Up | `camera_init` | FAST-LIO2 | `laserMapping` (Odometry + TF) |

---

## Key Answers

### 1. FAST-LIO2 `body` actual meaning

`body` = IMU frame. Since `imu_topic: "/trunk_imu"` (published in `imu_link` frame),
`body` ≡ `imu_link` (horizontal, body-aligned).

Evidence from `laserMapping.cpp:591-592`:
```cpp
odomAftMapped.header.frame_id = "camera_init";
odomAftMapped.child_frame_id = "body";
```

### 2. `/Odometry.pose` describes:

Pose of `body` (= IMU = `imu_link`) in `camera_init` frame.

### 3. `/cloud_registered` frame:

Published in `camera_init` frame (laserMapping.cpp:496,563,572).

### 4. Point cloud rotation before FAST-LIO2

**NO.** The `scan_to_pointcloud2.py` adapter does NOT rotate data — only changes message type and sets `frame_id = "laser_livox"`.

### 5. IMU data rotation before FAST-LIO2

**NO.** The IMU data is published by Gazebo in `imu_link` frame (horizontal, z-up) and consumed by FAST-LIO2 as-is.

### 6. `header.frame_id` consistency

- `/scan_pointcloud2`: frame_id=`laser_livox` — data IS in `laser_livox` frame ✓
- `/trunk_imu`: frame_id=`imu_link` — data IS in `imu_link` frame ✓
- `/Odometry`: frame_id=`camera_init`, child=`body` — correct given FAST-LIO2 convention ✓
- `/cloud_registered`: frame_id=`camera_init` — correct ✓

---

## TF Tree (Current, With Issues)

```
map ───────────────────────────────────────────────────── (Gazebo)
  ├── odom ── base ── trunk ── imu_link ──────────────── (robot_state_publisher)
  │                    ├── laser_livox ── livox_imu_link
  │                    └── real_sense ── real_sense_optical_frame
  │
  └── camera_init ── body ──────────────────────────────── (bridge + FAST-LIO2)
       (map→camera_init has WRONG Ry(-45°))
```

## TF Ownership

| Transform | Publisher |
|-----------|-----------|
| `map → odom` (identity here) | `gazebo_ros_api` |
| `odom → base` | Gazebo or controller |
| `base → *` (all robot links) | `robot_state_publisher` |
| `map → camera_init` (static) | `map_to_camera_init_bridge` |
| `camera_init → body` | FAST-LIO2 `laserMapping` |

### Conflict Check
- `body` and `imu_link`: same physical frame but DIFFERENT TF frames → potential confusion for navigation
- No duplicate publisher for any frame detected
