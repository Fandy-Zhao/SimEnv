# Repository Audit Report — FAST-LIO2 Frame Correction
## Date: 2026-07-14
## Branch: zzf/0714-fast-lio2-frame-fix (created from develop @ c6723988)

---

## 1. Sensor Mounting (from URDF: robot.xacro)

### TF Tree in URDF

```
world (Gazebo)
  └── base (root link, spawned by Gazebo)
        ├── trunk (identity: rpy="0 0 0", xyz="0 0 0")
        │     └── imu_link (identity: rpy="0 0 0", xyz="0 0 0")  ← TRUNK IMU
        ├── laser_livox (rpy="0 0.785 0", xyz="0.2 0 0.08")       ← LiDAR, 45° tilt
        │     └── livox_imu_link (rpy="0 0 0", xyz="-0.011 -0.02329 0.04412") ← LiDAR built-in IMU
        └── real_sense → real_sense_optical_frame
```

### Sensor Topics

| Physical Sensor | ROS Topic | frame_id (in header) | Orientation |
|----------------|-----------|---------------------|-------------|
| Trunk IMU (imu_link) | `trunk_imu` | `imu_link` | Horizontal, z-up |
| LiDAR (laser_livox) | `/scan` (PointCloud) | `laser_livox` | Tilted 45° forward (Ry(+45°)) |
| LiDAR built-in IMU (livox_imu_link) | `/livox/imu` | `livox_imu_link` | Tilted 45° forward (shares LiDAR orientation) |

### URDF Joint Details

- `laser_livox_joint`: parent=`base`, child=`laser_livox`, origin=`xyz="0.2 0 0.08" rpy="0 0.785 0"`
  - 0.785 rad = 45° — the LiDAR is pitched FORWARD by 45°
- `imu_joint`: parent=`trunk`, child=`imu_link`, origin=`xyz="0 0 0" rpy="0 0 0"` — IDENTITY
- `livox_imu_joint`: parent=`laser_livox`, child=`livox_imu_link`, origin=`xyz="-0.011 -0.02329 0.04412" rpy="0 0 0"` — same orientation as LiDAR

---

## 2. Data Pipeline

```
Gazebo LiDAR plugin (liblivox_laser_simulation.so)
  → /scan (sensor_msgs/PointCloud, frame_id="laser_livox")
  → scan_to_pointcloud2.py adapter
  → /scan_pointcloud2 (sensor_msgs/PointCloud2, frame_id="laser_livox")
  → FAST-LIO2 (preprocess.cpp → laserMapping.cpp)
  → /cloud_registered (frame_id="camera_init")
  → /Odometry (frame_id="camera_init", child_frame_id="body")

Gazebo IMU plugin (libgazebo_ros_imu_sensor.so)
  → /trunk_imu (sensor_msgs/Imu, frame_id="imu_link")
  → FAST-LIO2 (IMU_Processing.hpp)
```

### Adapter (scan_to_pointcloud2.py)
- Takes `/scan` PointCloud → republishes as PointCloud2
- Sets `header.frame_id = "laser_livox"` (unchanged from input)
- **NO rotation of point data** — message type conversion only
- Adds intensity=1.0 to every point (required by FAST-LIO2 lidar_type=4)

### Bridge (map_to_camera_init_bridge.py)
- Waits for first `/Odometry` message
- Looks up `map → imu_link` at the timestamp of that first Odometry
- **Applies Ry(-45°)** to the transform
- Publishes static TF: `map → camera_init`

---

## 3. FAST-LIO2 Configuration (simenv_mid360.yaml)

- `imu_topic: "/trunk_imu"` — uses horizontal trunk IMU (NOT the tilted livox IMU)
- `lid_topic: "/scan_pointcloud2"` — PointCloud2 in laser_livox frame
- `extrinsic_est_en: false` — extrinsic is fixed, not estimated online
- `extrinsic_T: [-0.085, 0.0, -0.198]`
- `extrinsic_R: [0.7071, 0, -0.7071, 0, 1, 0, 0.7071, 0, 0.7071]` — Ry(-45°)

---

## 4. FAST-LIO2 Source Code Verification

### Point Transformation (laserMapping.cpp:169,181,193)
```cpp
V3D p_global = s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos;
```
Where:
- `p_body` — point in **LiDAR frame** (misnamed; actually LiDAR, not body)
- `offset_R_L_I` — rotation from LiDAR to IMU (R_L_I)
- `offset_T_L_I` — translation from LiDAR to IMU
- `s.rot`, `s.pos` — IMU body pose in world (`camera_init`) frame

### Extrinsic Loading (laserMapping.cpp:818-820)
```cpp
Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
```

### Odometry Publishing (laserMapping.cpp:589-619)
```cpp
odomAftMapped.header.frame_id = "camera_init";
odomAftMapped.child_frame_id = "body";
// ...
br.sendTransform(tf::StampedTransform(transform, stamp, "camera_init", "body"));
```

### Key Convention
FAST-LIO2 treats `body` = IMU frame. Since we use `trunk_imu` (frame_id=`imu_link`, horizontal), `body` IS the horizontal IMU frame.

---

## 5. Navigation Configuration

From `unitree_move_base/config/costmap_params.yaml`:
```yaml
global_frame: odom
robot_base_frame: base
```

The `base` link is the URDF root, horizontal, z-up, same orientation as `imu_link` and `body`.

---

## 6. Current Issues Identified

### Issue 1: Duplicate Rotation in Bridge (CRITICAL)
The bridge applies Ry(-45°) when creating `map → camera_init`. This is WRONG because:
- The LiDAR 45° tilt is ALREADY handled by FAST-LIO2's `extrinsic_R`
- The bridge should simply do `map → camera_init = map → imu_link (at t=0)` without rotation
- Adding Ry(-45°) tilts the entire FAST-LIO2 world frame by 45°, causing Odometry axes to appear tilted

### Issue 2: Unconnected TF Trees
FAST-LIO2 publishes `camera_init → body` (body = IMU frame).
URDF publishes `base → imu_link` (imu_link = IMU frame, identity from base).
`body` and `imu_link` represent the same physical frame but are separate in the TF tree.
Navigation uses `base` as robot_base_frame.

### Issue 3: Odometry child_frame_id = "body" 
Navigation expects `base_link` (or in this project, `base`). The `body` frame is not directly recognized by navigation.

### Non-Issue: extrinsic_R/YAML
The extrinsic is mathematically correct: it correctly computes the LiDAR→IMU transform from the URDF joint definitions. Ry(-45°) is the right rotation here.
