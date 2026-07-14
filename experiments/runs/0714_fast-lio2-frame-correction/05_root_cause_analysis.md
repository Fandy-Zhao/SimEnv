# Root Cause Analysis — FAST-LIO2 Frame Issues
## Date: 2026-07-14
## Task: zzf/0714-fast-lio2-frame-fix

---

## Answers to Required Questions

### 1. Is LiDAR tilted independently?

**YES.** The LiDAR (`laser_livox`) is tilted 45° forward (Ry(+45°) = 0.785 rad) relative to `base`, as defined in `robot.xacro:146`:
```xml
<joint name="laser_livox_joint" type="fixed">
    <origin xyz="0.2 0 0.08" rpy="0 0.785 0"/>
    <parent link="base"/>
    <child link="laser_livox"/>
</joint>
```

### 2. Does IMU tilt with LiDAR?

**NO.** The trunk IMU (`imu_link`) is at identity rotation relative to `trunk`/`base` (`robot.xacro:103-107`):
```xml
<joint name="imu_joint" type="fixed">
    <parent link="trunk"/>
    <child link="imu_link"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
</joint>
```
The trunk IMU is horizontal, z-up, body-aligned.

**However**, there is a SECOND IMU: `livox_imu_link` (child of `laser_livox`) which shares the LiDAR's 45° tilt. This second IMU publishes on `/livox/imu` and is NOT used by FAST-LIO2 (the YAML uses `imu_topic: "/trunk_imu"`).

### 3. Is IMU data consistent with header.frame_id?

**YES.** `/trunk_imu` has `frame_id: imu_link`, and the data represents acceleration/angular velocity in the `imu_link` frame. The `imu_link` is horizontal (z-up), so gravity should appear mainly on the +Z axis (or -Z depending on Gazebo convention).

### 4. Is point cloud data consistent with header.frame_id?

**YES.** `/scan_pointcloud2` has `frame_id: laser_livox`, and the points are in the `laser_livox` frame (tilted 45° forward). No rotation has been applied by the adapter.

### 5. Does the PointCloud2 adapter rotate data?

**NO.** `scan_to_pointcloud2.py` performs message type conversion only (PointCloud → PointCloud2), with no geometric transformation. It sets `frame_id = "laser_livox"`.

### 6. Does the URDF already encode the sensor tilt?

**YES.** The 45° LiDAR tilt IS encoded in the URDF via the `laser_livox_joint` with `rpy="0 0.785 0"`.

### 7. Does FAST-LIO2 extrinsic_R duplicate the URDF rotation?

**NO — it's the necessary complement.** The extrinsic_R correctly computes the LiDAR→IMU relative transform:
```
R_imu_lidar = inverse(R_base_lidar) * R_base_imu
            = inverse(Ry(45°)) * Identity
            = Ry(-45°)
```
This is the correct and necessary transform for FAST-LIO2 to convert LiDAR measurements into the IMU frame. It does NOT "duplicate" the URDF — it's the mathematical consequence of the URDF joint definitions.

### 8. Why do point clouds appear correct?

The point cloud pipeline is mathematically consistent:
1. LiDAR data is generated in `laser_livox` frame (tilted 45°)
2. FAST-LIO2 applies `extrinsic_R = Ry(-45°)` to transform to IMU frame
3. FAST-LIO2 transforms from IMU→world using the estimated state
4. Result: correctly oriented point cloud in `camera_init` frame

When viewed in RViz in the `camera_init` frame, point clouds appear horizontal because FAST-LIO2 correctly compensates for the LiDAR tilt internally.

### 9. DIRECT ROOT CAUSE: Why is /Odometry X pointing downward?

**The `map_to_camera_init_bridge.py` applies a spurious Ry(-45°) rotation.**

Here's the mechanism:

1. FAST-LIO2 initializes `camera_init` as the identity pose of the `body` (IMU) frame at the first LiDAR frame. Since the IMU is horizontal (z-up), `camera_init` is a horizontal world frame.

2. The state `s.rot = Identity` at t=0 means `body` is aligned with `camera_init`. So Odometry (`camera_init → body`) starts at identity — body axes are:
   - +X forward
   - +Y left
   - +Z up (in `camera_init` frame)

3. The bridge publishes `map → camera_init` = `T(map→imu_link) * Ry(-45°)`.

4. When RViz resolves the `body` frame in the `map` frame:
   ```
   map → body = map → camera_init * camera_init → body
              = [T * Ry(-45°)] * [Identity]
              = T * Ry(-45°)
   ```
   
5. The `body` frame is rotated by -45° around Y in the `map` frame. At t=0, this means:
   - +X of `body` in `map` frame: forward component + downward component → **X points partly downward**
   - The rotation is 45°, so X has both forward and downward components

6. As the robot moves, the Odometry accumulates in `camera_init` (horizontal), but the bridge keeps `camera_init` tilted by -45° relative to `map`. The `/Odometry` visualization shows `body` axes in the `camera_init` frame, which IS correct internally — but the bridge tilt makes them appear wrong relative to the `map` frame.

**The Ry(-45°) in the bridge is the SOLE cause of the Odometry axes appearing wrong.**

### 10. What needs to be fixed?

**Primary fix:** Remove the Ry(-45°) from `map_to_camera_init_bridge.py`.

The bridge should publish:
```
map → camera_init = map → imu_link (at t=0, NO rotation added)
```

This makes `camera_init` align with `map` (both horizontal, z-up), and `body` appears correct in the `map` frame.

**Secondary consideration:** The `body` frame (FAST-LIO2's IMU frame) and `base` frame (URDF root) are both horizontal and represent the same physical chassis, but are separate TF frames. Navigation uses `robot_base_frame: base`. This disconnect is a pre-existing architectural issue, not introduced by the bridge rotation. Fixing it requires careful design (adding `body → base` static TF could create ownership conflicts).

**Non-issue:** FAST-LIO2's `extrinsic_R` and `extrinsic_T` are mathematically correct and should NOT be changed.

---

## Evidence Summary

| Item | Finding |
|------|---------|
| LiDAR tilt | 45° forward (Ry(+45°) in URDF) |
| Trunk IMU orientation | Horizontal, z-up (identity in URDF) |
| FAST-LIO2 uses which IMU | `/trunk_imu` (horizontal) — from YAML |
| extrinsic_R correctness | Ry(-45°) IS the correct LiDAR→IMU transform |
| Adapter rotation | NONE — correct |
| Bridge rotation | Ry(-45°) — **WRONG, must be removed** |
| Point cloud correct because | FAST-LIO2 extrinsic correctly handles the LiDAR tilt |
| Odometry X wrong because | Bridge tilts camera_init by -45°, which tilts body in map frame |
| Duplicate rotations | Bridge Ry(-45°) partially duplicates extrinsic_R effect on the WORLD frame |
| TF ownership conflicts | None detected (no duplicate publishers) |
