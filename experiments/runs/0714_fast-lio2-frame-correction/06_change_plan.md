# Change Plan — FAST-LIO2 Frame Correction
## Date: 2026-07-14

---

## Root Cause Summary

The `map_to_camera_init_bridge.py` applies a spurious **Ry(-45°)** rotation when publishing `map → camera_init`. This tilts the entire FAST-LIO2 world frame by 45°, causing Odometry axes to appear wrong in RViz. The 45° LiDAR tilt is already correctly handled by FAST-LIO2's `extrinsic_R`, so this bridge rotation is a duplicate.

---

## Change List

### File 1: `src/simenv_fast_lio2_integration/scripts/map_to_camera_init_bridge.py`

| Field | Detail |
|-------|--------|
| **Current problem** | Applies Ry(-45°) rotation to `map → imu_link` when creating `map → camera_init` |
| **Proposed change** | Remove the Ry(-45°) rotation; use `map → imu_link` directly as `map → camera_init` |
| **Risk** | LOW — the rotation is a duplicate of FAST-LIO2's correct extrinsic handling |
| **Verification** | Stationary test: Odometry X should point forward (not down), body axes should be horizontal |

**Changes:**
1. Remove `_rotate_by()` function call (line 76 — the Ry(-45°) application)
2. Use `body_in_map.transform` directly instead of `tf_aligned`
3. Update log message from `"imu_link + Ry(-45 deg)"` to `"imu_link (direct)"` or similar
4. Remove the `_rotate_by()` function if no longer used anywhere

### File 2: `src/simenv_fast_lio2_integration/launch/simenv_fast_lio2_mapping.launch`

| Field | Detail |
|-------|--------|
| **Current problem** | Comment line 49 says bridge looks up `map → laser_livox` but code actually looks up `map → imu_link` (stale) |
| **Proposed change** | Update comment to reflect actual behavior: `map → imu_link` |
| **Risk** | NONE — documentation only |
| **Verification** | Visual inspection |

### File 3: `src/simenv_fast_lio2_integration/config/simenv_mid360.yaml`

| Field | Detail |
|-------|--------|
| **Current problem** | No issue with extrinsic_R/T — they are correct |
| **Proposed change** | Add clarifying comment about bridge non-rotation; keep extrinsic unchanged |
| **Risk** | NONE — comment only |
| **Verification** | Visual inspection |

---

## Files NOT Modified

| File | Reason |
|------|--------|
| `src/FAST_LIO/**` | Core algorithm correct; extrinsic_R/T are mathematically correct |
| `src/simenv_fast_lio2_integration/scripts/scan_to_pointcloud2.py` | Correct — no rotation applied |
| `src/unitree_guide/**/robot.xacro` | URDF sensor poses are correct |
| `src/unitree_guide/**/gazebo.xacro` | Sensor plugin frame_ids are correct |
| `auto.sh` | Fast-LIO2 startup order and parameters correct |
| `generated_building/**` | Not related to frame issue |

---

## What This Fix Does

### Before (WRONG):
```
map → camera_init = map → imu_link * Ry(-45°)  ← tilts world frame!
camera_init → body = FAST-LIO2 state (body = IMU, horizontal in camera_init)
map → body = map → imu_link * Ry(-45°) * Identity = tilted by -45° in map
```

### After (CORRECT):
```
map → camera_init = map → imu_link  ← no tilt, aligned with map
camera_init → body = FAST-LIO2 state (body = IMU, horizontal in camera_init)
map → body = map → imu_link * Identity = horizontal in map ✓
```

---

## Why Point Clouds Won't Break

FAST-LIO2's point cloud registration is INTERNAL to the `camera_init` frame:
1. LiDAR data in `laser_livox` → FAST-LIO2 applies `extrinsic_R = Ry(-45°)` → IMU frame
2. IMU frame → `camera_init` (world) via estimated state
3. `/cloud_registered` published in `camera_init`

Removing the bridge rotation changes ONLY the `map → camera_init` link in the TF tree. It does NOT change how FAST-LIO2 processes points internally. The point cloud in `camera_init` frame remains identical.

When RViz displays `/cloud_registered` in the `map` frame, the visual result depends on the TF chain `map → camera_init`. With the fix:
- `map → camera_init` has correct rotation (no tilt)
- Points in `camera_init` → map appear with correct rotation
- Walls vertical, floor horizontal ✓

---

## Implementation Order

1. Edit `map_to_camera_init_bridge.py`: remove Ry(-45°), simplify code
2. Update launch file comment
3. Update YAML comment
4. Build check: `catkin_make` (this is a Python script, so mainly syntax check)
5. Runtime verification (see Phase 8)
