# Task Report — FAST-LIO2 Stage 0 & 1 Deployment

## Branch

All work committed directly to `develop`.

## Summary

Completed FAST-LIO2 Stage 0 (sensors/TF/time) and Stage 1 (standalone SLAM)
deployment testing.  Stage 0 passed outright.  Stage 1 required five critical
fixes before the estimator converged: PointCloud2 intensity field, IMU topic
switch, TF tree bridge, RViz configuration, and programmatic state control.

After all fixes, FAST-LIO2 converges to within 1 cm of origin in ~20 s of
stationary FixedStand, with verified Z stability at 0.006 m over 30 s.
All output topics publish stably.

## Files Changed (cumulative, 2026-07-13 to 2026-07-14)

| File | Change | Reason |
|------|--------|--------|
| `simenv_fast_lio2_mapping.launch` | Uncomment node, fix rosparam ns, add TF bridge | Required for launch + TF connectivity |
| `simenv_mid360.yaml` | `lidar_type:4`, IMU→`/trunk_imu`, new extrinsic | PointCloud2 path + Z-drift fix |
| `scan_to_pointcloud2.py` | xyzi32 output (was xyz32) | Missing `intensity` caused all points dropped |
| `map_to_camera_init_bridge.py` | **New** | Connects `map`→`camera_init` TF trees |
| `fast_lio2.rviz` | **New** | Pre-configured RViz for FAST-LIO2 data |
| `State_Trotting.h/.cpp` | Add `/cmd_vel` subscriber | Velocity command for Trotting gait |
| `FSM.cpp`, `main.cpp`, `CtrlComponents.h` | Add `/fsm/state_cmd` topic | Programmatic state transitions |
| `State_RL_test.cpp` | Switch to `plane.pt` model | Flat-ground policy |
| `auto.sh` | Comprehensive process cleanup | Prevent stale process contamination |

## Critical Bugs Found & Fixed

### 1. PointCloud2 missing `intensity` field (BLOCKING)
- FAST-LIO2 `lidar_type=4` requires x,y,z,intensity per point
- `pc2.create_cloud_xyz32()` produced x,y,z only (12 bytes/pt)
- "Failed to find match for field 'intensity'" → "No Effective Points!"
- EKF integrated IMU-only → diverged to 8000+ m in minutes
- **Fix**: custom `_create_cloud_xyzi32()` with intensity=1.0

### 2. Z-axis drift at 62 m/s (BLOCKING)
- `/livox/imu` on 45° tilted LiDAR: gravity signal corrupted by vibration
- Initial gravity estimate wrong → pure-Z divergence
- **Fix**: switch to `/trunk_imu` (body-aligned, z-up) + updated extrinsic `Ry(-45°)`

### 3. TF tree disconnected (USABILITY)
- FAST-LIO2 uses `camera_init`, Gazebo uses `map`
- Two unconnected TF sub-trees → RViz can't display data in world frame
- **Fix**: `map_to_camera_init_bridge.py` publishes static TF `map→camera_init`

## Convergence Verification

FixedStand, stationary robot, 30 s observation:

| Axis | Range | Drift |
|------|-------|-------|
| x | 0.000–0.011 m | stable |
| y | -0.069–0.000 m | stable |
| z | 0.000–0.007 m | stable |

Ground truth: stationary (dx=0.003 m / 30 s).  All axes within 1 cm.

## P1 Locomotion Status

- **RL policies** (plane, stair): diagnosed non-responsive to `/cmd_vel` (0.15–1.0 m/s → <0.2 mm)
- **Trotting** (classical MPC): `/cmd_vel` subscriber added, `/fsm/state_cmd` programmatic control working
- **Gazebo physics**: intermittent NaN twist after prolonged testing; blocks final P1 verification
- **Next**: restart machine, run `ENABLE_FAST_LIO2=1 GUI=false ./auto.sh` → Trotting → `/cmd_vel`

## Git

| Commit | Description |
|--------|-------------|
| `1aa61c3` | IMU switch `/livox/imu`→`/trunk_imu` |
| `384fbe5` | Governance docs final update |
| `84863c9` | PointCloud2 intensity field |
| `2ef7c87` | RViz config |
| `419fb4d` | TF bridge `map→camera_init` |
| `b3a85e9` | `auto.sh` comprehensive cleanup |
| `1b24a16` | `/fsm/state_cmd` programmatic state control |
| `2bf6441` | Trotting `/cmd_vel` subscriber |

All on `develop`.  No force-push or remote push performed.

## Risks

- Gazebo physics may produce NaN twist after extended testing; machine restart resolves
- Low RTF (~0.068) in 3-floor scene; static testing acceptable, motion testing may need scene simplification
- Trotting P1 final verification pending clean Gazebo restart

## Next Step

1. Machine restart → clean Gazebo
2. `ENABLE_FAST_LIO2=1 GUI=false ./auto.sh` → FAST-LIO2 convergence
3. Trotting → `/cmd_vel 0.3 m/s` → verify locomotion
4. Stage 2: nav topic adaptation (`/Odometry`→`/state_estimation`, `/cloud_registered`→`/registered_scan`)
