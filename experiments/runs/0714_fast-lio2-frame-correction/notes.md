# Experiment Notes — FAST-LIO2 Frame Correction
## Date: 2026-07-14
## Branch: zzf/0714-fast-lio2-frame-fix

## Summary

Diagnosed and fixed incorrect `/Odometry` body axes caused by a duplicate Ry(-45°) rotation in `map_to_camera_init_bridge.py`.

## Key Finding

The bridge was applying Ry(-45°) when publishing `map→camera_init`, tilting FAST-LIO2's entire world frame. The 45° LiDAR tilt was already correctly handled by FAST-LIO2's `extrinsic_R`. This was a duplicate rotation.

## Changes

1. `map_to_camera_init_bridge.py`: Removed Ry(-45°), removed unused `_rotate_by` and imports
2. `simenv_fast_lio2_mapping.launch`: Updated stale comment (laser_livox → imu_link)
3. `simenv_mid360.yaml`: Added rotation responsibility boundary documentation

## Verification

| Check | Result |
|-------|--------|
| Python syntax | PASS |
| YAML syntax | PASS |
| Launch XML syntax | PASS |
| catkin_make build | PASS |
| Rotation matrix validity (det≈1, orthogonal) | PASS |
| ROS runtime (stationary, straight, rotation, 5-min) | NOT RUN (ROS master not available) |

## Rotation Matrix

extrinsic_R = Ry(-45.0°), det=0.999981, orthogonality error=1.92e-05
