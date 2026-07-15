# fix/0714-fastlio2-startup-order — Experiment Notes

## Date
2026-07-14

## Changes Made
1. **auto.sh reorder**: Controller starts before FAST-LIO2, auto-commanded FixedStand, IMU upright check
2. **CONTROLLER_FOREGROUND default=1**: Controller runs in background first, then `fg` pulls it back after FAST-LIO2 init
3. **enable_adapter:=false**: Duplicate scan_to_pointcloud2 eliminated
4. **camera_init TF bridge**: Iterated `laser_livox` → `imu_link` → `imu_link+Ry(-45°)` for correct orientation

## Final TF Bridge
`map → imu_link → Ry(-45°) → camera_init`
- Point cloud: horizontal, correct orientation
- Odometry: X axis forward (correct)

## Commits
- db327fb1 refactor(auto.sh): default CONTROLLER_FOREGROUND=0, simplify startup logic
- fd9c4da4 feat(auto.sh): bring controller back to foreground after FAST-LIO2 init
- aaa4477d fix(fastlio2): use imu_link instead of laser_livox for camera_init TF bridge
- 03db4988 fix(fastlio2): add 180° Z flip to camera_init TF bridge
- fd28773b revert: remove 180° Z flip from camera_init TF bridge
- 035d964b fix(fastlio2): use laser_livox + Ry(135) for camera_init TF bridge
- b72ecab9 refactor(fastlio2): use imu_link for camera_init TF bridge again
- 446571e4 exp(fastlio2): try imu_link + Ry(-45 deg) for camera_init bridge
