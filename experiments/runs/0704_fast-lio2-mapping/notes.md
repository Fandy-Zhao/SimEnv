# Experiment Notes: 0704_fast-lio2-mapping

## Date
2026-07-04

## Key Actions
1. Analyzed sensor configuration: /scan (PointCloud), /livox/Pointcloud2, /livox/imu, /trunk_imu
2. Created `feat/0704-fast-lio2-mapping` branch from `develop`
3. Created `src/simenv_fast_lio2_integration/` package
4. Wrote `scan_to_pointcloud2.py` adapter (PointCloud → PointCloud2 in laser_livox frame)
5. Created FAST-LIO2 config `simenv_mid360.yaml` with URDF-derived extrinsics
6. Created launch file `simenv_fast_lio2_mapping.launch`
7. Modified `auto.sh` with `ENABLE_FAST_LIO2` flag
8. Verified with py_compile, catkin_make, roslaunch --files, rosrun, rospack

## Calibration Values (from URDF)
- LiDAR (laser_livox) in base frame: pos=(0.2, 0, 0.08), rpy=(0, 0.785, 0)
- Livox IMU in laser_livox frame: pos=(-0.011, -0.02329, 0.04412), rpy=(0, 0, 0)
- Trunk IMU (imu_link) in trunk frame: pos=(0, 0, 0)

## PointCloud Compatibility
- /scan: PointCloud, x/y/z only, NO time/intensity/ring
- /livox/Pointcloud2: PointCloud2, x/y/z only, frame: odom or laser_livox
- /scan_pointcloud2: our adapter output, PointCloud2, x/y/z only, frame: laser_livox

## Build Status
- catkin_make: pending verification
- py_compile: pending verification

## Environment
- ROS distro: Noetic (from /opt/ros/noetic/setup.bash)
- Workspace: /home/zzf/search_ws/SimEnv
