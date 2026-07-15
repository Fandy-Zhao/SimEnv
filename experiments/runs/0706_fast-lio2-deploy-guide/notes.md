# Experiment Notes — 0706 Fast-LIO2 Deployment Guide

## Task
整理 FAST-LIO2 在 SimEnv 中的部署流程、传感器配置与关键超参数说明文档。

## Date
2026-07-06

## Branch
docs/0706-fast-lio2-deploy-guide (from develop)

## Scan Results

### Workspace Structure
- SimEnv = catkin workspace root ✅
- `.catkin_workspace` present
- FAST_LIO at `src/FAST_LIO`
- livox_ros_driver at `src/livox_ros_driver`
- simenv_fast_lio2_integration at `src/simenv_fast_lio2_integration`

### Sensor Configuration Scan
Raw scan outputs saved to:
- `experiments/runs/0706_fast-lio2-deploy-guide/sensor_config_scan.txt`
- `experiments/runs/0706_fast-lio2-deploy-guide/fast_lio_param_scan.txt`
- `experiments/runs/0706_fast-lio2-deploy-guide/existing_docs_scan.txt`

### Key URDF/Xacro Findings
- LiDAR: base → laser_livox joint: xyz=(0.2, 0, 0.08), rpy=(0, 0.785, 0)
- Livox IMU: laser_livox → livox_imu_link joint: xyz=(-0.011, -0.02329, 0.04412), rpy=(0, 0, 0)
- Trunk IMU: trunk → imu_link joint: xyz=(0, 0, 0), rpy=(0, 0, 0)
- LiDAR plugin: liblivox_laser_simulation.so, 10 Hz, 24000 samples, 0.1-40m range
- Livox IMU plugin: libgazebo_ros_imu_sensor.so, 1000 Hz, noise=0.0
- Trunk IMU plugin: libgazebo_ros_imu_sensor.so, 1000 Hz, noise=0.0

### FAST_LIO Reference Config (mid360.yaml)
- Default lid_topic: "/livox/lidar" (overridden in simenv_mid360.yaml)
- Default imu_topic: "/livox/imu"
- lidar_type: 1, scan_line: 4, blind: 0.5
- extrinsic_T: [-0.011, -0.02329, 0.04412]
- extrinsic_R: identity

### Existing Reports
- ADR-0704-fast-lio2-mapping.md: architecture decisions
- 0704_fast-lio2-mapping.md: integration skeleton report
- 0704_fast-lio2-build-errors: build error logs and fixes
- 0704_build-with-venv.md: venv build setup
- 0704_cuda-gcc11-build: CUDA/gcc compatibility

## Documents Created
1. `docs/slam/fast_lio2_deployment_guide.md` — main deployment guide (15 sections)
2. `docs/reports/0706_fast-lio2-deploy-guide.md` — task report
3. `docs/decisions/ADR-0706-fast-lio2-deploy-guide.md` — architecture decision record

## Documents Updated
- `PROJECT_STATE.md`
- `CHANGELOG.md`
- `docs/module_status.md`

## Open Items for Future Work
- See §15 in deployment guide for 10 open questions
- Recommend next: `feat/0706-fast-lio2-runtime-smoke-test`
