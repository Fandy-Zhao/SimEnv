# Changelog

## 2026-07-06

### Documentation
- Added `docs/slam/fast_lio2_deployment_guide.md`: comprehensive FAST-LIO2 deployment guide covering repository layout, sensor topic mapping, parameter reference, pointcloud compatibility, IMU selection, extrinsic calibration, build environment, deployment steps, runtime validation checklist, common failure modes, experiment tracking parameters, and output contract for future navigation (docs/0706-fast-lio2-deploy-guide).
- Added `docs/decisions/ADR-0706-fast-lio2-deploy-guide.md`: architecture decisions for deployment guide (docs directory, no-vendor policy, default IMU selection, PointCloud time field risk, output contract).
- Added `docs/reports/0706_fast-lio2-deploy-guide.md`: task report with coverage table.

## 2026-07-04

### Documentation
- Fixed FAST-LIO2 workspace layout documentation: SimEnv is the catkin workspace root, FAST_LIO belongs at `SimEnv/src/FAST_LIO`, not nested under another `catkin_ws/`.
- FAST-LIO2 build environment audit: static checks all pass; catkin_make blocked by missing libtorch (C++ SDK) at hardcoded path in unitree_guide. Logs at `experiments/runs/0704_fast-lio2-build-check/`.
- `tools/build_with_venv.sh`: added auto-detection of pip torch CMake prefix (`torch.utils.cmake_prefix_path`), passes `-DCMAKE_PREFIX_PATH` to catkin_make without overwriting ROS paths.
- `tools/build_with_venv.sh`: auto-selects gcc-11/g++-11 for CUDA 11.8 compatibility; passes CC/CXX/CUDAHOSTCXX + CUDA paths to catkin_make. CUDA host compiler errors eliminated.
- Build fixes: unitree PIE linker (`-no-pie`), FAST_LIO C++14→17, livox_ros_driver C++11→17, PCL shared_ptr serialization, missing `<memory>` includes in Livox-SDK.

### Build Tooling
- Added `tools/build_with_venv.sh`: builds catkin workspace with project `.venv` Python, ensuring consistent interpreter for torch and other Python deps.
- README updated with venv setup and build instructions (torch 2.0.1 pin for Python 3.8 / ROS Noetic).

### Governance & Remote Configuration
- Added GitHub remote (`github` → `git@github.com:zzf/SimEnv.git`), origin retained as Gitee.
- Branch naming policy: maintenance/setup branches now use `zzf/MMDD-short-name`; `chore/` prefix is deprecated for this project.
- Initialized project governance skeleton (AGENTS.md, PROJECT_STATE.md, ROADMAP.md, docs/architecture.md, docs/module_status.md).

### FAST-LIO2 Mapping Integration (feat/0704-fast-lio2-mapping)
- Added `src/simenv_fast_lio2_integration/` ROS package with PointCloud adapter, FAST-LIO2 config, and launch files.
- Added `ENABLE_FAST_LIO2` optional flag in auto.sh.
- FAST-LIO2 operates as external catkin workspace dependency (not vendored).

## Historical (from git log)

### 2025
- `a46d947` — add LICENSE.
- `736ab90` — update README.md.
- `8191cff` — 电梯开关门优化
- `6d2aa9c` — 优化随机生成建筑
- `8ba1867` — 添加危险源及相关算法评估程序
