# Issue: FAST-LIO2 Reproducible External Build and Pointcloud Continuity

## Goal

Create an isolated branch and worktree from `master` `a423bcfd104659bfa05d286ccb79d6a03520b246`, make FAST-LIO2 and `livox_ros_driver` reproducibly build from fixed stable external sources, and diagnose/fix the simulated `/scan_pointcloud2` continuity path that can lead to FAST-LIO2 `No Effective Points!`.

## Scope

- Add a SimEnv-owned external dependency preparation layer for fixed FAST_LIO and `livox_ros_driver` source copies.
- Patch only staging copies so FAST_LIO builds as C++17 and `livox_ros_driver` can build message-only without Livox-SDK or network clone.
- Add reusable pointcloud/IMU/FAST-LIO output continuity diagnostics.
- Modify only FAST-LIO2 launch/config/adapter/auto/build tooling when directly required by the pointcloud input path.
- Update governance status documents.

## Non-Scope

- Do not change `src/unitree_guide/unitree_guide/unitree_guide/src/state_from_gazebo.cpp`.
- Do not alter Gazebo TF timestamp guards, `map -> odom`, `odom -> base`, or `/Odometry_gazebo`.
- Do not modify stable external repositories under `/home/zzf/search_ws/FAST_LIO` or `/home/zzf/search_ws/livox_ros_driver`.
- Do not tune FAST-LIO algorithm parameters unless runtime statistics prove a direct configuration mismatch.
- Do not change RL controller, Gazebo physics profile, navigation, exploration, or scene geometry.

## Acceptance Criteria

- Fixed external source HEADs and cleanliness are checked before staging.
- Staging excludes VCS/build/generated artifacts and is repeatable.
- `src/FAST_LIO` and `src/livox_ros_driver` resolve to staging symlinks ignored by Git.
- Formal build via `./tools/build_with_venv.sh` passes from a clean shell.
- FAST_LIO compile flags show C++17 and no effective C++14.
- `livox_ros_driver` generates messages without building the hardware node or cloning Livox-SDK.
- Runtime via `./auto.sh` proves a single `/scan_pointcloud2` publisher, continuous pointcloud and IMU input, sustained `/Odometry` and `/cloud_registered`, and stable FAST-LIO TF output.
- Protected `state_from_gazebo.cpp` SHA256 is unchanged.

## Risks

- Shared host may contain old ROS/Gazebo processes, so runtime validation must use isolated ROS/Gazebo master URIs.
- Gazebo runtime may be display or performance constrained in the current environment.
- External package CMake patches must remain minimal and source-only.
- Existing root workspace is dirty and must remain read-only.

## Impacted Modules

- `tools/external_deps`
- `tools/diagnostics`
- `tools/build_with_venv.sh` if needed
- `src/simenv_fast_lio2_integration`
- Governance docs: `PROJECT_STATE.md`, `CHANGELOG.md`, `docs/module_status.md`
