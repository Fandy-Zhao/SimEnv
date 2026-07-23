# FAST-LIO2 Dependency Provenance

Date: 2026-07-23

Task branch: `diagnose/0723-competition-rl-rtf-collapse`

## Verdict

`FAST_LIO_EXTERNAL_DEPENDENCY_REPRODUCIBLE`

SimEnv does not track or vendor the `fast_lio` package. The in-tree package
`simenv_fast_lio2_integration` is only the bridge/config layer and documents
FAST_LIO as an external ROS package placed at `SimEnv/src/FAST_LIO`.

A clean external candidate exists at `/home/zzf/search_ws/FAST_LIO` with a
fixed upstream remote and commit. That candidate has an uninitialized required
submodule, so reproducible restoration must include both the FAST_LIO commit
and the `include/ikd-Tree` submodule commit.

FAST-LIO also has a compile-time dependency on `livox_ros_driver` for
`CustomMsg` / `CustomPoint` message headers. SimEnv's runtime configuration uses
the PointCloud2 path, but upstream FAST-LIO still includes those Livox message
types at compile time. This transitive dependency is therefore restored from
its upstream remote at a fixed commit for build validation only.

## Provenance Table

| Field | Result |
|---|---|
| Original path | `/home/zzf/search_ws/SimEnv/src/FAST_LIO` absent; only `/home/zzf/search_ws/SimEnv/src/simenv_fast_lio2_integration` exists |
| Task worktree path | `/home/zzf/search_ws/SimEnv_worktrees/competition-rl-rtf-collapse/src/FAST_LIO` absent before restoration; only `src/simenv_fast_lio2_integration` exists |
| Git tracked | `fast_lio` package is not tracked by SimEnv; `simenv_fast_lio2_integration` is tracked |
| Ignored | `src/FAST_LIO` is not ignored by SimEnv `.gitignore` before restoration |
| Symlink | No `src/FAST_LIO` symlink found in original or task worktree |
| Submodule | SimEnv has no `.gitmodules` and no submodule entries; external FAST_LIO has required submodule `include/ikd-Tree` |
| Independent repo | Yes: `/home/zzf/search_ws/FAST_LIO` is an independent git repo |
| Remote URL | `https://github.com/hku-mars/FAST_LIO.git` |
| Commit SHA | FAST_LIO `7cc4175de6f8ba2edf34bab02a42195b141027e9` |
| Dirty state | Clean external FAST_LIO repo; no working tree or staged diff reported |
| License | `package.xml` declares `BSD`; repository `LICENSE` contains GPLv2 text. Treat as a license metadata conflict for redistribution review. |
| Required local modifications | Validation-only patches applied after restoration: build FAST_LIO as C++17 for Ubuntu/log4cxx compatibility; honor `BUILD_LIVOX_DRIVER_NODE=OFF` in `livox_ros_driver` so only message definitions are generated and the real hardware driver/Livox-SDK are not built |
| Required submodule | `include/ikd-Tree` at `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`, URL `https://github.com/hku-mars/ikd-Tree.git`, branch `fast_lio` |
| Required transitive dependency | `livox_ros_driver` from `https://github.com/Livox-SDK/livox_ros_driver.git` at `3d240d5666129e1a3052e78ee8487a04b08fdda3`; package license MIT |
| Reproducible acquisition method | Clone FAST_LIO from the remote, checkout `7cc4175de6f8ba2edf34bab02a42195b141027e9`, initialize submodules, verify `include/ikd-Tree` at `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`, then materialize a source-only copy under `src/FAST_LIO` without `.git` directories |
| Recommended integration method | For this governed validation, use a source-only restored external package under `src/FAST_LIO` and do not commit the external source. Long term, prefer a documented dependency bootstrap script or a real git submodule pinned to the same commits. |

## Audit Notes

- `rospack find fast_lio` failed before restoration.
- `rospack find simenv_fast_lio2_integration` succeeded before restoration.
- `tools/build_with_venv.sh` includes `fast_lio` in the default catkin
  whitelist, so the build expects the external package to be present.
- `src/simenv_fast_lio2_integration/launch/simenv_fast_lio2_mapping.launch`
  launches node `pkg="fast_lio"` / `type="fastlio_mapping"`.
- `src/simenv_fast_lio2_integration/README.md` explicitly says not to vendor
  FAST_LIO into SimEnv and to place external FAST_LIO at `SimEnv/src/FAST_LIO`.
- A local archive `/home/zzf/search_ws/FAST_LIO.zip` exists and contains a
  `.git` directory, but the reproducible source of truth is the remote URL and
  commit above, not the zip archive.

## Restoration Decision

Proceed with a temporary, source-only restoration for runtime validation:

1. Clone upstream FAST_LIO into the governed run directory.
2. Checkout FAST_LIO `7cc4175de6f8ba2edf34bab02a42195b141027e9`.
3. Initialize `include/ikd-Tree` and verify commit
   `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`.
4. Clone `livox_ros_driver` from `https://github.com/Livox-SDK/livox_ros_driver.git`,
   checkout `3d240d5666129e1a3052e78ee8487a04b08fdda3`, and export the
   `livox_ros_driver` ROS package into `src/livox_ros_driver` without `.git`
   metadata.
5. Export the main repo and submodule contents into `src/FAST_LIO` without
   `.git` metadata.
6. Keep `src/FAST_LIO` and `src/livox_ros_driver` uncommitted external source;
   commit only provenance, result summaries, scripts, and status documentation.

## Validation-Only Patch Summary

These changes are applied only to ignored external source restored for this run:

- `src/FAST_LIO/CMakeLists.txt`: replace hard-coded C++14 flags with C++17.
  Reason: ROS Noetic on this host includes log4cxx headers that require
  `std::shared_mutex`, available in C++17.
- `src/livox_ros_driver/CMakeLists.txt`: add
  `BUILD_LIVOX_DRIVER_NODE` option handling and skip the hardware driver target
  when it is `OFF`. Reason: SimEnv/FAST-LIO validation only needs
  `CustomMsg` / `CustomPoint` message generation; building the real driver
  triggers Livox-SDK and ROS/PCL compatibility issues unrelated to mapping.
