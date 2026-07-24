# Task Report

## Branch

`feat/0724-falco-dsv-single-floor-exploration-0p8`

## Summary

Prepared the single-floor FALCO + DSV exploration data path with a 0.8 m/s raw FALCO straight-line speed profile, DSV initialization/movement fixes, runtime terrain-map and boundary adapters, and a unified launch entry.

Initial implementation verdict: `FALCO_DSV_DATA_PATH_READY`.

Runtime validation update: `FALCO_DSV_EXPLORATION_BLOCKED`.

First failed runtime gate: `FAST_LIO_INPUT_BLOCKED`.

Shared dependency retry: `FALCO_DSV_EXPLORATION_BLOCKED`.

First failed retry gate: `FAST_LIO_BUILD_BLOCKED`.

Livox-SDK prefix retry: `FALCO_DSV_EXPLORATION_BLOCKED`.

First failed prefix retry gate: `FAST_LIO_BUILD_BLOCKED`.

## Validation

- `./tools/build_with_venv.sh`: PASS.
- Python compile for bridge scripts: PASS.
- Launch XML and `roslaunch --nodes/--files`: PASS.
- FALCO smoke: PASS.
- Raw path follower speed probes:
  - straight `0.803999543 m/s`
  - 30 deg turn `0.600000143 m/s`
  - 70 deg turn `0.203999937 m/s`
  - max angular `0.219911486 rad/s`

## Runtime Limit

S2 was attempted with `FLOOR_COUNT=1 GUI=false ./auto.sh` plus
`single_floor_exploration.launch`. It stopped before motion because `fast_lio`
was not discoverable in the task worktree, so `fast_lio/fastlio_mapping` could
not launch and `/Odometry` timed out. No claim is made for short closed-loop,
full exploration, collision-free operation, complete floor coverage, or return
home.

The shared dependency retry linked the fixed public FAST_LIO and
`livox_ros_driver` sources under ignored `src/external/` symlinks and confirmed
package discovery through `ROS_PACKAGE_PATH=$PWD/src`. Formal
`./tools/build_with_venv.sh` then stopped before `catkin_make` with exit code
`20` because the pinned shared `livox_ros_driver` CMake would auto-clone/build
Livox-SDK into `/home/zzf/search_ws/livox_ros_driver`; that mutation is
forbidden for this task. FAST-LIO runtime, terrain map, DSV/FALCO chain, short
closed loop, full exploration, and return home were not run in this retry.

The Livox-SDK prefix retry prepared a separate SDK install prefix under
`/home/zzf/search_ws/shared_ros_deps/Livox-SDK/9306596a2bf15c1343bc023b497465ed0a32909d/install`
from fixed commit `9306596a2bf15c1343bc023b497465ed0a32909d`. This resolved the
SDK discovery blocker: `livox_ros_driver` reported `find livox sdk library
success` during formal build, while shared FAST_LIO and livox checkout status
remained clean. The build then failed later because the unmodified shared
FAST_LIO source forces C++14 and the unmodified shared `livox_ros_driver`
hardware node still hits ROS Noetic/PCL compile errors. No runtime, navigation,
short-loop, full-exploration, or return-home stage was run after this build
failure.

## Runtime Evidence

- `fast_lio_input_blocked.txt`
- `auto_runtime.log`
- `navigation_runtime.log`
- `topic_hz_runtime.txt`
- `tf_snapshot_runtime.txt`
- `shared_dependency_audit.txt`
- `shared_dependency_commits.txt`
- `shared_dependency_links.txt`
- `build_shared_dependencies.log`
- `navigation_data_chain.txt`
- `terrain_map_metrics.csv`
- `dsv_frontier_metrics.csv`
- `falco_raw_cmd.csv`
- `bridge_cmd.csv`
- `closed_loop_metrics.csv`
- `full_exploration_metrics.csv`
- `livox_sdk_cmake_audit.txt`
- `livox_sdk_source_commit.txt`
- `livox_sdk_install_manifest.txt`
- `shared_source_status_before.txt`
- `shared_source_status_after.txt`
- `build_livox_sdk_prefix.log`
- `build_with_shared_sdk.log`
- `package_preflight.txt`

## Evidence

See `experiments/runs/0724_falco_dsv_single_floor_0p8/`.
