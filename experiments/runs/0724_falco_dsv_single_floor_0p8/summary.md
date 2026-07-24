# 2026-07-24 Shared FAST-LIO2 Dependency Retry

Verdict: `FALCO_DSV_EXPLORATION_BLOCKED`

First failed gate: `FAST_LIO_BUILD_BLOCKED`

## Shared Resources

- FAST_LIO: `/home/zzf/search_ws/FAST_LIO`
- ikd-Tree: `/home/zzf/search_ws/FAST_LIO/include/ikd-Tree`
- livox_ros_driver: `/home/zzf/search_ws/livox_ros_driver`

## Commits

- FAST_LIO: `7cc4175de6f8ba2edf34bab02a42195b141027e9`
- ikd-Tree: `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`
- livox_ros_driver: `3d240d5666129e1a3052e78ee8487a04b08fdda3`

## Result

`tools/prepare_shared_ros_deps.sh` created ignored `src/external/FAST_LIO` and
`src/external/livox_ros_driver` symlinks to the fixed shared sources. Package
discovery through `ROS_PACKAGE_PATH=$PWD/src` can find both `fast_lio` and
`livox_ros_driver`.

Formal `./tools/build_with_venv.sh` stopped before `catkin_make` with exit code
`20`. The pinned shared `livox_ros_driver` CMake lacks the local message-only
guard and would auto-clone/build Livox-SDK inside the public shared checkout
when no system `liblivox_sdk_static.a` exists. The build preflight refused that
mutation to preserve the shared source.

FAST-LIO runtime, terrain map, DSV/FALCO data chain, short closed loop, full
exploration, and return home were not run after this build blocker.
