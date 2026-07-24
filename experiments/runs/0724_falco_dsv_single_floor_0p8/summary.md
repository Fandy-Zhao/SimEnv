# 2026-07-24 Shared FAST-LIO2 Dependency Retry

Verdict: `FALCO_DSV_EXPLORATION_BLOCKED`

First failed gate: `FAST_LIO_BUILD_BLOCKED`

## Shared Resources

- FAST_LIO: `/home/zzf/search_ws/FAST_LIO`
- ikd-Tree: `/home/zzf/search_ws/FAST_LIO/include/ikd-Tree`
- livox_ros_driver: `/home/zzf/search_ws/livox_ros_driver`
- Livox-SDK prefix: `/home/zzf/search_ws/shared_ros_deps/Livox-SDK/9306596a2bf15c1343bc023b497465ed0a32909d/install`

## Commits

- FAST_LIO: `7cc4175de6f8ba2edf34bab02a42195b141027e9`
- ikd-Tree: `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`
- livox_ros_driver: `3d240d5666129e1a3052e78ee8487a04b08fdda3`
- Livox-SDK: `9306596a2bf15c1343bc023b497465ed0a32909d`
  (`v2.3.0-8-g9306596`, SDK version macros `2.3.0`)

## Result

`tools/prepare_shared_ros_deps.sh` created ignored `src/external/FAST_LIO` and
`src/external/livox_ros_driver` symlinks to the fixed shared sources. Package
discovery through `ROS_PACKAGE_PATH=$PWD/src` can find both `fast_lio` and
`livox_ros_driver`.

`tools/prepare_shared_ros_deps.sh --prepare` created an independent Livox-SDK
source/build/install prefix outside SimEnv and outside the public
`livox_ros_driver` checkout. The SDK install contains `include/livox_sdk.h` and
`lib/liblivox_sdk_static.a`. The SDK source is fixed at
`9306596a2bf15c1343bc023b497465ed0a32909d`; build used `-include memory` as a
compiler flag so the SDK source checkout itself stayed clean.

Formal `./tools/build_with_venv.sh` then entered catkin and `livox_ros_driver`
reported `find livox sdk library success`, resolving the SDK discovery blocker.
The build still failed with exit code `1`: unmodified shared FAST_LIO forces
C++14 and fails against Noetic/log4cxx `std::shared_mutex`, while unmodified
shared `livox_ros_driver` builds its hardware node and hits ROS/PCL
`std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>` message serialization
errors. Public FAST_LIO and livox checkout status remained clean before and
after build.

FAST-LIO runtime, terrain map, DSV/FALCO data chain, short closed loop, full
exploration, and return home were not run after this build blocker.
