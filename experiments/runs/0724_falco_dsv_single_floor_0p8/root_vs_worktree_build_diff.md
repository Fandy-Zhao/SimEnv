Root vs Worktree Build Difference Report
==========================================
Date: 2026-07-24

Root: /home/zzf/search_ws/SimEnv (master, ce73018c)
Worktree: /home/zzf/search_ws/SimEnv_worktrees/falco-dsv-single-floor-0p8 (feat/0724-falco-dsv-single-floor-exploration-0p8, bae41931)

1. LIVOX HARDWARE NODE (MOST CRITICAL)
   Root:    Patches livox_ros_driver to guard hardware node behind BUILD_LIVOX_DRIVER_NODE option.
            Builds with BUILD_LIVOX_DRIVER_NODE=OFF. No Livox-SDK needed.
   Worktree: Links directly to UNPATCHED public livox_ros_driver. No BUILD_LIVOX_DRIVER_NODE guard.
            build_with_venv.sh safety check at line ~272 detects this and exits with error 20
            unless Livox-SDK lib is found.
   Impact:  BLOCKING. Worktree cannot build without either the patch or Livox-SDK.

2. C++ STANDARD
   Root:    Patched FAST_LIO uses C++17. build_with_venv.sh sets -DCMAKE_CXX_STANDARD=17.
            Consistent C++17 throughout.
   Worktree: Public FAST_LIO uses C++14 (ADD_COMPILE_OPTIONS(-std=c++14)).
            build_with_venv.sh sets -DCMAKE_CXX_STANDARD=17.
            CONFLICT: C++14 vs C++17 mismatch.
   Impact:  BLOCKING. C++ standard mismatch will cause compilation/linking errors
            with ROS Noetic PCL (ABI incompatibility).

3. ROS/PCL SHARED_PTR COMPAT
   Root:    C++17 throughout means std::shared_ptr. PCL in Noetic was compiled with C++14,
            but consistent use of C++17 with proper flags avoids ABI issues.
   Worktree: Mix of C++14 (FAST_LIO) and C++17 (workspace) creates ABI conflict.
   Impact:  BLOCKING.

4. LIVOX-SDK PATH
   Root:    Not required (message-only livox build).
   Worktree: Required by build_with_venv.sh checks. Expected at
            /home/zzf/search_ws/shared_ros_deps/Livox-SDK/9306596a.../install.
   Impact:  BLOCKING (if Livox-SDK not pre-installed).

5. CATKIN WHITELIST/BLACKLIST
   Root:    Same whitelist. BUILD_LIVOX_DRIVER_NODE=OFF.
   Worktree: Same whitelist + BUILD_LIVOX_DRIVER_NODE=OFF.
            BUT the shared source safety check blocks the build before catkin runs.
   Impact:  Same whitelist, but blocked earlier.

6. SOURCE PATH STRUCTURE
   Root:    src/FAST_LIO -> /tmp/simenv-fast-lio2-deps/.../FAST_LIO (PATCHED)
            src/livox_ros_driver -> /tmp/.../livox_ros_driver/livox_ros_driver (PATCHED)
   Worktree: src/external/FAST_LIO -> /home/zzf/search_ws/FAST_LIO (UNPATCHED)
            src/external/livox_ros_driver -> /home/zzf/search_ws/livox_ros_driver (UNPATCHED)
   Impact:  ROOT CAUSE. The worktree approach of linking to unpatched public sources
            cannot work without the patches.

7. BUILD_TOOL (build_with_venv.sh)
   Root:    Simpler version. No prepare_shared_ros_deps.sh call. No Livox-SDK checks.
            No shared-source safety guard.
   Worktree: Added prepare_shared_ros_deps.sh --check-only call, Livox-SDK checks,
            CMAKE_LIBRARY_PATH/Livox-SDK prefix propagation, shared-source safety guard.
   Impact:  The additions in the worktree version assume a different dependency strategy
            (src/external/ + Livox-SDK) that doesn't match the root's proven approach.

ROOT CAUSE SUMMARY:
The worktree introduced a "shared_ros_deps" strategy that links directly to
unpatched public sources. The root workspace uses a "temp staging + patches"
strategy via prepare_fast_lio2_deps.sh. The unpatched public sources cannot
compile in this environment because:
1. FAST_LIO uses C++14 but the workspace uses C++17
2. livox_ros_driver lacks the BUILD_LIVOX_DRIVER_NODE guard

MINIMAL FIX RECOMMENDATION:
Revert build_with_venv.sh to root version (remove shared-deps additions).
Use prepare_fast_lio2_deps.sh --prepare to set up patched temp staging.
Clean build and continue.
