Build Fix Decision
==================
Date: 2026-07-24
Decision: Revert to root workspace's proven approach

Problem:
  Worktree used "shared_ros_deps" strategy: src/external/ symlinks to
  UNPATCHED public sources. This cannot compile because:
  1. Public FAST_LIO uses C++14, workspace uses C++17
  2. Public livox_ros_driver lacks BUILD_LIVOX_DRIVER_NODE guard
  3. Worktree's build_with_venv.sh had extra Livox-SDK checks

Selected Fix: Revert to root's temp staging + patches approach

Changes made:
  1. tools/build_with_venv.sh: Reverted to root version (removed shared-deps
     checks, Livox-SDK requirements, CMAKE_LIBRARY_PATH additions)
  2. Removed src/external/FAST_LIO and src/external/livox_ros_driver symlinks
  3. Ran tools/external_deps/prepare_fast_lio2_deps.sh --prepare:
     - Copies FAST_LIO and livox_ros_driver to /tmp/simenv-fast-lio2-deps/
     - Applies fast_lio-cxx17.patch (C++14->C++17)
     - Applies livox-driver-message-only.patch (BUILD_LIVOX_DRIVER_NODE guard)
     - Creates src/FAST_LIO and src/livox_ros_driver symlinks
  4. Cleaned build/ and devel/ (stale CMake cache from broken approach)

Why this is minimal:
  - Uses existing prepare_fast_lio2_deps.sh tool already in the worktree
  - Uses existing patches already in the worktree
  - Reverts build_with_venv.sh to match the known-good root version
  - No new dependencies, no new patches, no new tools
  - Reuses already-prepared temp staging from root setup
