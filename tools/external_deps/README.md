# FAST-LIO2 External Dependency Staging

This directory contains SimEnv-owned tooling for reproducible FAST-LIO2 builds.
The stable external repositories remain read-only:

- `/home/zzf/search_ws/FAST_LIO`
- `/home/zzf/search_ws/livox_ros_driver`

Run from a SimEnv worktree:

```bash
./tools/external_deps/prepare_fast_lio2_deps.sh --check
./tools/external_deps/prepare_fast_lio2_deps.sh --prepare
./tools/external_deps/prepare_fast_lio2_deps.sh --clean-links
```

`--prepare` copies fixed commits into `/tmp/simenv-fast-lio2-deps`, applies the
local patches in this directory, and maps the staged packages into `src/` with
symlinks. The symlinks are added to `.git/info/exclude` and must not be
committed.

The patches are intentionally local to staging:

- `fast_lio-cxx17.patch` removes FAST_LIO's hard-coded C++14 flags and sets
  C++17.
- `livox-driver-message-only.patch` keeps message generation active while
  making the real Livox hardware driver node opt-in with
  `BUILD_LIVOX_DRIVER_NODE=ON`.
