# 0721 Unitree Runtime Rebuild Summary

Verdict: `UNITREE_RUNTIME_REBUILT_FIXEDSTAND_FAIL`

## Build Matrix

- NVCC default probe: failed, because default `/usr/bin/gcc` cannot execute
  `cc1plus`.
- NVCC with `-ccbin /usr/bin/g++-11`: passed.
- Target Unitree/Torch/Gazebo build: passed.
- `tools/build_with_venv.sh` Unitree profile: passed.
- Full un-whitelisted catkin build: failed on missing `move_base_msgs` while
  configuring `unitree_move_base`; not a Unitree/Torch `cc1plus` failure.

## Artifact Provenance

Runtime artifacts resolve to:

- `/home/zzf/search_ws/SimEnv_worktrees/unitree-runtime-rebuild/devel/lib/unitree_guide/junior_ctrl`
- `/home/zzf/search_ws/SimEnv_worktrees/unitree-runtime-rebuild/devel/lib/unitree_guide/state_from_gazebo`
- `/home/zzf/search_ws/SimEnv_worktrees/unitree-runtime-rebuild/devel/lib/libunitreeFootContactPlugin.so`
- `/home/zzf/search_ws/SimEnv_worktrees/unitree-runtime-rebuild/devel/lib/libunitreeDrawForcePlugin.so`
- `/home/zzf/search_ws/SimEnv_worktrees/unitree-runtime-rebuild/devel/lib/libunitree_legged_control.so`

`junior_ctrl` links to LibTorch/CUDA under
`/home/zzf/third_party/libtorch-2.0.1-cu118-cxx11-abi` and
`/usr/local/cuda-11.8`.

## Runtime Gates

- C0-A native FixedStand: failed.
  - Run 01: `FAIL_ATTITUDE`, FSM state 2, final base height `0.335063 m`,
    min base height `0.110049 m`, max tilt `1.854429 deg`, RTF median
    `0.858318`.
  - Run 02: blocked by `/clock` stall after Gazebo startup; terminated.
- C0-B competition FixedStand: not run because C0-A did not pass 3/3.
- C0-C earth FixedStand: not run because C0-A did not pass 3/3.
- Earth RL with stair policy: not run; gate blocked before policy switch.

## Validation Checks

- `bash -n`: passed.
- Python `py_compile`: passed.
- Python unit tests: passed, 3 tests.
- `git diff --check`: passed.
- `gz sdf -k earth.world`: passed.
- XML parse: failed on existing generated file
  `src/unitree_guide/unitree_guide/generated_building/multi_floor_building.world`;
  `earth.world` SDF check passed.

## Answered Questions

1. `cc1plus` root cause: `/usr/bin/gcc` selects GCC 12, but GCC 12 `cc1plus`
   is absent on this host.
2. Fix path: use GCC/G++ 11 explicitly and pass CUDA host compiler/`-ccbin`.
3. Did target rebuild pass: yes.
4. Did `build_with_venv.sh` compile: yes for the Unitree runtime profile.
5. Did full catkin pass: no, blocked by missing `move_base_msgs`.
6. Artifact provenance: current worktree `devel`, not stale 0720 artifacts.
7. C0-A result: fail, not 3/3.
8. C0-B result: not run due gate.
9. C0-C result: not run due gate.
10. G1-P/G1-S result: not run due gate.
11. Earth FixedStand result: not run due gate.
12. RL/stair policy result: policy exists and is hashed; not run.
13. RTF: C0-A run 01 median `0.858318`, mean `1.544759`; run 02 clock stalled.
