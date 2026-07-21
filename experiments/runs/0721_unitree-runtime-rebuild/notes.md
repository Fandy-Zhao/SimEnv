# 0721 Unitree Runtime Rebuild Notes

## Scope

Governed rebuild and retest for the Earth RL runtime chain. This task did not
modify `earth.world`, spawn pose, physics/contact settings, FSM/controller/RL
logic, URDF, gains, estimator, IK, gait code, or fall validators.

## Toolchain Diagnosis

- Default `/usr/bin/gcc` is GCC 12.3.0, but its C++ frontend is missing:
  `/usr/lib/gcc/x86_64-linux-gnu/12/cc1plus` is absent.
- `/usr/bin/g++` and explicit `/usr/bin/gcc-11`/`/usr/bin/g++-11` resolve
  `cc1plus` under GCC 11.
- Default `nvcc` fails with `gcc: fatal error: cannot execute 'cc1plus'`.
- `nvcc -ccbin /usr/bin/g++-11 /tmp/nvcc_probe.cu` succeeds.
- `CUDAHOSTCXX=/usr/bin/g++-11` alone did not change direct `nvcc` behavior;
  CMake must pass the host compiler or `-ccbin`.

## Build Results

- Target Unitree/Torch build passed with explicit GCC/G++ 11 and
  `-DCMAKE_CUDA_HOST_COMPILER=/usr/bin/g++-11`.
- User-requested build entry was honored through the same tracked
  `tools/build_with_venv.sh` in this governed worktree. The absolute script
  `/home/zzf/search_ws/SimEnv/tools/build_with_venv.sh` is root-bound by its
  `SCRIPT_DIR` logic, so the effective command used the worktree copy plus a
  temporary `.venv` symlink to the main workspace venv.
- `build_with_venv.sh` Unitree profile passed:
  `unitree_legged_msgs;unitree_guide;unitree_legged_control;unitree_gazebo`.
- Full un-whitelisted `catkin_make` failed at configure because
  `unitree_move_base` requires missing `move_base_msgs`; this is separate from
  the Unitree/Torch rebuild.

## Runtime Results

- C0-A native Unitree FixedStand run 01 entered FSM state 2 and ended upright,
  but the validation window saw `min_base_height=0.11004896024519946 m`, below
  the current `0.12 m` FixedStand threshold. Verdict: `FAIL_ATTITUDE`.
- C0-A native Unitree FixedStand run 02 stalled: Gazebo services responded and
  physics reported `pause: False`, but `/clock` produced no new messages.
  The run was manually terminated and classified as runtime gate failure.
- C0-A did not reach 3/3 pass, so C0-B, C0-C, G1-P/G1-S, Earth FixedStand, and
  RL trials were not entered.

## Policy Provenance

- Requested later RL policy exists:
  `/home/zzf/search_ws/SimEnv/src/unitree_guide/logs/policy_act_inference_stair.pt`
- SHA256:
  `2d5aa72511c0c6609c02f4105845eee6974d3d73431497f8f35306da9588fe14`
- It was not used in runtime because C0-A blocked the RL gate.
