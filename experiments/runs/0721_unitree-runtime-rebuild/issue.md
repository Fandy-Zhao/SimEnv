# Issue: Unitree Runtime Rebuild & Retest

**Goal:** diagnose and fix or isolate the `nvcc cannot execute cc1plus` build failure, rebuild the Unitree/Gazebo/Torch runtime chain from this worktree, prove artifact provenance, then retest FixedStand gates before any Earth RL policy evaluation.

**Scope:**
- Stage A toolchain diagnosis for gcc/g++/nvcc/cc1plus.
- Stage B rebuild attempts for Torch-enabled Unitree runtime, Unitree/Gazebo targets, and full catkin.
- Stage C FixedStand recovery gates: native Unitree, competition, then Earth.
- Stage D RL zero/low-speed only if all FixedStand gates pass.

**Non-Scope:**
- `earth.world`, spawn z, Gazebo physics, friction/contact parameters.
- FSM, FixedStand controller, RL policy, observation/action, gait, IK, estimator, URDF/xacro, model weights, KP/KD, or fall validator changes.
- System CUDA/GCC installation changes or global shell configuration edits.
- RL evaluation before rebuilt Competition FixedStand passes.

**Acceptance Criteria:**
- `cc1plus` root cause classified from evidence, not guessed.
- NVCC probe result saved with verbose output.
- Key Unitree/Gazebo/Torch runtime targets rebuilt or task marked with an explicit blocked verdict.
- Artifact paths and hashes prove runtime binaries/plugins come from this worktree before tests.
- C0-A/C0-B/C0-C gates are run in order only after successful rebuild.

**Risks:**
- Host toolchain may require system package repair outside repository scope.
- Full catkin may remain blocked by unrelated navigation/UAV dependencies.
- Low RTF may block motion conclusions even after build recovery.

**Impacted Modules:**
- `unitree_guide`, `unitree_gazebo`, `unitree_legged_control`, and generated runtime artifacts.
- Validation scripts and reports under `experiments/runs/0721_unitree-runtime-rebuild/`.
