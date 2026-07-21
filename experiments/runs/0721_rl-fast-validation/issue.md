# Issue: RL Fast Validation Experiment

**Goal:** create governed scaffolding for RL fast validation experiments, separating
preflight, capture, metrics, and replay concerns without implementing controller
semantics.

**Scope:**
- Pure Python metric helpers (`rl_fast_metrics.py`) covering evaluation windows,
  thresholds, verdict priority, NaN/Inf detection, local-frame displacement,
  port allocation, artifact validation, policy SHA256 validation, and clock/master
  failure classification.
- Bash runner skeleton (`run_rl_fast_smoke.sh`) with port allocation, PID tracking,
  cleanup trap, and capture placeholder.
- Python capture placeholder (`rl_fast_capture.py`) writing preflight/blocked output
  artifacts.
- Offline replay scaffold (`replay_rl_state.py`) with fixture parser, metadata
  validator, policy hash verifier, and explicit `OFFLINE_REPLAY_SCAFFOLD_ONLY` status.
- Unit tests for all pure helpers.
- Thin build wrapper (`tools/build_rl_fast.sh`) for the Unitree RL runtime profile.

**Non-Scope:**
- RL policy, FSM, observation/action semantics, estimator, gait, IK, controller
  gains, URDF/xacro, spawn, Gazebo physics, `earth.world`, or fall thresholds.
- C++ files.
- Live Gazebo or ROS runtime execution.

**Acceptance Criteria:**
- `bash -n` clean on all `.sh` files.
- `python3 -m py_compile` clean on all `.py` files.
- `python3 -m unittest discover` passes all tests.
- `tools/build_rl_fast.sh` propagates `build_with_venv.sh` exit code.

**Risks:**
- Live ROS capture pieces are intentionally marked `TASK_PARTIAL` until implemented.
- Replay scaffold cannot process C++ observation/action data.
