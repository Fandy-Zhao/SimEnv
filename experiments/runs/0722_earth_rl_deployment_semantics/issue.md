# Issue: Earth RL Deployment Semantics

Date: 2026-07-22
Branch: fix/0722-earth-rl-deployment-semantics
Baseline: e7fbbe639412fbd528a2fc35dc3009aa18c9af83

## Goal

Diagnose why the Unitree A1 enters the RL FSM state with `policy_act_inference_stair.pt` and immediately becomes unstable or falls under zero command or `vx=0.10` in `WORLD_MODE=earth PHYSICS_PROFILE=normal`.

If evidence identifies one semantic root cause, apply a minimal fix without changing the policy file, physics profile, broad gains, LowCmd scheduling, or hiding failure by freezing RL output.

## Scope

- Reproduce the FixedStand-to-RL transition from a fresh epoch.
- Capture transition semantics from one second before RL entry through at least three seconds after.
- Inspect policy dimensions, observation order and scaling, history initialization, command/state transition, action scaling, joint target semantics, and relevant robot state metrics.
- Add focused diagnostics and/or one minimal semantic fix when supported by evidence.
- Validate zero-command RL stability and, only if it passes, `vx=0.10` forward motion.

## Non-Scope

- No edits to `CHANGELOG.md`, `PROJECT_STATE.md`, `docs/module_status.md`, or `experiments/runs/0722_earth_rl_fastcheck/summary.md`.
- No LowCmd scheduling changes.
- No `.pt` model edits.
- No switch back to the plane policy.
- No physics profile changes.
- No broad Kp/Kd tuning.
- No action freezing or failure masking.

## Acceptance Criteria

`RL_DEPLOYMENT_PASS` requires:

- Zero-command RL remains stable for at least 8 simulation seconds.
- No fall or collapse, no NaN/Inf, no large action saturation, no instant joint target jump.
- Base height remains out of the fall zone and posture does not diverge.
- Then `vx=0.10` remains stable for 5-8 simulation seconds, moves forward above noise, and does not show sustained reverse or violent oscillation.

Otherwise report `RL_DEPLOYMENT_FAIL` with evidence.

## Risks

- Gazebo runtime may be unavailable or too slow in the current environment.
- Interactive FSM switching may require tmux/keyboard automation.
- A detected symptom may be caused by another subagent's LowCmd scheduling area, which is explicitly out of scope.
- TorchScript semantics may be inferable only from runtime tensors because the training config is not present.

## Expected Impacted Modules

- `src/unitree_guide/unitree_guide/unitree_guide/src/FSM/State_RL_test.cpp`
- `src/unitree_guide/unitree_guide/unitree_guide/include/FSM/State_RL_test.h`
- Experiment evidence under `experiments/runs/0722_earth_rl_deployment_semantics/`
