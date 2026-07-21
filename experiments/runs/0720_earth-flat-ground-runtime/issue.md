# Issue: Earth Flat-Ground Runtime Validation

**Branch:** `test/0720-earth-flat-ground-runtime`
**Commit:** `5f5f9045`
**Date:** 2026-07-20

## Goal
Validate that commit `5f5f9045` removes the Earth benchmark platform contact trap at runtime, restores FixedStand stability, and gates RL zero/low-speed tests only after FixedStand passes.

## Scope
- Cherry-pick and validate `5f5f9045` in a runnable Unitree/Gazebo worktree.
- Run G0 world-only, G1 initial-contact, C0 competition FixedStand, E0 Earth FixedStand, and gated RL cases.
- Capture model lists, pose/velocity time series, RTF, process snapshots, logs, and metrics.

## Non-Scope
- World geometry, spawn height, URDF/xacro, Gazebo physics, controller, estimator, gait, observation/action, policy weights, or fall-threshold changes.
- Entering RL before E0 passes 3/3.
- Replacing prior results; new policy-file experiments must use separate case IDs.

## Acceptance Criteria
- [ ] `platform_1` and `platform_2` absent from runtime model list.
- [ ] G1 shows no obvious initial contact explosion or severe foot/base penetration evidence.
- [ ] C0 competition FixedStand has no obvious regression.
- [ ] E0 Earth FixedStand passes 3 independent 15 s simulation-time runs before RL is attempted.
- [ ] RL zero and forward cases are only evaluated after E0 passes.
- [ ] RTF is recorded and low-RTF limitations are explicitly reported.

## Risks
- Low RTF may block trustworthy RL performance conclusions.
- Contact evidence is limited if no direct contact sensors are available.
- Full `catkin_make -j` may remain blocked by unrelated local dependencies.

## Impacted Modules
- `unitree_guide` runtime benchmark tooling and reports.
- `earth.world` is validated but not modified by this task.

<!-- Filled by Codex after runtime execution -->
