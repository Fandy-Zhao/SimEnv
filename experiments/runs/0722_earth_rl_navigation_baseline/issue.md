# Issue: Earth RL Navigation Baseline

## Goal
Establish a governed baseline on `master` for Earth RL navigation with a
validated flat-ground policy, known speed range, and preserved interfaces.

## Scope
- Audit local `master` and avoid duplicate merge if the validated Earth RL
  fixes are already present.
- Confirm build PASS with `WORLD_MODE=earth`, `PHYSICS_PROFILE=normal`.
- Recommend `policy_act_inference_plane.pt` for flat ground.
- Validate speed range `0.20–0.40 m/s`; confirm `vx=0.10` ineffective.
- Verify RL zero upright, IMU fallback, LowCmd ≈500 Hz.

## Non-scope
- Navigation package code, new policies, control-parameter tuning.
- Remote push.

## Acceptance criteria
- Build PASS, RL zero min height ≥0.31 m, max tilt ≤1.01 deg.
- Plane policy body vx tracks command within ~0.78 gain at vx=0.20.
- IMU fallback `using_imu_policy_input=1`.
- LowCmd median ≈500 Hz.

## Risks
- Speed-range floor at vx=0.10 remains unresolved.
- No navigation planner integration yet validated.
- Stair/obstacle behavior remains unvalidated by this flat-ground comparison.
- RTF fluctuation remains recorded but non-blocking.

## Impacted modules
`unitree_guide` (State_RL, LowCmd transport, IMU fallback, policy loader).
