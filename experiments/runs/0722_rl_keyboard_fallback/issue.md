# Issue: RL Keyboard Command Fallback

## Goal

Let RL mode respond to keyboard `w/a/s/d` movement input when no fresh
`/cmd_vel` command is available, matching the operator experience of Trotting
while preserving `/cmd_vel` control for navigation.

## Scope

- Add a `State_RL` command resolver that prefers fresh `/cmd_vel`.
- Fall back to `_lowState->userValue` for keyboard velocity commands when
  `/cmd_vel` is absent or timed out.
- Use the same keyboard mapping convention as Trotting.
- Reject non-finite `/cmd_vel` values by commanding zero.

## Non-scope

- No RL policy retraining or policy file changes.
- No control gain or gait tuning.
- No navigation planner integration.
- No changes to Trotting behavior.

## Acceptance Criteria

- Build passes for the Unitree runtime profile.
- Existing `/cmd_vel` control path remains primary.
- Keyboard fallback maps `w/s` to forward/backward velocity, `a/d` to lateral
  velocity, and `j/l` to yaw rate.

## Risks

- `vx=0.10 m/s` remains an RL policy deadband limitation and is not solved by
  adding keyboard fallback.
