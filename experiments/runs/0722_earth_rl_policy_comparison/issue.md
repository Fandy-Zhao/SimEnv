# Issue: Earth RL Stair and Plane Policy Comparison

Date: 2026-07-22

## Goal

Compare the current stair and plane TorchScript policies on Earth flat ground,
recommend a policy for Earth motion, and carry only validated Earth RL timing
and deployment fixes toward `master`.

## Scope

- Add a minimal runtime policy-path override if the controller does not already
  support one.
- Run a fair stair-vs-plane speed comparison using the same Earth world,
  physics profile, spawn pose, command topic, FSM flow, data capture, and
  body-frame motion metrics.
- Record compact JSON/Markdown evidence under this experiment directory.
- Merge eligible comparison artifacts back to the current RL task branch.
- Validate a separate master merge candidate before fast-forwarding `master`.

## Non-Scope

- Do not modify `.pt` files.
- Do not tune control parameters, observation/action scales, gains, or physics
  to favor a policy.
- Do not submit raw CSV, runtime logs, PID files, generated worlds, build, or
  devel outputs.
- Do not push a remote branch.

## Acceptance Criteria

- Stair and plane policy files resolve to different SHA256 values.
- Runtime loader logs prove the intended policy path/SHA256 for each policy.
- Each policy completes at least zero velocity and one nonzero speed point.
- Comparison uses body-frame velocity and displacement.
- Recommended policy is evidence-based and does not rely on controller changes.
- Master is updated only through a validated merge candidate and fast-forward.

## Risks

- Gazebo runtime may show RTF variation; record it but do not gate solely on it.
- Low-speed command response may be policy-limited rather than fixable in
  deployment code.
- Repeated full sweeps are intentionally skipped for speed, so results are
  quick comparison evidence rather than statistical performance claims.
