# Issue: G2 Fast Exit Gate A

## Goal

Use minimal runtime evidence to decide whether the current G2 blocker can be
classified as Trotting-specific, and whether RL deployment validation may
proceed.

## Scope

- Record the current Gate P git/worktree state.
- Add diagnostic-only P0/P1/P2 fast-exit probe tooling.
- Run P0 FixedStand-only before any Trotting or RL active test.
- Publish the Gate A verdict and RL entry authorization.

## Non-Scope

- No Trotting control parameter, gait, estimator, contact, physics, URDF/SDF,
  policy, observation, history, action, or LowCmd behavior changes.
- No root workspace edits, cleanup, staging, or commit.
- No P1/P2 execution after P0 shared-base failure.
- No active RL action.

## Acceptance Criteria

- P0 produces structured evidence or a clearly reported runtime/tool blocker.
- If P0 fails, Gate A returns `G2_FAST_EXIT_SHARED_BASE_FAILURE`.
- Documentation records deferred G2 work and RL authorization boundaries.

## Risks

- Runtime startup can generate tracked scene/log files; these must not be
  committed as code changes.
- Missing foot-contact samples can block FixedStand command acceptance and must
  be treated as shared-base evidence, not Trotting evidence.

## Impacted Modules

- `experiments/runs/0718_g2_trotting_motion_baseline/`
- `docs/active/0718-g2-trotting-motion-baseline/`
