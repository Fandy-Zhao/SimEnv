# Issue: Earth RL Timebase Fast Validation

Date: 2026-07-22
Branch: `fix/0722-earth-rl-timebase-fast-validation`
Worktree: `/home/zzf/search_ws/SimEnv_worktrees/earth-rl-fastcheck`
Baseline: `master` at `38947a556342b4bafa20a84ff56e8065ce4f358f`

## Goal

Validate Earth `PHYSICS_PROFILE=normal` RL controller behavior with priority on simulation-time control semantics, stable RTF, actual policy path, RL state movement, and a short forward-speed range.

## Scope

- Audit `/fsm/state_cmd` to RL transition, observation/history updates, policy inference, action output, and low-level command publication.
- Build only with `tools/build_with_venv.sh`.
- Launch only with `WORLD_MODE=earth PHYSICS_PROFILE=normal GUI=False ./auto.sh`.
- Collect small evidence for RTF, policy path, timebase, and first RL speed smoke.
- Apply only evidence-backed minimal fixes.

## Non-Scope

- No `.pt` modification, retraining, reward tuning, broad parameter search, physics weakening, FAST-LIO2/nav changes, or unrelated refactor.
- No merge back to `master`.

## Acceptance Criteria

- Isolated branch/worktree with clean baseline.
- Build pass or documented blocker.
- Independent verdicts for baseline, build, RTF, policy path, timebase, RL smoke, tracking, and fix status.
- Summary and governance docs updated.

## Risk Points

- RTF may be unstable under Earth normal.
- Wall-time loops may consume repeated simulation states.
- Runtime policy path may differ from source defaults.
- RL motion failure may reflect policy capability after infrastructure fixes.
