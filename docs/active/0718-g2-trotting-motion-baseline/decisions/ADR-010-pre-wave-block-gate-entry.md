# ADR-010: Pre-WAVE Block Gate Entry via Validator No-Defect Exception

Date: 2026-07-19

Status: Accepted

## Context

Gate V (validator frame semantics) returned verdict `G2_VALIDATOR_NO_DEFECT`:

- The `FALL_DETECTED` verdict was not triggered by a `roll≈180°` quaternion defect.
- The actual predicate is `min(model_states.z) < 0.12 m`.
- Normal FixedStand height (~0.326 m) does not trigger fall.
- All four old G2 trials retain `FALL_DETECTED` after offline reclassification.
- Normalized-quaternion tilt + height re-judgment still supports fall.
- No validator frame/pose semantic defect was found.
- A fresh D0 capture did not reproduce a first non-finite value.

Therefore Gate V produced no production validator fix to merge.

Gate P (pre-WAVE block reason) requires a decision on whether to proceed with
locomotion diagnostics despite the absence of a validator fix.

## Decision

Gate P may begin under either of two conditions:

1. **`G2_VALIDATOR_FIX_PASS`**: A confirmed validator defect was fixed, verified,
   and integrated.
2. **`G2_VALIDATOR_NO_DEFECT`** (this path): Runtime and offline evidence confirm
   the fall judgments are physically meaningful and no validator modification
   needs to be merged. The absence of a validator defect is itself a diagnostic
   result that narrows the root-cause search space.

This ADR activates the second path.

Gate P remains **diagnostic only**. It does not authorize:

- Any control parameter modification.
- Any readiness threshold change.
- Any WaveGenerator state-transition semantic change.
- Any fall-guard removal or bypass.
- Any numerical fix.
- Any G2-R recovery branch creation.

## Consequences

1. A new branch `diagnose/0719-g2-pre-wave-block-reason` was created from the
   integrated G2 baseline (which includes cherry-picked Gate V diagnostic
   evidence).
2. All Gate P work is performed in worktree
   `/home/zzf/search_ws/SimEnv_worktrees/g2-pre-wave-block-reason`.
3. Root workspace (`/home/zzf/search_ws/SimEnv`) is NOT modified.
4. G2-R remains **NOT AUTHORIZED** until `G2_D1_PASS_ROOT_CAUSE_IDENTIFIED`
   is reached.
5. The first failing checkpoint must be unambiguously identified before any
   recovery branch can be proposed.
6. Real physical falls must not be attributed to any specific control module
   without event-timeline evidence confirming the causal order.

## Validation

- Gate V verdict: `G2_VALIDATOR_NO_DEFECT` (see ADR-009).
- Gate V evidence cherry-picked into baseline: commit `af99255b`.
- Gate P branch: `diagnose/0719-g2-pre-wave-block-reason` at `af99255b`.
- Worktree clean on creation.
