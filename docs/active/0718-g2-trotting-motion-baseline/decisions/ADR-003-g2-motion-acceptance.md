# ADR-003: G2 Motion Acceptance Criteria

**Status:** Accepted
**Date:** 2026-07-18

## Context
G2-B needs a quantitative pass/fail standard for Trotting motion tracking. Criteria must be measurable from sim ground truth without controller instrumentation.

## Decision
Adopt the six AC-G2B-01 through AC-G2B-06 criteria defined in `acceptance-criteria.md`:
- Tracking error ≤ 0.08 m/s at 0.10 m/s (primary pass threshold).
- Tracking ratio target [0.70, 1.30] at 0.30 and 0.50 m/s.
- Monotonicity, drift, stop behavior, catastrophic failure, and trial validity gates as defined.

## Rationale
- 0.08 m/s at low speed prevents 0.10 m/s from passing due to drift or uncontrolled residual motion.
- Higher speeds use tracking ratio because relative error is more meaningful for speed scaling and monotonicity.
- Drift, stop, and failure criteria ensure the trial measures locomotion, not a stumble or collapse.

## Consequences
- If AC-G2B-02 fails at any speed, root-cause classification (ADR-004) is triggered.
- Failure of AC-G2B-05 (catastrophic failure) at any trial invalidates the entire speed epoch and requires immediate investigation before continuing the matrix.
