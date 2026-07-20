# G2-D1 Gate V: Fall Validator Frame Semantics

## Goal

Determine whether the G2 fall validator has a reproducible pose/frame semantic
defect, fix only the validator/data semantics if confirmed, and reclassify the
four existing G2 smoke trials without modifying locomotion behavior.

## Scope

- Freeze existing D0/G2-B evidence from the four baseline trials.
- Audit model/link pose source, quaternion order, quaternion direction, Euler
  conversion, height source, and fall predicate.
- Add validator semantics tests and offline reclassification tooling.
- Run a minimal FixedStand/Trotting smoke revalidation only after the semantics
  defect is confirmed and fixed.
- Publish `g2-validator-semantics-report.md` and ADR-009.

## Non-Scope

- No Trotting, WaveGenerator, scheduler, estimator, contact readiness, gait,
  torque, joint mapping, physics, spawn pose, or RL policy behavior changes.
- No G2-R recovery branch authorization.
- No full G2 matrix rerun.
- No deletion, overwrite, or mutation of existing raw trial evidence.

## Acceptance Criteria

- Existing four raw trials remain untouched.
- Pose source and quaternion/frame semantics are explicitly documented.
- Validator defect verdict is evidence-backed, not based on visual impression
  alone.
- If fixed, the fix changes validator semantics rather than weakening thresholds.
- Offline reclassification preserves non-fall invalid reasons such as
  `WAVE_ALL_NOT_REACHED` and `GAIT_NOT_ADVANCING`.
- Unit tests cover quaternion/frame and fall predicate cases.

## Risks

- Existing raw trial fields may lack link-level base/trunk pose, limiting
  offline certainty.
- Model origin height may be a poor proxy for trunk/body height.
- A normal robot model pose may have roll close to pi in Gazebo's frame
  convention.
- Removing a false-positive fall reason may still leave all trials invalid due
  to Pre-WAVE blockers.
- Runtime smoke tests can be affected by low RTF and ROS/Gazebo process residue.
