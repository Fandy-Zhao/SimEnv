# ADR-004: G2 Root Cause Classification

**Status:** Accepted
**Date:** 2026-07-18

## Context
If G2-B tracking acceptance criteria fail, a structured root-cause classification prevents premature or unfocused controller/model edits (per G2 non-goals).

## Decision
Classification proceeds through this ordered checklist:
1. **R1 Command path**: `/cmd_vel` to raw command, resolved command, saturation, and internal target.
2. **R2 Estimator**: Gazebo body velocity vs estimated body velocity, including bias, lag, and frame consistency.
3. **R3 Gait/foot placement**: phase, contact timing, foot placement, stance/swing, and step length.
4. **R4 LowCmd/joint path**: joint order, `q/dq/tau`, gains, torque limits, saturation, and zero-command recovery.
5. **R5 Model/physics**: mass, inertia, collision, friction, plugin torque limits, and solver settings only after controller-side evidence and any official baseline comparison justify it.

## Rationale
- The order favors clear code/interface and estimator/frame errors before gait, joint-path, or physics explanations.
- The final report names one primary root cause and at most two secondary candidates.

## Consequences
- Root cause documented in `g2-root-cause-report.md`.
- Fix must be single-variable, targeted, and re-validated per `g2-recovery-report.md`.
- G2-R starts only after G2-B completes and a single primary cause has enough evidence for a minimal targeted branch.
