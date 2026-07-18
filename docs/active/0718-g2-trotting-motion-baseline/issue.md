# G2: Trotting Motion Baseline

## Branch
`test/0718-g2-trotting-motion-baseline` (from `master` at `fab433ba`)

## Prerequisite
G1_R_PASS — contact chain, FSM command, scheduler, and sim-time alignment verified.

## Goal
Measure and validate A1 Trotting motion tracking performance at 0.1, 0.3, 0.5 m/s, identify root causes of any tracking failures, and apply targeted single-cause fixes.

## Scope
- Baseline measurement (no controller modification)
- Root cause classification
- Single-variable targeted repair
- Post-fix re-validation

## Non-goals
- RL policy modification
- Physics/contact parameter changes (unless proven necessary via evidence)
- Navigation/sensor changes
- Multi-floor locomotion

## Acceptance Criteria (Initial)
- Speed monotonicity: v_actual(0.1) < v_actual(0.3) < v_actual(0.5)
- Tracking: absolute error <= 0.08 m/s at 0.1 m/s
- Lateral drift ratio <= 5%
- Stop time: 1.0s to reach <= 0.05 m/s after zero command
- No falls, no passive entry, no sustained joint/torque saturation
