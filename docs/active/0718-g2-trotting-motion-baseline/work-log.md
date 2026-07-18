# Work Log: 0718 G2 Trotting Motion Baseline

## 2026-07-18: G2 Branch Created

### Branch
`test/0718-g2-trotting-motion-baseline` from `master` at `fab433ba`

### Context
G1_R_PASS achieved. Contact chain (C0-C8), FSM command chain (F0-F7), scheduler, and sim-time alignment all verified. Ready for G2 Trotting motion baseline measurement.

### Prerequisites Verified
- Contact plugins loaded and publishing
- FSM command chain functional (PASSIVE→FixedStand→Trotting)
- WAVE_ALL entered with finite contact forces
- Scheduler running at 500 Hz sim-time
- Timing tests 13/13 PASSED
- Build 0 errors

### Next Steps
1. Create G2 baseline trial runner
2. Measure Trotting at vx=0.1, 0.3, 0.5 m/s (3 epochs each)
3. Record Gazebo ground truth and controller state
4. Compute tracking, drift, stopping metrics
5. Classify root cause if tracking fails

## 2026-07-19: G2-B Governance and Tooling

### Added
- Completed G2-B documentation scaffold: test plan, acceptance criteria,
  architecture, risk register, evidence index, reports, and ADR-001 through
  ADR-004.
- Added G2-B-only experiment tooling under
  `experiments/runs/0718_g2_trotting_motion_baseline/`.
- Added pure metric tests for velocity frame transform, yaw wrap, sim-time
  window extraction, stop threshold detection, invalid filtering, median
  aggregation, and drift ratio.

### Validation
- `/usr/bin/python3 -m py_compile` passed for new Python scripts.
- `/usr/bin/python3 -m unittest` passed 6 metric tests.
- `bash -n` passed for the new G2 shell scripts.

### Notes
- No controller, URDF/SDF, Gazebo physics, gait, estimator, scheduler, contact
  threshold, or RL policy changes were made.
- Runtime smoke evidence and reports are now recorded; current G2 verdict is
  `G2_BASELINE_INCONCLUSIVE`.

## 2026-07-19: G2-B Runtime Smoke Trials

### Matrix
- Ran one trial each for `vx=0.00`, `vx=0.10`, `vx=0.30`, and `vx=0.50`.
- All four trials produced truth, timing, contact, joint, event, status, and
  metrics files.

### Result
- Valid trials: 0/4.
- Invalid reasons in every trial: `FALL_DETECTED`, `WAVE_ALL_NOT_REACHED`,
  `GAIT_NOT_ADVANCING`.
- Nonzero command trials reached resolved command values in controller timing:
  0.1, 0.3, and 0.5 m/s respectively.
- Controller pane for `vx=0.50` captured non-finite Trotting output with `q=0`,
  followed by wave cancellation.

### Verdict
- `G2_BASELINE_INCONCLUSIVE`.
- G2-R not started; next gate is a focused pre-WAVE non-finite output
  diagnostic branch.
