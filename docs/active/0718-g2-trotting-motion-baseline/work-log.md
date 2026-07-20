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

## 2026-07-19: G2-D1 Gate V Started

### Branch
`fix/0719-g2-fall-validator-frame-semantics` from
`test/0718-g2-trotting-motion-baseline` at `e5e27cfe`.

### Isolation
- Root workspace is currently on
  `diagnose/0719-g2-pre-wave-numerical-validity` with uncommitted runtime
  evidence; it is treated as read-only for this Gate.
- Existing baseline trial directories under
  `experiments/runs/0718_g2_trotting_motion_baseline/baseline/` are read-only
  inputs and must not be overwritten.

### Gate
Gate V will audit and, if confirmed, correct fall-validator pose/frame
semantics before any Gate P Pre-WAVE block diagnostics.

## 2026-07-19: G2-D1 Gate V Completed

### Commit
`2ac4cd0d test(g2v): record fall validator semantics verdict`

### Verdict
**G2_VALIDATOR_NO_DEFECT**

- No validator frame/pose semantic defect found.
- FALL_DETECTED confirmed physically meaningful.
- No production validator fix to merge.
- Gate P entered via ADR-010 no-defect exception.

### Gate V Evidence
- ADR-009: validator semantics fix.
- 21 files: report, tests, offline reclassification tools, evidence.
- Cherry-picked into G2 baseline at `af99255b`.

## 2026-07-19: G2-D1 Gate P Started

### Branch
`diagnose/0719-g2-pre-wave-block-reason` from
`test/0718-g2-trotting-motion-baseline` at `af99255b` (includes Gate V evidence).

### Worktree
`/home/zzf/search_ws/SimEnv_worktrees/g2-pre-wave-block-reason`

### Entry
Entered via ADR-010 (`G2_VALIDATOR_NO_DEFECT` exception path).

### Commit `1a524244`
```
test(g2d1): add pre-wave readiness and block diagnostics
```

### Completed
1. Static control-path audit → `g2-pre-wave-static-audit.md`
2. C++ diagnostic fields (PreWaveDiagnostics struct, 6 new CSV columns)
3. Python analysis module (`prewave_analyze.py`, 23 unit tests)
4. Build verified, G1 timing tests 13/13 PASS
5. ADR-010, report structure, evidence index updated

### Key Static Finding
`checkStepOrNot()` returns false for vx=0 (stable robot). Wave CANNOT start
with zero velocity command. This is expected behavior — not a defect.

### Pending
- Trial execution: P0 (FixedStand), P1 (vx=0), P2 (vx=0.10), P3 (vx=0.50 conditional)
- Event timeline alignment and first failing checkpoint determination
- Gate P verdict
- G2-R authorization decision

## 2026-07-20: Gate A G2 Fast Exit

### Branch
`diagnose/0719-g2-pre-wave-block-reason` at
`d24924702aaadc828a1690f2690c19d329554d9b` before this task's local edits.

### Added
- Fast-exit P0/P1/P2 diagnostic runner under
  `experiments/runs/0718_g2_trotting_motion_baseline/`.
- Zero-command offline classification:
  `EXPECTED_NO_STEP_TRIGGER` when `resolved_vx <= 0.03` and no wave start is
  expected.
- ADR-011 and `g2-fast-exit-report.md`.

### Runtime Evidence
- `p0_fixedstand_run_01`: probe bug evidence only; CSV parser failed on an
  empty timing field and no Gate verdict is claimed from this run.
- `p0_fixedstand_run_02`: valid P0 evidence. Result `FAIL` with
  `CONTACT_NOT_READY`, `FIXEDSTAND_NOT_ENTERED`, and `FALL_DETECTED`;
  `min_model_height=0.05698662028992169 m`, `foot_samples=0`, final FSM
  remained `PASSIVE`.

### Gate A Verdict
`G2_FAST_EXIT_SHARED_BASE_FAILURE`.

### RL Authorization
- `RL_SHADOW_ONLY_AUTHORIZED`
- `RL_ACTIVE_NOT_AUTHORIZED`

P1/P2 and active RL were not run because P0 did not establish the shared
FixedStand/contact base.
