# G2 Fast Exit Report

## Branch

`diagnose/0719-g2-pre-wave-block-reason`

## Baseline

- Gate P HEAD: `d24924702aaadc828a1690f2690c19d329554d9b`
- G2 baseline: `test/0718-g2-trotting-motion-baseline` at `af99255b`
- Stable master worktree HEAD observed: `ee800833`

## Scope Reduction

Gate A intentionally stops the complete G2 speed matrix and asks only whether
the shared FixedStand/contact/FSM base is stable enough to attribute remaining
failure to Trotting-specific code.

## Deferred G2 Work

- `vx=0.30`, `vx=0.50`
- 3 trials per speed
- Full velocity tracking, stopping, drift, and gait-performance scoring
- Trotting recovery/fix work

## P0 FixedStand Result

**FAIL.** Valid evidence is
`experiments/runs/0718_g2_trotting_motion_baseline/fast_exit/p0_fixedstand_run_02/`.

Key runtime facts from `probe_status.json`:

- `result`: `FAIL`
- `reasons`: `CONTACT_NOT_READY`, `FALL_DETECTED`, `FIXEDSTAND_NOT_ENTERED`
- `final_fsm_state`: `1` (`PASSIVE`)
- `foot_samples`: `0`
- `min_model_height`: `0.05698662028992169 m`
- `truth_samples`: `3116`
- `joint_samples`: `22070`
- `timing_rows`: `32971`

The controller log shows `/fsm/state_cmd data=2` callbacks were received, but
the controller remained in the pre-FixedStand waiting path and no four-foot
contact evidence was captured.

`p0_fixedstand_run_01` is retained as tool-failure evidence only: it exposed a
probe CSV parsing bug and does not carry a Gate verdict.

## P1 Trotting Zero-Command Result

**NOT RUN.** Gate rules require P0 PASS before entering Trotting. P0 failed.

The offline analyzer was updated so future `vx=0` trials distinguish
`EXPECTED_NO_STEP_TRIGGER` from missing wave-start when a step is actually
required.

## P2 Trotting Low-Speed Result

**NOT RUN.** Gate rules require P0 PASS before P2. P0 failed.

## Shared-Base Assessment

`G2_FAST_EXIT_SHARED_BASE_FAILURE`.

The minimum evidence loop did not establish a stable FixedStand shared base.
The failure happens before Trotting entry, so this run cannot classify the
blocker as Trotting-specific.

## Trotting-Specific Assessment

Not authorized for runtime attribution in this gate. No P1/P2 evidence was
collected because P0 failed.

## First Failing Checkpoint

`P0_FAIL_SHARED_BASE_CONTACT_OR_FIXEDSTAND`.

The first observed blockers are missing four-foot contact samples, failure to
enter FixedStand, and physical fall below the `0.12 m` safety line.

## Evidence Paths

- P0 valid evidence:
  `experiments/runs/0718_g2_trotting_motion_baseline/fast_exit/p0_fixedstand_run_02/`
- Probe tool-failure evidence:
  `experiments/runs/0718_g2_trotting_motion_baseline/fast_exit/p0_fixedstand_run_01/`
- Analyzer/tests:
  `experiments/runs/0718_g2_trotting_motion_baseline/diagnostics/pre_wave_block_reason/`
- Runner:
  `experiments/runs/0718_g2_trotting_motion_baseline/run_g2_fast_exit_probe.sh`

## Tests

- `/usr/bin/python3 -m py_compile` for the fast-exit probe and analyzer.
- `/usr/bin/python3 -m unittest .../test_prewave.py`: 28 tests passed.
- `bash -n` for the fast-exit runner.
- `git diff --check` is clean after restoring runtime-generated scene/log
  files from the task worktree.

## Commits

This report is recorded by commit `test(g2fast): record shared-base fast exit`.

## G2 Fast Exit Verdict

`G2_FAST_EXIT_SHARED_BASE_FAILURE`

## RL Entry Authorization

- `RL_SHADOW_ONLY_AUTHORIZED`
- `RL_ACTIVE_NOT_AUTHORIZED`

No active RL action may be applied until the shared FixedStand/contact base is
recovered and revalidated.

## Remaining Risks

- P0 failed before FixedStand; this does not prove Trotting is healthy or
  unhealthy.
- Missing foot-contact samples must be investigated before using the result to
  reason about Trotting or active RL.
- Full G2 performance baseline remains deferred, not completed.
