# G2-B Evidence Index

## Summary Evidence

| ID | Evidence Type | Path | Status |
|----|---------------|------|--------|
| E01 | Aggregated trial CSV | `experiments/runs/0718_g2_trotting_motion_baseline/summary.csv` | RECORDED |
| E02 | Aggregated trial JSON | `experiments/runs/0718_g2_trotting_motion_baseline/summary.json` | RECORDED |
| E03 | Baseline report | `docs/active/0718-g2-trotting-motion-baseline/g2-baseline-report.md` | RECORDED |
| E04 | Root-cause report | `docs/active/0718-g2-trotting-motion-baseline/g2-root-cause-report.md` | RECORDED |
| E05 | Recovery report | `docs/active/0718-g2-trotting-motion-baseline/g2-recovery-report.md` | NOT STARTED |
| E06 | Validator semantics report | `docs/active/0718-g2-trotting-motion-baseline/g2-validator-semantics-report.md` | RECORDED |

## Trial Evidence

Each trial directory contains `manifest.json`, `environment.txt`, `git.txt`,
`binary_hashes.txt`, `rosparams.yaml`, `trial_status.json`,
`trial_metrics.json`, `events.csv`, `ground_truth.csv`, `controller_state.csv`,
`joint_state.csv`, `foot_force.csv`, `auto.log`, `gazebo.log`, and
`controller.log`.

| Speed | Trial | Path | Status |
| ----: | ----- | ---- | ------ |
| 0.00 | `vx_000_run_01` | `experiments/runs/0718_g2_trotting_motion_baseline/baseline/vx_000/vx_000_run_01/` | INVALID |
| 0.10 | `vx_010_run_01` | `experiments/runs/0718_g2_trotting_motion_baseline/baseline/vx_010/vx_010_run_01/` | INVALID |
| 0.30 | `vx_030_run_01` | `experiments/runs/0718_g2_trotting_motion_baseline/baseline/vx_030/vx_030_run_01/` | INVALID |
| 0.50 | `vx_050_run_01` | `experiments/runs/0718_g2_trotting_motion_baseline/baseline/vx_050/vx_050_run_01/` | INVALID |

## Linkage

- `trial_status.json` files feed `summary.csv` and `summary.json`.
- `controller_state.csv` verifies FSM, wave status, resolved command, phase,
  contact, and gait-cycle sequences.
- `controller.log` from `vx_050_run_01` captures the first observed non-finite
  Trotting output and wave cancellation.

## Gate V Validator Semantics Evidence

| ID | Evidence Type | Path | Status |
|----|---------------|------|--------|
| V01 | Static pose/validator audit | `experiments/runs/0718_g2_trotting_motion_baseline/diagnostics/validator_semantics/pose_validator_static_audit.md` | RECORDED |
| V02 | Existing trial pose summary | `experiments/runs/0718_g2_trotting_motion_baseline/diagnostics/validator_semantics/existing_trial_pose_summary.csv` | RECORDED |
| V03 | Existing trial fall timeline | `experiments/runs/0718_g2_trotting_motion_baseline/diagnostics/validator_semantics/existing_trial_fall_timeline.json` | RECORDED |
| V04 | Offline reclassification CSV | `experiments/runs/0718_g2_trotting_motion_baseline/diagnostics/validator_semantics/offline_reclassification/offline_reclassification.csv` | RECORDED |
| V05 | Offline reclassification JSON | `experiments/runs/0718_g2_trotting_motion_baseline/diagnostics/validator_semantics/offline_reclassification/offline_reclassification.json` | RECORDED |
| V06 | Validator semantics tests | `experiments/runs/0718_g2_trotting_motion_baseline/diagnostics/validator_semantics/test_g2_validator_semantics.py` | PASS |

Gate V verdict: `G2_VALIDATOR_NO_DEFECT` for the suspected frame/pose
false-positive mechanism. `FALL_DETECTED` is preserved in all four original
trials.

## Gate P Pre-WAVE Block Diagnostics Evidence (G2-D1)

| ID | Evidence Type | Path | Status |
|----|---------------|------|--------|
| P01 | Gate P entry ADR | `docs/active/0718-g2-trotting-motion-baseline/decisions/ADR-010-pre-wave-block-gate-entry.md` | RECORDED |
| P02 | Static control path audit | `docs/active/0718-g2-trotting-motion-baseline/g2-pre-wave-static-audit.md` | RECORDED |
| P03 | Pre-wave analysis module | `experiments/runs/0718_g2_trotting_motion_baseline/diagnostics/pre_wave_block_reason/prewave_analyze.py` | RECORDED |
| P04 | Pre-wave unit tests (23) | `experiments/runs/0718_g2_trotting_motion_baseline/diagnostics/pre_wave_block_reason/test_prewave.py` | PASS |
| P05 | C++ diagnostic fields | `src/unitree_guide/unitree_guide/unitree_guide/include/control/CtrlComponents.h` | COMPILED |
| P06 | Timing CSV extension | `src/unitree_guide/unitree_guide/unitree_guide/include/common/TimingDiagnostics.h` | COMPILED |
| P07 | G1 timing regression (13 tests) | `build/test_results/unitree_guide/gtest-timing_alignment_test.xml` | PASS |
| P08 | Pre-wave block report | `docs/active/0718-g2-trotting-motion-baseline/g2-pre-wave-block-report.md` | PENDING |

Gate P diagnostics committed at `1a524244`. Ready for trial execution.
G2-R remains NOT AUTHORIZED.

## Gate A Fast Exit Evidence

| ID | Evidence Type | Path | Status |
|----|---------------|------|--------|
| A01 | Fast-exit issue | `experiments/runs/0720_g2_fast_exit/issue.md` | RECORDED |
| A02 | Fast-exit probe script | `experiments/runs/0718_g2_trotting_motion_baseline/g2_fast_exit_probe.py` | RECORDED |
| A03 | Fast-exit runner | `experiments/runs/0718_g2_trotting_motion_baseline/run_g2_fast_exit_probe.sh` | RECORDED |
| A04 | P0 tool-failure run | `experiments/runs/0718_g2_trotting_motion_baseline/fast_exit/p0_fixedstand_run_01/` | TOOL_FAIL |
| A05 | P0 FixedStand evidence | `experiments/runs/0718_g2_trotting_motion_baseline/fast_exit/p0_fixedstand_run_02/` | FAIL |
| A06 | Gate A report | `docs/active/0718-g2-trotting-motion-baseline/g2-fast-exit-report.md` | RECORDED |
| A07 | ADR-011 | `docs/active/0718-g2-trotting-motion-baseline/decisions/ADR-011-g2-fast-exit-and-rl-entry.md` | ACCEPTED |

Gate A verdict: `G2_FAST_EXIT_SHARED_BASE_FAILURE`.
RL authorization: `RL_SHADOW_ONLY_AUTHORIZED`, `RL_ACTIVE_NOT_AUTHORIZED`.
