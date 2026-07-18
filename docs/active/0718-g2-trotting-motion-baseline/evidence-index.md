# G2-B Evidence Index

## Summary Evidence

| ID | Evidence Type | Path | Status |
|----|---------------|------|--------|
| E01 | Aggregated trial CSV | `experiments/runs/0718_g2_trotting_motion_baseline/summary.csv` | RECORDED |
| E02 | Aggregated trial JSON | `experiments/runs/0718_g2_trotting_motion_baseline/summary.json` | RECORDED |
| E03 | Baseline report | `docs/active/0718-g2-trotting-motion-baseline/g2-baseline-report.md` | RECORDED |
| E04 | Root-cause report | `docs/active/0718-g2-trotting-motion-baseline/g2-root-cause-report.md` | RECORDED |
| E05 | Recovery report | `docs/active/0718-g2-trotting-motion-baseline/g2-recovery-report.md` | NOT STARTED |

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
