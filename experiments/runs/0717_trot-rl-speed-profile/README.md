# Trotting / RL Speed-Profile Experiment (single-floor)

## Raw data

Each trial lives under `raw/<mode_speed>/` and contains:

| File | Content |
|---|---|
| `ground_truth.csv` | Reference pose per timestamp |
| `trial_metrics.json` | Aggregated per-epoch metrics |
| `auto.log` | Sim-side automation log |
| `capture.log` | Capture-node log |
| `junior_ctrl.log` | Controller log (optional, RL trials only) |

Expected tags (six trials):

- `trotting_010`, `trotting_050`, `trotting_100`
- `rl_010`, `rl_050`, `rl_100`

## Figures

`plot_speed_profile.py` reads completed `trial_metrics.json` files and produces:

- `trajectory_planar.png` — XY trajectory overlay
- `rtf_mobility_relation.png` — real-time factor versus mobility metric

## Runtime prerequisite

The runner needs a verified Torch-enabled controller build. Set
`SIMENV_BINARY_DEVEL` to its `devel` directory before calling
`run_speed_trial.sh` or `run_all.sh`; the runner overlays only its binaries and
removes the matching source path from ROS package lookup.

## Rules

- Use only **complete** trials — no partial or aborted runs.
- Epochs must not **overlap** in time (disjoint intervals only).
- Do not hand-pick epochs; the full qualifying span of each trial must be used.
