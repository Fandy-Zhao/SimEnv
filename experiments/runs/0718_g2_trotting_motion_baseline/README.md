# G2 Trotting Motion Baseline Experiments

This directory contains G2-B-only tooling and evidence for the Trotting motion
baseline. Scripts here may start isolated ROS/Gazebo trials, publish FSM and
`/cmd_vel` commands, collect evidence, and compute metrics. They must not modify
controller parameters, Gazebo physics parameters, robot model files, or control
logic during G2-B.

Expected layout:

```text
manifests/
baseline/vx_000/
baseline/vx_010/
baseline/vx_030/
baseline/vx_050/
invalid/
ablations/
post_fix/
plots/
summary.csv
summary.json
```

Raw per-trial CSV files can be large. Keep only compact summaries, manifests,
selected plots, and report evidence in commits unless a raw file is explicitly
needed for review.
