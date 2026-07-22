# Competition RL RTF Collapse Diagnostics

This report records the governed setup for the competition-mode RL RTF
collapse diagnosis. The task intentionally adds diagnostic harnesses and
static audit notes only; it does not change runtime behavior, launch defaults,
controller logic, mapping logic, or physics parameters.

## Artifacts
- `experiments/runs/0723_competition_rl_rtf_collapse/issue.md`
- `experiments/runs/0723_competition_rl_rtf_collapse/static_audit.md`
- `tools/diagnostics/run_competition_rl_rtf_matrix.sh`
- `tools/diagnostics/sample_runtime_metrics.py`
- `tools/diagnostics/check_mapping_pipeline.py`

## Validation Scope
The scripts support dry-run metadata generation, runtime metric snapshots, and
mapping-pipeline inspection. Full M0-M8 runtime execution still requires a
display/ROS/Gazebo session and enough wall-clock time to reach the requested
simulation-time windows.
