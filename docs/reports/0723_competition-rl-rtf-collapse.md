# Competition RL RTF Collapse Diagnostics

This report records the governed setup and runtime validation for the
competition-mode RL RTF collapse diagnosis. Runtime behavior, physics
parameters, observation/action logic, and FAST-LIO2 parameters were not changed.

## Artifacts
- `experiments/runs/0723_competition_rl_rtf_collapse/issue.md`
- `experiments/runs/0723_competition_rl_rtf_collapse/static_audit.md`
- `tools/diagnostics/run_competition_rl_rtf_matrix.sh`
- `tools/diagnostics/run_core_runtime_matrix.py`
- `tools/diagnostics/sample_runtime_metrics.py`
- `tools/diagnostics/check_mapping_pipeline.py`
- `experiments/runs/0723_competition_rl_rtf_collapse/summary.md`
- `experiments/runs/0723_competition_rl_rtf_collapse/mapping_pipeline.md`

## Validation Scope
The master merge preserves diagnostic tools and evidence only. It intentionally
excludes transient process snapshots and cleanup PID records from M0/M1/M6; the
retained artifacts are metrics, topic rates, mapping pipeline snapshots,
commands, environment records, summaries, and FAST-LIO2 provenance notes.

## Build Resolution
`tools/build_with_venv.sh` is worktree-aware and built inside:

```text
/home/zzf/search_ws/SimEnv_worktrees/competition-rl-rtf-collapse
```

The first build failed on ROS message generation because the local venv used
`empy 4.2.1`. Reinstalling `empy==3.3.4` in the task worktree venv fixed the
Noetic `em.RAW_OPT` compatibility issue. The final build produced
`devel/setup.bash` and `devel/lib/unitree_guide/junior_ctrl` in the task
worktree.

## Runtime Core Matrix

| Case | Competition | Mapping | Policy loaded | RL active | Torch limit | Mean RTF | p10 RTF | Policy Hz | Verdict |
| ---- | ----------: | ------: | ------------: | --------: | ----------: | -------: | ------: | --------: | ------- |
| M0 | 0 | 0 | 0 | 0 | 0 | 0.640749 | 0.636691 |  | `CASE_COMPLETE` |
| M1 | 1 | 0 | 0 | 0 | 0 | 0.989895 | 0.987226 |  | `CASE_COMPLETE` |
| M2 | 1 | 0 | 0 | 0 | 0 | 0.165125 | 0.143740 |  | `CASE_COMPLETE` |
| M3 | 1 | 0 | 0 | 0 | 0 |  |  |  | `NO_CLOCK` |
| M4 | 1 | 1 | 0 | 0 | 0 | 0.134005 | 0.110661 |  | `CASE_COMPLETE` |
| M5 | 1 | 1 | 1 | 0 | 0 | 0.138520 | 0.113486 |  | `CASE_COMPLETE` |
| M6 | 1 | 0 | 1 | 1 | 0 | 0.989091 | 0.986998 | 50.0085 | `CASE_COMPLETE` |
| M7 | 1 | 1 | 1 | 1 | 0 | 0.087573 | 0.018263 | 48.4947 | `CASE_COMPLETE` |
| M8 | 1 | 1 | 1 | 1 | 1 | 0.058942 | 0.021545 | 22.5537 | `CASE_COMPLETE` |

## Interpretation
- Earth baseline remains above 0.6 in this run (`mean=0.640749`).
- Competition minimal physics/control without mapping or RL did not reproduce
  the collapse (`mean=0.989895`).
- RL active without mapping did not reproduce the collapse (`mean=0.989091`).
- Policy inference frequency is confirmed near 50 Hz (`50.0085 Hz` wall time).
- Competition sensor data alone reproduces the first major RTF collapse:
  M1 `0.989895` -> M2 `0.165125`, before PointCloud2 conversion, FAST-LIO2,
  or RL inference is required.
- FAST-LIO2 mapping adds a secondary cost after the sensor-layer collapse:
  M4 mean RTF is `0.134005`.
- M7/M8 show additional RL/thread-limit cost after the sensor/mapping path is
  already slow, not the primary break.
- M3 `NO_CLOCK` flags converter-path startup instability for follow-up.

Final status for this stage: `RUNTIME_MATRIX_COMPLETE_SHORT_WINDOW`,
`SENSOR_LAYER_RTF_COLLAPSE_CONFIRMED`,
`ROOT_CAUSE_IDENTIFIED_SENSOR_LAYER`.
