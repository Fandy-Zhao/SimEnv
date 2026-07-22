# Competition RL RTF Collapse Diagnostics

This report records the governed setup and partial runtime validation for the
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
| M4 | 1 | 1 | 0 | 0 | 0 |  |  |  | `CASE_NOT_RUN_FAST_LIO_MISSING` |
| M5 | 1 | 1 | 1 | 0 | 0 |  |  |  | `CASE_NOT_RUN_FAST_LIO_MISSING` |
| M6 | 1 | 0 | 1 | 1 | 0 | 0.989091 | 0.986998 | 50.0085 | `CASE_COMPLETE` |
| M7 | 1 | 1 | 1 | 1 | 0 |  |  |  | `CASE_NOT_RUN_FAST_LIO_MISSING` |
| M8 | 1 | 1 | 1 | 1 | 1 |  |  |  | `CASE_NOT_RUN_FAST_LIO_MISSING` |

## Interpretation
- Earth baseline remains above 0.6 in this run (`mean=0.640749`).
- Competition minimal physics/control without mapping or RL did not reproduce
  the collapse (`mean=0.989895`).
- RL active without mapping did not reproduce the collapse (`mean=0.989091`).
- Policy inference frequency is confirmed near 50 Hz (`50.0085 Hz` wall time).
- Full mapping cases are blocked because `rospack find fast_lio` fails in this
  hermetic worktree. Therefore the combined RL + FAST-LIO2 root cause remains
  unverified.

Final status for this stage: `RUNTIME_CORE_MATRIX_PARTIAL`,
`MAPPING_PIPELINE_FAIL`, `ROOT_CAUSE_PARTIAL`.
