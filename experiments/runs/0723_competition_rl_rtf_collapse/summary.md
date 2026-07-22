# Competition RL RTF Collapse Runtime Summary

Status:
- `BUILD_PASS`
- `RUNTIME_CORE_MATRIX_PARTIAL`
- `ROOT_CAUSE_PARTIAL`

## Build
The build completed inside the task worktree:

```text
/home/zzf/search_ws/SimEnv_worktrees/competition-rl-rtf-collapse/devel/setup.bash
/home/zzf/search_ws/SimEnv_worktrees/competition-rl-rtf-collapse/devel/lib/unitree_guide/junior_ctrl
```

The first build failed because the local venv used `empy 4.2.1`, which lacks
ROS Noetic's expected `em.RAW_OPT`. Reinstalling `empy==3.3.4` in the task
worktree venv fixed message generation. System ROS was not modified.

## Core Runtime Matrix

| Case | Competition | Mapping | Policy loaded | RL active | Torch limit | Mean RTF | p10 RTF | Policy Hz | Verdict |
| ---- | ----------: | ------: | ------------: | --------: | ----------: | -------: | ------: | --------: | ------- |
| M0 | 0 | 0 | 0 | 0 | 0 | 0.640749 | 0.636691 |  | `CASE_COMPLETE` |
| M1 | 1 | 0 | 0 | 0 | 0 | 0.989895 | 0.987226 |  | `CASE_COMPLETE` |
| M4 | 1 | 1 | 0 | 0 | 0 |  |  |  | `CASE_NOT_RUN_FAST_LIO_MISSING` |
| M5 | 1 | 1 | 1 | 0 | 0 |  |  |  | `CASE_NOT_RUN_FAST_LIO_MISSING` |
| M6 | 1 | 0 | 1 | 1 | 0 | 0.989091 | 0.986998 | 50.0085 | `CASE_COMPLETE` |
| M7 | 1 | 1 | 1 | 1 | 0 |  |  |  | `CASE_NOT_RUN_FAST_LIO_MISSING` |
| M8 | 1 | 1 | 1 | 1 | 1 |  |  |  | `CASE_NOT_RUN_FAST_LIO_MISSING` |

## Incremental Evidence
- M0 -> M1: RTF increased from `0.640749` to `0.989895`; competition scene
  without sensors/mapping/RL did not reproduce the RTF collapse.
- M1 -> M6: RTF changed from `0.989895` to `0.989091`; RL active without
  mapping did not create a significant RTF break in this run.
- M6 policy inference was measured at `50.0085 Hz` wall time.
- M1 -> M4, M4 -> M5, M4 -> M7, M6 -> M7, and M7 -> M8 are not valid because
  FAST-LIO2 is unavailable in this worktree.

## Verdicts
- `EARTH_BASELINE_PASS`: M0 mean RTF `0.640749`, p10 `0.636691`.
- `COMPETITION_PHYSICS_BOTTLENECK_NOT_CONFIRMED`: M1 mean RTF `0.989895`.
- `RL_INFERENCE_50HZ_CONFIRMED`: M6 policy wall Hz `50.0085`.
- `RL_INFERENCE_OVERHEAD_NOT_SIGNIFICANT`: M6 RTF remains near M1 with mapping
  disabled.
- `MAPPING_PIPELINE_FAIL`: external `fast_lio` package is missing.
- `ROOT_CAUSE_PARTIAL`: mapping/RL combined-resource and Torch-thread cases
  remain unverified.
