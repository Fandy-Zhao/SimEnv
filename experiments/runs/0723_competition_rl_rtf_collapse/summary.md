# Competition RL RTF Collapse Runtime Summary

Status:
- `BUILD_PASS`
- `FAST_LIO_EXTERNAL_DEPENDENCY_REPRODUCIBLE`
- `RUNTIME_MATRIX_COMPLETE_SHORT_WINDOW`
- `ROOT_CAUSE_IDENTIFIED_SENSOR_LAYER`

## Build
The build completed inside the task worktree:

```text
/home/zzf/search_ws/SimEnv_worktrees/competition-rl-rtf-collapse/devel/setup.bash
/home/zzf/search_ws/SimEnv_worktrees/competition-rl-rtf-collapse/devel/lib/unitree_guide/junior_ctrl
```

The first build failed because the local venv used `empy 4.2.1`, which lacks
ROS Noetic's expected `em.RAW_OPT`. Reinstalling `empy==3.3.4` in the task
worktree venv fixed message generation. System ROS was not modified.

FAST-LIO2 was restored as an uncommitted external source dependency from
`https://github.com/hku-mars/FAST_LIO.git` at
`7cc4175de6f8ba2edf34bab02a42195b141027e9`, with `ikd-Tree` at
`e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4` and transitive
`livox_ros_driver` at `3d240d5666129e1a3052e78ee8487a04b08fdda3`.
See `fast_lio_provenance.md`.

## Core Runtime Matrix

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

M0, M1, and M6 are the original long-window core baseline runs. M2, M3, M4,
M5, M7, and M8 are short-window follow-up runs (`warmup_sim=1`,
`sample_sim=4`) because sensor-on RTF was too low for the original 45 s
simulation-time window to finish economically.

## Incremental Evidence
- M0 -> M1: RTF increased from `0.640749` to `0.989895`; competition scene
  without sensors/mapping/RL did not reproduce the RTF collapse.
- M1 -> M6: RTF changed from `0.989895` to `0.989091`; RL active without
  mapping did not create a significant RTF break in this run.
- M6 policy inference was measured at `50.0085 Hz` wall time.
- M1 -> M2: enabling competition sensor data drops mean RTF from `0.989895`
  to `0.165125`. This is the first confirmed RTF collapse point.
- M3 (`sensor data + PointCloud2 converter`, no FAST-LIO2) failed as
  `NO_CLOCK` in the 60 s startup-clock window, showing clock instability before
  FAST-LIO2 or RL inference is required.
- M2 -> M4: adding FAST-LIO2 mapping lowers mean RTF from `0.165125` to
  `0.134005`, a secondary cost after the sensor-layer collapse.
- M4 -> M5: loading the RL policy without entering RL does not materially
  worsen RTF (`0.134005` -> `0.138520`).
- M4/M5 -> M7: enabling RL plus referee/building-control lowers RTF further
  to `0.087573`, but the main break already exists in M2.
- M7 -> M8: Torch thread limiting did not improve this run (`0.087573` ->
  `0.058942`) and reduced measured policy wall rate to `22.5537 Hz`.

## Verdicts
- `EARTH_BASELINE_PASS`: M0 mean RTF `0.640749`, p10 `0.636691`.
- `COMPETITION_PHYSICS_BOTTLENECK_NOT_CONFIRMED`: M1 mean RTF `0.989895`.
- `RL_INFERENCE_50HZ_CONFIRMED`: M6 policy wall Hz `50.0085`.
- `RL_INFERENCE_OVERHEAD_NOT_SIGNIFICANT`: M6 RTF remains near M1 with mapping
  disabled.
- `FAST_LIO_EXTERNAL_DEPENDENCY_REPRODUCIBLE`: external FAST-LIO2 provenance
  and transitive Livox dependency were fixed to commits and rebuilt locally.
- `SENSOR_LAYER_RTF_COLLAPSE_CONFIRMED`: M2 reproduces the collapse with
  competition sensor data enabled and no PointCloud2 converter, FAST-LIO2, or
  RL.
- `FAST_LIO_SECONDARY_COST`: M4 is worse than M2 but is not the first break.
- `RL_NOT_PRIMARY_ROOT_CAUSE`: M6 is healthy without mapping; M7/M8 add cost
  only after the sensor/mapping path is already slow.
- `ROOT_CAUSE_IDENTIFIED_SENSOR_LAYER`: first confirmed culprit layer is
  competition sensor/Gazebo load, with M3 clock instability as an additional
  converter-path startup risk.
