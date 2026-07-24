# FALCO A1 Real-Cloud R3 Tuning Summary

Date: 2026-07-24
Branch: `feat/0723-falco-dsv-navigation-integration`
Baseline HEAD: `8f5c89ee`

## Verdict

`FALCO_A1_REAL_PATH_READY`

R3 is ready for the next gated stage using real FAST-LIO2 `/cloud_registered`
with `checkObstacle=true`. R4-R6 were not run in this task.

## What Changed

- Added `falco_a1.yaml` as the SimEnv A1-specific FALCO configuration.
- Switched FALCO launch defaults to the A1 profile while keeping launch-time
  overrides for the tuned parameters.
- Added opt-in FALCO diagnostics for real-cloud filtering, candidate path
  counts, selected path group/rotation, collision score distribution, and
  command/path output.
- Preserved the bridge safety gate: `/navigation/enabled=false` keeps
  `/cmd_vel` zero.

## Selected Parameters

- `minRelZ=-0.25`, `maxRelZ=0.25`
- `vehicleLength=0.56`, `vehicleWidth=0.43`
- `pointPerPathThre=2`
- `adjacentRange=3.5`, `pathScale=1.0`, `minPathScale=0.75`,
  `minPathRange=1.0`, `goalClearRange=0.5`
- `autonomyMode=true`, `autonomySpeed=0.10`

## Key Evidence

- Old/default real-cloud R3 remained blocked with obstacle checking enabled:
  the planner selected rotation/zero useful forward output.
- `minRelZ=-0.35` still selected a rotation-only response.
- `minRelZ=-0.25` repeatedly produced a forward local path and raw forward
  FALCO command.
- The selected medium A1 footprint was derived from the robot xacro geometry:
  standing lidar-inclusive footprint about `0.434 m x 0.302 m`, with selected
  runtime footprint `0.56 m x 0.43 m`.
- `reg_long_front_3m` showed obstacle checking remained active: only about
  `4533-4817` of `6174` candidate paths were free, while the planner still
  selected a valid local forward segment.

## Artifacts

- `a1_geometry_audit.md`
- `parameter_matrix.csv`
- `cloud_statistics.csv`
- `candidate_path_statistics.csv`
- `selected_parameters.yaml`
- `obstacle_regression.md`
