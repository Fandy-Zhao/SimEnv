# Task Report

## Branch

`feat/0724-falco-dsv-single-floor-exploration-0p8`

## Summary

Prepared the single-floor FALCO + DSV exploration data path with a 0.8 m/s raw FALCO straight-line speed profile, DSV initialization/movement fixes, runtime terrain-map and boundary adapters, and a unified launch entry.

Initial implementation verdict: `FALCO_DSV_DATA_PATH_READY`.

Runtime validation update: `FALCO_DSV_EXPLORATION_BLOCKED`.

First failed runtime gate: `FAST_LIO_INPUT_BLOCKED`.

## Validation

- `./tools/build_with_venv.sh`: PASS.
- Python compile for bridge scripts: PASS.
- Launch XML and `roslaunch --nodes/--files`: PASS.
- FALCO smoke: PASS.
- Raw path follower speed probes:
  - straight `0.803999543 m/s`
  - 30 deg turn `0.600000143 m/s`
  - 70 deg turn `0.203999937 m/s`
  - max angular `0.219911486 rad/s`

## Runtime Limit

S2 was attempted with `FLOOR_COUNT=1 GUI=false ./auto.sh` plus
`single_floor_exploration.launch`. It stopped before motion because `fast_lio`
was not discoverable in the task worktree, so `fast_lio/fastlio_mapping` could
not launch and `/Odometry` timed out. No claim is made for short closed-loop,
full exploration, collision-free operation, complete floor coverage, or return
home.

## Runtime Evidence

- `fast_lio_input_blocked.txt`
- `auto_runtime.log`
- `navigation_runtime.log`
- `topic_hz_runtime.txt`
- `tf_snapshot_runtime.txt`

## Evidence

See `experiments/runs/0724_falco_dsv_single_floor_0p8/`.
