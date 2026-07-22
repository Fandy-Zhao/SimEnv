# Earth RL Policy Comparison

Date: 2026-07-22
Branch: `test/0722-earth-rl-policy-comparison`
Base task branch HEAD: `1e8e58f640dbe4dd4f162cf54a3cced46f1c7c53`

## Goal

Compare the current stair and plane TorchScript RL policies on Earth flat
ground with identical launch/control parameters, recommend the better policy
from evidence, and keep the comparison free of control-parameter tuning.

## Runtime Policy Selection

`State_RL` now accepts a minimal runtime policy override:

1. ROS param `/rl_policy_path`
2. Environment variable `RL_POLICY_PATH`
3. Default `src/unitree_guide/logs/policy_act_inference_stair.pt`

The controller logs configured path, resolved realpath, SHA256, file existence,
and load success before running inference.

Observed loader proofs:

| Policy | Path | SHA256 | Loaded |
| --- | --- | --- | --- |
| stair | `/home/zzf/search_ws/SimEnv/src/unitree_guide/logs/policy_act_inference_stair.pt` | `2d5aa72511c0c6609c02f4105845eee6974d3d73431497f8f35306da9588fe14` | yes |
| plane | `/home/zzf/search_ws/SimEnv/src/unitree_guide/logs/policy_act_inference_plane.pt` | `e886847fe266e3c2f7c08825fceeaecfa75c7eac5f780b25b6d4dca173ff8bef` | yes |

## Method

Launch command template:

```bash
WORLD_MODE=earth PHYSICS_PROFILE=normal GUI=False \
ENABLE_SENSOR_DATA=0 ENABLE_FAST_LIO2=0 ENABLE_RVIZ=0 \
ENABLE_POINTCLOUD_CONVERTER=0 ENABLE_REFEREE_ODOM=0 \
ENABLE_GROUND_TRUTH=0 START_BUILDING_CONTROL=0 \
LOWCMD_APPLY_DIAGNOSTICS_ENABLED=1 \
UNITREE_RL_DIAG_PATH=experiments/runs/0722_earth_rl_policy_comparison/raw/<policy>_rl_diag.csv \
RL_POLICY_PATH=<policy-path> ./auto.sh
```

Sequence per policy: FixedStand for 3 sim-s, RL zero for 3 sim-s, then
`vx = 0.00, 0.10, 0.20, 0.30, 0.40 m/s`, with 2 sim-s settle and 6 sim-s
effective capture per speed. Metrics are reported in the robot body frame.

## Results

| Policy | vx cmd | Median body vx | Tracking gain | Yaw drift | Min height | Verdict |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| stair | 0.00 | -0.000 m/s | n/a | 0.94 deg | 0.308 m | `TRACKING_PASS` |
| stair | 0.10 | 0.001 m/s | 0.008 | 0.18 deg | 0.309 m | `NO_EFFECTIVE_MOTION` |
| stair | 0.20 | 0.018 m/s | 0.092 | 2.37 deg | 0.313 m | `MOTION_PASS` |
| stair | 0.30 | 0.231 m/s | 0.770 | 14.73 deg | 0.309 m | `TRACKING_PASS` |
| stair | 0.40 | 0.317 m/s | 0.792 | 11.86 deg | 0.307 m | `TRACKING_PASS` |
| plane | 0.00 | -0.004 m/s | n/a | -0.40 deg | 0.311 m | `TRACKING_PASS` |
| plane | 0.10 | -0.000 m/s | -0.001 | 0.02 deg | 0.314 m | `NO_EFFECTIVE_MOTION` |
| plane | 0.20 | 0.187 m/s | 0.934 | 5.11 deg | 0.312 m | `TRACKING_PASS` |
| plane | 0.30 | 0.277 m/s | 0.923 | 5.62 deg | 0.313 m | `TRACKING_PASS` |
| plane | 0.40 | 0.342 m/s | 0.855 | 5.73 deg | 0.314 m | `TRACKING_PASS` |

LowCmd timing from controller timing diagnostics:

| Policy | LOWCMD samples | Median LOWCMD Hz | Mean LOWCMD Hz |
| --- | ---: | ---: | ---: |
| stair | 32229 | 500.0 | 456.4 |
| plane | 44658 | 500.0 | 493.4 |

## Recommendation

Recommend `policy_act_inference_plane.pt` for the master short regression and
for Earth flat-ground RL deployment checks. Both policies fail to produce
effective motion at `vx=0.10 m/s`, so the low-speed deadband remains a known
behavioral limitation. Plane is clearly better from `0.20` through `0.40 m/s`:
it tracks commanded speed more closely, has lower yaw drift at the useful speed
points, and remains upright without falls.

No control parameters were tuned during this comparison.

## Validation

- Comparison worktree build: PASS via `./tools/build_with_venv.sh`
- Script syntax: PASS via `python3 -m py_compile`
- C++ whitespace check: PASS via `git diff --check`
- Runtime policy loader proof: PASS for stair and plane
- Controller regression: PASS for zero command and nonzero policy sweeps, with
  median LowCmd cadence at 500 Hz

## Residual Risks

- The dedicated joint-controller apply-side trace did not produce separate
  `*_lowcmd_apply.csv` files in this quick run; cadence evidence comes from
  `unitree_timing.csv` LOWCMD events.
- RTF is non-blocking by task definition and varied across windows; comparison
  conclusions use simulation-time windows and body-frame motion metrics.
- `vx=0.10 m/s` remains ineffective for both policies and should not be treated
  as solved by selecting the plane policy.
