# FALCO Speed Profile Summary

## Verdict

`FALCO_SPEED_PROFILE_PASS` for isolated FALCO path follower semantics.

## Profile

- Straight or small heading error: target `0.80 m/s`.
- Ordinary turn around 30 deg: target `0.60 m/s`.
- Large heading error above 60 deg: target `0.20 m/s`.
- Maximum angular speed: `0.22 rad/s`.
- Linear slope limit: existing `maxAccel=0.6 m/s^2`.
- Angular slope limit: `maxYawAccel=1.0 rad/s^2`.
- Path or odometry stale for `0.5 s`: raw FALCO command becomes zero.

## Evidence

| Probe | Samples | Max Linear | Max Abs Angular |
|---|---:|---:|---:|
| Straight path | 281 | 0.803999543 | 0.000000000 |
| 30 deg path | 280 | 0.600000143 | 0.219911486 |
| 70 deg path | 280 | 0.203999937 | 0.219911486 |

CSV files:

- `falco_pathfollower_cmd_straight.csv`
- `falco_pathfollower_cmd_turn30.csv`
- `falco_pathfollower_cmd_turn70.csv`

## Bridge

Bridge limits are aligned at `max_linear_x=0.80`, `max_angular_z=0.22`. The speed probes measured raw FALCO output directly on `/navigation/falco/cmd_vel_stamped`; the 0.8 m/s result is not caused by bridge clipping.
