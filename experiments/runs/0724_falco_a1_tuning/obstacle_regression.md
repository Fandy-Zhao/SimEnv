# R3 Obstacle-Checked Regression

Date: 2026-07-24

Scope: FALCO local path generation only, using real FAST-LIO2 `/Odometry` and
`/cloud_registered`. `/navigation/check_obstacle` was published `true` for
each case. `/navigation/enabled=false` was held during command checks, so
Trotting `/cmd_vel` remained gated to zero and R4 was not entered.

## Result

Verdict: `FALCO_A1_REAL_PATH_READY`

The selected A1 profile (`minRelZ=-0.25`, `vehicleLength=0.56`,
`vehicleWidth=0.43`, `pointPerPathThre=2`) produced nonzero FALCO raw commands
for repeated forward waypoints and directional turn commands for left/right
offset waypoints while obstacle checking remained enabled.

## Evidence

- `reg_front_1`, `reg_front_2`, `reg_front_3`: raw FALCO forward velocity was
  positive (`0.095` to `0.100 m/s`) with near-zero yaw command after the
  waypoint settled.
- `reg_left_02`: selected a left-turn local response (`raw angular z` about
  `+0.785 rad/s`) for a `0.8 m` forward, `0.2 m` lateral waypoint.
- `reg_right_02`: selected a right-turn local response (`raw angular z` about
  `-0.785 rad/s`) for a `0.8 m` forward, `-0.2 m` lateral waypoint.
- `reg_long_front_3m`: kept `checkObstacle=true`, published a local forward
  segment, and diagnostics showed obstacle filtering was active:
  `candidate_paths=6174`, `free_paths` about `4533-4817`, and nonzero
  collision counts on rejected paths. This confirms the planner did not simply
  bypass obstacle checking for long goals.
- In all cases sampled here, `/navigation/enabled=false` kept the final
  `/cmd_vel` sample at zero.

## Limitations

This is a local-planner readiness gate, not a full navigation success proof.
The long-front regression confirms obstacle scoring is active, but it does not
prove whole-building traversal, door/elevator handling, or DSV exploration.
R4 Trotting motion, R5 DSV integration, and R6 full exploration remain future
gates.
