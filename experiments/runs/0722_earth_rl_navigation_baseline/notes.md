# Notes: Earth RL Navigation Baseline

## Audit
- `master` already merged; no duplicate merge.
- HEAD: `4eeae260b452a296b17f30afaea3b7b2edb7c636`.

## Build
- Build PASS, `WORLD_MODE=earth`, `PHYSICS_PROFILE=normal`.

## Runtime
- Plane policy SHA:
  `e886847fe266e3c2f7c08825fceeaecfa75c7eac5f780b25b6d4dca173ff8bef`.
- RL zero: min height 0.311609 m, max tilt 1.008 deg.
- vx=0.20: median body vx 0.155124 m/s, tracking gain 0.775622.
- IMU fallback: `using_imu_policy_input=[1]`.
- LowCmd median: 500 Hz.
- Recorded non-blocking RTF fluctuation: final governance windows showed RTF
  medians around 14 sim-time / wall-time in this headless run.

## Preserved interfaces
- `/cmd_vel`
- `/fsm/state_cmd`
- `/clock`
- Robot state / odometry source
- IMU input
- RL policy runtime selection
- LowCmd transport/application cadence

## Known limitations
- `vx=0.10 m/s` deadband remains.
- Stair policy and obstacle/stair behavior remain unvalidated by this
  flat-ground baseline.
- RTF fluctuation remains recorded but non-blocking.

## Remote push
No.
