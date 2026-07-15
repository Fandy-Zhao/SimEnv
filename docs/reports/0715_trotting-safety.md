# Task Report: A1 Trotting command and finite-output repair

## Outcome

Trotting now accepts continuous `/cmd_vel`, remains finite under zero and
forward commands, rejects non-finite input, and stops after a stale command.
The robot moved in the commanded body-forward direction, but its measured
average planar speed was only about `0.121 m/s` for a `0.3 m/s` request, so
directional locomotion passes while velocity calibration remains incomplete.

The prior immediate NaN was not caused by one fault alone. The uninitialized
yaw filter was real, but instrumentation identified the first runtime
non-finite output as inverse-kinematics joint position. Gazebo 11 reported the
folded calf angle as `+3.5866 rad` instead of the equivalent negative URDF
angle, and the robot was spawned close to the folded-knee singularity.

## Governance

- Previous task branch `exp/0715-trotting-rl-cmd-validation` was merged into
  `master` with merge commit `7eb2a947`.
- New task branch: `fix/0715-trotting-safety`.
- No remote push or branch deletion was performed.
- Issue and experiment evidence:
  `experiments/runs/0715_trotting-safety/`.

## Reference provenance

The user-provided `/home/zzf/search_ws/unitree_rl` is commit `0ccd0e7` with
remote [dstx123/unitree_rl](https://github.com/dstx123/unitree_rl). It is a
third-party RL deployment derived from Unitree Guide, not a Unitree Robotics
official repository, and it contains no `State_Trotting`.

Useful transferable behavior from its `State_RL` was applied selectively:

- initialize `_dYawCmdPast` before the first filtered command;
- bound external inputs/outputs;
- establish an explicit control contract before publishing joint targets.

The classical gait remains based on
[Unitree's official unitree_guide](https://github.com/unitreerobotics/unitree_guide).
Neither reference was copied wholesale: the third-party model is for a
different RL contract, while the official classic stance values were unstable
under this project's current Gazebo contact/gain tuning.

## Cause analysis

### 1. Uninitialized yaw filter

`State_Trotting::_dYawCmdPast` was consumed in
`0.9 * previous + 0.1 * requested` without initialization. It could contaminate
yaw target, balance control, foot placement, IK, and motor output. It is now
initialized in the class and reset on every Trotting entry.

### 2. Wrapped calf feedback and singular startup pose

The first field-level run produced:

```text
foot_p=1 foot_v=1 force_g=1 force_b=1 q=0 qd=1 tau=1
```

The FixedStand command was `q=-1.8`, but the calf controller published
`q=+3.5866` and saturated at `-33.5 Nm`. Gazebo 11 had selected the positive
equivalent angle for the bounded revolute joint. Direct PD subtraction and A1
kinematics expect the signed URDF interval.

The controller now maps revolute feedback through
`atan2(sin(q), cos(q))` before velocity, PD, and state publication. The launch
pose also changes from the near-limit `(-/+ hip, 1.36, -2.65)` configuration to
the existing local FixedStand target `(0, 0.9, -1.8)`, avoiding folded-knee IK
singularity on Trotting entry.

### 3. Missing command/output safety contract

Trotting previously treated the first `/cmd_vel` as active forever and allowed
NaN input or downstream NaN joint targets to reach motor topics. The repaired
path:

- rejects non-finite Twist values;
- clamps x/y/yaw commands before and after yaw filtering;
- expires programmatic commands after `0.5 s` wall time (configurable with
  `trotting_cmd_vel_timeout`, valid range `0.1--5.0 s`);
- verifies estimator, command, foot, force, IK, and torque values;
- holds current measured joint positions with zero velocity/torque if a finite
  check fails.

## Runtime evidence

Headless setup used seed `71501`, one floor, no RViz/FAST-LIO2/nonessential
sensors, Gazebo `2 ms / 500 Hz / ODE 40`, and controller `dt=0.002`. The build
cache had `UNITREE_ENABLE_TORCH_POLICY=ON`.

State commands were published continuously while physics was paused, then
physics was unpaused. This avoids the known startup window in which a one-shot
FixedStand command is cleared before full joint feedback is ready.

### Zero command

After entering Trotting, more than 40 simulated seconds completed without a
finite-guard warning. An adjacent settled interval moved about `0.0206 m` over
`21.206 s`; IMU and Gazebo states stayed finite, with body height around
`0.34 m`.

### Forward command pair

| Sample | Sim time | Position `(x, y, z)` m | Yaw |
|---|---:|---|---:|
| Before | `128.338 s` | `(-1.761917, 2.454028, 0.342231)` | `-26.2 deg` |
| After | `204.346 s` | `(7.194509, 0.410898, 0.331091)` | approximately preserved |

The `linear.x=0.3` window produced `(dx,dy)=(8.956426,-2.043130) m`, distance
`9.186509 m`, average `0.120862 m/s`, and displacement heading `-12.85 deg`.
That is a strong body-forward response at the initial negative yaw, but only
about 40% of requested speed over this long open-loop window.

### Safety inputs

- Stopping the publisher produced `Trotting /cmd_vel timed out; commanding zero
  velocity`.
- Publishing `.nan` produced `Trotting rejected a non-finite /cmd_vel and
  commanded a stop`.
- The model state after both checks remained finite.

## Files changed

- `State_Trotting.h/.cpp`: initialize/reset command state, finite validation,
  clamp/filter/freshness logic, and safe motor-output fallback.
- `joint_controller.cpp`: normalize Gazebo revolute feedback to the signed
  angle branch used by URDF commands and kinematics.
- `multi_floor_gazeboSim.launch`: start A1 joints at the local FixedStand pose
  instead of the folded-knee limit.
- `CHANGELOG.md`, `PROJECT_STATE.md`, `docs/module_status.md`: project status.
- `experiments/runs/0715_trotting-safety/*`: issue, reproduction, and evidence.

## Tests

- `catkin_make -j`: pass after controller, gait, and stance changes.
- Sanitized-system-Python `roslaunch --files unitree_guide
  multi_floor_gazeboSim.launch`: pass (the unsanitized IDE shell still exhibits
  the already-known Miniconda 3.13/xacro incompatibility).
- `git diff --check`: pass after generated runtime artifacts were restored.
- Headless FixedStand -> Trotting transition: pass.
- Continuous zero Twist finite/stability gate: pass after Trotting settles.
- Continuous body-forward Twist: directional movement pass; speed tracking
  fail/needs calibration.
- Stale Twist watchdog: pass.
- Non-finite Twist rejection: pass.
- A direct official `0.67/-1.3` stance A/B was rejected because it rolled under
  current contact/gain parameters.

## Risks

- FixedStand and the transition into Trotting still show a sizable transient;
  navigation should wait for a low-velocity/upright readiness gate.
- The long forward test crossed a large open-loop distance and did not isolate
  contact/collision effects; repeat with a short simulated-time harness for
  accurate gain calibration.
- The joint normalization assumes A1/Go1 bounded revolute coordinates whose
  intended command interval is within `[-pi, pi]`.
- RL policy behavior is unchanged and remains blocked on missing training and
  joint-order metadata.
- The AMP recording thread and hard-coded output path in Trotting remain
  unrelated technical debt.

## Next steps

1. Add a reusable simulated-time locomotion test harness that publishes at
   20 Hz and samples exact 2--5 s windows.
2. Gate Trotting entry on upright pose, low body velocity, and non-singular
   leg geometry.
3. Tune command-to-speed response and validate forward/lateral/yaw separately.
4. Keep RL repair as a separate task until policy joint ordering,
   normalization, and export metadata are recovered.
