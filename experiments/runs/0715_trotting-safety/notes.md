# Trotting safety experiment notes

## Governance

- Parent branch: `master` at merge commit `7eb2a947`.
- Task branch: `fix/0715-trotting-safety`.
- Previous task merged with `--no-ff`; no push and no branch deletion.

## Reference audit

- User-provided reference: `/home/zzf/search_ws/unitree_rl`.
- Reference commit: `0ccd0e7` (`master`).
- Reference remote: `https://github.com/dstx123/unitree_rl.git`.
- Provenance: third-party project; README says it is based on Unitree Guide and
  thanks Unitree's `rl_ws`. It is not an official Unitree Robotics remote.
- The reference contains `State_RL` but no `State_Trotting`.
- Relevant transferable behavior:
  - `_init_buffers()` sets `_dYawCmdPast=0.0` before the first filtered command;
  - observations and actions are clipped;
  - policy inference has an explicit 50 Hz period and joint targets are
    interpolated before 500 Hz publication.
- Current Trotting remains a classical gait/balance/IK controller at the main
  500 Hz loop. Its dynamics should not be converted to 50 Hz.

## Initial fault path

`State_Trotting::getUserCmd()` computes
`0.9 * _dYawCmdPast + 0.1 * requested_yaw_rate`, while `_dYawCmdPast` has no
initializer and `enter()` does not reset it. An indeterminate first yaw command
can flow through `_yawCmd`, `_Rd`, balance control, foot placement, IK and motor
commands. The previous zero-Twist test observed Gazebo/IMU NaN immediately
after the FixedStand -> Trotting transition.

## Planned validation

- Build: `catkin_make -j` using the existing Torch-enabled cache/profile.
- Runtime: deterministic headless `auto.sh`, default 2 ms physics/controller
  period, no RViz/FAST-LIO2.
- State sequence: repeated `2` to FixedStand, then repeated `4` to Trotting.
- Commands: continuous zero Twist, then continuous `linear.x=0.3` Twist; record
  `/clock`, `/gazebo/get_model_state`, and `/trunk_imu` immediately around each
  command window.

## Results

### Root cause refinement

The first guard-instrumented run showed that `foot_p`, `foot_v`, balance
forces, `qd`, and torque were finite, while only IK position `q` was not. The
calf feedback exposed the upstream cause:

- FixedStand command: `q=-1.8`, `Kp=140`, `Kd=7`.
- Gazebo feedback before normalization: `q=+3.5866` with saturated
  `tauEst=-33.5`; this is the wrapped equivalent of about `-2.6966`.
- The original spawn pose placed each calf at `-2.65`, near the folded knee
  limit. Gazebo 11's positive angle representation broke the PD difference and
  the Trotting IK entered a near-singular configuration.

The implemented repair normalizes bounded revolute feedback with
`atan2(sin(q), cos(q))`, starts simulation at the existing FixedStand target
`(0, 0.9, -1.8)`, initializes the Trotting yaw filter, and blocks non-finite
output.

An attempted official-classic pose `(0, 0.67, -1.3)` was rejected: with this
project's current ramped FixedStand gains/contact parameters it rolled and
translated strongly. The accepted configuration therefore preserves the local
FixedStand target rather than copying upstream geometry without retuning.

### Accepted runtime

- `catkin_make -j`: pass with `UNITREE_ENABLE_TORCH_POLICY=ON`.
- Zero-Twist Trotting: no finite-guard errors over more than 40 simulated
  seconds; model/IMU remained finite. During an adjacent settled window from
  `107.132 s` to `128.338 s`, planar movement was about `0.0206 m`.
- Forward paired sample (`linear.x=0.3`):
  - before at `128.338 s`: `(-1.761917, 2.454028, 0.342231)`, yaw about
    `-26.2 deg`;
  - after at `204.346 s`: `(7.194509, 0.410898, 0.331091)`;
  - displacement `(8.956426, -2.043130) m`, distance `9.186509 m` over
    `76.008 s`, average planar speed `0.120862 m/s`, displacement heading
    `-12.85 deg`.
- Directional response: pass; exact `0.3 m/s` tracking: fail/under-speed.
- Stale command: watchdog warning observed and zero command selected.
- `.nan` Twist: rejected; stop warning observed; subsequent model state finite.
- Controller log contained no Trotting non-finite-output warning in the
  accepted run.

### Residual behavior

FixedStand-to-Trotting startup still has appreciable transient translation in
this contact model. Once Trotting settles under zero command it is nearly
stationary, but precise navigation should wait for an upright/low-velocity gate
and later calibrate command tracking.
