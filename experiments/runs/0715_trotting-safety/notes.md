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

## Nominal-stance and readiness-gate extension (2026-07-16)

### Implementation contract

- A1's single foot-space nominal stance remains
  `z=-0.318 m`; the backing member is now explicitly named
  `_feetPosNominalStance`.
- FixedStand computes its twelve joint targets through
  `robotModel->getQ(getFeetPosIdeal(), BODY)` on entry. The duplicated
  `[0, 0.9, -1.8]` target array is gone.
- Trotting inherits estimator body position and all current global foot
  positions. Its body-height target follows a smoothstep from inherited height
  to `0.318 m` over `0.75 s`.
- Wave readiness requires the height transition to finish, desired all-stance,
  four fresh measured foot forces above `1 N`, body speed below `0.12 m/s`,
  angular speed below `0.35 rad/s`, and absolute roll/pitch below 10 degrees for
  a continuous `0.20 s`.

### Contact and model root cause

`CtrlComponents::contact` is a desired gait schedule, not physical contact.
Gazebo publishes four 100 Hz WrenchStamped topics, but IOROS did not consume
them. The controller now copies their force magnitudes and one-second wall-time
freshness into `LowlevelState`; the real SDK path copies A1 foot-force fields.

The first contact-enabled A/B still made the nominal IK stand oscillate and
fall. Gazebo model inspection then showed every foot link and collision twice.
`a1_description/xacro/leg.xacro` assigned the same foot child to both a fixed
joint and a zero-range revolute joint. This violates URDF's single-parent tree
and prevented the fixed foot from being lumped into the calf as expected by the
contact sensor. The repository's generated `a1.urdf` and the reference layout
only use the fixed joint. Removing the extra joint restored one collision per
foot, correct contact sensing, and static stability. A trial using Gazebo's
native joint velocity did not remove the physical oscillation and was reverted.

### Final headless evidence

Configuration: seed 71502, one floor/two rooms, GUI/RViz/FAST-LIO2/sensor data
disabled, Gazebo 2 ms step at 500 Hz, ODE 40 iterations, controller `dt=0.002`.

- FixedStand before transition at `56.370 s`:
  position `(-0.001115, 2.307273, 0.326198) m`; linear and angular speeds were
  approximately `2.4e-5 m/s` and `3.7e-5 rad/s`. Foot force magnitudes were
  approximately `[10.9, 11.1, 12.6, 12.9] N`.
- Trotting entered at `61.810 s`, inherited estimated height `0.306 m`, and
  announced its `0.75 s` transition to `0.318 m`.
- Readiness became true at `62.780 s`, after `0.20 s` stable, with
  `|v|=0.006 m/s`, `|w|=0.000 rad/s`, and forces
  `[9.2, 9.5, 11.3, 11.6] N`.
- Zero-command sample at `72.146 s` remained upright at
  `(-0.000942, 2.297635, 0.349350) m`; velocity was about `3e-6` and no
  non-finite guard fired.
- Forward pair: from `106.914 s`,
  `(-0.000926, 2.297628, 0.349349) m`, to `113.834 s`,
  `(-0.007170, 4.188549, 0.348097) m`. Displacement was
  `(-0.006244, +1.890921) m`, or `1.890931 m / 6.920 s = 0.273256 m/s`.
  With initial yaw near +90 degrees, world +Y is the correct body-forward
  response to `linear.x=0.3`.
- The controller logged wave start at `108.184 s`, explicitly after height,
  velocity, attitude, and four-foot contact checks passed.
- The final accepted log had no Trotting non-finite-output error. A stale zero
  publisher produced the expected timeout-to-stop warning at `81.664 s`.

### Rejected independent-foot compatibility A/B

An additional `preserveFixedJoint` trial restored four independent foot bodies,
four collisions, and `/ground_truth/FR_foot` publication, but did not retain
stable dynamics. After recovery time the model still slid continuously, FR
calf velocity reached `4.86 rad/s`, and calf torque saturated at `33.5 Nm`.

The accepted configuration therefore uses the canonical single fixed foot
joint with Gazebo lumping. The four force topics remain valid because their
sensors bind the lump collisions. Legacy `/ground_truth/*_foot` pose-topic
compatibility is a residual issue; Trotting estimation uses kinematics and is
unaffected.
