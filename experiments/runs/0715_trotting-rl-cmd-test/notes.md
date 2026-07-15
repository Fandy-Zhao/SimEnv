# Trotting/RL headless command-control experiment

## Plan

1. Record repository/build provenance and statically trace FSM and `/cmd_vel`.
2. Launch a deterministic one-floor headless scene with nonessential mapping,
   RViz, and sensor processing disabled to improve real-time factor.
3. Confirm `/clock`, controller subscribers, joint feedback, and upright fixed
   stand.
4. Test Trotting with a zero-command settling period, forward command, stop,
   and ground-truth snapshots.
5. Recover to FixedStand (or restart if necessary), then repeat for RL.
6. Inspect controller/Gazebo logs and calculate displacement, velocity,
   orientation, height, and simulated elapsed time.
7. Record findings, risks, and the official-source comparison recommendation.

## Test configuration

- Working branch: `exp/0715-trotting-rl-cmd-validation`
- Interface interpretation: requested `/cmd` means the implemented
  `geometry_msgs/Twist` topic `/cmd_vel`.
- GUI: disabled.
- RViz and FAST-LIO2: disabled.
- Scene: one floor, reduced room/hazard count, fixed seed.
- Controller: current Torch-enabled `junior_ctrl` build.

## Results

### Provenance and startup

- Baseline commit on `master`: `2a36abbe`.
- `UNITREE_ENABLE_TORCH_POLICY=ON`; `junior_ctrl` linked libtorch and loaded
  `policy_act_inference_plane.pt`.
- Official comparison: `unitreerobotics/unitree_guide` commit
  `fdf4d23de6affe8ee38fb4d892f61053fa1fcbcb` (2024-05-07).
- With FAST-LIO2 disabled, `auto.sh` does not issue FixedStand. An early
  one-shot state command can also be consumed before complete feedback and
  lost. Repeating FixedStand around unpause produced a reliable baseline.
- The 4 ms / 250 Hz / ODE 20 profile destabilized FixedStand and was rejected.
  Accepted tests used 2 ms / 500 Hz / ODE 40 and controller `dt=0.002`.

### FixedStand baseline

At `/clock=15.302 s`: position approximately
`(-0.01156, 2.29873, 0.12001)`; velocities were near zero and pose was finite.

### Trotting

- A zero Twist was published before `/fsm/state_cmd data: 4`.
- FSM confirmed `fixed stand -> trotting`.
- Before any nonzero command, `a1_gazebo`, `/ground_truth/base_w`, and
  `/trunk_imu` developed non-finite velocities. The model and both processes
  remained present/alive.
- Verdict: entry succeeds; zero-command Trotting is numerically unsafe, so
  movement control and normal walking fail.

### RL

- Separate restart confirmed stable FixedStand and `fixed stand -> RL`.
- Zero command remained finite but drifted and rotated.
- Strict paired `linear.x=+0.3 m/s` measurement:
  - before: `/clock=34.748 s`, `(x,y,z)=(0.022906,1.974482,0.101569)`, yaw
    about `+26.6 deg`;
  - after: `/clock=39.434 s`, `(0.022591,1.938075,0.102628)`, yaw about
    `+17.0 deg`;
  - over `4.686 s`: `dx=-0.000315 m`, `dy=-0.036406 m`, yaw about `-9.6 deg`.
- Body +X should have produced substantial world +X/+Y motion at the starting
  yaw. Magnitude and direction were wrong, with uncontrolled turning.
- Verdict: policy inference is finite, but `/cmd_vel` tracking fails.

### Root-cause findings

1. Trotting declares `_dYawCmdPast` without initialization and uses it on the
   first command-filter update. Garbage/non-finite data can propagate through
   `_yawCmd`, `_Rd`, balance forces, IK, and motor commands. This is the leading
   explanation for immediate NaN; the exact first non-finite value still needs
   an instrumented run.
2. Official Trotting is a useful A/B baseline, but it also leaves this member
   uninitialized. This fork additionally adds `/cmd_vel`, fixed limits, and an
   AMP thread.
3. RL reorders feedback/actions with
   `{3,4,5,0,1,2,9,10,11,6,7,8}`, while the default joint tensor's hip signs
   match the current Gazebo FR/FL/RR/RL order. This likely observation/action
   ordering mismatch explains left/right asymmetry and yaw drift.
4. RL policy training metadata is absent. Inference also writes `_lowCmd` from
   a worker thread without synchronization; the thread flag is set after
   creation; commands have no finite/range/freshness checks.

### Official-reference decision

- Necessary for a controlled Trotting A/B against the fork and custom model.
- Insufficient for RL: official `unitree_guide` has no equivalent RL state or
  policy. The original training/export repository is required.
