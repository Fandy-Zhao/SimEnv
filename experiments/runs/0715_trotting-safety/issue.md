# Issue: Trotting command safety and finite-output repair

## Background

The headless locomotion validation on `exp/0715-trotting-rl-cmd-validation`
showed that entering Trotting with a zero `/cmd_vel` makes Gazebo model and IMU
velocities non-finite before a nonzero command is sent. Static analysis found
that `_dYawCmdPast` is consumed by the yaw-rate filter before initialization.

The user-provided reference workspace is
`/home/zzf/search_ws/unitree_rl` at commit `0ccd0e7`. Its remote is
`https://github.com/dstx123/unitree_rl.git`; it is a third-party RL deployment
derived from `unitree_guide`, not an official Unitree Robotics repository. It
does not contain `State_Trotting`, but its `State_RL::_init_buffers()` explicitly
zeros `_dYawCmdPast` and its policy path clips observations/actions.

## Scope

- Initialize and reset all Trotting command/filter state on construction and
  state entry.
- Reject non-finite `/cmd_vel`, clamp accepted commands, and stop on stale
  programmatic commands.
- Prevent non-finite gait/IK/torque values from reaching Gazebo motor topics.
- Preserve keyboard/joystick fallback and the classical A1 gait algorithm.
- Validate FixedStand -> Trotting under headless `auto.sh`, then exercise zero
  and forward `/cmd_vel` at a continuous publish rate.

## Out of scope

- Replacing the classical Trotting controller with the third-party RL policy.
- Repairing the separate `State_RL_test` policy contract.
- Modifying policy `.pt` files, Gazebo scene generation, FAST-LIO2, or system
  ROS Noetic.

## Acceptance criteria

1. `catkin_make -j` passes with the Torch-enabled locomotion build profile.
2. Trotting entry and a zero Twist keep model pose/velocity and IMU finite.
3. A repeated bounded forward Twist produces a finite, directionally sensible
   response, or the residual gait failure is captured with adjacent simulated
   time and pose evidence.
4. A non-finite or stale Twist cannot be propagated as a motor command.
5. Project status and the task report record source provenance, tests, risks,
   and follow-up work.
