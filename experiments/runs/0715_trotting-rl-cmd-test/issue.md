# Issue: Trotting/RL headless command-control validation

## Goal

Run `auto.sh` without a GUI and determine whether the Unitree A1 can enter the
Trotting and RL FSM states, accept velocity commands, move in the commanded
direction, and maintain a normal walking posture.

## Scope

- Start the current working tree with `GUI=false`, `ENABLE_RVIZ=false`, and a
  deterministic reduced scene suitable for repeatable locomotion checks.
- Switch states through `/fsm/state_cmd`.
- Send velocity commands through the implemented `/cmd_vel` interface.
- Measure Gazebo ground-truth displacement, velocity, orientation, height, FSM
  process health, and visible controller/runtime errors.
- Compare Trotting and RL behavior and decide whether an official
  `unitree_guide` source comparison is necessary.
- Produce experiment notes and a detailed report.

## Non-scope

- No controller tuning or source-code fix is authorized by this diagnostic
  issue.
- No changes to the system ROS Noetic installation.
- No GUI/RViz validation, real-robot command, navigation-stack, or SLAM quality
  evaluation.
- No deletion or modification of pre-existing uncommitted work.

## Acceptance criteria

- Headless `auto.sh` reaches a healthy ROS/Gazebo/controller startup.
- Trotting and RL each receive an explicit FSM command and `/cmd_vel` stimulus.
- Each mode has before/during/after ground-truth evidence and controller logs.
- Normal walking is assessed from pose stability, height/orientation, joint
  feedback, progress, and absence of crashes/non-finite values.
- Failures are localized to command routing, state transition, policy/control,
  physics, or test-environment causes where evidence permits.
- The report states whether comparison with official `unitree_guide` is needed
  and what should be compared.

## Risks

- Gazebo real-time factor is known to be low, so wall-clock command duration
  may provide little simulated motion.
- Trotting and RL may destabilize the robot or terminate `junior_ctrl`.
- The current branch contains pre-existing uncommitted RL and changelog edits;
  results describe that exact working tree and must not be confused with a
  clean baseline.
- A shared ROS master or stale Gazebo process can invalidate measurements.

## Expected impacted modules

- Runtime under test: `unitree_guide`, `auto.sh`, Gazebo generated scene.
- Documentation only: this experiment directory, `docs/reports/`, and one
  project status document.
