# Task Report — `auto.sh` Controller Keyboard Interaction

## Branch

`fix/0715-auto-keyboard`

## Summary

Fixed the controller startup path used with FAST-LIO2. `junior_ctrl` now reads
from `/dev/tty` even while it is initially backgrounded for startup ordering.
The script later waits for that process instead of using Bash `fg`, so the
terminal remains the keyboard-control session.

## Validation

- Bash syntax and static launch-path assertions passed.
- Full runtime launch was not run because it would terminate the active ROS /
  Gazebo session and overwrite its generated scene.

## Risk

Keyboard control requires an interactive controlling terminal. Non-TTY use is
supported through `/fsm/state_cmd` and `/cmd_vel` only.
