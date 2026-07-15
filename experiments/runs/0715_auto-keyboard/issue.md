# Issue: Preserve keyboard control through `auto.sh` startup

## Goal

Ensure that `junior_ctrl` can read keyboard input from the terminal that runs
`auto.sh`, including when FAST-LIO2 requires the controller to start before
mapping.

## Scope

- Controller stdio and lifecycle handling in `auto.sh`.
- Operator documentation and task-status records.

## Non-scope

- No changes to ROS Noetic, Gazebo, controller keyboard mappings, or SLAM
  logic.

## Acceptance criteria

1. The background controller receives stdin from `/dev/tty`, not `/dev/null`.
2. Foreground mode waits for the controller after startup without relying on
   non-interactive Bash job control.
3. Non-terminal launches report that keyboard input is unavailable.

## Risk

Full runtime startup is destructive to the currently running simulation
session because `auto.sh` deliberately cleans up stale Gazebo/ROS processes.
