# Task Report — Dedicated Terminal Launch from Snap Code

## Summary

`auto.sh` already requested two GNOME Terminal windows, but the standard
`gnome-terminal` wrapper inherited Snap Code's library environment and failed
before creating either window. The launcher now invokes `gnome-terminal.real`
with a minimal desktop-session environment, then the terminal command sources
ROS and the workspace normally. If the controller or RViz command exits, the
terminal remains open in an interactive diagnostic shell instead of flashing
closed. Because `rosrun` itself uses `exec`, each launched command is isolated
in a child shell so the outer diagnostic shell always survives.

The default backend is now tmux. Each command is hosted in a named detached
session and GNOME Terminal only attaches to it, so a failed GUI terminal no
longer terminates the controller or RViz command. The multiline launch command
is written to a runtime script under `logs/`, avoiding tmux `/bin/sh` parsing
of Bash-specific quoted text.

## Scope

The change is limited to `auto.sh`; it does not modify the system terminal,
Snap installation, or ROS Noetic.

## Validation

- The original wrapper reproducibly failed with a GLIBC symbol lookup error.
- A clean direct-terminal probe executed successfully.
- `bash -n auto.sh` and an isolated tmux runtime-script probe both pass.
- A controlled full startup with `TERMINAL_BACKEND=tmux` retained both
  `simenv-junior_ctrl` and `simenv-rviz` sessions after startup; each pane was
  a live interactive Bash process (`dead=0`). FAST-LIO2 `/Odometry` published
  at about 10 Hz.
- Sending `SIGINT` to the launcher completed the normal trap cleanup and
  removed both tmux sessions and all checked runtime processes.

# Task Report

## Branch

`fix/0715-build-auto-startup`

## Files Changed

- `auto.sh`: tmux-backed terminal sessions and cleanup.
- `CHANGELOG.md`, `PROJECT_STATE.md`, `docs/module_status.md`, and
  `docs/quick-start.md`: operational status and reattach instructions.
- This report and `experiments/runs/0715_auto-terminal-launch/notes.md`:
  diagnostic evidence and smoke-test results.

## Git

Implementation commit: `61d2d03f fix(auto): retain terminal sessions with tmux`.

## Risk and Next Step

The automated runner cannot send keyboard input through a desktop-attached tmux
client. Start `./auto.sh`, attach with `tmux attach-session -t
simenv-junior_ctrl`, and verify `2` (stand) from the intended user terminal.
