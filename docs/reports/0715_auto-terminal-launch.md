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

## Scope

The change is limited to `auto.sh`; it does not modify the system terminal,
Snap installation, or ROS Noetic.

## Validation

- The original wrapper reproducibly failed with a GLIBC symbol lookup error.
- A clean direct-terminal probe executed successfully.
- Script syntax and the patched launcher are validated before commit.
- A non-interactive full `auto.sh` smoke test reached FAST-LIO2 and 10 Hz
  odometry, but cannot certify GUI-window persistence because its VTE children
  exited under the command-runner timeout harness. A direct controller-terminal
  test retained its shell; final confirmation requires the user's interactive
  terminal session.
