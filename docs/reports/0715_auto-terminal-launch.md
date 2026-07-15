# Task Report — Dedicated Terminal Launch from Snap Code

## Summary

`auto.sh` already requested two GNOME Terminal windows, but the standard
`gnome-terminal` wrapper inherited Snap Code's library environment and failed
before creating either window. The launcher now invokes `gnome-terminal.real`
with a minimal desktop-session environment, then the terminal command sources
ROS and the workspace normally.

## Scope

The change is limited to `auto.sh`; it does not modify the system terminal,
Snap installation, or ROS Noetic.

## Validation

- The original wrapper reproducibly failed with a GLIBC symbol lookup error.
- A clean direct-terminal probe executed successfully.
- Script syntax and the patched launcher are validated before commit.
