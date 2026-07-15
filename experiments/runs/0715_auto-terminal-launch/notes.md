# Dedicated Terminal Launch Notes

## Validation

| Command | Result |
| --- | --- |
| `gnome-terminal -- ...` from Snap Code environment | FAIL, exit 127 with GLIBC symbol lookup error |
| clean `/usr/bin/gnome-terminal.real -- ...` probe | PASS, command in new terminal executed |
| `bash -n auto.sh` | PASS |
| `gnome-terminal.real` + `exec bash -i` hold probe | PASS: terminal child remains alive after the launch command exits |
| `rosrun` failure inside terminal child shell | PASS: outer diagnostic shell survives `rosrun`'s `exec` |
| `GUI=false ENABLE_RVIZ=1 ./auto.sh` under a non-interactive `timeout` harness | Startup and `/Odometry` at 10 Hz PASS; its VTE children exited before the harness ended, so this is not accepted as proof of interactive-window survival |
| Direct controller terminal with a live `junior_ctrl` child | PASS: the VTE terminal shell remained alive while the controller process ran |

## Implementation

`launch_in_terminal()` runs `gnome-terminal.real` through `env -i`, retaining
only user identity and desktop-session variables. The command inside the new
terminal sources ROS Noetic and workspace setup again, then restores the
required Gazebo and ROS paths. When its main command exits, it replaces the
wrapper with an interactive shell rather than reading one byte; this preserves
the terminal and its error output until the user types `exit`.
Terminal commands run in a child shell so `rosrun` cannot replace that outer
shell with RViz and skip the preservation step.

## Residual interactive check

The Codex command runner has no controlling terminal. Its full `auto.sh`
timeout harness successfully reached FAST-LIO2 and 10 Hz odometry, but the VTE
children exited before timeout. The direct controller-terminal test did retain
its VTE child. A final manual run from the user's interactive VS Code terminal
is required to confirm window persistence in that exact session type.
