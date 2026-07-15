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
| tmux generated-runtime-script probe | PASS: named session stayed alive and sourced ROS Noetic |
| `TERMINAL_BACKEND=tmux ./auto.sh` controlled startup | PASS: `simenv-junior_ctrl` and `simenv-rviz` both had a live Bash pane (`dead=0`); `/Odometry` averaged about 10 Hz |
| `SIGINT` cleanup after tmux smoke test | PASS: trap removed both sessions and checked ROS/runtime processes |

## Implementation

`launch_in_terminal()` runs `gnome-terminal.real` through `env -i`, retaining
only user identity and desktop-session variables. The command inside the new
terminal sources ROS Noetic and workspace setup again, then restores the
required Gazebo and ROS paths. When its main command exits, it replaces the
wrapper with an interactive shell rather than reading one byte; this preserves
the terminal and its error output until the user types `exit`.
Terminal commands run in a child shell so `rosrun` cannot replace that outer
shell with RViz and skip the preservation step.

tmux 3.2a was installed as an authorised, isolated system package. `auto.sh`
uses it by default to host `simenv-junior_ctrl` and `simenv-rviz`; GUI terminals
only attach to those sessions.

The first integration smoke test exposed a tmux `/bin/sh` incompatibility with
the Bash-specific escaping produced by `printf %q` for multiline commands. The
launcher now writes each command to `logs/<title>.tmux.sh` and starts that file
directly; the file is retained as runtime diagnostics.

The smoke test then exposed and corrected a second issue in the inherited
session environment construction: `GAZEBO_MODEL_PATH` was split across two
quoted fragments, yielding invalid Bash in the generated script. It is now
exported as one quoted assignment.

## Residual interactive check

The Codex command runner has no controlling terminal, so it cannot type a
keyboard command into an attached controller pane. It has verified the durable
tmux side: both named sessions remained alive through full startup, and cleanup
removed them. A user-side manual interaction remains the final confirmation
that the controller accepts `2`, `4`, or `6` in the particular desktop terminal
used to attach to `simenv-junior_ctrl`.
