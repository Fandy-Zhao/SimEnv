# Dedicated Terminal Launch Notes

## Validation

| Command | Result |
| --- | --- |
| `gnome-terminal -- ...` from Snap Code environment | FAIL, exit 127 with GLIBC symbol lookup error |
| clean `/usr/bin/gnome-terminal.real -- ...` probe | PASS, command in new terminal executed |
| `bash -n auto.sh` | PASS |

## Implementation

`launch_in_terminal()` runs `gnome-terminal.real` through `env -i`, retaining
only user identity and desktop-session variables. The command inside the new
terminal sources ROS Noetic and workspace setup again, then restores the
required Gazebo and ROS paths.
