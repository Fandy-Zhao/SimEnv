# Issue: Restore full catkin configuration and fail fast for missing controller

## Goal

Allow the workspace to configure without the optional legacy PS3 USB utility
and make `auto.sh` fail before simulation startup if the required controller
binary is absent.

## Scope

- Optional `ps3joy` `sixpair` build dependency handling.
- `auto.sh` controller-artifact preflight.
- Build and startup documentation/status records.

## Non-scope

- Do not install or modify system ROS Noetic packages.
- Do not port the obsolete libusb-0.1 `sixpair` source to libusb-1.0.

## Acceptance criteria

1. Missing legacy `libusb` no longer blocks workspace configuration.
2. `auto.sh` reports a missing `junior_ctrl` before generating a scene or
   launching Gazebo.
3. Relevant controller and FAST-LIO2 packages rebuild successfully.
