# auto.sh event-driven startup and RTF cleanup report

## Outcome

`auto.sh` now advances on observable ROS/Gazebo state with configurable
wall-time limits. Optional legacy conversion and Livox rendering are off by
default, while compatibility switches remain available. The official build,
Cases A-D, controlled failure cleanup, and a simple standalone RTF smoke pass.

## Changed behavior

- Structured stages validate master, Gazebo, robot/controllers, supervisor,
  controller, FixedStand, sensors, FAST-LIO2, navigation, state transitions,
  and recorder prerequisites.
- State requests are continuously published only until the corresponding
  latched output is observed and stable.
- Exit restores exploring/navigation/FSM safety and zero velocity, then
  recursively terminates only tracked task processes.
- `ENABLE_POINTCLOUD_CONVERTER` and `ENABLE_LIDAR_VISUALIZATION` default to
  false. Setting either to `1` restores the optional behavior.

## Validation and risk

See `experiments/runs/0728_auto-event-driven-rtf-cleanup/runtime_validation.md`
for commands and observations. The final candidate mean RTF was `0.037064`
over 20 wall seconds. No matched numerical improvement is claimed because the
fresh old-launcher comparison was invalidated by concurrent external load.
The user's final instruction authorized merge after the simple RTF smoke.

The remaining risk is environmental variability at very low RTF. Timeouts are
wall-clock based and configurable, and topic sampling uses a 12-second inner
window to avoid false negatives when simulated sensor rates become sparse in
wall time.
