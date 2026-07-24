# Boundary Validation

Runtime boundary node was launched by `single_floor_exploration.launch`, and `/navigation/boundary` appeared in the topic-hz capture.

The validation did not proceed to waypoint/boundary containment because the first gate failed before valid FAST-LIO2 input:

`FAST_LIO_INPUT_BLOCKED`

Root cause:

- `rospack find fast_lio` failed in the task worktree after sourcing `/opt/ros/noetic/setup.bash` and `devel/setup.bash`.
- `auto.sh` reported `fast_lio: NOT_FOUND`.
- `logs/fast_lio2.log` reported it could not launch `fast_lio/fastlio_mapping`.
- `/Odometry` timed out, so the navigation data path had no real state or registered cloud input.

Boundary itself was not identified as the first blocker.
