# Failure Diagnostics

## Runtime Scope

`auto.sh` + Gazebo + full FAST-LIO2 + Trotting closed-loop validation was not executed in this turn. The reason is scope hygiene: `auto.sh` regenerates tracked scene/log/result artifacts under the worktree, and the safe work completed here already established build, launch, adapter, and isolated FALCO speed semantics. Full S2-S5 should be run in a controlled runtime window with generated artifacts reviewed and excluded before commit.

## Gates Not Claimed

- `SHORT_CLOSED_LOOP_PASS` not claimed.
- `FULL_EXPLORATION_PASS` not claimed.
- `RETURN_HOME_PASS` not claimed.
- Real-cloud terrain-map point statistics were not finalized; current parameters are initial candidates.

## Observations

- A synthetic cloud-only FALCO smoke proved the launch/bridge path but did not generate useful nonzero local-planner speed; direct path follower probes were used to validate speed scheduling.
- `registered_cloud_to_terrain_map.py` correctly refused to publish transformed terrain when no `camera_init -> map` TF existed in the synthetic smoke.
- Disabled bridge does not continuously publish zero if it has already published one zero; this avoids extra command spam but means `rostopic echo -n 3 /cmd_vel` can wait indefinitely in disabled state.
