# Experiment Notes

- Date: 2026-07-17
- Branch: `exp/0717-trot-rl-speed-profile`
- Baseline: `abd0f940` (`master` integration head)
- Initial worktree status: clean
- Read before edits: `AGENTS.md`, `auto.sh`, previous mapping trial capture/runner,
  `PROJECT_STATE.md`, `CHANGELOG.md`, and `docs/module_status.md`.
- Fixed test scene: `FLOOR_COUNT=1`, `SEED=77`, spawn `(0.0, 2.3, 0.6, 1.5708)`.
- Each test is a new `auto.sh` epoch, uses FixedStand (`2`) then FSM `4`/`6`, a
  0.5 s simulated forward segment and a 0.25 s simulated zero command segment.
- `real_time_factor = active_sim_elapsed_s / active_wall_elapsed_s`; actual mobility
  is Gazebo-truth XY path length divided by active simulated seconds.
- FAST-LIO2 is intentionally disabled for this control-only benchmark; sensor/Gazebo
  and the Torch-enabled controller remain active. This avoids interpreting mapping
  CPU cost as locomotion capability.
- The runner holds `/tmp/simenv-gazebo.lock`; it uses `/usr/bin/python3`, monotonic
  wall-clock limits, a unique tmux session per trial, and kills only its own process
  group during cleanup.
