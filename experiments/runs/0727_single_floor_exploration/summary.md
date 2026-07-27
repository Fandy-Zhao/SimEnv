# Single-floor exploration result

Verdict: `SINGLE_FLOOR_EXPLORATION_BLOCKED`.

The repaired runtime produced a real 2D map, a DSV goal, waypoint/path traffic,
non-zero velocity commands from a single bridge publisher, Trotting state, and
a 0.24 m measured robot trajectory. The recorder saved all available partial
results and a 12 MB topic-whitelist rosbag.

This was not a complete exploration. The final run covered 14.022 simulated
seconds in 463.149 wall seconds (RTF 0.0303), reached zero goals, recorded no
natural completion method, and did not establish explicit frontier generation
or repeatability. The branch therefore was not eligible for local master merge.

Artifacts remain under the absolute directory:
`/home/zzf/search_ws/SimEnv_worktrees/single-floor-exploration-artifacts-0727/experiments/runs/0727_single_floor_exploration/artifacts/`.
