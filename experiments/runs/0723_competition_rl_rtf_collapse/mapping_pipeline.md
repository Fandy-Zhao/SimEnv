# Mapping Pipeline Status

Verdict: `MAPPING_PIPELINE_RESTORED_WITH_EXTERNAL_DEPENDENCIES`

The hermetic task worktree does not track FAST-LIO2, but the dependency was
restored for validation from reproducible external sources. See
`fast_lio_provenance.md` for remote URLs, commits, license notes, and
validation-only patches.

Evidence:

```text
rospack find fast_lio
/home/zzf/search_ws/SimEnv_worktrees/competition-rl-rtf-collapse/src/FAST_LIO

roslaunch --nodes simenv_fast_lio2_integration simenv_fast_lio2_mapping.launch
/laserMapping
/state_estimation_relay
/registered_scan_relay
/odometry_tf_bridge
/map_to_camera_init_bridge

roslaunch --args=laserMapping simenv_fast_lio2_integration simenv_fast_lio2_mapping.launch
/home/zzf/search_ws/SimEnv_worktrees/competition-rl-rtf-collapse/devel/lib/fast_lio/fastlio_mapping __name:=laserMapping
```

Implication:
- M4, M5, M7, and M8 are valid short-window mapping cases in this worktree.
- The first RTF collapse does not require FAST-LIO2: M2 already drops to mean
  RTF `0.165125` with competition sensor data enabled.
- FAST-LIO2 adds secondary cost: M4 mean RTF is `0.134005`.
- M3 produced `NO_CLOCK` when the PointCloud2 converter path was enabled
  without FAST-LIO2, so converter startup/clock behavior remains a risk to
  inspect separately from FAST-LIO2 compute cost.
