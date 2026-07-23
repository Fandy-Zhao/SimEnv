# Runtime Before Fix

## Attempted Startup

Command:

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash 2>/dev/null || true
GUI=False ./auto.sh
```

Result:

```text
exit:1
ERROR: junior_ctrl is not built: /home/zzf/search_ws/SimEnv_worktrees/fast-lio2-tf-fix/devel/lib/unitree_guide/junior_ctrl
```

The isolated task worktree was created from clean `master` and has no `devel/setup.bash`.
The root worktree has a `devel/` directory, but it was not reused because this task explicitly
requires root state preservation and worktree isolation.

## ROS Graph Checks

With no simulation running:

```text
rosparam get /use_sim_time -> ERROR: Unable to communicate with master!
rostopic list -> ERROR: Unable to communicate with master!
```

## Evidence Status

Runtime reproduction is blocked in this clean worktree until the workspace is built and external
FAST_LIO dependencies are available. Static evidence is recorded separately; runtime claims are
not treated as proven.
