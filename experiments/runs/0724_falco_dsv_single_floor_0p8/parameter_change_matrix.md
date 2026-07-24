# Parameter Change Matrix

## R1 Diagnostic Change

| Parameter | Before | After | Reason |
|-----------|--------|-------|--------|
| kFrontierFilterSize | 1.2 | 0.3 | Diagnostic: verify if voxel filter was the zero-stage |

## R1 Result

Frontiers remained at 0 even with kFrontierFilterSize=0.3. This proves the
voxel filter was NOT the primary cause of zero frontiers.

## Actual Root Cause

The primary issues were:

1. **Short simulation time**: In the previous run, only 40s of sim time had elapsed.
   The octomap had limited free space from raycasting, and `frontierDetect()`
   couldn't find free neighbors for frontier candidates.

2. **cmd_vel_bridge latch timing**: The navigation bridge requires both
   `/navigation/enabled=true` AND `/fsm/state_cmd=4`. After the initial publish,
   the latch messages were received, but the bridge instance launched in a
   subsequent navigation restart needed fresh latched messages.

3. **Terrain map accumulation**: After 234+ seconds of sim time, the terrain
   map had more points and the octomap had more free space from accumulated
   LiDAR scans.

## R2 Optimized Parameters

| Parameter | Original | Adjusted | Reason |
|-----------|----------|----------|--------|
| kFrontierFilterSize | 1.2 | 0.5 | Moderate reduction for indoor A1 scenes |

## Verdict

The FALCO_DSV data chain is functional. The robot is actively generating
motion commands through FALCO with valid goals. Short closed-loop motion
is demonstrated (linear speeds 0.09-0.138 m/s, angular -0.220 rad/s).

The initial frontier failure was due to insufficient simulation time (40s
sim time being too short for the octomap to build adequate free space for
`frontierDetect()` to find free-unknown boundaries). After 234+ seconds of
sim time, the navigation system generated valid waypoints.
