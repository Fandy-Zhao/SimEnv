# Issue: Event-driven startup and RTF payload cleanup

## Goal

Replace fixed-delay and fire-and-forget startup behavior in `auto.sh` with bounded ROS/Gazebo event and state checks, then remove only the confirmed optional runtime load from the legacy PointCloud/Livox converter and LiDAR visualization.

## Scope

- Audit the complete competition startup dependency chain.
- Add stage logs, wall-time timeouts, functional readiness checks, confirmed supervisor state transitions, and idempotent safe cleanup.
- Default `ENABLE_POINTCLOUD_CONVERTER` off while preserving the explicit compatibility switch.
- Default A1 Livox ray visualization off without changing scan semantics.
- Build and validate through the project entrypoints, collect runtime and RTF evidence, update governance documents, commit, and conditionally fast-forward local `master`.

## Non-scope

- Any RealSense/depth-camera code, parameters, topics, conditions, or behavior.
- Collision geometry, physics profiles, robot dynamics or spawn position.
- FAST-LIO2, DSV, FALCO, RL/trotting controller core algorithms.
- Building generation, hazards, judging interfaces, or user runtime artifacts.
- Remote push.

## Acceptance Criteria

- Governance and static audit evidence are complete.
- Critical startup phases use bounded functional readiness checks, not arbitrary fixed sleeps.
- State changes use supervisor request topics and output confirmation.
- Legacy converter is off by default and explicitly re-enableable.
- LiDAR visualization is off without changing `/scan` behavior.
- Formal build, runtime cases A-D, controlled failure cleanup, depth-camera audit, and RTF comparison pass.
- FAST-LIO2 + DSV + FALCO exploration loop is demonstrated.
- No merge occurs unless every core acceptance gate passes.

## Risks

- Low RTF may require generous wall-time bounds while retaining finite failure behavior.
- ROS topic presence alone can mask stale or invalid data; checks must validate fresh content.
- Root workspace contains user-owned generated data and must not be synchronized destructively.
- Runtime availability (display, hardware resources, stale ROS processes) may prevent a full acceptance pass and therefore block merge.

## Expected Modules

- `auto.sh`
- Unitree A1 Gazebo description/launch (LiDAR-only configuration boundary)
- Startup/runtime governance documents and task evidence
