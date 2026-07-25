# Issue: FALCO + DSV Single-Floor Exploration 0.8 m/s

## Goal

Prepare the SimEnv single-floor indoor autonomous exploration chain:

FAST-LIO2 -> DSV-Planner -> FALCO -> safety bridge -> Trotting `/cmd_vel`.

The immediate target is a continuous, boundary-constrained, single-floor exploration runtime with FALCO producing a raw straight-line target near 0.8 m/s and reducing speed during turns.

## Scope

- Tune FALCO A1 speed semantics and safety bridge limits.
- Repair DSV initialization and movement/stuck detection for A1.
- Add lightweight `/navigation/terrain_map` and `/navigation/boundary` adapters using only public runtime topics/parameters.
- Add a unified single-floor exploration launch entry.
- Record build and interface evidence under this run directory.

## Non-Scope

- No Trotting, RL controller, Unitree controller, FAST-LIO2 core, physics, collision geometry, sensor extrinsics, scene generator, or ground-truth changes.
- No Gazebo direct startup for formal runtime validation; use `auto.sh`.
- No fixed waypoint loop or ground-truth navigation input.

## Acceptance Gates

- `BASELINE_PASS`
- `SCOPE_AUDIT_PASS`
- `BUILD_PASS`
- `FALCO_SPEED_PROFILE_PASS` when raw FALCO command evidence is available.
- `DSV_INIT_PASS`
- `DSV_MOVEMENT_DETECTION_PASS`
- `TERRAIN_MAP_PASS`
- `SINGLE_FLOOR_CONSTRAINT_PASS`
- `BOUNDARY_PASS`
- `NO_MOTION_GATE_PASS`
- Short-loop and full-exploration gates only if runtime checks are actually executed.

## Risks

- Runtime Gazebo/ROS display availability may block S2-S5 validation.
- FALCO/DSV parameters are initial candidates and require live cloud/frontier metrics before being called final.
- Vendored DSV uses global ROS parameter paths; namespace behavior must be verified by launch/build checks.

## Expected Impacted Modules

- `src/navigation/simenv_navigation_bridge`
- `src/navigation/simenv_navigation_bringup`
- `src/navigation/vendor/falco/local_planner`
- `src/navigation/vendor/dsv/dsvplanner`
- Governance docs and this experiment run directory
