# Issue: FALCO + DSV-Planner navigation integration

## Goal

Integrate the minimum required FALCO and DSV-Planner ROS Noetic source packages into SimEnv, add SimEnv-specific navigation bridge/bringup packages, compile through `tools/build_with_venv.sh`, and validate ROS interfaces in staged checks.

## Scope

- Audit upstream source repositories under `/home/zzf/search_ws`.
- Import only necessary ROS packages under `src/navigation/vendor/`.
- Add `simenv_navigation_bridge` and `simenv_navigation_bringup`.
- Adapt topics with launch remaps and bridge nodes instead of hard-coding SimEnv names in vendor code.
- Update navigation source documentation and project status documentation.

## Non-Scope

- Gazebo collision geometry, building generation, physics/RTF optimization.
- Unitree controller core, RL policy, FAST-LIO2 algorithm core.
- Upstream simulator, robot model, joystick, LOAM, or Velodyne simulator packages.

## Acceptance Criteria

- Third-party source provenance and copied/excluded package lists are documented.
- `local_planner`, `dsvplanner`, `simenv_navigation_bridge`, and `simenv_navigation_bringup` are discoverable by ROS package tooling.
- Formal build is attempted only through `tools/build_with_venv.sh` and logged.
- Static launch/interface checks and smoke scripts are added or run where dependencies allow.

## Risks

- DSV-Planner may require map interfaces not currently produced by FAST-LIO2.
- Vendor source may require system ROS/C++ dependencies not installed locally.
- Runtime smoke may be limited by Gazebo/display/RTF constraints.
