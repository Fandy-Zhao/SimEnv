# Issue: Flatten `earth.world` for Motion Validation

## Goal

Fix the `WORLD_MODE=earth` motion benchmark world so the A1 robot spawns on a single flat ground plane instead of on raised box platforms that can intersect leg or foot collisions and cause FixedStand rollover.

## Scope

- Modify `src/unitree_guide/unitree_ros/unitree_gazebo/worlds/earth.world`.
- Remove the raised platform models, including their visual and collision geometry.
- Preserve the existing `WORLD_MODE=competition` default and launch/spawn behavior.
- Record static and runtime validation evidence for the flat earth world.

## Non-Scope

- Do not modify FSM logic, RL policy, observations, actions, gait, IK, estimator, controller gains, model weights, URDF/xacro collision geometry, or Gazebo physics parameters.
- Do not change robot spawn height as a workaround.
- Do not change generated competition scene behavior.

## Acceptance Criteria

- `earth.world` parses as XML/SDF.
- `earth.world` contains `sun` and a single effective ground plane include.
- `earth.world` no longer contains `platform_1`, `platform_2`, or platform collisions/visuals.
- `WORLD_MODE` default remains `competition`.
- `WORLD_MODE=earth` still resolves to the repository `earth.world`.
- Robot spawn for earth mode remains recorded as `x=0.0 y=0.0 z=0.6 roll=0 pitch=0 yaw=0.0`.
- Static checks pass: XML parse, world content check, `git diff --check`, and `bash -n auto.sh`.
- Runtime smoke is attempted or explicitly recorded as blocked with reason.

## Risks

- Gazebo runtime may be unavailable in headless or unbuilt environments.
- Removing platforms changes only the benchmark world; any previous tests that depended on platforms must use a separate world.
- `ground_plane` is an included Gazebo model, so the static check validates the include rather than expanding model geometry.

## Expected Impacted Modules

- `unitree_guide`: benchmark-only Gazebo world file.
- Governance docs and experiment records for task evidence.
