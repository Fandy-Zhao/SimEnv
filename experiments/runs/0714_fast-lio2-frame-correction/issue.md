# Issue: FAST-LIO2 LiDAR/IMU frame correction

## Task Goal

Diagnose and fix coordinate frame issues in FAST-LIO2's LiDAR, IMU, Odometry, and TF chain. The primary symptom is that `/Odometry` shows incorrect body axes in RViz (X axis pointing downward or not forward), while point cloud registration appears correct.

Root cause candidates: duplicate rotations, incorrect extrinsic parameters, TF ownership conflicts, or inconsistent `frame_id` assignments across the sensor→SLAM→navigation pipeline.

## Modification Scope

- `src/simenv_fast_lio2_integration/` — adapter scripts, launch files, YAML config
- Possible URDF/Xacro fixes for sensor mounting angles
- Possible static TF adjustments
- Governance documentation updates

## Explicit Non-Scope

- FAST_LIO core algorithm (filtering, matching, ikd-Tree, state estimation)
- Controller/FSM logic
- TARE/DSV-Planner/FALCO
- Danger source recognition
- Training modules
- `auto.sh` flow (unless a frame fix requires a minor adjustment)

## Acceptance Criteria

1. `base_link` satisfies: +X forward, +Y left, +Z up
2. `/Odometry` body axes match the physical robot chassis orientation
3. Point cloud mapping remains correct (no degradation from current quality)
4. No duplicate rotations exist in the sensor→SLAM pipeline
5. No duplicate TF publishers for the same frame
6. `./auto.sh` default flow is not broken
7. FAST-LIO2 disabled mode still works
8. Stationary test: no continuous drift, IMU gravity direction correct
9. Straight-line test: robot moves along `base_link` +X

## Risk Points

- Current correct point cloud may be the result of two compensating errors; fixing one without the other would break mapping
- FAST-LIO2 `extrinsic_R` direction convention must be verified from source code, not assumed
- URDF sensor poses may already encode the 45° tilt; adding it again in extrinsic would double-rotate
- TF tree may have ownership conflicts between `robot_state_publisher` and FAST-LIO2

## Expected Impacted Modules

- `simenv_fast_lio2_integration` (adapter, config, launch)
- URDF/Xacro sensor model definitions
- FAST-LIO2 YAML configuration
- Navigation/exploration frame parameters (if they reference `body` or `base_link`)
