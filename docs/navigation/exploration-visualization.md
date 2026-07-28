# Single-Floor Exploration Visualization

## Overview

After a single-floor exploration run completes and the
`exploration_result_recorder` saves all artifacts, you can generate the
final **offline** overview image **without** ROS, Gazebo, or a roscore.

The renderer produces one composite plot with four core layers:

| Layer | Source | Description |
|-------|--------|-------------|
| **Ground Truth Layout** | `layout_metadata.json` | Building walls, rooms, lobby, corridor, stairs, elevator, furniture — in the Gazebo world frame, transformed to the map frame |
| **Explored Area** | `map/occupancy_grid.csv` | Known (free + occupied) cells overlaid as a semi-transparent mask; unknown areas shown in gray |
| **Robot Trajectory** | `route/trajectory.csv` | Full `/Odometry` path with START (green circle) and END (red square) markers |
| **DSV Goals** | `goals/goals_unique.csv` | Reached goals (green circles) and unreached goals (red X marks), each numbered |

**Important**: The ground truth layout is used **only** for offline visualization
and post-hoc evaluation. It **never** enters the online exploration, planning, or
control pipeline (DSV-Planner, FALCO, FAST-LIO2, cmd_vel_bridge, Unitree
controller).

## Usage

```bash
# From the repository root, with a completed run directory:
/usr/bin/python3 tools/render_single_floor_exploration.py \
    --run-dir <RUN_DIR>

# If the layout metadata is not auto-detected, specify it explicitly:
/usr/bin/python3 tools/render_single_floor_exploration.py \
    --run-dir <RUN_DIR> \
    --layout-metadata generated_building/layout_metadata.json

# Override robot spawn parameters (if different from the default):
/usr/bin/python3 tools/render_single_floor_exploration.py \
    --run-dir <RUN_DIR> \
    --spawn-x 0.0 --spawn-y 2.3 --spawn-yaw-deg 90

# Also export a standalone truth layout JSON for downstream analysis:
/usr/bin/python3 tools/render_single_floor_exploration.py \
    --run-dir <RUN_DIR> --export-layout-json
```

## Requirements

- Python 3 with `matplotlib`, `numpy`, `PyYAML`
- No ROS, no Gazebo, no roscore needed
- No build step needed

## Outputs

| File | Description |
|------|-------------|
| `plots/final_exploration_overview.png` | 4-layer composite visualization (200 dpi) |
| `plots/final_exploration_overview_meta.yaml` | Structured metadata: run info, layout source, coordinate transform, trajectory stats, goal stats, exploration coverage |
| `layout/layout_truth.json` | (optional) Standalone truth layout export |

## Coordinate Transform

The ground truth layout uses the Gazebo **world** frame. Exploration data
(odometry, occupancy grid, goals) uses the **map** (odom) frame from FAST-LIO2.

The transform `T_truth_to_map` is derived from the robot spawn pose recorded in
`scene_manifest.json` (field `robot_start`):

```
p_map = R(-spawn_yaw) · (p_world − spawn_position)
```

Default values: `spawn_x=0.0`, `spawn_y=2.3`, `spawn_yaw=π/2 (90°)`.

Override with `--spawn-x`, `--spawn-y`, `--spawn-yaw-deg` if the scene
configuration differs.

## Coverage Ratio

The `coverage_ratio` in the metadata YAML is computed as:

```
coverage_ratio = known_area_m² / explorable_area_m²
```

where `explorable_area_m²` is the sum of lobby, corridor, and room areas from
the ground truth layout.

The ratio is set to `null` when a reliable denominator is not available.

## Limitations

- This is an **offline** tool — it does not run during exploration.
- The ground truth layout reflects the **generated building** at scene creation
  time. If the Gazebo world is manually modified after generation, the layout
  may not match.
- Only the **first floor** (`floor_index=0`) is rendered.
- Doors are shown as small markers; their open/closed state during exploration
  is not reflected.
