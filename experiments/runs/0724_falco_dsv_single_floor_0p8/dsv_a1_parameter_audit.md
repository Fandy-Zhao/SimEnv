# DSV A1 Parameter Audit

Date: 2026-07-24

## A1 Robot Geometry

| Parameter | Current Value | A1 Recommended | Risk |
|-----------|--------------|----------------|------|
| robot length | 0.56 m | 0.56 m | OK |
| robot width | 0.43 m | 0.43 m | OK |
| collision_radius | 0.35 m | 0.30–0.35 m | OK |
| vehicle_height | 0.65 m | 0.60–0.70 m | OK |
| robot_self_filter_radius | 0.32 m | 0.30–0.35 m | OK |

## Frontier Extraction Pipeline (CRITICAL)

| Parameter | Current Value | Suggested Range | Risk |
|-----------|--------------|-----------------|------|
| **kFrontierFilterSize** | **1.2 m** | 0.3–0.6 m | **HIGH — 1.2m voxel collapses sparse frontiers to zero** |
| kFrontierResolution | 0.5 m | 0.3–0.6 m | MEDIUM |
| kEffectiveUnknownNumAroundFrontier | 2 | 1–3 | LOW |
| kSearchRadius | 8.0 m | 6–12 m | MEDIUM — 8m from stationary position may miss doorways |
| kSearchBoundingX | 22 m | OK for 20m building | LOW |
| kSearchBoundingY | 38 m | OK for 36m building | LOW |
| kSearchBoundingZ | 0.25 m | 0.20–0.30 m | MEDIUM |
| kFrontierNeighbourSearchRadius | 5.0 m | 3–8 m | LOW |

## Graph Connectivity

| Parameter | Current Value | Suggested Range | Risk |
|-----------|--------------|-----------------|------|
| kMinVertexDist | 0.6 m | 0.4–0.8 m | MEDIUM |
| kConnectVertexDistMax | 2.5 m | 1.5–3.0 m | MEDIUM |
| kMaxVertexDiffAlongZ | 0.25 m | 0.15–0.25 m | MEDIUM |
| kMaxVertexAngleAlongZ | 38.0° | 30–45° | LOW |
| kCollisionCheckDistace | 0.35 m | 0.30–0.35 m | MEDIUM |
| kLookAheadDist | 1.2 m | 0.8–1.5 m | LOW |
| kWaypointProjectionDistance | 0.6 m | 0.4–0.8 m | LOW |

## Occupancy Grid

| Parameter | Current Value | Suggested Range | Risk |
|-----------|--------------|-----------------|------|
| kMapWidth | 40 m | OK | LOW |
| kGridSize | 0.10 m | 0.05–0.15 m | MEDIUM — 10cm is fine |
| kObstacleHeightThre | 0.12 m | 0.10–0.15 m | LOW |
| kFlyingObstacleHeightThre | 1.10 m | 1.00–1.50 m | LOW |

## Single Floor Z Constraints

| Parameter | Current Value | Suggested Range | Risk |
|-----------|--------------|-----------------|------|
| max_goal_z_deviation | 0.20 m | 0.15–0.25 m | MEDIUM |
| max_frontier_z_deviation | 0.30 m | 0.25–0.40 m | MEDIUM |
| max_extension_along_z | 0.08 m | 0.05–0.10 m | LOW |
| search_bounding_z | 0.25 m | 0.20–0.30 m | MEDIUM |

## Octomap

| Parameter | Current Value | Suggested Range | Risk |
|-----------|--------------|-----------------|------|
| resolution | 0.20 m | 0.10–0.30 m | LOW |
| sensorMaxRange | 12.0 m | 10–15 m | LOW |
| probabilityHit | 0.65 | 0.60–0.70 | LOW |

## HIGHEST RISK: kFrontierFilterSize = 1.2m

This is the primary suspect for zero frontiers. The frontier extraction applies
a 1.2m voxel grid filter at THREE stages in the pipeline:

1. `getUnknowPointcloudInBoundingBox()` — first filter on raw frontier candidates
2. `localFrontierUpdate()` — second filter after visibility check
3. `gloabalFrontierUpdate()` — third filter on global frontier update

With a stationary robot and limited LiDAR scans (~40s sim time), the boundary
between free and unknown space is narrow. A 1.2m voxel filter collapses all
frontier points into very few or ZERO clusters.

Combined with `kFrontierNeighbourSearchRadius = 5m` in `globalFrontiersNeighbourCheck()`,
even the few surviving points may fail if they're too sparse.

## SECONDARY RISK: Cold-start initialization

`interface.skipInitialMotion: true` — robot does NOT perform any initial motion.
With no motion, the octomap has limited free-space rays, making frontier detection
harder. The `initializationTimeout: 5.0s` with `continueAfterInitializationTimeout: true`
allows the planner to start without proper initialization.

## TERTIARY RISK: Terrain map sparsity

840 terrain points in a 40x40m grid = very sparse. The octomap also starts with
limited data. Frontier detection relies on free/unknown adjacency in the octomap,
which requires sufficient free-space rays.
