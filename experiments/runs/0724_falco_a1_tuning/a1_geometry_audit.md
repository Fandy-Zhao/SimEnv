# A1 Robot Geometry Audit for FALCO Footprint Tuning

**Source:** `multi_floor_gazeboSim.launch` loads `robot.xacro` → `const.xacro` + `leg.xacro` + `gazebo.xacro`.

## Base Frame

- Root link: **`base`** (no `base_link` exists). Tiny 1 mm visual box, no collision.
- `floating_base` (fixed): `base` → `trunk` at `(0,0,0)`.
- `imu_link` attaches to `trunk` at `(0,0,0)`.
- `laser_livox` and `real_sense` attach to `base`.

## Trunk (collision geometry from `const.xacro`)

| Parameter | Value (m) |
|-----------|-----------|
| trunk_length (X) | 0.267 |
| trunk_width (Y)  | 0.194 |
| trunk_height (Z) | 0.114 |

Collision box centered at `(0,0,0)` in trunk frame → XY half-extents: ±0.1335 (X), ±0.097 (Y).

## Leg Chain (standing pose: hip=0, thigh=0.9 rad, calf=-1.8 rad)

| Link | Collision shape | Dims (m) | Origin rel. to parent |
|------|----------------|----------|----------------------|
| Hip | cylinder (axis X) | r=0.046, len=0.04 | at hip_joint |
| Thigh shoulder | cylinder (axis X) | r=0.041, len=0.032 | y=±(0.032/2+0.065)=±0.081 from hip |
| Thigh | box | 0.2×0.0245×0.034 | rpy=(0,π/2,0), z=-0.1 |
| Calf | box | 0.2×0.016×0.016 | rpy=(0,π/2,0), z=-0.1 |
| Foot | sphere | r=0.02 | z=-0.2 from calf |

Key offsets: `leg_offset_x=0.1805`, `leg_offset_y=0.047`, `thigh_offset=0.0838`, `thigh_length=calf_length=0.2`.

**Foot centers in trunk frame (standing, computed):**
- FR: `( 0.1805, -0.1308, -0.249)` — FL: `( 0.1805,  0.1308, -0.249)`
- RR: `(-0.1805, -0.1308, -0.249)` — RL: `(-0.1805,  0.1308, -0.249)`

Foot sphere r=0.02 → ground XY extent: X=±0.2005, Y=±0.1508.

## Sensors

| Sensor | Joint origin (base frame) | Notes |
|--------|--------------------------|-------|
| livox_mid360 | `(0.2, 0, 0.08)`, rpy=(0,0.785,0) | mesh collision, ~0.065 m dia → X forward ~0.233 |
| real_sense (D415) | `(0.28, 0, 0.043)` | small body, adds ~0.03–0.05 forward |

## Conservative 2D Footprint (standing A1, ground-plane projection)

| Bound | Origin | Extent (m) |
|-------|--------|------------|
| Rear (X-) | rear foot rear edge | -0.2005 |
| Front (X+) | lidar front edge | +0.233 |
| Left/Right (Y±) | foot outer edge | ±0.1508 |

**Occupied XY extent:** length ~0.434 m, width ~0.302 m *(from collision geometry)*.
If including RealSense overhang: length up to ~0.53 m *(inferred from mount pos, mesh not parsed)*.

## Sweep Candidates for `vehicleLength` / `vehicleWidth`

Per-side margins 0.03, 0.06, 0.10 m; base occupied = 0.44×0.31 m (lidar-inclusive, no RealSense overhang).

| Margin/side | vehicleLength (m) | vehicleWidth (m) |
|-------------|-------------------|------------------|
| 0.03 | 0.50 | 0.37 |
| 0.06 | 0.56 | 0.43 |
| 0.10 | 0.64 | 0.51 |

All values ≥ occupied footprint. Recommend starting with 0.56×0.43 (0.06/side) and tuning upward if collisions persist.
