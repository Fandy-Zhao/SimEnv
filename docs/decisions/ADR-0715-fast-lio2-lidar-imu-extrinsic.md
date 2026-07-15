# ADR-0715: FAST-LIO2 uses the direct LiDAR-to-IMU point transform

## Status

Accepted (2026-07-15). Supersedes the `extrinsic_R` direction stated in ADR-0714.

## Context

`laser_livox` is mounted relative to the body-aligned `imu_link` at
`xyz=(0.2, 0, 0.08)`, `rpy=(0, 0.785, 0)`.  The Livox plugin publishes each
`/scan` point in the local `laser_livox` frame.  FAST-LIO2 then evaluates:

```
p_imu = extrinsic_R * p_lidar + extrinsic_T
```

The former configuration supplied the inverse of this transform
(`Ry(-45°)`, `[-0.085, 0, -0.198]`).  It therefore rotated LiDAR points an
additional 90° relative to the body when FAST-LIO2 formed the map.

## Decision

Use the direct `imu_link -> laser_livox` TF values in FAST-LIO2:

```yaml
extrinsic_T: [0.2, 0.0, 0.08]
extrinsic_R: [0.7071, 0, 0.7071,
              0,      1, 0,
              -0.7071, 0, 0.7071]
```

The bridge remains a direct `map -> imu_link` copy to `map -> camera_init`.
It owns world-frame connection only and applies no sensor rotation.

## Validation

- `tf_echo imu_link laser_livox` reports `Ry(+44.977°)` and `[0.2, 0, 0.08]`.
- FAST-LIO2 source applies the configured values as `p_imu = R * p_lidar + T`.
- `check_fast_lio2_extrinsics.py` compares the YAML against `robot.xacro`.

## Consequences

- Registered clouds and the `body` odometry axes use the body convention:
  +X forward, +Y left, +Z up.
- Existing running FAST-LIO2 nodes retain old startup parameters until they
  are restarted.
