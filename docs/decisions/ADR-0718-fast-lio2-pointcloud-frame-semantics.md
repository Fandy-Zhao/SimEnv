# ADR-0718: Preserve PointCloud frame semantics in the FAST-LIO2 adapter

## Decision

The default SimEnv PointCloud-to-PointCloud2 adapter performs no geometric
transformation. It preserves incoming point coordinates and header. Optional
rotations remain supported only with an explicit, distinct output frame name.

## Context

The Livox plugin emits local `+X` points in `laser_livox`; the previous adapter
rotated them by default but retained that frame name. FAST-LIO2 correctly uses
the physical LiDAR-to-IMU transform, so it cannot safely distinguish this
mislabelled input from valid LiDAR data.

## Consequences

- FAST-LIO2 extrinsics, A1 mounting, and world bridge remain unchanged.
- Configurations that deliberately rotate adapter points must now provide a TF
  for `rotated_frame_id`; implicit frame forgery is rejected at startup.
- A fresh runtime mapping validation remains required after the blocked
  isolated sensor-path repair.
