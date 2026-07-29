# Recorder Frame and Route Metrics Fix — Validation Notes

## Runtime Preservation

- Existing `auto.sh`, ROS master, Gazebo, FAST-LIO2, DSV, exploration,
  localPlanner, and pathFollower processes were not stopped or restarted.
- No ROS parameters or topics were changed or published.
- No workspace build was run because the live runtime remains active and the
  change is Python/launch-only.

## Unit and Fixture Validation

- Shared route fixtures cover valid map motion, NaN/Inf, backward and duplicate
  timestamps, excessive speed, excessive step, and frame mismatch.
- Recorder fixtures cover map passthrough, camera_init-to-map transformation,
  missing-TF drops, and source-frame changes without mixed output frames.
- Synthetic normal route: 1.0 m, 2 accepted, 0 rejected.
- Synthetic divergent route: 1.0 m, 2 accepted, 1 speed rejection; all four
  source samples remain present.
- Recorder and renderer return identical metrics for the same fixture.

## Historical Artifact Validation

- Source run: `manual_full_20260729_001913`.
- Read-only snapshot: `/tmp/recorder_frame_route_test_d2MvnaJG`.
- Legacy frame: `camera_init`; target frame: `map`.
- Renderer warning: `TRAJECTORY_FRAME_MISMATCH`.
- Overlay allowed: false.
- Old renderer route: 1,848,936.807 m.
- New diagnostic route: 0.0 m; 0 accepted and 17,900 frame-mismatch rejects.
- Original trajectory SHA-256 remained
  `a4e9f8baa5d71dbdec54ca76fcf14d6269ff2f49cc202677d34e8992186a3bb7`.

## Residual Risk

- This change does not repair FAST-LIO2 divergence. A divergent map-frame pose
  remains visible in trajectory CSV and will produce rejected route segments.
- TF behavior is unit-tested with a deterministic buffer; it was not validated
  by restarting the currently running exploration instance.
