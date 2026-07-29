# Recorder Frame and Route Metrics Fix

## Goal

Record exploration trajectories in the `map` frame and make the recorder and
offline renderer use one shared, auditable route-segment policy.

## Scope

- Transform non-`map` trajectory poses through TF before recording.
- Drop and count samples whose transform is unavailable.
- Add shared route validation, evaluation, aggregation, and reject counters.
- Make the renderer refuse unsafe legacy non-`map` overlays.
- Add unit and offline fixture coverage.

## Non-Scope

- FAST-LIO2 divergence, sensor calibration, DSV, Graph Planner, FALCO, control,
  physics, and map generation.
- Modification of existing exploration artifacts or the live runtime.
- Merge to `master` or remote push.

## Acceptance Criteria

- Every recorded trajectory row has `frame_id=map`.
- Failed transforms never enter the trajectory buffer and increment a counter.
- Recorder and renderer return identical route metrics for the same samples.
- Invalid frame/time/speed/step segments are retained as samples but rejected
  from distance accumulation with explicit reason counters.
- Historical `camera_init` artifacts are diagnosed but not overlaid or edited.
- Python syntax, unit tests, offline fixtures, and `git diff --check` pass.

## Risks

- TF may be temporarily unavailable during startup; dropped samples must remain
  visible in metadata.
- Upstream map-frame localization may still diverge; filtering route segments
  must not be presented as proof that localization is healthy.
- Existing dirty generated/log/result artifacts must remain outside the commit.

## Impacted Modules

- `simenv_navigation_bridge`
- offline exploration renderer
- exploration recorder launch/configuration
