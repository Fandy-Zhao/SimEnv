# FALCO + DSV Real Runtime Validation Report

Date: 2026-07-23

Branch: `feat/0723-falco-dsv-navigation-integration`

Initial HEAD: `01776ecdb45cd00dfcaafcde0b89699f0444f4a4`

## Result

Overall verdict: `FALCO_DSV_RUNTIME_TIMING_BLOCKED`.

R2 passed with real Gazebo, LiDAR, FAST-LIO2, navigation relays, and TF. R3 did
not pass because FALCO did not produce a useful nonzero path from the real
registered cloud and manual waypoint under the observed low-RTF runtime. R4-R6
were intentionally not run.

## Evidence

Primary evidence is under
`experiments/runs/0723_falco_dsv_runtime/`.

- `interface_audit.md`: source and launch interface contract.
- `build_after_fast_lio_deps.log`: formal build after FAST-LIO2 dependency
  staging.
- `topic_types.txt`, `topic_hz.txt`, `topic_frames.txt`: real FAST-LIO2 topic
  ownership, type, frequency, and frame samples.
- `r2_message_validity.txt`: finite odometry and nonempty cloud checks.
- `tf_echo_base_frames.txt`, `tf_monitor.txt`: TF evidence.
- `tf_frames.gv`, `tf_frames.pdf`: audited TF diagram generated from saved
  echo/monitor evidence because native `view_frames` timed out.
- `falco_real_input_retry.log`: R3 FALCO output evidence.
- `rtf_metrics.csv`, `motion_metrics.csv`: timing and motion summary.

## Changes Made

- Added `runtime_real_data.launch` to relay global FAST-LIO2 topics into the
  `/navigation` namespace before starting FALCO, optional DSV, and the gated
  bridge.
- Added launch-time safety limit overrides to `navigation_bridge.launch`.
- Documented the real-data runtime launch in `src/navigation/README.md`.

## Scope Audit

- Collision changed: No.
- Physics changed: No.
- Robot dynamics changed: No.
- Trotting core changed: No.
- RL changed: No.
- FAST-LIO2 core changed: No.
- Vendor planner core changed: No.
- Building generation logic changed: No.

## Next Steps

1. Stabilize R3 timing enough for continuous `/navigation/state_estimation` and
   `/navigation/registered_scan` before re-testing FALCO path quality.
2. Inspect FALCO obstacle/path parameters against FAST-LIO2 `camera_init`
   registered clouds without disabling obstacle checking.
3. Re-run R3 until FALCO produces a nonzero, finite, reasonable path.
4. Only then run R4 low-speed Trotting and the required stop/timeout/state-gate
   tests.
