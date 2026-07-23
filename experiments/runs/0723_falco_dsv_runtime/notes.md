# Notes: FALCO + DSV Real Runtime Validation

## Baseline

- Worktree: `/home/zzf/search_ws/SimEnv_worktrees/falco-dsv-navigation`
- Branch: `feat/0723-falco-dsv-navigation-integration`
- Initial HEAD: `01776ecdb45cd00dfcaafcde0b89699f0444f4a4`
- Root workspace `/home/zzf/search_ws/SimEnv` is intentionally preserved.

## Initial Verdicts

- Governance: `GOVERNANCE_IN_PROGRESS`
- R0: `R0_INTERFACE_AUDIT_PASS`
- R1: `R1_BUILD_PASS`
- R2: `R2_FAST_LIO_REAL_DATA_PASS`, `R2_TF_PASS`
- Fix needed before R3: add a real-data navigation launch that relays global
  FAST-LIO2 `/state_estimation` and `/registered_scan` into the `/navigation`
  namespace expected by validation and DSV config.
- R3: `R3_FALCO_REAL_INPUT_FAIL`, `R3_MOTION_GATE_PASS`
- R4: `NOT_RUN`; blocked by R3.
- R5: `NOT_RUN`; blocked by R3/R4 prerequisites.
- R6: `R6_DSV_FALCO_TROTTING_NOT_RUN`; blocked by R3/R4/R5 prerequisites.

## Final Verdict

`FALCO_DSV_RUNTIME_TIMING_BLOCKED`

R2 real data and TF passed. R3 connected FALCO to real FAST-LIO2 inputs through
the new runtime relays, but FALCO produced only a zero one-pose path and zero
raw velocity for the tested waypoint. R4-R6 were not run.
