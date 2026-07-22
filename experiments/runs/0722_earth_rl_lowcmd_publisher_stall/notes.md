# LowCmd Publisher Stall Investigation

Date: 2026-07-22
Branch: fix/0722-earth-rl-lowcmd-publisher-stall

## Objective

Root-cause the LowCmd publisher stall where RL-state command generation and delivery rates fall below expected thresholds, building on the transport-layer fix from the 0722_earth_rl_lowcmd_500hz investigation.

## Allowed Scope

- Read-only analysis of TimingDiagnostics CSV (FSM/LOWCMD events) and LowCmd apply CSV (CMD_RECEIVE/CMD_APPLY events).
- Compute per-window metrics across FixedStand (last 5 s) and RL-zero (first 8.5 s after FSM switch).
- No production code changes, no control/policy/launch edits, no existing experiment file overwrites.

## Validation

```bash
/usr/bin/python3 -m py_compile experiments/runs/0722_earth_rl_lowcmd_publisher_stall/analyze_combo_timing.py
```

## Combo Evidence

Input data came from temporary integration branch `test/0722-earth-rl-candidate-integration`
at `94188231`.

- FixedStand last 5 s before RL switch: LOWCMD publish median `333.33 Hz`,
  CMD_RECEIVE median `333.33 Hz`, CMD_APPLY-new median `333.33 Hz`,
  max apply gap `10 ms`, apply sequence jumps `223`.
- RL zero first 8.5 s after switch: LOWCMD publish median `500.00 Hz`,
  CMD_RECEIVE median `333.33 Hz`, CMD_APPLY-new median `333.33 Hz`,
  max apply gap `43 ms`, apply sequence jumps `433`.
- RL zero stability itself passed in the combo run: post duration `9.124 s`,
  minimum base height `0.3119586857 m`, maximum tilt `5.148136463 deg`,
  no fall.

Conclusion: the combo still fails the LowCmd 475-525 Hz receive/apply gate.
The remaining issue is not solved by commit `31109221` in this runtime setup.
