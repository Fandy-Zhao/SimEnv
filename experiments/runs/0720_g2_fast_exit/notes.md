# G2 Fast Exit Notes

## Commands

```bash
PROBE_MODE=p0_fixedstand PROBE_ID=p0_fixedstand_run_01 COMMAND_VX=0.0 ROS_PORT=13120 \
  bash experiments/runs/0718_g2_trotting_motion_baseline/run_g2_fast_exit_probe.sh

PROBE_MODE=p0_fixedstand PROBE_ID=p0_fixedstand_run_02 COMMAND_VX=0.0 ROS_PORT=13121 \
  bash experiments/runs/0718_g2_trotting_motion_baseline/run_g2_fast_exit_probe.sh
```

## Results

- `p0_fixedstand_run_01`: tool-failure evidence only. The first probe crashed
  while converting an empty timing CSV field.
- `p0_fixedstand_run_02`: valid P0 evidence. Result `FAIL` with
  `CONTACT_NOT_READY`, `FIXEDSTAND_NOT_ENTERED`, and `FALL_DETECTED`.

## Key P0 Metrics

- final FSM state: `1` (`PASSIVE`)
- final wave status: `1`
- foot samples: `0`
- min model height: `0.05698662028992169 m`
- max abs tilt: `0.09249959008750497 rad`
- truth samples: `3116`
- joint samples: `22070`
- timing rows: `32971`

## Verdict

`G2_FAST_EXIT_SHARED_BASE_FAILURE`

`RL_SHADOW_ONLY_AUTHORIZED`; `RL_ACTIVE_NOT_AUTHORIZED`.

## Commit

This run summary is recorded by commit
`test(g2fast): record shared-base fast exit`.
