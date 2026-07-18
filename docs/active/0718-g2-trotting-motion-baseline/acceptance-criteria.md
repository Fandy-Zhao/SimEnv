# G2-B Acceptance Criteria

## AC-G2B-01: Speed Monotonicity
The valid-trial medians must satisfy:

```text
median_vx(0.10) < median_vx(0.30) < median_vx(0.50)
```

The `vx=0.00` trials are used for drift, attitude, and derived stability thresholds, not the monotonicity chain.

## AC-G2B-02: Tracking Accuracy
At `vx=0.10`, absolute tracking error must be `<= 0.08 m/s`, and steady speed must not stay above `0.30 m/s`. At `vx=0.30` and `vx=0.50`, tracking ratio target is `[0.70, 1.30]`.

## AC-G2B-03: Lateral Drift
Recommended threshold: lateral drift ratio `<= 0.05`, where the ratio is `abs(lateral_displacement) / max(abs(forward_displacement), epsilon)`.

## AC-G2B-04: Stop Behavior
For every valid nonzero-speed trial, speed must reach `<= 0.05 m/s` within 1.0 s after the zero command, and the stopping-window tail mean speed must be `<= 0.05 m/s`. If the threshold is not reached within the 2.0 s stop window, record `NOT_STOPPED_WITHIN_WINDOW`.

## AC-G2B-05: No Catastrophic Failure
No falls, no sustained joint/torque saturation, no passive entry across all trials.

## AC-G2B-06: Trial Validity
At least 3 valid epochs per speed are required for a speed verdict. A speed with fewer valid trials is `INCONCLUSIVE`; invalid trials remain evidence and must include explicit reason codes.

## Non-goals
- RL policy or controller modification
- Physics/contact parameter changes (unless root-cause evidence demands)
- Navigation, sensor, or multi-floor locomotion evaluation
