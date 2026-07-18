# G2-B Root Cause Report

## Status

Preliminary classification only. G2-R is not authorized from this evidence
because G2-B has zero valid trials.

## Trigger Condition

The baseline did not reach speed-tracking acceptance checks. It failed earlier:
all four smoke trials were invalid before WAVE_ALL/gait execution.

## Observed Failure

| Speed | Observed facts | Candidate |
| ----: | -------------- | --------- |
| 0.00 | FixedStand and Trotting entered; WAVE_ALL never reached; gait cycle stayed 0; fall detected | Pre-gait Trotting validity failure |
| 0.10 | Resolved vx reached 0.1; WAVE_ALL never reached; gait cycle stayed 0; fall detected | Pre-gait Trotting validity failure |
| 0.30 | Resolved vx reached 0.3; WAVE_ALL never reached; gait cycle stayed 0; fall detected | Pre-gait Trotting validity failure |
| 0.50 | Resolved vx reached 0.5; WAVE_ALL never reached; gait cycle stayed 0; fall detected; controller log captured non-finite Trotting output with `q=0` | Pre-gait Trotting output/joint-target failure |

## ADR-004 Classification

- R1 Command path: unlikely as primary for nonzero trials. `resolved_vx`
  reached each commanded value.
- R2 Estimator: not ruled out, but no valid gait window exists to compare
  estimator speed tracking.
- R3 Gait/foot placement: strongest current category. WAVE_ALL never starts,
  gait cycle never advances, and the controller cancels wave after non-finite
  Trotting output in the captured `vx=0.50` pane.
- R4 LowCmd/joint path: secondary candidate because the non-finite output log
  specifically reports `q=0` while other output groups were finite.
- R5 Model/physics: not promoted from this evidence alone.

## Primary Root Cause

Not yet confirmed.

Current strongest hypothesis: Trotting pre-wave output calculation produces a
non-finite joint target (`q=0` in the controller log), which cancels wave and
prevents gait advancement; the robot then remains in an invalid fall/posture
state.

## Secondary Hypotheses

- Estimator/body posture at Trotting entry is already outside the assumptions
  needed by IK or joint target generation.
- Joint feedback normalization or joint target source differs between the G1
  validated run and this isolated G2 runtime.

## Recommended Next Investigation

Create a separate diagnostic branch before any fix. Capture Trotting pre-wave
inputs and outputs at the first non-finite frame: body pose, foot goals,
measured joint q, IK result, qGoal, qdGoal, tau, and finite flags. Do not
change Kp/Kd, gait period, friction, robot model, or physics parameters.
