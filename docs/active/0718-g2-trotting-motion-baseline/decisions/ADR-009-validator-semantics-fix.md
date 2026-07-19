# ADR-009: Fall Validator Pose Semantics

**Status:** Accepted for Gate V evidence
**Date:** 2026-07-19

## Context

G2-B marked all four smoke trials invalid with `FALL_DETECTED`,
`WAVE_ALL_NOT_REACHED`, and `GAIT_NOT_ADVANCING`. The baseline report also
showed roll peaks near 180 degrees and minimum model height around 0.079 m.
Before any locomotion recovery, Gate V must determine whether `FALL_DETECTED`
is a validator frame/pose false positive.

## Decision

The existing G2-B fall predicate is not a roll-based predicate. It is a
height-only predicate over the Gazebo model pose recorded from
`/gazebo/model_states`:

```text
min(model_pose.z over the trial) < 0.12 m
```

Gate V will not remove `FALL_DETECTED` from existing trials. Offline
reclassification with an explicit tilt+height semantic validator still marks
all four trials as fallen. Existing D0 FixedStand pose probes show normal
model/link/base_w height near 0.326 m with roll/pitch near zero, so the normal
standing pose is not inherently classified as fall by the model-state height
predicate.

## Rationale

- `/gazebo/model_states` orientation is stored in ROS `(x,y,z,w)` order and is
  converted with a standard normalized ZYX decomposition.
- The old runtime predicate does not use roll/pitch at all, so `roll≈180°` is
  diagnostic evidence from the fallen data, not the reason the validator failed.
- Four old trials cross both explicit semantic criteria: low model height
  (`~0.079 m`) and large body tilt (`>170 deg`).
- D0 FixedStand probes record model/link/base_w height around 0.326 m and
  near-zero roll/pitch; these do not meet the old model-state fall predicate.

## Consequences

- Gate V verdict is `G2_VALIDATOR_NO_DEFECT` for the specific suspected
  frame/pose false positive.
- No locomotion, WaveGenerator, scheduler, spawn, URDF/SDF, physics, or
  threshold behavior is changed by this Gate.
- Gate P remains necessary because all trials still fail before WAVE_ALL/gait
  advancement and `FALL_DETECTED` appears to be a symptom or concurrent failure,
  not a reason to authorize G2-R.
- Future validators should report both height and tilt evidence explicitly
  instead of relying on a single scalar `z` reason.
