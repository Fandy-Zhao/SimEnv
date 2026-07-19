# G2-D1 Validator Semantics Report

## Previous validator

The G2-B runtime capture writes Gazebo model pose from `/gazebo/model_states`
and marks a trial fallen when any recorded model pose satisfies:

```text
z < 0.12 m
```

The previous validator does **not** directly use roll, pitch, angular velocity,
base/trunk contact, or foot contact to set `FALL_DETECTED`.

## Pose source

Static code evidence:

- `g2_capture_trial.py` subscribes to `/gazebo/model_states`.
- It selects `message.name.index("a1_gazebo")`.
- It records `pose.position.{x,y,z}` and `pose.orientation.{x,y,z,w}`.
- It writes those samples to each trial `ground_truth.csv`.

Runtime D0 pose-probe evidence from the existing diagnostic branch records
`model_states`, `link_states`, `ground_truth/base_w`, `ground_truth/base_trunk`,
and `trunk_imu` summaries. Normal FixedStand samples show model/link/base_w
height near `0.326 m`, roll/pitch near zero, and quaternion norm near one.

## Frame semantics

- Raw quaternion order is ROS `(x,y,z,w)`.
- Gazebo pose orientation is the model/canonical-link pose in the world frame.
- The Euler conversion is standard ZYX Tait-Bryan `(roll,pitch,yaw)` and uses
  normalized quaternions in the Gate V offline checker.
- `roll≈180°` in the old trials is diagnostic evidence that the model became
  inverted or nearly inverted; it is not the runtime predicate that created
  `FALL_DETECTED`.

See:

- `experiments/runs/0718_g2_trotting_motion_baseline/diagnostics/validator_semantics/pose_validator_static_audit.md`

## Confirmed defect

`FALL_VALIDATOR_SEMANTIC_DEFECT_CONFIRMED` is **not** supported by the current
evidence.

The old validator is incomplete because it is height-only, but the suspected
false-positive mechanism is not reproduced:

- FixedStand D0 model-state pose does not meet the `z < 0.12 m` predicate.
- Existing G2-B trials meet both old low-height evidence and explicit tilt
  evidence when reprocessed offline.

## Physical posture evidence

Existing D0 FixedStand pose probes:

- `model_states.z_min ≈ 0.326 m`
- `link_states.z_min ≈ 0.326 m`
- `ground_truth/base_w.z_min ≈ 0.326 m`
- roll/pitch magnitudes are below one degree
- quaternion norms are approximately one

Existing G2-B baseline trials:

| Trial | First legacy height fall | First semantic tilt/height fall | Min model z | Max tilt |
| ----- | -----------------------: | -------------------------------: | ----------: | -------: |
| `vx_000_run_01` | `1.672 s` | `1.144 s` | `0.078638 m` | `170.857 deg` |
| `vx_010_run_01` | `2.262 s` | `1.584 s` | `0.078657 m` | `174.194 deg` |
| `vx_030_run_01` | `1.592 s` | `1.158 s` | `0.078576 m` | `171.100 deg` |
| `vx_050_run_01` | `2.788 s` | `2.470 s` | `0.078760 m` | `173.694 deg` |

The old baseline data does not include per-sample trunk/base collision contact,
so trunk/base ground contact is not proven from old CSVs alone.

## Minimal fix

No production fall-validator fix is applied in Gate V because the specific
suspected frame/pose false positive was not confirmed.

Gate V adds an offline semantic checker that:

- normalizes quaternions before Euler/tilt computation;
- computes body +Z tilt relative to world +Z;
- keeps the legacy height evidence;
- exposes first fall time for both legacy height and explicit semantic
  tilt/height criteria.

This is analysis tooling, not a controller or runtime behavior change.

## Unit tests

Added ROS-free tests for:

- identity quaternion;
- normal yaw 180 degrees;
- 180-degree roll;
- inverse quaternion sign;
- quaternion normalization;
- normal FixedStand;
- side fall by tilt;
- low body/model height;
- first semantic-vs-height fall time.

## Offline reclassification

Offline reclassification preserves all invalid reasons:

| Trial | Old fall result | New fall result | Remaining invalid reasons | Reclassified status |
| ----- | --------------- | --------------- | ------------------------- | ------------------- |
| `vx_000_run_01` | true | true | `FALL_DETECTED;GAIT_NOT_ADVANCING;WAVE_ALL_NOT_REACHED` | INVALID |
| `vx_010_run_01` | true | true | `FALL_DETECTED;GAIT_NOT_ADVANCING;WAVE_ALL_NOT_REACHED` | INVALID |
| `vx_030_run_01` | true | true | `FALL_DETECTED;GAIT_NOT_ADVANCING;WAVE_ALL_NOT_REACHED` | INVALID |
| `vx_050_run_01` | true | true | `FALL_DETECTED;GAIT_NOT_ADVANCING;WAVE_ALL_NOT_REACHED` | INVALID |

Generated files:

- `experiments/runs/0718_g2_trotting_motion_baseline/diagnostics/validator_semantics/existing_trial_pose_summary.csv`
- `experiments/runs/0718_g2_trotting_motion_baseline/diagnostics/validator_semantics/existing_trial_fall_timeline.json`
- `experiments/runs/0718_g2_trotting_motion_baseline/diagnostics/validator_semantics/offline_reclassification/offline_reclassification.csv`
- `experiments/runs/0718_g2_trotting_motion_baseline/diagnostics/validator_semantics/offline_reclassification/offline_reclassification.json`

## Runtime revalidation

Gate V reused existing D0 FixedStand/Trotting pose-probe artifacts as runtime
evidence for pose semantics:

- FixedStand pose probes show model/link/base_w upright and above the fall
  height threshold.
- Trotting zero-command and `vx=0.10` pose probes from D0 show model/link/base_w
  upright and above the threshold in those diagnostic windows.

No new locomotion behavior was changed or tuned in this Gate.

## Remaining invalid reasons

All four original G2-B trials remain invalid for:

- `FALL_DETECTED`
- `WAVE_ALL_NOT_REACHED`
- `GAIT_NOT_ADVANCING`

Gate P must therefore locate the first Pre-WAVE blocker without assuming that
fall was a validator false positive.

## Verdict

`G2_VALIDATOR_NO_DEFECT`

This verdict is limited to the suspected validator frame/pose false-positive
mechanism. It does not claim the locomotion root cause is fixed, and it does
not authorize G2-R.
