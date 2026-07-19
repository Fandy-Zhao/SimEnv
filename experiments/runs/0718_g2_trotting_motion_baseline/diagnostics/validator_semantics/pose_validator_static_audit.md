# Pose / Validator Static Audit — G2 Trotting Baseline

## Finding Summary
The validator sources pose from a single Gazebo `ModelStates` topic, derives Euler angles via a standard ZYX-intrinsic conversion, computes body-frame velocity via yaw rotation, and classifies falls using **only** a height threshold (`z < 0.12 m`). No orientation-based fall criteria exist. The Euler conversion assumes unit quaternions without re-normalization. Frame semantics are internally consistent but the fall predicate is purely scalar — it cannot distinguish a collapsed robot from a robot that is genuinely low (e.g., crouching).

## Frame-Semantic Table

| Data | Source | Object | Parent frame | Child frame | Quaternion order | Direction | Units |
|---|---|---|---|---|---|---|---|
| `pose.position` | `/gazebo/model_states` | `a1_gazebo` model (base link) | `world` | — | — | — | m |
| `pose.orientation` | `/gazebo/model_states` | `a1_gazebo` model/canonical link | `world` | model/canonical link | `[x, y, z, w]` (ROS) | child pose in parent; rotation matrix convention body→world | dimensionless (unit) |
| `twist.linear` | `/gazebo/model_states` | `a1_gazebo` model (base link) | `world` | — | — | — | m/s |
| `twist.angular.z` | `/gazebo/model_states` | `a1_gazebo` model (base link) | `world` | — | — | — | rad/s |
| `roll, pitch, yaw` | `quaternion_to_euler()` in `g2_metrics.py:18` | derived from orientation | — | — | ZYX intrinsic | body pose relative to world | rad |
| `body_vx, body_vy` | `world_to_body_velocity()` in `g2_metrics.py:39` | derived from `twist.linear + yaw` | — | — | — | world→body | m/s |
| `roll_peak_deg, pitch_peak_deg` | `compute_truth_metrics()` | derived → `math.degrees()` | — | — | — | — | deg |
| foot force | `/visual/{FR,FL,RR,RL}_foot_contact/the_force` | contact sensor | sensor | — | — | — | N |

## Audit Notes

### Gazebo Pose Source
**Code evidence** (`g2_capture_trial.py:107`): subscribes to `/gazebo/model_states`, matches `model_name` (default `a1_gazebo`). `ModelStates` carries the **full-model** pose from Gazebo's physics engine — not a per-link pose.

### Model / Link
**Code evidence** (`g2_capture_trial.py:119-121`): `message.name.index(self.model_name)` selects the model by name; `message.pose[index]` is the model pose reported by Gazebo. **Inference requiring runtime confirmation**: Gazebo model pose is typically the pose of the model's canonical link; for this A1 URDF the relevant canonical/body link must be confirmed from runtime `ModelStates`, `LinkStates`, or `get_link_state` evidence before calling it definitively `base` or `trunk`.

### Raw Quaternion Order
**Code evidence** (`g2_capture_trial.py:122-124`): `pose.orientation.x, .y, .z, .w` → `[x, y, z, w]` order. ROS `geometry_msgs/Pose` uses `(x, y, z, w)`.

### Quaternion Direction
**Code evidence** (`g2_metrics.py:18-32`): `quaternion_to_euler` uses formulas consistent with the standard rotation matrix R(q) that maps body-frame coordinates to world-frame coordinates. In ROS/Gazebo pose semantics, the quaternion represents the child/model orientation in the parent/world frame. `world_to_body_velocity` (`g2_metrics.py:39-43`) then separately rotates world planar velocity **into** the body frame via the yaw inverse (`c*vx + s*vy, -s*vx + c*vy`), matching R_z(yaw)^T.

### Normalization
**Code evidence**: `quaternion_to_euler` does NOT call any normalization routine — it assumes the input quaternion has unit norm. Gazebo publishes normalized quaternions per its internal convention. **Unknown**: whether accumulated floating-point drift in `ModelStates` (published at sim rate) ever produces quaternions with `|q| != 1` at the subscriber side.

### Euler Convention
**Code evidence** (`g2_metrics.py:18-32`): the three equations match the standard **intrinsic ZYX** Tait-Bryan decomposition: yaw (Z), pitch (Y'), roll (X''). Output order is `(roll, pitch, yaw)`. No alternative decomposition (e.g., XYZ, ZYZ) is in use.

### Normal Standing Reference
**Inference**: normal standing A1 on flat ground would be `(roll≈0, pitch≈0, yaw=arbitrary, z≈0.25–0.30 m)`. The code computes `abs(roll)` and `abs(pitch)` statistics but imposes **no** roll/pitch threshold on validity — orientation is measured but not gated.

### Height Source
**Code evidence** (`g2_capture_trial.py:130`): `pose.position.z` — the Z coordinate of the model's origin in the world frame.

### Fall Predicate
**Code evidence** (`g2_capture_trial.py:302`): `min(row["z"] for row in capture.truth_rows) < 0.12` — a single scalar threshold applied to the **entire** trial duration (not just the active command window). The threshold is `0.12 m`. No roll/pitch/angular-velocity criteria contribute. A robot that crouches below 12 cm is classified as fallen even without orientation/contact evidence; a robot tilted on its side with model origin at 15 cm would not be caught by this predicate.

## Unknowns Requiring Runtime Confirmation
1. Whether the A1 base-link origin in the URDF/SDF coincides with the CoM or a geometric corner — affects what `z < 0.12` physically means.
2. Whether the ground-plane Z in the simulation is exactly 0.0 and perfectly flat.
3. Whether `ModelStates` quaternion norm ever deviates measurably from 1.0 at the ROS subscriber.
4. Whether the model name `a1_gazebo` resolves to the base link or to the full kinematic tree in this specific world file.
