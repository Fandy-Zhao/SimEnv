# G2-B Test Plan: Trotting Motion Baseline

## Speed Matrix
| vx (m/s) | Epochs | Command window |
|-----------|--------|----------------|
| 0.00      | 3      | zero-speed Trotting/stand window |
| 0.10      | 3      | 5.0 s nonzero command + 2.0 s stop |
| 0.30      | 3      | 5.0 s nonzero command + 2.0 s stop |
| 0.50      | 3      | 5.0 s nonzero command + 2.0 s stop |

## Procedure per Trial
1. Start a fresh isolated ROS/Gazebo runtime using the frozen G2 configuration.
2. Wait for spawn, `/gazebo/model_states`, foot-force topics, and advancing `/clock`.
3. Repeatedly publish `data=2` until FixedStand is observed or timed out.
4. Hold FixedStand for 1.0 s of sim time.
5. Repeatedly publish `data=4` until Trotting is observed or timed out.
6. Wait for `WAVE_ALL`, changing phase, and advancing gait-cycle sequence.
7. Publish zero command for 1.5 s of sim time.
8. Publish target `vx` for 5.0 s of sim time, with `vy=0` and `wz=0`.
9. Publish zero command for 2.0 s of sim time and record the stopping response.
10. Stop only the scoped trial processes and preserve evidence.

## Validity Checks
- Robot spawned and data topics remained available.
- Four foot-force topics existed, their callback sequences advanced, and force data stayed fresh.
- FSM entered FixedStand, then Trotting.
- Wave status entered `WAVE_ALL`, phase changed, and gait-cycle sequence advanced.
- No sim-time reset, pause, controller restart, fall, data loss, or contact-readiness failure.
- RTF is recorded and flagged when low, but not used alone to invalidate a trial.

## Metrics (per trial)
- Steady-window mean/median/std/min/max body-frame `vx`.
- Absolute tracking error and tracking ratio for nonzero commands.
- Forward/lateral displacement, drift ratio, yaw drift, and yaw-rate RMS.
- Roll/pitch/base-height statistics.
- Stop latency, stop time to 0.10 and 0.05 m/s, stop distance, and tail speed.
- Contact/gait duty, contact loss ratio, unexpected contact ratio, and gait frequency.
- Estimator-vs-truth bias/RMSE/correlation/lag where timing diagnostics are present.

## Data Recording
- Gazebo ground truth from `/gazebo/model_states`.
- Controller timing diagnostics CSV from `TimingDiagnostics`.
- Foot-force samples from `/visual/*_foot_contact/the_force`.
- Joint states from `/a1_gazebo/joint_states` when available.
- Event timeline, manifest, environment, git, rosparam, hashes, logs, status, metrics, and plots.
