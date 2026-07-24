# Changelog

## 2026-07-24 — Shared FAST-LIO2 dependency retry

- Added `tools/prepare_shared_ros_deps.sh` to validate fixed shared FAST-LIO2
  source checkouts and wire them into the task worktree under ignored
  `src/external/` symlinks without cloning, pulling, patching, or modifying the
  public sources.
- Added `/src/external/` to `.gitignore` so the shared dependency symlinks stay
  out of the SimEnv Git index.
- Added a `tools/build_with_venv.sh` preflight that refuses to build the pinned
  shared `livox_ros_driver` when its CMake would auto-clone/build Livox-SDK
  inside `/home/zzf/search_ws/livox_ros_driver`.
- Current gate remains blocked at `FAST_LIO_BUILD_BLOCKED`; FAST-LIO runtime,
  DSV/FALCO, short loop, full exploration, and return-home validation were not
  executed in this retry.

## 2026-07-24 — FALCO + DSV single-floor data path

- Added `single_floor_exploration.launch` for an already running `auto.sh` +
  FAST-LIO2 stack, starting navigation relays, terrain-map adapter, runtime
  boundary, DSV, FALCO, and the `/cmd_vel` bridge without launching Gazebo,
  FAST-LIO2, controller, RViz, or rosbag.
- Added SimEnv-owned terrain-map and boundary adapter nodes under
  `simenv_navigation_bridge`.
- Updated FALCO A1 speed semantics so raw path-following commands reach
  `0.8 m/s` on straight paths, reduce near `0.6 m/s` for ordinary turns, and
  reduce near `0.2 m/s` for large heading errors with `0.22 rad/s` angular cap.
- Repaired DSV initialization and movement detection semantics for zero initial
  motion, windowed stuck detection, bounded replanning, single-floor goal Z, and
  parameterized planner services.
- Formal `./tools/build_with_venv.sh` passed; isolated FALCO path follower
  probes passed. Full `auto.sh` closed-loop exploration was not run.
- Runtime validation was then attempted with the required launch entries and
  stopped at `FAST_LIO_INPUT_BLOCKED`: this worktree does not expose a
  discoverable `fast_lio` ROS package, so `fast_lio/fastlio_mapping` could not
  launch and `/Odometry` timed out. No motion stage was entered.

## 2026-07-24 — FALCO A1 real-cloud R3 tuning

- Added `falco_a1.yaml` as the default FALCO profile for SimEnv Unitree A1
  real FAST-LIO2 `/cloud_registered` validation with `checkObstacle=true`.
- Added launch-time overrides for A1 footprint, height band, path scale/range,
  obstacle threshold, and opt-in FALCO diagnostics.
- Added throttled `falco_diag` logging in FALCO local planner to record cloud
  filtering, candidate/free path counts, selected path, collision score
  distribution, and output command/path state.
- Recorded selected-parameter, pointcloud, candidate-path, and obstacle
  regression evidence under `experiments/runs/0724_falco_a1_tuning/`.
- Kept `/navigation/enabled=false` command gating during all R3 checks; R4-R6
  were not run.

## 2026-07-24 — FALCO R3 real-data diagnosis

- Updated `runtime_real_data.launch` to relay directly from FAST-LIO2
  `/Odometry` and `/cloud_registered` by default, avoiding a runtime dependency
  on intermediate Stage 2 relay nodes for R3 FALCO validation.
- Recorded R3 A/B evidence under
  `experiments/runs/0724_falco_r3_diagnosis/`: direct-source inputs publish
  at about 10 Hz, `checkObstacle=true` yields the zero one-pose path, and
  temporary `checkObstacle=false` yields a multi-pose path plus nonzero raw
  FALCO velocity while `/cmd_vel` remains gated to zero.
- Did not run R4 Trotting, R5 DSV, or R6 full exploration.

## 2026-07-23 — FALCO + DSV real runtime validation

- Added `runtime_real_data.launch` for connecting an already running
  SimEnv + FAST-LIO2 stack to the `/navigation` namespace without starting
  Gazebo, FAST-LIO2, robot models, joystick, RViz, or rosbag.
- Added launch-time command bridge limit overrides for low-speed validation.
- Recorded real runtime evidence through R3 under
  `experiments/runs/0723_falco_dsv_runtime/`.
- Confirmed R2 real FAST-LIO2 data and TF pass; R3 remains blocked because
  FALCO did not produce a useful path from real registered clouds under low RTF.
- Refreshed the evidence bundle on 2026-07-24 with `baseline.txt`, latest R0/R1
  command outputs, and audited `tf_frames.gv`/`tf_frames.pdf` artifacts.

## 2026-07-23 — FALCO + DSV-Planner source integration

- Added minimum navigation vendor sources under `src/navigation/vendor/`:
  FALCO `local_planner` plus the DSV-Planner package closure.
- Added `simenv_navigation_bridge` with a gated, rate-limited
  `TwistStamped -> Twist` command bridge for Trotting `/cmd_vel`.
- Added `simenv_navigation_bringup` launch/config files for FALCO-only,
  DSV-only, and combined DSV + FALCO startup without launching Gazebo,
  FAST-LIO2, robot models, joystick, RViz, or rosbag recording by default.
- Added source vendored ROS1 `octomap_msgs` and `octomap_ros` dependencies
  because this host's apt sources do not provide Noetic binary packages.
- Extended `tools/build_with_venv.sh` package whitelist for the navigation
  packages while preserving the existing runtime whitelist entries.

## 2026-07-23 — FAST-LIO2 clean runtime validation

- Added governed runtime evidence for the clean external FAST-LIO2 build under
  `experiments/runs/0723_fast-lio2-runtime-validation/`.
- Confirmed `/scan_pointcloud2` is owned by the current adapter process,
  `/Odometry` and `/cloud_registered` are owned by `laserMapping`, and the
  pointcloud/odometry chain resumes after Gazebo pause/unpause.
- Confirmed the PointCloud2 contract (`laser_livox`, `24000` points,
  `x/y/z/intensity`, `point_step=16`) and that FAST-LIO2 no longer shows a
  sustained empty-pointcloud failure in the clean runtime.

## 2026-07-23 — FAST-LIO2 reproducible external build and diagnostics

- Added `tools/external_deps/prepare_fast_lio2_deps.sh` to stage fixed,
  clean external FAST_LIO and `livox_ros_driver` sources outside the worktree,
  apply SimEnv-owned patches to the staging copies, and map them into `src/`
  through ignored symlinks.
- Added FAST_LIO C++17 and `livox_ros_driver` message-only staging patches so
  simulation builds do not require Livox-SDK hardware-node linkage or network
  clone.
- Added `tools/diagnostics/check_fast_lio2_input.py` with unit coverage for
  pointcloud finite/blind/timestamp checks and FAST-LIO output presence.
- Validated the formal clean-shell `./tools/build_with_venv.sh` build for the
  runtime whitelist. Runtime pointcloud continuity remains a follow-up gate
  because repeated isolated runs polluted the ROS master with stale node
  registrations before a clean 30-second diagnostic verdict was captured.

## 2026-07-23 — FAST-LIO2 TF repeated timestamp fix

- Fixed `state_from_gazebo` to use a single guarded callback timestamp for
  both referee TF edges and `/Odometry_gazebo`, skip zero/repeated sim-time
  publications, recover after `/clock` rollback, and avoid link-state backlog
  with queue depth 1.
- Changed FAST-LIO2 mapping launch so `odometry_tf_bridge` defaults off;
  `laserMapping` remains the default `camera_init -> body` TF owner and the
  bridge is opt-in for estimators that do not publish TF.
- Added governed TF ownership/root-cause records under
  `experiments/runs/0723_fast_lio2_tf_repeated_data/`.

## 2026-07-23 — competition RL RTF collapse diagnostics

- Added governed competition RL RTF collapse diagnostic artifacts: static
  audit, M0-M8 dry-run-first matrix harness, runtime metrics sampler, and
  mapping-pipeline checker.
- Left simulation launch defaults, controller logic, mapping logic, generated
  scenes, logs, and results unchanged.
- Added a core runtime matrix runner and recorded partial runtime evidence:
  M0/M1/M6 complete, RL policy inference measured at about 50 Hz, and full
  mapping cases blocked by missing external `fast_lio`.
- Propagated diagnostic and thread-limit environment variables into the
  dedicated `junior_ctrl` launch environment so M8-style thread-limit tests can
  be valid once FAST-LIO2 is available.
- Resolved FAST-LIO2 dependency provenance, restored source-only external
  dependencies for validation, rebuilt successfully, and completed short-window
  M2/M3/M4/M5/M7/M8 follow-up runs.
- Identified the first confirmed RTF collapse at the competition sensor layer:
  M2 mean RTF `0.165125` versus M1 `0.989895`, before PointCloud2 conversion,
  FAST-LIO2, or RL inference are required. M3 produced `NO_CLOCK`; M4/M5/M7/M8
  completed but are downstream/secondary costs.

## 2026-07-22 — RL keyboard command fallback

- Added a `State_RL` command fallback so keyboard `w/a/s/d` and `j/l` input
  can drive RL mode when no fresh `/cmd_vel` is available.
- Preserved `/cmd_vel` as the primary command source for navigation and added a
  non-finite `/cmd_vel` guard that commands zero.
- Set the controller default RL policy to the Earth flat-ground recommended
  `policy_act_inference_plane.pt`; runtime policy overrides remain available.

## 2026-07-22 — quick-start auto.sh parameter docs

- Updated `docs/quick-start.md` with current `auto.sh` environment parameters,
  including Earth mode, physics profiles, RL policy override, tmux backend,
  auto-unpause, sensor toggles, and Gazebo physics overrides.

## 2026-07-22 — auto.sh RL policy override visibility

- Added explicit `auto.sh` startup-summary reporting for `RL_POLICY_PATH`.
- Exported `RL_POLICY_PATH` into the dedicated `junior_ctrl` terminal
  environment when set, while preserving controller-side priority
  `/rl_policy_path` -> `RL_POLICY_PATH` -> stair default.

## 2026-07-22 — Earth RL navigation baseline

- Audited local `master` and confirmed the validated Earth RL fixes are present
  at
  `4eeae260b452a296b17f30afaea3b7b2edb7c636`.
- Baseline build PASS with `WORLD_MODE=earth` and `PHYSICS_PROFILE=normal`.
- Plane policy `policy_act_inference_plane.pt` confirmed as recommended
  flat-ground policy; runtime policy selection preserved.
- RL zero and IMU fallback pass; LowCmd median ≈500 Hz. `vx=0.10 m/s`
  remains ineffective and obstacle/stair behavior remains outside this
  flat-ground baseline.

## 2026-07-22 — Earth RL stair vs plane policy comparison

- Added a runtime RL policy path override to `State_RL` with priority
  `/rl_policy_path`, `RL_POLICY_PATH`, then the existing stair default.
- Logged policy configured path, resolved realpath, SHA256, existence, and load
  success so runtime comparisons prove the actual TorchScript artifact used.
- Compared stair and plane policies on Earth flat ground from `vx=0.00` through
  `0.40 m/s` without control-parameter tuning. Plane is recommended for the
  master short regression because it tracks `0.20` to `0.40 m/s` more closely
  and with lower yaw drift; both policies still show no effective motion at
  `vx=0.10 m/s`.

## 2026-07-22 — Earth RL LowCmd and IMU merge validation

- Merged the validated `fix/0722-earth-rl-lowcmd-publisher-stall` branch into
  `fix/0722-earth-rl-timebase-fast-validation` with a no-ff merge commit.
- Preserved the stair policy path while bringing in LowCmd queue depth 1,
  `tcpNoDelay()`, persistent ROS callback spinners, simulation-time LowCmd
  cadence scheduling, staged T1-T4 diagnostics, and Earth IMU policy-input
  fallback.
- Validated the task worktree build and short Earth RL regression:
  FixedStand/RL zero T0-T2 median cadence is 500 Hz, T3/T4 are 1000 Hz,
  `using_imu_policy_input=1`, policy inputs are finite, and RL zero does not
  fall over 9 sim-s.

## 2026-07-22 — Earth RL timebase fastcheck

- Added governed evidence under `experiments/runs/0722_earth_rl_fastcheck/`
  for Earth `PHYSICS_PROFILE=normal` RL timing, RTF, policy-path, and first
  speed-smoke validation.
- Fixed the RL default policy path so runtime `State_RL` loads
  `src/unitree_guide/logs/policy_act_inference_stair.pt` instead of the older
  plane policy.
- Validation result: build passes and policy path passes after the fix, but
  RTF stability fails (`median=0.750945`, `p10=0.407949`) and the first
  `vx=0.10 m/s` RL smoke is unstable.

## 2026-07-21 — RL fast validation infrastructure

- Added `tools/build_rl_fast.sh` as a thin Unitree runtime-profile wrapper over
  the tracked `tools/build_with_venv.sh`.
- Added governed RL fast-validation scaffold under
  `experiments/runs/0721_rl-fast-validation/`, including metrics helpers,
  F0/F1/F2 FixedStand smoke runner, live capture, replay scaffold, tests, build
  logs, provenance, and summary evidence.
- Added `docs/reports/0721_rl-fast-validation.md` documenting the current
  `FAIL_BASE_HEIGHT` F0 native FixedStand result. No RL/controller/world/physics
  semantics changed.

## 2026-07-21 — Unitree runtime rebuild and retest

- Added governed rebuild/retest evidence under
  `experiments/runs/0721_unitree-runtime-rebuild/`, including toolchain probes,
  build logs, artifact manifests, runtime capture scripts, metrics helpers,
  and C0-A raw runtime logs.
- Confirmed the Torch/CUDA build blocker is the host GCC 12 `cc1plus` gap; the
  Unitree runtime profile builds successfully with GCC/G++ 11 via
  `tools/build_with_venv.sh`.
- Stopped runtime validation at C0-A native FixedStand because it did not pass
  3/3. No world, controller, policy, spawn, physics, URDF, gain, estimator,
  gait, IK, or fall-validator code was changed.

## 2026-07-20 — Earth flat-ground runtime validation

- Cherry-picked `5f5f9045` into the runnable earth RL benchmark worktree and
  added governed runtime validation scripts, metrics, tests, and reports under
  `experiments/runs/0720_earth-flat-ground-runtime/`.
- Confirmed at runtime that `platform_1` and `platform_2` are absent from the
  Earth world model list; G0 passed with median RTF about `0.983`.
- Stopped before Earth E0 and all RL cases because G1 initial-contact gating
  failed and C0 competition FixedStand also failed in the selected runtime
  artifact environment. No controller, policy, URDF/xacro, spawn, world, or
  physics behavior was changed by this validation task.

## 2026-07-20 — Earth flat-ground fix

- Removed the raised `platform_1` and `platform_2` models from
  `unitree_gazebo/worlds/earth.world`, including their visual and collision box
  geometry.
- Preserved the `sun` include, single `ground_plane` include, physics settings,
  `WORLD_MODE=competition` default, and `WORLD_MODE=earth` spawn pose
  (`x=0.0 y=0.0 z=0.6 yaw=0.0`).
- Added governed issue, notes, and report evidence for static XML/SDF checks
  and the remaining built-worktree runtime validation gap.

## 2026-07-20 — Earth RL motion benchmark

- Added `WORLD_MODE=earth` as an isolated launch mode that resolves the tracked
  `earth.world`, skips competition scene generation, and defaults optional
  mapping/competition nodes off while preserving explicit environment
  overrides.
- Added earth-world benchmark capture helpers and static tests under
  `experiments/runs/0720_earth-rl-motion/`.
- Ran earth world/topic smoke and E0 FixedStand. Earth launches independently,
  but E0 fails at body attitude (`max_roll_deg=91.911513`), so active RL trials
  are blocked before policy attribution.

## 2026-07-18 — FAST-LIO2 runtime point-cloud orientation

- Added startup diagnostics to `auto.sh` so branch, HEAD, ROS overlay paths,
  package resolution, and the active adapter path are visible before Gazebo
  starts.
- Added a runtime smoke checker proving `/scan` and `/scan_pointcloud2` are
  coordinate-identical in `laser_livox`, verifying the 45° LiDAR TF, and
  optionally checking `/cloud_registered` ground-plane alignment.
- Runtime validation showed the adapter is correct in a worktree containing
  `69ff34e7`; lingering `-X/-Z` observations point to old branch/overlay or
  Python environment contamination, not a remaining adapter rotation.

## 2026-07-18 — FAST-LIO2 point-cloud frame semantics

- Made the PointCloud-to-PointCloud2 adapter a coordinate-preserving format
  conversion by default; optional rotations now require an explicit output
  frame name rather than being published as `laser_livox`.
- Added adapter frame-semantics regression coverage and documented the
  LiDAR-plugin to FAST-LIO2 coordinate responsibility boundary.

## 2026-07-20 — G2 Fast Exit Gate A

- Added diagnostic-only G2 fast-exit P0/P1/P2 probe tooling with private ROS
  master orchestration and scoped cleanup.
- Updated pre-wave offline classification so stable zero-command Trotting can
  be marked `EXPECTED_NO_STEP_TRIGGER` instead of a missing wave-start failure.
- Ran P0 FixedStand-only. The valid P0 run failed before FixedStand with no
  foot-contact samples, final FSM `PASSIVE`, and minimum model height
  `0.05698662028992169 m`.
- Published `G2_FAST_EXIT_SHARED_BASE_FAILURE`; P1/P2 and active RL were not
  run. RL is limited to shadow/static validation until P0 is recovered.

## 2026-07-19 — G2-D1 fall validator semantics gate

- Added Gate V validator-semantics evidence and tests for G2-D1.
- Audited `/gazebo/model_states` pose semantics, quaternion order, body tilt,
  and the existing height-only fall predicate.
- Reclassified the four existing G2-B smoke trials offline without modifying
  raw trial files; all four remain invalid with `FALL_DETECTED`,
  `WAVE_ALL_NOT_REACHED`, and `GAIT_NOT_ADVANCING`.
- Published ADR-009 and `g2-validator-semantics-report.md` with verdict
  `G2_VALIDATOR_NO_DEFECT` for the suspected frame/pose false positive.

## 2026-07-19 — G2 Trotting baseline tooling

- Added the G2-B governance scaffold for Trotting motion baseline testing,
  including runtime-configuration, valid-trial, motion-acceptance, and
  root-cause-classification ADRs.
- Added G2-B-only trial tooling under
  `experiments/runs/0718_g2_trotting_motion_baseline/` for isolated ROS master
  trials, Gazebo truth capture, controller timing CSV capture, foot-force
  logging, metric aggregation, and pure metric tests.
- Recorded one smoke trial at each G2 speed point. All four were invalid before
  WAVE_ALL/gait execution, so the current baseline verdict is
  `G2_BASELINE_INCONCLUSIVE` and no recovery fix was applied.
- No controller, model, or Gazebo physics parameters were changed; no G2
  baseline pass/fail claim is made.

## 2026-07-17 — Single-floor Trotting/RL speed profile

- Added reproducible, isolated ROS-master speed trials for Trotting and RL at
  0.1, 0.5, and 1.0 m/s, with Gazebo-truth planar traces and per-epoch metrics.
- Recorded the current RTF/mobility relationship and explicitly classified the
  0.5 s windows as short-response evidence, not steady-state calibration.
- Added a report, raw-data artifact guide, summary CSV/JSON, and generated
  planar-trajectory and RTF/mobility figures.

## 2026-07-17 — FAST-LIO2 Stage 2 navigation interface

- Added transparent, configurable `/state_estimation` and `/registered_scan`
  relays while preserving legacy `/Odometry` and `/cloud_registered` topics.
- Kept `map → camera_init` TF ownership unchanged and added static contract
  tests for topic/frame responsibility.
- Package build passes; full-workspace CUDA build and five-minute isolated
  Gazebo validation remain blocked and are documented in the task report.
- Added an Odometry-to-TF bridge for `camera_init → body` and validated the
  revised 150-second runtime target with both navigation topics at 10 Hz.

## 2026-07-17

- Completed the Gazebo–unitree_guide–RL timing-alignment review. Policy/history
  now remain stationary during simulation pause, state and action snapshots
  prevent torn LowCmd, reset clears the prior epoch, and reset-generation
  validation rejects in-flight stale policy output. Added the final report and
  five timing regression tests.

## 2026-07-16

### Gazebo RL Timing Diagnostics

- Added opt-in buffered CSV diagnostics for FSM iterations, simulation/state
  generations, policy wait exit reasons, history timestamps, action
  generations, LowCmd sends, and torn-action detection.
- Recorded an RTF 0.276 baseline: policy remained 49.25 Hz in simulation time
  with no normal-run wall overtime, while pause advanced policy/history/action
  and 28 LowCmd copies overlapped an action write.
- Changed the simulation policy wait caller so wall overtime is diagnostic only:
  it keeps waiting from the same simulation-time origin and does not advance
  observation, history, inference, or action until the full simulation period.
- Replaced inference-thread writes to shared LowCmd with a complete policy
  output snapshot applied by the FSM thread, and build observations from a
  locked LowState/base snapshot plus an independently sequenced command.
- Runtime history updates now require a new state generation, increasing
  simulation timestamp, and a full 20 ms policy interval; the existing
  repeated-current-observation entry initialization is preserved.
- Promoted the Gazebo microsecond clock to an atomic 64-bit value, explicitly
  propagated `use_sim_time` into Gazebo launch, and reset policy tensors,
  history timestamps, command/action snapshots, and transition state when
  simulation time moves backward or jumps forward. A pause does not advance or
  invalidate the last complete action.
- Added production-backed regression tests for simulation wait outcomes,
  history deduplication/reset across the former 32-bit boundary, and concurrent
  complete-generation action publication.

### Gazebo Simulation-Time Locomotion Control

- Changed WaveGenerator, estimator propagation, Trotting height/readiness, and
  desired body/yaw integration to use advancing Gazebo `/clock` time instead
  of wall/system time or a fixed controller-loop increment.
- The Gazebo FSM now skips gait, estimator, and control-target updates while
  simulation time is unchanged, and resets to all stance on a sustained pause,
  backward clock, or excessive forward jump.
- Added latched Wave cancellation for excessive roll/pitch, scheduled
  stance-foot contact loss, and non-finite output. The abort path stops further
  gait/IK calculation until Trotting is re-entered.
- Forced-RTF test at about `0.10` measured `0.946 s` of simulation time for the
  configured `0.75 + 0.20 s` entry gates, while `9.459 s` elapsed on the wall.
  Pause/resume, tilt abort, and `0.080 s` four-foot contact-loss abort passed.

### A1 Nominal Stance and Trotting Readiness Gate

- Consolidated A1 nominal stance in foot space and changed FixedStand to derive
  joint targets through inverse kinematics instead of a duplicated
  `[0, 0.9, -1.8]` array.
- Trotting now inherits current body height and global foot positions, uses a
  0.75 s smoothstep to nominal height, and suppresses wave until velocity,
  attitude, desired all-stance, and four measured contacts are stable.
- Added fresh foot-force feedback to the ROS simulation and real SDK state
  paths; the prior gait contact vector remains only a desired schedule.
- Removed an invalid second parent joint from every A1 foot in xacro. This
  eliminated duplicate Gazebo foot collisions and made the IK-derived nominal
  FixedStand settle instead of oscillating/falling.
- Final headless test held zero Trotting upright and moved 1.891 m in 6.920 s
  (`0.273 m/s`) in the correct body-forward direction for `linear.x=0.3`.

## 2026-07-15

### A1 Trotting Safety and Gazebo Joint-Angle Repair
- Reset Trotting command/yaw-filter state on entry, reject non-finite Twist,
  clamp commands, expire stale `/cmd_vel` after 0.5 s, and block non-finite
  gait/IK/motor output with a measured-position hold.
- Normalize Gazebo bounded revolute feedback to the signed angle branch used by
  A1 URDF commands and kinematics; this fixes calf feedback such as `+3.5866`
  representing about `-2.6966` and corrupting PD/IK.
- Start the A1 simulation at the local FixedStand joint target rather than the
  folded-knee limit. Headless Trotting stayed finite under zero/forward Twist
  and moved in the commanded direction; exact velocity tracking remains slow
  (`~0.121 m/s` average for `0.3 m/s` requested).

### RL Entry Real-Command Guard
- Disabled the FreeDog real-robot command initialization in `State_RL::enter()`.
  Its surrounding `real` condition had been commented out, so it previously
  executed even when entering RL in Gazebo simulation.

### Dedicated Controller & RVIZ Terminals (fix/0715-build-auto-startup)
- Added a tmux-backed terminal mode (default). `junior_ctrl` and RViz run in
  named sessions that survive a GUI-terminal failure and can be reattached with
  `tmux attach-session -t simenv-junior_ctrl` or `simenv-rviz`.
- Run the tmux command from a generated runtime script rather than shell-quoted
  multiline text, preventing `/bin/sh` from rejecting Bash-specific escaping
  and immediately closing the session.
- Corrected the per-session `GAZEBO_MODEL_PATH` export so the generated Bash
  script remains syntactically valid and both named sessions stay alive.
- Validated full startup: both sessions remained live, `/Odometry` published at
  about 10 Hz, and `SIGINT` removed sessions and runtime processes cleanly.
- At each new startup, explicitly stop the prior controller and RViz sessions
  and remove their runtime records; attached GUI clients now exit with the
  session instead of remaining as orphaned shells.
- Fixed terminal creation when `auto.sh` is launched from Snap Code: it now
  starts `gnome-terminal.real` with a clean desktop environment instead of the
  Snap-contaminated D-Bus wrapper, which failed with a GLIBC symbol error.
- Keep a diagnostic interactive shell open after either dedicated command
  exits, so a controller/RViz failure is visible instead of flashing and
  immediately closing its terminal window.
- Run each terminal command in a child shell because `rosrun` ends in `exec`;
  this prevents an RViz launch failure from replacing the outer shell and
  bypassing the terminal-preservation logic.
- `auto.sh` now launches `junior_ctrl` and rviz in dedicated `gnome-terminal`
  windows (fallback: `xterm` → background).  This guarantees a real TTY for
  keyboard state switching (`2`=stand, `4`=trot, `6`=RL), eliminating the
  unreliable `/dev/tty`-redirection approach that caused segfaults and missing
  keyboard input.
- Removed `CONTROLLER_FOREGROUND` variable; the controller always runs in its
  own terminal.  Added `ENABLE_RVIZ` (default: 1).
- Replaced the end-of-script `wait` with a `trap cleanup INT TERM` + infinite
  sleep loop — Ctrl‑C in the main terminal now cleanly stops all ROS processes.

### FSM Nullptr Segfault Fix (fix/0715-build-auto-startup)
- Guarded keyboard `4`/`6` mappings, ROS `/fsm/state_cmd` cases `4`/`6`, and
  `State_FixedStand::checkChange()` TROTTING/RL returns behind `#ifndef
  UNITREE_DISABLE_TORCH_POLICY`.  When Torch is OFF (default), pressing these
  keys or publishing the matching rostopic no longer triggers a transition to a
  nullptr state object → segfault.
- Added a nullptr safety check in `FSM::run()` so that any future unguarded
  path produces a warning instead of a crash.
- Updated `auto.sh` help text to indicate that 4=Trotting and 6=RL require a
  Torch-enabled build.

### Controller Terminal Keyboard Fix (fix/0715-auto-keyboard)
- `auto.sh` now explicitly assigns `/dev/tty` to background `junior_ctrl`.
  This preserves keyboard input when FAST-LIO2 startup requires the controller
  to run before mapping.
- Replaced non-interactive `fg %1` with `wait`, keeping the invoking terminal
  attached to the controller after startup.  Non-TTY launches now warn and
  document ROS-topic control as the supported alternative.

### Build and Controller Startup Guard (fix/0715-build-auto-startup)
- Updated the default `build_with_venv.sh` profile to build every `auto.sh`
  runtime dependency (FAST-LIO2, Mid-360 sensor plugin, `junior_ctrl`, Unitree
  Gazebo plugins) while excluding unrelated locally added packages such as
  legacy `ps3joy`.
- Added an early `junior_ctrl` artifact check in `auto.sh`, preventing scene
  generation and Gazebo startup when the controller has not been built.

### FAST-LIO2 LiDAR–IMU Axis Correction (fix/0715-fast-lio2-axis)
- Corrected `simenv_mid360.yaml` to the direct FAST-LIO2 point transform
  `p_imu = R_L_I * p_lidar + T_L_I`: `Ry(+45°)` and `[0.2, 0, 0.08]`.
  The prior inverse (`Ry(-45°)`, `[-0.085, 0, -0.198]`) conflicted with the
  `laser_livox`-frame points produced by the simulator.
- Added `check_fast_lio2_extrinsics.py`, which derives the expected transform
  from `robot.xacro`, and installed both it and the TF bridge with catkin.
- Updated the deployment guide and ADRs.  A mapper restart is required before
  the corrected parameters affect a running session.

## 2026-07-14

### FAST-LIO2 Frame Correction (zzf/0714-fast-lio2-frame-fix)
- **Root cause**: `map_to_camera_init_bridge.py` applied a spurious **Ry(-45°)** rotation when publishing `map→camera_init`. The 45° LiDAR tilt was already handled by FAST-LIO2's `extrinsic_R`, so this was a duplicate rotation that tilted the entire `camera_init` world frame, causing `/Odometry` body axes to appear incorrect (X pointing downward instead of forward).
- **Fix**: Removed the Ry(-45°) rotation from the bridge. `map→camera_init` is now a direct copy of `map→imu_link` (the trunk IMU pose) without any extra rotation. Removed unused `_rotate_by()` function and `math`/`tf.transformations` imports.
- **Documentation**: Updated stale bridge comment in launch file (was "map→laser_livox", corrected to "map→imu_link"). Added rotation responsibility boundary documentation to YAML config. Created ADR-0714 (frame convention decision record).
- **Verified**: Python syntax, YAML syntax, launch XML, catkin_make build, rotation matrix validity (det≈1, orthogonal). Runtime tests pending (ROS master not available).
- **ADR**: `docs/decisions/ADR-0714-fast-lio2-frame-convention.md`

### FAST-LIO2 Startup Order Fix
- **Controller before FAST-LIO2**: `auto.sh` now starts `junior_ctrl` BEFORE FAST-LIO2 when `ENABLE_FAST_LIO2=1`. Controller is forced to background mode, FixedStand is auto-commanded via `/fsm/state_cmd` (`data: 2`), and the script waits for IMU confirmation (gravity aligned to Z, `linear_acceleration.z >= 9`) before launching FAST-LIO2. Prevents incorrect gravity initialization that caused continuous Z drift in `/cloud_registered`.
- **Duplicate adapter fix**: `simenv_fast_lio2_mapping.launch` now defaults `enable_adapter:=false`. `auto.sh` starts `scan_to_pointcloud2.py` once; the launch file no longer starts a second instance. Pass `enable_adapter:=true` when launching standalone.
- **FSM reference**: Use `rostopic pub /fsm/state_cmd std_msgs/Int8 "data: N"` for programmatic state control (2=FixedStand, 4=Trotting, 6=RL).
- **Controller foreground**: Default `CONTROLLER_FOREGROUND=1`. When `ENABLE_FAST_LIO2=1`, controller starts in background for auto-stabilisation, then `fg` pulls it back after FAST-LIO2 is running. Simple foreground when FAST-LIO2 is disabled.

### FAST-LIO2 Runtime Validation
- **PointCloud2 format fix**: `scan_to_pointcloud2.py` now outputs x,y,z,intensity (xyzi32) instead of xyz32.  FAST-LIO2 `lidar_type=4` requires the intensity field for curvature-based feature extraction; without it every point was rejected as "not effective" and the EKF diverged to 8000+ meters within minutes.
- **TF bridge**: New `map_to_camera_init_bridge.py` publishes a static TF `map→camera_init` by looking up `map→laser_livox` at startup, connecting the previously separate Gazebo and FAST-LIO2 TF trees.
- **RViz config**: Added `config/fast_lio2.rviz` pre-configured for Odometry, Laser_map, cloud_registered, path, and TF displays with Fixed Frame: map.
- **IMU topic fix**: Switched from `/livox/imu` (45° tilted, vibration-corrupted gravity) to `/trunk_imu` (body-aligned, z-up).  Updated LiDAR→IMU extrinsic to `Ry(-45°)`.  Eliminated pure-Z drift of 62 m/s; all axes now within 1 cm after 30 s.
- **Verified convergence**: With robot in FixedStand, FAST-LIO2 converges to ~1 cm from origin within 20 s. All output topics publish stably.

### Locomotion
- **RL diagnosis**: Both `policy_act_inference_plane.pt` and `policy_act_inference_stair.pt` confirmed non-responsive to `/cmd_vel` velocity commands (0.15-1.0 m/s produce <0.2 mm motion).
- **Trotting `/cmd_vel`**: Added `ros::Subscriber` to `State_Trotting` (classical MPC gait). `/fsm/state_cmd` topic (`std_msgs/Int8`) enables programmatic state transitions: 2=FixedStand, 4=Trotting.

### Infrastructure
- **`auto.sh` cleanup**: Replaced 5-line pkill with comprehensive cleanup covering Gazebo core, SimEnv launch, unitree controller, FAST-LIO2, sensor bridges, and stale rostopic processes.

## 2026-07-13

### Documentation
- **New `docs/cli-reference.md`**: comprehensive CLI quick-reference covering all commands: build, auto.sh env vars (6 categories), FAST-LIO2 launch, scene generation, door/elevator control, controller, evaluation, diagnostics, file paths, rosbag.
- **Updated `docs/README.md`**: added FAST-LIO2 deployment guide and CLI reference to the documentation index.
- **Updated `docs/quick-start.md`**: added `ENABLE_FAST_LIO2=1` usage, FAST-LIO2 related env vars, fixed `ROBOT_Y` default from `-2.2` to `2.3`.

### FAST-LIO2 Deployment Test (Stage 0 and initial Stage 1 L1/L2)
- **Stage 0 (PASS)**: Verified Gazebo sensors, simulation time, and TF tree. All checks passed: `/use_sim_time=true`, `/clock`@500Hz, `/scan`@10Hz (`laser_livox` frame), `/livox/imu`@400Hz (`livox_imu_link` frame). TF tree correct: `base→laser_livox` (45° pitch) `→livox_imu_link` (extrinsic matches config).
- **Stage 1 initial L1/L2 check (superseded by runtime retest below)**: FAST-LIO2 node launched without crash and registered all output topics. Discovered and fixed 3 launch bugs:
  - FAST-LIO2 node was commented out in `simenv_fast_lio2_mapping.launch`
  - YAML config loaded in wrong namespace (`ns="laserMapping"`) — FAST_LIO uses public `NodeHandle`, reads from `/common/lid_topic`; fixed to root-level `<rosparam>`
  - Removed redundant `<param>` tags (superseded by YAML rosparam load)

### FAST-LIO2 Runtime Retest
- Corrected FAST-LIO2's input type contract: SimEnv's `/scan_pointcloud2`
  adapter publishes `sensor_msgs/PointCloud2`, so `simenv_mid360.yaml` now
  uses `preprocess.lidar_type: 4` (MARSIM/standard PointCloud2), rather than
  Livox `CustomMsg` mode (`1`).
- Verified the corrected launch publishes finite `/Odometry` and
  `/cloud_registered` messages.
- Marked Stage 1 static localization **failed**: 60 s wall-clock capture
  (4.300 s ROS time) drifted 325.253 m and 51.607° while Gazebo truth moved
  only 0.0476 m over 1.020 s. The 5 m and loop tests are deferred by the
  stage gate.

### FAST-LIO2 Controlled P0 Follow-up
- **Superseded the static-failure interpretation**: the former capture ran
  without `junior_ctrl`, and the A1 was falling/rolling rather than stationary.
  In fixed-stand mode FAST-LIO2 moved 0.001967 m / 0.066852° over 60 s
  wall-clock (4.300 s ROS time), with effectively static Gazebo truth.
- **Fixed FAST-LIO2 MARSIM initialization**: initialized
  `last_lidar_end_time_` in the external FAST_LIO `ImuProcess` constructor and
  `Reset()` to avoid undefined startup/reset behavior in the PointCloud2 path;
  the `fast_lio` package selectively rebuilt successfully.
- **P1 explicitly blocked**: the ABI-isolated `junior_ctrl` build excludes the
  Torch-dependent trot/RL states, so a real 5 m or loop locomotion test needs
  an approved Torch-policy rebuild.  The strict 60 s ROS-time P0 capture also
  remains pending because of low simulation RTF.

### FAST-LIO2 Extended P0 Attempt
- The latest ROS-time fixed-stand capture reached 28.900 s before a second
  workspace (`/home/zzf/桌面/unitree_ex`) joined the same ROS master and
  registered duplicate `/gazebo` and `/robot_state_publisher` names. Its
  missing `hustw_description` package then ended the shared session; this is a
  cross-workspace ROS-master collision, not a FAST-LIO2 crash.
- In that interrupted window Gazebo truth changed 0.018344 m / 0.592970° and
  FAST-LIO2 changed 0.074965 m / 0.788825°. P0 is not accepted: ROS-master
  isolation and a more stationary support/control condition are required.

### FAST-LIO2 Reduced-duration P0
- Measured RTF=0.068 and completed a separate 10 s ROS-time diagnostic window
  to avoid an excessive wall-clock test. FAST-LIO2 changed 0.286359 m /
  1.216603°, versus Gazebo truth 0.004874 m / 0.774496°; all values were
  finite and `/cloud_registered` remained available, but P0 still fails.
- Added duration and output-tag environment options to the P0 capture helper,
  so reduced-duration tests do not overwrite the 60 s evidence.

### P1 Locomotion Analysis & Trotting `/cmd_vel` Fix
- **RL policy diagnosis**: Both `policy_act_inference_plane.pt` and `policy_act_inference_stair.pt` confirmed non-responsive to `/cmd_vel` velocity commands. At 0.3 m/s and 1.0 m/s, robot moves <0.2 mm. RL inference outputs constant standing actions regardless of command.
- **Trotting `/cmd_vel` addition**: Added `ros::Subscriber` for `/cmd_vel` to `State_Trotting` (classical MPC gait controller, no RL dependency). When active, overrides keyboard WASD with `cmd_vel.linear.x/y` and `angular.z`, saturated to robot limits (vx[-0.8,0.8], vy[-0.6,0.6], wz[-1.0,1.0]). Keyboard fallback preserved when no `/cmd_vel` received.
- **State chain**: `Passive → 2(FixedStand) → 4(Trotting)`. Trotting originally keyboard-only (WASD→vx, AD→vy, JL→wz, IK→height).

### FAST-LIO2 P0 Cause Diagnostic
- Added a synchronized truth/IMU/odometry diagnostic and ran it for 10 s.
  FAST-LIO2 tracked the remaining physical truth rotation closely (0.364035°
  versus 0.325467°); truth and IMU gyro rates agreed, while acceleration
  reached 87.112 m/s². This identifies residual fixed-stand contact/rotational
  motion as the primary P0 cause, rather than standalone FAST-LIO2 divergence.

## 2026-07-06

### Build Fix
- Fixed `unitree_guide/junior_ctrl` Torch ABI pollution: `find_package(Torch)` injected `-D_GLIBCXX_USE_CXX11_ABI=0` globally, causing `ros::init` undefined references. Added `UNITREE_ENABLE_TORCH_POLICY` option (default OFF) to isolate Torch flags; excluded 3 torch-dependent source files; guarded transitive header includes in `FSM.h`/`FSM.cpp`. `junior_ctrl` now compiles and links with ROS Noetic (fix/0704-unitree-torch-abi-isolation).

### Documentation
- Added `docs/slam/fast_lio2_deployment_guide.md`: comprehensive FAST-LIO2 deployment guide covering repository layout, sensor topic mapping, parameter reference, pointcloud compatibility, IMU selection, extrinsic calibration, build environment, deployment steps, runtime validation checklist, common failure modes, experiment tracking parameters, and output contract for future navigation (docs/0706-fast-lio2-deploy-guide).
- Added `docs/decisions/ADR-0706-fast-lio2-deploy-guide.md`: architecture decisions for deployment guide.
- Added `docs/reports/0706_fast-lio2-deploy-guide.md`: task report with coverage table.

## 2026-07-04

### Documentation
- Fixed FAST-LIO2 workspace layout documentation: SimEnv is the catkin workspace root, FAST_LIO belongs at `SimEnv/src/FAST_LIO`, not nested under another `catkin_ws/`.
- FAST-LIO2 build environment audit: static checks all pass; catkin_make blocked by missing libtorch (C++ SDK) at hardcoded path in unitree_guide. Logs at `experiments/runs/0704_fast-lio2-build-check/`.
- `tools/build_with_venv.sh`: added auto-detection of pip torch CMake prefix (`torch.utils.cmake_prefix_path`), passes `-DCMAKE_PREFIX_PATH` to catkin_make without overwriting ROS paths.
- `tools/build_with_venv.sh`: auto-selects gcc-11/g++-11 for CUDA 11.8 compatibility; passes CC/CXX/CUDAHOSTCXX + CUDA paths to catkin_make. CUDA host compiler errors eliminated.
- Build fixes: unitree PIE linker (`-no-pie`), FAST_LIO C++14→17, livox_ros_driver C++11→17, PCL shared_ptr serialization, missing `<memory>` includes in Livox-SDK.

### Build Tooling
- Added `tools/build_with_venv.sh`: builds catkin workspace with project `.venv` Python, ensuring consistent interpreter for torch and other Python deps.
- README updated with venv setup and build instructions (torch 2.0.1 pin for Python 3.8 / ROS Noetic).

### Governance & Remote Configuration
- Added GitHub remote (`github` → `git@github.com:zzf/SimEnv.git`), origin retained as Gitee.
- Branch naming policy: maintenance/setup branches now use `zzf/MMDD-short-name`; `chore/` prefix is deprecated for this project.
- Initialized project governance skeleton (AGENTS.md, PROJECT_STATE.md, ROADMAP.md, docs/architecture.md, docs/module_status.md).

### FAST-LIO2 Mapping Integration (feat/0704-fast-lio2-mapping)
- Added `src/simenv_fast_lio2_integration/` ROS package with PointCloud adapter, FAST-LIO2 config, and launch files.
- Added `ENABLE_FAST_LIO2` optional flag in auto.sh.
- FAST-LIO2 operates as external catkin workspace dependency (not vendored).

## Historical (from git log)

### 2025
- `a46d947` — add LICENSE.
- `736ab90` — update README.md.
- `8191cff` — 电梯开关门优化
- `6d2aa9c` — 优化随机生成建筑
- `8ba1867` — 添加危险源及相关算法评估程序
