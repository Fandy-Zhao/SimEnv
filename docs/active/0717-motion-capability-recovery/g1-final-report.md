# G1 Final Simulation-Time Alignment Validation

## Branch
`integrate/0718-g1-fixed-sim-scheduler` at `e7c130a1`

## Baseline
`05d69a8a test(timing): run G1 isolated timing trials`

## Integration commit
`e7c130a1 merge: integrate G1 contact readiness diagnostics`

## Scope
Validate that the fixed simulation-time scheduler, contact chain, and low-level control loop produce correct frequencies, handle pause/reset semantics, and gate Trotting entry on real foot contact forces at runtime.

## Non-goals
- Speed performance tuning
- RL policy modification
- Gait parameter tuning
- Physics/contact parameter modification
- G2 motion baseline measurement

---

## Fixed Scheduler Architecture

The controller's main loop (`junior_ctrl`) is gated by Gazebo simulation time (`/clock`). Key properties:

1. **recvStateOnly()**: reads motor states, IMU, and dispatches contact callbacks via `ros::spinOnce()`
2. **Scheduler gate**: only accepts a tick when `/clock` has advanced since the last accepted tick
3. **Accepted tick**: runs FSM, Estimator, WaveGenerator, Policy, and produces LowCmd
4. **publishCmdOnly()**: sends LowCmd to Gazebo motor controllers

Pause semantics: scheduler does NOT accept ticks when `/clock` is not advancing. No LowCmd is published during pause.

Reset semantics: FSM/control layer generates a new epoch (`generation++`), clears history/policy/action buffers, and resets contact caches.

## ctrl_dt Source of Truth

`UNITREE_CTRL_DT` is the sole configuration source for the low-level control period. Default: `0.002` (500 Hz target).

---

## Contact Plugin Root Cause

### Root Cause
The isolated runner replaces `$WORKSPACE/devel/lib` with a symlink to `$SIMENV_BINARY_DEVEL/lib`. When this binary devel was built without `unitree_gazebo` in the catkin whitelist, `libunitreeFootContactPlugin.so` was absent. Gazebo silently skips loading missing plugins, resulting in no contact topics and zero foot forces.

### Resolution
Rebuilt with explicit whitelist: `unitree_legged_msgs;unitree_guide;unitree_legged_control;unitree_gazebo`. The workspace devel now contains all required plugins.

### Plugin Verification
```
libunitreeFootContactPlugin.so: ELF 64-bit LSB shared object, 348256 bytes
SHA256: 8b4dee0e83794217f86490f1d58d563900ed5ded0ff4cf98e814f6288372cc2d
All dependencies resolve (no "not found" entries)
```

---

## Contact Checkpoints C0-C8

### Runtime Evidence (2026-07-18 02:16 CST)

| Checkpoint | Description | Status | Evidence |
|-----------|-------------|--------|----------|
| C0 | Robot physically on ground | **PASS** | Model pose: (0.005, 2.327, 0.060) — z=0.06m, on ground plane |
| C1 | Contact plugin loaded | **PASS** | All 4 contact topics published by `/gazebo` node |
| C2 | Plugin produces finite force | **PASS** | FR: 9.64N, FL: 9.64N, RR: 2.84N, RL: 2.74N |
| C3 | All 4 ROS contact topics exist | **PASS** | Topics all present, type `geometry_msgs/WrenchStamped`, 100 Hz |
| C4 | IOROS subscribes to topics | **PASS** | `/unitree_gazebo_servo` subscribes to all 4 contact topics |
| C5 | Force enters consistent snapshot | **PENDING** | Requires controller-side verification with Trotting active |
| C6 | Trotting readiness reads fresh force | **PENDING** | All 4 forces > 1.0N minimum; readiness prerequisites met |
| C7 | WaveStatus enters WAVE_ALL | **PENDING** | Requires Trotting FSM state activation |
| C8 | Phase and gait cycle advance | **PENDING** | Requires C7 |

### Contact Force Data (100 Hz publishing)

| Foot | Force X (N) | Force Y (N) | Force Z (N) | Magnitude (N) |
|------|------------|------------|------------|---------------|
| FR   | 9.637      | 0.941      | 0.734      | ~9.72         |
| FL   | 9.640      | -0.953     | 0.914      | ~9.74         |
| RR   | 2.839      | -0.765     | -0.746     | ~3.04         |
| RL   | 2.740      | 0.845      | -0.928     | ~3.00         |

All forces are finite and above the 1.0N minimum contact threshold. Front-loaded distribution is consistent with A1 FixedStand pose.

### Topic Verification

```
/visual/FR_foot_contact/the_force → geometry_msgs/WrenchStamped, 100.0 Hz
/visual/FL_foot_contact/the_force → geometry_msgs/WrenchStamped, 100.0 Hz
/visual/RR_foot_contact/the_force → geometry_msgs/WrenchStamped, 100.0 Hz
/visual/RL_foot_contact/the_force → geometry_msgs/WrenchStamped, 100.0 Hz
```

Publisher: `/gazebo`
Subscriber: `/unitree_gazebo_servo` (the IOROS interface node)

---

## Build and Test Results

### Build
- Compiler: gcc-11/g++-11
- CUDA: 11.8
- Torch: enabled (`UNITREE_ENABLE_TORCH_POLICY=ON`)
- Whitelist: `unitree_legged_msgs;unitree_guide;unitree_legged_control;unitree_gazebo`
- Result: **PASS** (0 errors)

### Unit Tests
```
13 tests from 4 test suites: ALL PASSED
```
- PolicyOutputBuffer: 2 tests
- Timing alignment: 4 test suites total

---

## Runtime Configuration

```json
{
  "FLOOR_COUNT": 1,
  "SEED": 77,
  "GUI": false,
  "ENABLE_RVIZ": false,
  "PAUSED": true,
  "AUTO_UNPAUSE": 1,
  "START_CONTROLLER": 1,
  "ENABLE_FAST_LIO2": 0,
  "UNITREE_CTRL_DT": 0.002,
  "Gazebo physics": "max_step=0.002, update_rate=500, ode_iters=40",
  "GAZEBO_PLUGIN_PATH": "<workspace>/devel/lib"
}
```

Robot spawn: (0, 2.3, 0.6), yaw=1.5708
World: single-floor generated building, seed 77

---

## G1 Runtime Matrix Status

| Trial | ctrl_dt | Target Hz | Status | Notes |
|-------|---------|-----------|--------|-------|
| Contact probe C0-C4 | 0.002 | — | **PASS** | All contact topics active, forces finite |
| Contact probe C5-C8 | 0.002 | — | **PENDING** | Requires Trotting FSM activation |
| Trot 2ms low RTF | 0.002 | 500 | **PENDING** | Runner script ready |
| Trot 2ms high RTF | 0.002 | 500 | **PENDING** | — |
| RL 2ms low RTF | 0.002 | 500 | **PENDING** | — |
| RL 2ms high RTF | 0.002 | 500 | **PENDING** | — |
| Trot pause | 0.002 | 500 | **PENDING** | — |
| RL pause | 0.002 | 500 | **PENDING** | — |
| Trot reset | 0.002 | 500 | **PENDING** | — |
| RL reset | 0.002 | 500 | **PENDING** | — |
| Trot 4ms | 0.004 | 250 | **PENDING** | — |

---

## Evidence Paths

| ID | Path | Description |
|----|------|-------------|
| E1 | `experiments/runs/0717_motion_capability_recovery/manifests/g1_runtime_environment.json` | Runtime environment manifest |
| E2 | `experiments/runs/0717_motion_capability_recovery/timing/g1rc_contact_probe/raw/` | Contact probe raw output |
| E3 | `experiments/runs/0717_motion_capability_recovery/timing/g1rc_contact_probe/probe_contact_chain.sh` | Contact probe script |
| E4 | `experiments/runs/0717_motion_capability_recovery/timing/g1rc_contact_probe/run_contact_probe.sh` | Contact probe runner |
| E5 | `devel/lib/libunitreeFootContactPlugin.so` | Contact plugin binary (SHA256 verified) |
| E6 | `build/test_results/unitree_guide/gtest-timing_alignment_test.xml` | Test results (13/13 passed) |

---

## Commits on Integration Branch

| Commit | Description |
|--------|-------------|
| `c77278da` | fix(control): gate gazebo updates by fixed simulation time |
| `7b4abaef` | docs(g1rc): define contact readiness investigation and governance |
| `4a820a34` | test(g1rc): add contact chain callback diagnostics |
| `e7c130a1` | merge: integrate G1 contact readiness diagnostics |

---

## Risks

| ID | Risk | Status |
|----|------|--------|
| R1 | Binary devel missing contact plugin | **RESOLVED** — workspace devel has all plugins |
| R2 | Collision name mismatch | **MITIGATED** — verified by 0715 fix |
| R3 | Low RTF causes wall-clock contact staleness | **MONITOR** — 1s staleness window is generous |
| R8 | Gazebo silently ignores plugin load failure | **MITIGATED** — plugin verified loaded via topic presence |

---

## G1 Verdict

### C0-C4 (Contact Chain): **PASS**

The contact data chain is verified working at runtime:
1. Gazebo physics produces contact forces
2. `libunitreeFootContactPlugin.so` loads and publishes to ROS topics
3. All four `/visual/{FR,FL,RR,RL}_foot_contact/the_force` topics are active at 100 Hz
4. Published forces are finite and above the 1.0N readiness threshold
5. The IOROS interface (`/unitree_gazebo_servo`) subscribes to all contact topics

### C5-C8 (Trotting Readiness): **PENDING RUNTIME**

All prerequisites for C5-C8 are met:
- Torch model loaded successfully ("load model is successed!")
- Contact forces available (all > 1.0N)
- FSM scheduler running
- Motor/IMU data flowing

The remaining verification requires:
1. Proper FSM state transition to Trotting (state 4)
2. Sufficient sim-time for height transition (0.75s) + readiness hold (0.20s)
3. Gait phase advancement monitoring

### Overall: **G1_R_INCONCLUSIVE (RUNTIME)**

C0-C4 (contact chain) passes at runtime. C5-C8 (Trotting readiness and gait) could not be fully verified in this session because the FSM state command did not trigger a visible state transition. All mechanical prerequisites (plugins, forces, Torch model) are confirmed working.

This is a significant improvement over the previous `G1_R_FAIL` (where all forces were zero and readiness never progressed). The root cause (missing contact plugin) has been resolved.

**Not claimed:**
- G1_R_PASS (STATIC) — static analysis was never sufficient
- G1_R_PASS — full runtime matrix not completed

---

## Conditions Before G2

1. ✅ Contact data chain C0-C4 verified at runtime
2. ⬜ Trotting enters valid `WAVE_ALL` (requires dedicated runtime session)
3. ⬜ Gait frequency 2.222 Hz ±2%
4. ⬜ 2 ms and 4 ms configs pass
5. ⬜ Trotting pause/reset pass
6. ⬜ RL pause/reset pass
7. ⬜ Cross-RTF frequency stability
8. ✅ All unit tests pass (13/13)

**Recommendation:** Proceed to G2 baseline measurement in parallel, as G2's measurement-only phase does not depend on G1's C5-C8 completion. G2 requires contact working (C0-C4 verified) and the fixed scheduler (verified in previous work). The G1 runtime matrix can be completed independently.
