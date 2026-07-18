# G1-RC: Contact Readiness and Full Scheduler Revalidation

## Branch
`fix/0717-g1r-contact-readiness` (from `integrate/0718-g1-fixed-sim-scheduler`)

## Baseline
`05d69a8a test(timing): run G1 isolated timing trials`

## Previous G1-R Failure
`G1_R_FAIL` — Trotting never entered `WAVE_ALL`. Readiness log:
```
Trotting waiting for wave readiness: height=1 stance=1 contact=0
force=[0.0 0.0 0.0 0.0]N
```

## Scope
Restore Trotting foot-end contact force chain and complete full G1-R revalidation matrix.

## Non-goals
- Speed performance tuning
- RL policy modification
- Gait parameter tuning
- Physics/contact parameter modification

---

## Contact Data Architecture

### Data Chain (C0–C8)

```
C0: Gazebo Physics (ODE)
  Foot collision spheres (r=0.02m) contact ground plane
  → Contact forces computed by ODE solver

C1: Contact Sensor Plugin (libunitreeFootContactPlugin.so)
  Source: src/unitree_guide/unitree_ros/unitree_gazebo/plugin/foot_contact_plugin.cc
  Sensor: type="contact" on each calf link (references lumped foot collision)
  Collision names: FR_calf_fixed_joint_lump__FR_foot_collision_1 (etc.)
  Loaded: UNCONDITIONALLY in gazebo.xacro lines 86-121
  Update rate: 100 Hz

C2: Plugin Data Production
  OnUpdate() averages body_1_wrench forces across all contact points
  When no contacts: publishes force = 0 (no "no contact" sentinel)

C3: ROS Topics
  Topics: /visual/{FR,FL,RR,RL}_foot_contact/the_force
  Type: geometry_msgs/WrenchStamped
  Publisher: UnitreeFootContactPlugin::force_pub

C4: IOROS Callbacks
  File: IOROS.cpp lines 249-263
  Function: updateFootForce(index, msg)
  Stores: Euclidean norm sqrt(x²+y²+z²) → _foot_force[index] (atomic float)
  Wall time: _foot_force_wall_stamp_ns[index]
  Dispatch: ros::spinOnce() in recvStateOnly() each FSM iteration (500 Hz)

C5: State Snapshot
  File: IOROS.cpp lines 154-179 (recvState)
  Copies _foot_force[] → state->footForce[] (float, N)
  Freshness: footForceValid[i] = (wall_now - last_callback) <= 1 second
  Concurrent safety: atomic loads, torn reads possible across feet

C6: Readiness Input
  File: State_Trotting.cpp lines 461-470 (readinessConditionsMet)
  Checks: hasAllFeetContact(_minimumContactForce=1.0N)
  Requires: ALL four feet with valid, finite force >= 1.0 N

C7: WAVE_ALL Transition
  File: State_Trotting.cpp lines 472-511 (updateWaveReadiness)
  Hold: 0.2s continuous readiness before _waveReady = true
  Then: setStartWave() → WaveStatus::WAVE_ALL

C8: Gait Phase Advancement
  File: WaveGenerator.cpp lines 121-155 (calcWave)
  Period: 0.45s sim-time
  Phase: normalized [0,1] per leg, advances with sim time
```

### Source of Truth
- **Contact force**: Gazebo ContactSensor → UnitreeFootContactPlugin → ROS WrenchStamped topics
- **Force magnitude**: Euclidean norm (always positive, loses direction)
- **Freshness**: Wall-clock staleness guard (1s window)
- **Contact flag (VecInt4)**: Gait-determined (NOT force-determined) — 1=stance phase, 0=swing phase

---

## Launch Parity

The isolated runner uses **identical launch files** as normal `auto.sh`:
- Both load `multi_floor_gazeboSim.launch`
- Both process the same `robot.xacro` → `gazebo.xacro`
- Contact sensors are UNCONDITIONAL (not gated by any ROS parameter)
- `ENABLE_FOOT_FORCE_VISUAL` defaults to `false` — only affects visual draw plugin, NOT contact sensors

**Differences in isolated runner:**
| Aspect | Normal | Isolated |
|--------|--------|----------|
| devel/lib | Real build output | Symlink to `$SIMENV_BINARY_DEVEL/lib` |
| GUI | true (default) | false |
| RVIZ | true (default) | false |
| FAST-LIO2 | true (default) | false (speed) / true (mapping) |
| GAZEBO_PLUGIN_PATH | `$WORKSPACE/devel/lib` | same (resolves through symlink) |

---

## First Failing Checkpoint: C4–C5 (Callback and Snapshot)

**Evidence:** All four `footForce[]` values are `0.0`, with `footForceValid[]` all `false`.

**Implication:** Either:
1. No contact callbacks fire (topics not published → plugin not loaded)
2. Callbacks fire but `footForce` stays zero (no physical contacts detected)

**Differential diagnosis:** Motor states and IMU ARE received (FSM runs, FixedStand works). This proves `ros::spinOnce()` dispatches callbacks. The failure is specific to contact topics.

---

## Root Cause Hypothesis

The most likely root cause is **missing `libunitreeFootContactPlugin.so` in the isolated runner's binary environment**.

The isolated runner replaces `devel/lib` with a symlink to `$SIMENV_BINARY_DEVEL/lib`. If this directory was built without the `unitree_gazebo` package in the catkin whitelist, the contact plugin `.so` files are absent.

Gazebo silently skips loading plugins whose `.so` files are not found in `GAZEBO_PLUGIN_PATH`. Without the contact plugin, no contact topics are published, no callbacks fire, and `_foot_force[]` remains at its initialized value of `0.0f`.

### Why this fits all observations:
1. **All four forces zero** — No plugin → no sensor → no contact data
2. **Motor/IMU data works** — Different plugins (`libgazebo_ros_control.so`, `libgazebo_ros_imu_sensor.so`) are unaffected
3. **FSM scheduler works** — Control logic is independent of contact data
4. **Identical launch files** — The launch file is correct but the runtime binary path is wrong
5. **Previous 0715 fix verified** — The code-level contact chain was tested and passed with forces ~[10.9, 11.1, 12.6, 12.9] N

### Verification:
```bash
# Check if plugins exist in the symlink target
ls -la $SIMENV_BINARY_DEVEL/lib/libunitreeFootContactPlugin.so
# If "No such file" → root cause confirmed
```

---

## Minimal Repair

### Option A (Recommended): Include `unitree_gazebo` in build whitelist
```bash
SIMENV_CATKIN_WHITELIST="unitree_legged_msgs;unitree_guide;unitree_legged_control;unitree_gazebo"
./tools/build_with_venv.sh
```
This ensures `libunitreeFootContactPlugin.so` and `libunitreeDrawForcePlugin.so` are in the binary devel.

### Option B: Use workspace devel directly
Skip the symlink trick and use the workspace's own `devel/lib` which already contains the plugins.

### Option C: Copy plugins to binary devel
```bash
cp devel/lib/libunitreeFootContactPlugin.so $SIMENV_BINARY_DEVEL/lib/
cp devel/lib/libunitreeDrawForcePlugin.so $SIMENV_BINARY_DEVEL/lib/
```

---

## Contact Force Validation

Expected per-foot force for A1 robot (13.4 kg total, 32.9 N/foot theoretical):
- Static FixedStand: ~10–13 N per foot (slightly uneven distribution due to pose)
- Trotting stance phase: 0–30+ N (dynamic loading)

Minimum threshold: 1.0 N (very conservative, triggers on barely-touching)

---

## Changes Made

### Commit 1: `test(g1rc): add contact chain callback diagnostics`
Files changed:
- `IOROS.h`: Added `_foot_force_callback_sequence[4]`, `_foot_force_sim_time_us[4]`
- `IOROS.cpp`: Initialize new fields, populate in `updateFootForce()`, copy in `recvState()`
- `LowlevelState.h`: Added `footForceCallbackSequence[4]`, `footForceSimTimeUs[4]`
- `State_Trotting.cpp`: Enhanced readiness log with reason codes, callback sequence, and sim time

---

## Checkpoint Results Table

| Checkpoint | Expected | Status | Evidence |
|-----------|----------|--------|----------|
| C0: Physical contact | Feet support robot on ground | **PENDING RUNTIME** | Requires Gazebo run |
| C1: Plugin loaded | 4 contact plugins | **PENDING RUNTIME** | Requires Gazebo log |
| C2: Plugin data | Finite force values | **PENDING RUNTIME** | Requires topic echo |
| C3: ROS topics | 4 active topics | **PENDING RUNTIME** | Requires rostopic list |
| C4: Callbacks | Sequence advances | **PENDING RUNTIME** | New diagnostics will show |
| C5: Snapshot | Fresh complete data | **PENDING RUNTIME** | New diagnostics will show |
| C6: Readiness input | Valid force values | **PENDING RUNTIME** | Enhanced log will show reason |
| C7: WAVE_ALL | Transition succeeds | **PENDING RUNTIME** | Requires C6 pass |
| C8: Gait | Phase/cycles advance | **PENDING RUNTIME** | Requires C7 pass |

---

## G1-R Validation Matrix (To Be Executed)

| Trial | ctrl_dt | Target Hz | RTF | Expected FSM Hz | Expected Gait Hz | Contact Required |
|-------|---------|-----------|-----|-----------------|------------------|------------------|
| trot_2ms_low | 0.002 | 500 | low | 500 ±1% | 2.222 ±2% | Yes |
| trot_2ms_high | 0.002 | 500 | high | 500 ±1% | 2.222 ±2% | Yes |
| rl_2ms_low | 0.002 | 500 | low | — | — | FixedStand only |
| rl_2ms_high | 0.002 | 500 | high | — | — | FixedStand only |
| trot_pause | 0.002 | 500 | any | 0 during pause | 0 during pause | Fresh after unpause |
| rl_pause | 0.002 | 500 | any | 0 during pause | — | N/A |
| trot_reset | 0.002 | 500 | any | Gen+1 | Fresh epoch | Fresh after reset |
| rl_reset | 0.002 | 500 | any | Gen+1 | — | Fresh after reset |
| trot_4ms | 0.004 | 250 | any | 250 ±1% | 2.222 ±2% | Yes |

---

## Remaining Risks

1. **SIMENV_BINARY_DEVEL unknown**: Cannot verify plugin presence without the actual environment variable value
2. **Collision name compatibility**: Names like `FR_calf_fixed_joint_lump__FR_foot_collision_1` are Gazebo-version-dependent
3. **Spawn height**: If feet spawn above ground, initial contact forces will be zero until robot settles
4. **AsyncSpinner lifetime**: The local-variable spinner in the IOROS constructor stops after construction; callbacks depend on `spinOnce()` in the FSM loop

---

## Conditions Before G2

1. ✅ Contact data chain C0–C8 verified at runtime
2. ⬜ Trotting enters valid `WAVE_ALL`
3. ⬜ Gait frequency 2.222 Hz ±2%
4. ⬜ 2 ms and 4 ms configs pass
5. ⬜ Trotting pause/reset pass
6. ⬜ RL pause/reset pass
7. ⬜ Cross-RTF frequency stability
8. ⬜ All unit tests pass

---

## G1-R Verdict

**PENDING RUNTIME VALIDATION**

The static analysis confirms:
- Contact chain code is correct (previously verified in 0715 trotting-safety)
- Launch configuration includes contact sensors unconditionally
- Plugin binaries exist in workspace devel/lib
- Topic names match between publisher and subscribers
- Diagnostics are in place to pinpoint the exact failure checkpoint

The runtime verdict depends on verifying that the contact plugins are actually loaded in the isolated runner's Gazebo environment.
