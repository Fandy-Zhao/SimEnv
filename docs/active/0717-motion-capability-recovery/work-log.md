# Work Log: 0717 Motion Capability Recovery

## 2026-07-18: G1-RC Contact Readiness Investigation

### Branch
`fix/0717-g1r-contact-readiness` (to be created from `integrate/0718-g1-fixed-sim-scheduler`)

### Context
G1-R validation at baseline `05d69a8a` returned `G1_R_FAIL` because Trotting never entered `WAVE_ALL`. The readiness log showed `contact=0 force=[0.0 0.0 0.0 0.0]N`, indicating the contact force data chain was broken.

### Static Analysis Findings

1. **Contact sensor plugins** (`libunitreeFootContactPlugin.so`): Always loaded unconditionally in `gazebo.xacro` (lines 86-121). Built by `unitree_gazebo/CMakeLists.txt`. Present in `devel/lib/`.

2. **Topic chain**: Gazebo contact sensor → `/visual/{FR,FL,RR,RL}_foot_contact/the_force` (WrenchStamped) → IOROS::updateFootForce() → `_foot_force[]` (atomic float) → recvState() → LowlevelState.footForce[] → State_Trotting readiness check.

3. **Previous fix (0715)**: The IOROS contact force consumption was added and verified working with forces ~[10.9, 11.1, 12.6, 12.9] N. The leg.xacro duplicate joint bug was fixed.

4. **Launch parity**: Isolated runner uses identical launch files as normal auto.sh. Differences: binary isolation via devel symlink, env vars (GUI=false, etc.).

5. **Most likely root cause**: The isolated runner's `$SIMENV_BINARY_DEVEL/lib` symlink target may not include `libunitreeFootContactPlugin.so`, causing Gazebo to silently skip loading the contact sensors.

### Action Plan
1. Add contact chain diagnostics to IOROS and State_Trotting
2. Rebuild with proper whitelist including `unitree_gazebo`
3. Verify plugins are in devel/lib
4. Run smoke test with runtime topic probe
5. Complete G1-R matrix
6. Publish report

### Files to modify
- `src/unitree_guide/unitree_guide/unitree_guide/src/interface/IOROS.cpp` (diagnostics)
- `src/unitree_guide/unitree_guide/unitree_guide/include/interface/IOROS.h` (diagnostics)
- `src/unitree_guide/unitree_guide/unitree_guide/src/FSM/State_Trotting.cpp` (enhanced readiness logging)
- `src/unitree_guide/unitree_guide/unitree_guide/include/message/LowlevelState.h` (contact age tracking)

### Build configuration
- `SIMENV_CATKIN_WHITELIST="unitree_legged_msgs;unitree_guide;unitree_legged_control;unitree_gazebo"`
- `CC=/usr/bin/gcc-11 CXX=/usr/bin/g++-11 CUDAHOSTCXX=/usr/bin/g++-11`
- `UNITREE_ENABLE_TORCH_POLICY=ON`

## 2026-07-18 02:00: G1-RC → Integration Merge

### Merge
- Source: `fix/0717-g1r-contact-readiness` (`7b4abaef`)
- Target: `integrate/0718-g1-fixed-sim-scheduler` (`c77278da`)
- Merge commit: `e7c130a1` — `merge: integrate G1 contact readiness diagnostics`
- Strategy: `--no-ff`, no conflicts

### Files merged (12 files, +672/-2)
- Contact chain diagnostics in IOROS.h, IOROS.cpp, LowlevelState.h, State_Trotting.cpp
- G1-RC report, work-log, risk-register, evidence-index
- 3 ADRs (contact force source of truth, readiness freshness, runtime matrix)
- Contact probe script

### Next
- Rebuild binary devel with `unitree_gazebo` in whitelist
- Runtime contact probe (C0-C8)
- Full G1 runtime matrix

## 2026-07-18 02:09: Binary Devel Rebuild

### Build
- Compiler: gcc-11/g++-11, CUDA 11.8
- Whitelist: `unitree_legged_msgs;unitree_guide;unitree_legged_control;unitree_gazebo`
- Torch: enabled (`UNITREE_ENABLE_TORCH_POLICY=ON`)
- Result: PASS (0 errors, warnings only in QuadProg++ register specifiers)

### Plugin Verification
- `libunitreeFootContactPlugin.so`: 348256 bytes, SHA256 `8b4dee0e...`
- `libunitreeDrawForcePlugin.so`: 386112 bytes
- `libunitree_legged_control.so`: present
- All dependencies resolve (no missing .so)

### Unit Tests
- `catkin_make run_tests_unitree_guide_gtest_timing_alignment_test`: 13/13 PASSED

## 2026-07-18 02:16: Contact Probe Runtime Results

### C0-C4: PASS

| Checkpoint | Result | Key Evidence |
|-----------|--------|-------------|
| C0: Physical contact | PASS | Robot pose z=0.060m, on ground |
| C1: Plugin loaded | PASS | All 4 topics published by /gazebo |
| C2: Finite forces | PASS | FR=9.64N, FL=9.64N, RR=2.84N, RL=2.74N |
| C3: 4 topics exist | PASS | All at 100 Hz, type geometry_msgs/WrenchStamped |
| C4: IOROS subscribes | PASS | /unitree_gazebo_servo subscribed to all 4 topics |

### C5-C8: PENDING
- Torch model loaded: "load model is successed!"
- All forces > 1.0N minimum threshold
- FSM state command to Trotting sent but state transition not confirmed
- Requires dedicated runtime session with proper FSM state management

### Root Cause Resolution
The primary root cause (missing `libunitreeFootContactPlugin.so` in isolated runner's binary devel) is **RESOLVED**. Using the workspace devel with `unitree_gazebo` in the build whitelist ensures all contact plugins are available.

## 2026-07-18: G1 Final Report

### Verdict: G1_R_INCONCLUSIVE (RUNTIME)
- C0-C4 (contact chain): RUNTIME PASS
- C5-C8 (Trotting readiness): PENDING — all prerequisites met but full verification requires dedicated runtime session
- Contact plugin root cause RESOLVED
- Unit tests: 13/13 PASSED
- Build: PASS

### Next
- Complete C5-C8 in dedicated runtime session
- Run full G1 runtime matrix (9 trials)
- Merge to master after G1_R_PASS
- Proceed to G2 baseline measurement

## 2026-07-18 14:23: G1-F FSM Command Chain Investigation

### Branch
`fix/0718-g1-fsm-command-chain` from `integrate/0718-g1-fixed-sim-scheduler`

### Static Audit
Complete call chain traced and verified:
- `/fsm/state_cmd` subscriber at main.cpp:183
- Maps `data=4` → `UserCommand::START` (main.cpp:175)
- Latch `pendingStateCmd` in CtrlComponents.h:86
- FSM::run() applies after recvStateOnly() (FSM.cpp:99-101)
- State_FixedStand::checkChange() maps START → TROTTING (FixedStand.cpp:174-175)
- `UNITREE_DISABLE_TORCH_POLICY` NOT defined → all Torch code active

### Root Cause
**Controller starts in PASSIVE (state=1).** From PASSIVE, `data=4` (START) is silently ignored — only `data=2` (L2_A→FixedStand) triggers transition. The previous probe sent `data=4` while still in PASSIVE.

### Fix
**No code fix needed.** Procedural: send `data=2` before `data=4`.

### F0-F7 Runtime Results

| Checkpoint | Result | Evidence |
|-----------|--------|----------|
| F0 topic | **PASS** | `/fsm/state_cmd` exists |
| F1 subscriber | **PASS** | `/unitree_gazebo_servo` subscribed |
| F2 callback | **PASS** | seq=1,49,77,88 observed |
| F3 mapping | **PASS** | raw=4→mapped=1 (START) |
| F4 arbitration | **PASS** | ROS overrides keyboard NONE |
| F5 guard | **PASS** | FixedStand→Trotting accepted |
| F6 transition | **PASS** | "Switched from fixed stand to trotting" |
| F7 target | **PASS** | "Trotting entry: inherited body height" |

### C5-C8 Runtime Results

| Checkpoint | Result | Evidence |
|-----------|--------|----------|
| C5 snapshot | **PASS** | force=[8.9 9.0 11.2 11.6]N, callback seq > 0 |
| C6 readiness | **PASS** | "wave ready after 0.20 s" |
| C7 WAVE_ALL | **PASS** | "wave started after height, stance, and contact readiness" |
| C8 gait | **PRELIMINARY** | FSM state=4 (TROTTING) sustained; full freq measurement pending |

### Commits
```
18c75a11 docs(g1f): publish FSM command chain recovery report
23d97ec6 test(g1f): add FSM command chain diagnostics and arbitration ADR
```

### Updated G1 Status
- C0-C8: C0-C7 PASS, C8 preliminary pass
- F0-F7: ALL PASS
- Timing tests: 13/13 PASSED
- Full G1 runtime matrix: PENDING (9 trials)
- Verdict: approaching G1_R_PASS, blocked by full matrix execution
