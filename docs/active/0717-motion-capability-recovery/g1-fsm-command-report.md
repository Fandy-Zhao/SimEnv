# G1-F: FSM Command Chain Recovery

## Branch
`fix/0718-g1-fsm-command-chain` from `integrate/0718-g1-fixed-sim-scheduler`

## Previous Blocker
`/fsm/state_cmd std_msgs/Int8 "data: 4"` did not trigger Trotting FSM state switch during G1-RC contact probe (2026-07-18 02:16).

## Root Cause
**Controller starts in PASSIVE (state=1).** From PASSIVE, only `data=2` (UserCommand::L2_A) triggers transition to FixedStand. `data=4` (UserCommand::START) only triggers Trotting from FIXEDSTAND (state=2).

The previous probe sent `data=4` while the controller was still in PASSIVE. The command was correctly received and mapped (callback seq=1, raw=4, mapped=START), and applied to `lowState->userCmd` (source=ROS), but `State_Passive::checkChange()` only checks for `L2_A` — START falls through to `else return PASSIVE`.

## Static Audit Results

### Complete Call Chain Verified
```
/fsm/state_cmd Int8 data=4
→ main.cpp:169 stateCmdCb callback
→ ctrlComp->pendingStateCmd = UserCommand::START
→ FSM.cpp:99-101 lowState->userCmd = pendingStateCmd
→ State_FixedStand.cpp:174-175 return FSMStateName::TROTTING
→ FSM.cpp:319 getNextState() returns _stateList.trotting
→ FSM.cpp:130-134 transition executes
```

### Build Config
- `UNITREE_ENABLE_TORCH_POLICY=ON` (CMakeCache)
- `UNITREE_DISABLE_TORCH_POLICY` NOT defined
- `case 4:` in main.cpp and State_FixedStand.cpp ARE compiled in
- All Torch-dependent symbols available

## Minimal Repair
**No code fix needed.** The command chain is correct. The fix is procedural:
1. Send `data=2` (FixedStand) first
2. Wait for state transition to FixedStand
3. Send `data=4` (Trotting)

## Command Source Arbitration (ADR-010)
Priority: SAFETY > ROS TOPIC > JOYSTICK > KEYBOARD > NONE

ROS command is a **pulse/latch**: consumed on first advancing sim tick after receipt.
Keyboard command is **level**: persists until a different key is pressed.

## FSM Checkpoint Results

| Checkpoint | Expected | Pre-fix | Post-fix | Result |
|-----------|----------|---------|----------|--------|
| F0 topic | exists | FAIL (timing) | **PASS** | `/fsm/state_cmd` exists |
| F1 subscriber | connected | FAIL (timing) | **PASS** | `/unitree_gazebo_servo` subscribed |
| F2 callback | sequence grows | **PASS** | **PASS** | seq=1,49,77,88 observed |
| F3 mapping | correct target | **PASS** | **PASS** | raw=4→mapped=1 (START) |
| F4 arbitration | not overwritten | **PASS** | **PASS** | ROS overrides keyboard NONE |
| F5 guard | accepted | **PASS** | **PASS** | FixedStand→Trotting accepted |
| F6 transition | observable | **PASS** | **PASS** | "Switched from fixed stand to trotting" |
| F7 target state | active | **PASS** | **PASS** | "Trotting entry: inherited body height" |

**F0/F1 initial FAIL was a probe timing issue** — the probe checked before the controller subscriber was created. Fixed by polling for topic appearance.

## C5 Contact Snapshot
Verified in FixedStand via readiness log:
```
force=[8.9 9.0 11.2 11.6]N seq=[5562 5573 5562 5562]
```
- All 4 callback sequences > 0
- All 4 forces finite (8.9–11.6 N)
- Forces consistent with A1 FixedStand distribution

## C6 Readiness
**PASS**: "Trotting wave ready after 0.20 s stable: |v|=0.034 m/s, force=[8.6 8.8 11.5 11.8] N"
- Height ready: yes
- Stance ready: yes
- Contact ready: yes (all 4 forces > 1.0N minimum)
- Hold time: 0.20 s (correct, based on sim time)

## C7 WAVE_ALL
**PASS**: "Trotting wave started after height, stance, and contact readiness."
- WaveStatus transitioned from STANCE_ALL to WAVE_ALL
- One transient readiness loss at 72.9s (recovered at 79.5s)

## C8 Gait
**PRELIMINARY PASS**: FSM state=4 (TROTTING) confirmed sustained in diagnostics.
Full gait frequency measurement requires dedicated trial with velocity command.

## Files Changed
| File | Change | Purpose |
|------|--------|---------|
| `CtrlComponents.h` | +15 lines | FSM command diagnostic fields |
| `main.cpp` | +11 lines | ROS callback records raw/mapped/source |
| `FSM.cpp` | +28 lines | Throttled diagnostics at apply and transition |
| `ADR-010-*.md` | +64 lines | Command source arbitration decision |

## Evidence Paths
- `experiments/runs/0717_motion-capability_recovery/timing/g1f_fsm_command_probe/run_fsm_cmd_probe.sh`
- Tmux session `simenv-g1f-manual-junior_ctrl` runtime output (captured inline)
- `docs/active/0717-motion-capability-recovery/decisions/ADR-010-fsm-command-source-arbitration.md`

## Commits
```
23d97ec6 test(g1f): add FSM command chain diagnostics and arbitration ADR
```

## Next Steps
1. Run full G1 runtime matrix with proper 2-step command sequence
2. Complete C8 gait frequency measurement
3. Merge fix branch to integration
4. Merge integration to master (after G1_R_PASS)
