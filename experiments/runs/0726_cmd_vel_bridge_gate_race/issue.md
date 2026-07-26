# Issue: cmd_vel_bridge Gate Initialization Race

## Task Goal
Fix the race condition where `cmd_vel_bridge` fails to open its safety gate,
preventing FALCO commands from reaching `/cmd_vel` and the robot.

## Root Cause Analysis (Confirmed)

### Architecture
The navigation state management has a request/response design:

```
External User/auto.sh
    → /navigation/request_enabled    (std_msgs/Bool)   → nav_state_supervisor
    → /navigation/request_fsm_state  (std_msgs/Int8)    → nav_state_supervisor
    → /navigation/request_exploring  (std_msgs/Bool)    → nav_state_supervisor

nav_state_supervisor
    → /navigation/enabled            (std_msgs/Bool, latch) → cmd_vel_bridge
    → /fsm/state_cmd                 (std_msgs/Int8, latch) → cmd_vel_bridge, unitree controller
    → /navigation/start_exploring    (std_msgs/Bool, latch)
        + periodic republish at 1 Hz
```

### The Bug

`auto.sh` publishes state transitions directly to the **OUTPUT** topics,
NOT to the **REQUEST** topics:

```bash
# auto.sh line 970-971: Safe defaults — published to OUTPUT topics
rostopic pub /navigation/enabled       std_msgs/Bool  "data: false" -1
rostopic pub /navigation/start_exploring std_msgs/Bool "data: false" -1

# auto.sh line 981: Trotting command — published to OUTPUT topic
rostopic pub /fsm/state_cmd std_msgs/Int8 "data: 4" -1

# auto.sh line 986: Navigation enable — published to OUTPUT topic
rostopic pub /navigation/enabled std_msgs/Bool "data: true" -1
```

Meanwhile, `nav_state_supervisor` subscribes to `/navigation/request_enabled`,
`/navigation/request_fsm_state`, and `/navigation/request_exploring` —
topics that `auto.sh` **never publishes to**.

### Consequence Chain

1. Supervisor starts with defaults: enabled=False, fsm=2
2. auto.sh publishes `/navigation/enabled=true` and `/fsm/state_cmd=4` as one-shots
3. Bridge briefly sees correct state → but then:
4. **Supervisor's periodic 1 Hz republish sends (enabled=False, fsm=2)**
5. Bridge receives stale state → gate closes again
6. Supervisor's latched values remain (enabled=False, fsm=2)
7. Any bridge restart gets the wrong latched state → gate stays closed

### Evidence
- /navigation/falco/cmd_vel_stamped published at ~50 Hz (PASS)
- cmd_vel_bridge receives FALCO commands (PASS)
- Bridge gate condition: `enabled=T, trotting=T, fresh=T` → FAIL
- /cmd_vel remains all zeros (FAIL)
- Direct rostopic pub to /cmd_vel makes robot turn (PASS — confirms robot OK)

## Fix Strategy

1. **auto.sh**: Publish state changes to the REQUEST topics so the supervisor
   tracks correct state. Keep direct publishes to output topics as well for
   belt-and-suspenders (direct controller commanding).

2. **cmd_vel_bridge.py**: Add diagnostic logging for gate state transitions
   to make future race conditions visible.

3. **nav_state_supervisor.py**: Minor hardening of initial publish timing.

## Modification Scope
- `auto.sh`: navigation state publishing section (lines ~969-992)
- `src/navigation/simenv_navigation_bridge/scripts/cmd_vel_bridge.py`: add gate state logging
- `src/navigation/simenv_navigation_bridge/scripts/nav_state_supervisor.py`: shorten initial fsm republish

## Non-Scope
- No permanent gate bypass
- No hardcoded enabled=true or fsm=4
- No modification of safety semantics
- No modification of unitree controller or FAST-LIO2
- No modification of DSV or FALCO

## Acceptance Criteria
1. Bridge gate opens without manual rostopic pub
2. Bridge works when started before OR after supervisor
3. Bridge restart recovers current gate state
4. Navigation disable or FSM!=4 reliably closes gate
5. 3 independent cold starts all pass
6. Robot actually moves (yaw changes, not just joint vibration)
