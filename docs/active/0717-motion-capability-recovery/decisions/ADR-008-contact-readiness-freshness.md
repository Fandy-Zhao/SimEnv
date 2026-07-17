# ADR-008: Contact Readiness Freshness

## Status
Accepted

## Context
The Trotting readiness gate requires fresh contact force data before allowing transition to `WAVE_ALL`. "Fresh" must be defined in a way that is correct under variable simulation RTF, Gazebo pause/resume, and controller reset.

## Decision

### Freshness Definition
Contact data is considered **fresh** when:
1. At least one callback has been received (`_foot_force_callback_sequence[i] > 0`)
2. The wall-clock time since the last callback is ≤ 1 second
3. The force magnitude is finite and ≥ `_minimumContactForce` (1.0 N default)

### Wall Clock vs Sim Time
- **Freshness uses wall-clock time**: `(WallTime::now() - last_callback_wall_time) <= 1 second`
- **Rationale**: The contact sensor plugin publishes at 100 Hz sim-time rate. At any RTF ≥ 0.01, this produces at least one message per wall-clock second. A 1-second wall-clock window is conservative and avoids false staleness.
- **Pause safety**: During Gazebo pause, `FSM::updateControlTime()` returns false, so the readiness check (which evaluates freshness) is not reached. This prevents false staleness during pause.
- **Sim-time age** is tracked diagnostically (`footForceSimTimeUs`) but not used for gating, because the plugin's `header.stamp` is set by Gazebo and may not be reliable in all configurations.

### Reset Behavior
On controller reset:
1. `controlResetGeneration` increments
2. Previous `footForceValid` flags become stale (new epoch)
3. The FSM's `updateControlTime()` detects the generation change and resets
4. Readiness must be re-established with fresh contact data from the new epoch

### Pause Behavior
During pause:
1. Sim time stops advancing → `updateControlTime()` returns false
2. FSM logic (including readiness evaluation) is skipped
3. Contact callback wall-time stamps continue aging
4. On resume, the first FSM iteration calls `recvStateOnly()` which evaluates freshness
5. If the pause was longer than 1 wall-second, `footForceValid` will be false
6. This is correct behavior: the controller should re-verify contact before resuming gait

## Consequences
- A 1-second wall-clock staleness window is generous; it would only false-trigger at RTF < 0.01 (far below normal operating range)
- Wall-clock freshness means contact data can go stale during long pauses, requiring re-verification — this is a safety feature
- Per-foot callback sequence counters enable precise diagnostics of which foot's data is missing
