# ADR-009: G1-R Runtime Validation Matrix

## Status
Accepted

## Context
G1-R validation requires verifying low-level timing, pause, reset, and multi-configuration behavior under effective Trotting gait. A minimum matrix of trials must pass before G2 (motion performance) can begin.

## Decision

### Required Trials

| # | Trial | ctrl_dt | Sim Time | Description |
|---|-------|---------|----------|-------------|
| 1 | trot_2ms_low_rtf | 0.002 | ≥25s | Trotting vx=0.1, low CPU load |
| 2 | trot_2ms_high_rtf | 0.002 | ≥25s | Trotting vx=0.1, high CPU load |
| 3 | rl_2ms_low_rtf | 0.002 | ≥25s | RL policy, low CPU load |
| 4 | rl_2ms_high_rtf | 0.002 | ≥25s | RL policy, high CPU load |
| 5 | trot_pause | 0.002 | ≥5s wall pause | Pause during WAVE_ALL, verify freeze |
| 6 | rl_pause | 0.002 | ≥5s wall pause | Pause during RL, verify freeze |
| 7 | trot_reset | 0.002 | Full reset cycle | Reset during WAVE_ALL, verify new epoch |
| 8 | rl_reset | 0.002 | Full reset cycle | Reset during RL, verify new epoch |
| 9 | trot_4ms | 0.004 | ≥25s | Trotting at 250 Hz control rate |

### Measurement Windows
- **Warm-up**: 5 s sim-time after FSM enters TROTTING
- **Measurement**: At least 20 s sim-time of continuous valid gait
- **Exclusion**: FixedStand transitions, contact warm-up, pause/reset intervals

### Acceptance Criteria

#### Frequency
- FSM Hz_sim = target ±1%
- Estimator Hz_sim = target ±1%
- Wave Hz_sim = target ±1%
- LowCmd Hz_sim = target ±1%
- RL Policy Hz = 50 ±1%
- Gait Hz = 2.222 ±2%

#### Cross-RTF Stability
- Same config, different RTF: frequency difference ≤ 1% (control) / ≤ 2% (gait)

#### Pause
- All stateful sequences: delta = 0 during pause
- Contact snapshot: not invalidated across pause boundary
- Phase: frozen at pause value

#### Reset
- One logical reset → generation +1
- Scheduler, Estimator, Wave, contact, RL synchronized to new epoch
- No stale snapshots, actions, or contact data
- No burst catch-up

#### Contact
- Four foot plugins loaded
- Four callbacks observable (sequence > 0)
- Forces finite and ≥ 1.0 N during stance
- Snapshot fresh at readiness evaluation
- Reset clears old data; new epoch data restores readiness

### Data Collection Per Trial
- `manifest.json`: trial parameters
- `rosparams.yaml`: ROS parameter snapshot
- `environment.txt`: env vars
- `git.txt`: git status
- `plugin_inventory.txt`: loaded Gazebo plugins
- `topic_inventory.txt`: rostopic list/info/hz
- `node_inventory.txt`: rosnode list/info
- `auto.log`: auto.sh output
- `gazebo.log`: gzserver output
- `capture.log`: rostopic echo samples
- `timing.csv`: FSM timing records
- `contact.csv`: per-foot contact force time series
- `event_summary.json`: key events (state transitions, readiness, wave start)
- `trial_metrics.json`: aggregated metrics
- `trial_status.json`: pass/fail verdict

## Consequences
- Nine trials provide comprehensive coverage of G1-R requirements
- Cross-RTF trials catch timing bugs that only manifest under load
- Pause/reset trials verify the scheduler's correctness under edge cases
- The matrix gates G2 entry: no G2 before all trials pass
