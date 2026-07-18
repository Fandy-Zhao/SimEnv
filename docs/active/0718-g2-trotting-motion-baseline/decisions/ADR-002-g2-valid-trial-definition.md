# ADR-002: G2 Valid Trial Definition

**Status:** Accepted
**Date:** 2026-07-18

## Context
Trials may fail for reasons unrelated to motion tracking (contact loss, FSM regression, RTF collapse). A validity gate prevents contaminated data from entering metric computation.

## Decision
A trial is valid iff ALL of:
1. Robot spawns normally and data topics remain available.
2. Four foot-force topics exist, callback sequences advance, and force data stays fresh.
3. FSM enters FixedStand.
4. FSM enters Trotting.
5. Wave status enters `WAVE_ALL`.
6. Phase values change and gait-cycle sequence advances.
7. No sim-time reset, pause, controller restart, or robot fall occurs.
8. Required ground-truth, timing, contact, event, status, and metrics files are complete.
9. RTF is recorded and flagged when low; RTF alone does not invalidate the trial.

## Rationale
- Contact readiness, FSM state, WAVE_ALL, and phase/gait advancement prove that the trial actually exercised Trotting.
- Sim-time reset, pause, controller restart, fall, or missing data breaks the window definitions and invalidates the trial.
- RTF is an operational concern unless it causes missing, stale, or corrupted data.

## Consequences
- A speed with fewer than 3 valid epochs is `INCONCLUSIVE`.
- Invalid trials are logged but not included in metric computation.
- Invalid trials use explicit reason codes such as `CONTACT_NOT_READY`, `FSM_TRANSITION_FAILED`, `WAVE_ALL_NOT_REACHED`, `GAIT_NOT_ADVANCING`, `FALL_DETECTED`, `DATA_INCOMPLETE`, `SIM_TIME_RESET`, or `CONTROLLER_EXITED`.
