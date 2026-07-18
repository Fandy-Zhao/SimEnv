# ADR-001: G2 Runtime Configuration

**Status:** Accepted
**Date:** 2026-07-18

## Context
G2-B requires a fixed runtime configuration to ensure reproducible baseline trials. Key parameters: sim-time windows per speed, epoch count, RTF threshold, data recording rates.

## Decision
- Sim-time windows: 15s for vx=0.00, 30s for vx=0.10/0.30/0.50.
- 3 epochs per speed.
- RTF flag threshold: 0.5 (flag, do not reject).
- Ground truth recording: 50 Hz.
- Controller state and contact: 500 Hz.
- Fresh spawn per trial (no pause/reset reuse).

## Rationale
- 15s for stationary baseline is sufficient to confirm zero-velocity stability.
- 30s at commanded speeds provides ≥20s of steady-state data after settling.
- 3 epochs balances statistical robustness against wall-clock cost.
- 0.5 RTF threshold from G1 timing validation experience.
- Fresh spawn avoids G1 pause/reset unverified matrix risk (R02).

## Consequences
- Wall-clock runtime per speed: ~30s × 3 epochs / RTF. At RTF=0.5, ~3 min/speed.
- Full matrix: ~12 min wall-clock minimum. Lower RTF extends linearly.
