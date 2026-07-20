# ADR-011: G2 Fast Exit and RL Entry

## Status

Accepted on 2026-07-20.

## Context

Gate V found no fall-validator semantic defect. Gate P diagnostics are
available, but the project needs a fast decision on whether the current failure
is Trotting-specific enough to pause the full G2 baseline and enter RL
deployment validation.

## Decision

Run a minimum Gate A sequence with P0 FixedStand-only first. If P0 fails, stop
Trotting runtime attribution and authorize only RL shadow/static validation.

## Why the Full G2 Speed Matrix Is Deferred

The full G2 matrix would measure performance after a stable shared locomotion
base exists. P0 failed before FixedStand, so additional Trotting speeds would
not distinguish Trotting gait performance from shared startup/contact/FSM
failure.

## Deferred G2 Items

- `vx=0.30` and `vx=0.50`
- Three trials per speed
- Velocity tracking, stop, drift, and gait-cycle performance scoring
- Trotting recovery/fix branch

## Required Shared-Base Conditions

- Four-foot contact evidence is present.
- `data=2` reaches and sustains FixedStand.
- Model/base height stays above the safety line.
- LowCmd, joint feedback, estimator inputs, and timing records remain finite.

## Trotting-Specific Failures That Do Not Block RL Shadow

After P0 PASS, failures at Trotting enter, STANCE_ALL output, readiness/wave
request, or gait-only execution may block G2 performance but still allow
shadow-only RL observation/history/action validation.

This gate did not reach that case because P0 failed.

## RL Shadow and Active Boundary

Shadow validation may construct observations, update history, run inference,
and inspect candidate action/LowCmd without applying RL action to the final
LowCmd.

Active validation applies policy output to the robot. Active validation is not
authorized after P0 failure.

## Active-RL Stop Conditions

Active RL is blocked by any of:

- P0 shared-base failure
- non-finite candidate/final LowCmd
- unsafe height or tilt
- contact loss
- joint target jump, limit violation, or sustained saturation
- observation/history/action contract mismatch

## Remaining G2 Debt Tracking

The G2 performance baseline remains deferred in the G2 work log, risk register,
and evidence index. A future branch must first recover and revalidate P0 before
resuming P1/P2/P3 or the full speed matrix.
