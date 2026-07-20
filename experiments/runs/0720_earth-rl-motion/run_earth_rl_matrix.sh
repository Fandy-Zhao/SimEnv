#!/usr/bin/env bash
set -euo pipefail

RUN_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

run_trial() {
  local trial_id="$1"
  local state="$2"
  local vx="$3"
  local duration="$4"
  local stop_duration="${5:-0.0}"
  TRIAL_ID="$trial_id" \
  TRIAL_STATE="$state" \
  COMMAND_VX="$vx" \
  TRIAL_DURATION="$duration" \
  STOP_DURATION="$stop_duration" \
    "$RUN_ROOT/run_earth_rl_trial.sh"
}

run_trial E0_fixedstand fixedstand 0.00 15.0
run_trial E1_rl_zero rl 0.00 15.0
run_trial E2_rl_vx005 rl 0.05 15.0
run_trial E3_rl_vx010 rl 0.10 20.0
run_trial E5_rl_stop rl 0.10 8.0 10.0
