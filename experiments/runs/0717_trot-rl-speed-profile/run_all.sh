#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec 9>/tmp/simenv-gazebo.lock
flock 9
for mode in trotting rl; do
  for speed in 0.1 0.5 1.0; do
    MODE="$mode" SPEED="$speed" "$ROOT/run_speed_trial.sh"
  done
done
/usr/bin/python3 "$ROOT/plot_speed_profile.py"
