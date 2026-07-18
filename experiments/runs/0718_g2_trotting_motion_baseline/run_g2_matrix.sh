#!/usr/bin/env bash
set -euo pipefail

RUN_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SPEEDS=(0.00 0.10 0.30 0.50)
EPOCHS="${EPOCHS:-3}"

for speed in "${SPEEDS[@]}"; do
  tag="$(printf 'vx_%03d' "$(awk "BEGIN {printf \"%d\", ${speed} * 100}")")"
  for epoch in $(seq 1 "$EPOCHS"); do
    run_id="${tag}_run_$(printf '%02d' "$epoch")"
    echo "=== G2 trial $run_id speed=$speed ==="
    COMMAND_VX="$speed" VX_TAG="$tag" RUN_ID="$run_id" bash "$RUN_ROOT/run_g2_trial.sh" || true
  done
done

/usr/bin/python3 "$RUN_ROOT/summarize_g2.py" --root "$RUN_ROOT"
