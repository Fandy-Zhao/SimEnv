#!/usr/bin/env python3
"""Aggregate G2 trial_status.json files into compact CSV/JSON summaries."""

from __future__ import annotations

import argparse
import csv
import json
import os
import statistics
from typing import Dict, List


FIELDS = [
    "trial_id",
    "command_vx",
    "trial_result",
    "invalid_reasons",
    "steady_mean_vx",
    "steady_median_vx",
    "absolute_tracking_error",
    "tracking_ratio",
    "lateral_drift_ratio",
    "yaw_drift_deg",
    "stop_time_to_0_05",
    "tail_speed_mean",
    "truth_samples",
    "timing_rows",
]


def load_trials(root: str) -> List[Dict[str, object]]:
    trials = []
    baseline = os.path.join(root, "baseline")
    for dirpath, _, filenames in os.walk(baseline):
        if "trial_status.json" not in filenames:
            continue
        with open(os.path.join(dirpath, "trial_status.json")) as handle:
            trial = json.load(handle)
        trial["_path"] = dirpath
        trials.append(trial)
    return sorted(trials, key=lambda item: str(item.get("trial_id", "")))


def median(values):
    finite = [float(value) for value in values if value is not None]
    return statistics.median(finite) if finite else None


def speed_summary(trials: List[Dict[str, object]]) -> Dict[str, object]:
    by_speed: Dict[str, List[Dict[str, object]]] = {}
    for trial in trials:
        by_speed.setdefault("%.2f" % float(trial["command_vx"]), []).append(trial)
    summary = {}
    for speed, rows in sorted(by_speed.items()):
        valid = [row for row in rows if row.get("trial_result") in ("PASS", "FAIL")]
        summary[speed] = {
            "total_runs": len(rows),
            "valid_runs": len(valid),
            "invalid_runs": len(rows) - len(valid),
            "median_vx": median(row.get("steady_median_vx") for row in valid),
            "median_tracking_ratio": median(row.get("tracking_ratio") for row in valid),
            "median_drift": median(row.get("lateral_drift_ratio") for row in valid),
            "median_stop_time_to_0_05": median(row.get("stop_time_to_0_05") for row in valid),
            "result": "INCONCLUSIVE" if len(valid) < 3 else "PENDING_REVIEW",
        }
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True)
    args = parser.parse_args()
    trials = load_trials(args.root)
    with open(os.path.join(args.root, "summary.csv"), "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS, lineterminator="\n")
        writer.writeheader()
        for trial in trials:
            row = {field: trial.get(field) for field in FIELDS}
            if isinstance(row.get("invalid_reasons"), list):
                row["invalid_reasons"] = ";".join(row["invalid_reasons"])
            writer.writerow(row)
    output = {"schema_version": 1, "trial_count": len(trials), "speeds": speed_summary(trials)}
    with open(os.path.join(args.root, "summary.json"), "w") as handle:
        json.dump(output, handle, indent=2, sort_keys=True)
    print(json.dumps(output, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
