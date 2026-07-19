#!/usr/bin/env python3
"""Freeze existing G2-B pose evidence and reclassify fall semantics offline."""

from __future__ import annotations

import argparse
import csv
import json
import os
from pathlib import Path
from typing import Dict, List

import g2_validator_semantics as validator


TRIALS = [
    ("vx_000", "vx_000_run_01"),
    ("vx_010", "vx_010_run_01"),
    ("vx_030", "vx_030_run_01"),
    ("vx_050", "vx_050_run_01"),
]


def read_csv(path: Path) -> List[Dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def read_json(path: Path) -> Dict[str, object]:
    with path.open() as handle:
        return json.load(handle)


def write_csv(path: Path, rows: List[Dict[str, object]]) -> None:
    fieldnames = sorted({key for row in rows for key in row})
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def event_time(rows: List[Dict[str, str]], event: str) -> float | None:
    for row in rows:
        if row.get("event") == event:
            return float(row["sim_time"])
    return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline-root", required=True, help="Path to existing baseline/ directory")
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()

    baseline_root = Path(args.baseline_root)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    offline_dir = output_dir / "offline_reclassification"
    offline_dir.mkdir(parents=True, exist_ok=True)

    pose_rows: List[Dict[str, object]] = []
    timeline: Dict[str, object] = {"schema_version": 1, "trials": {}}
    reclass_rows: List[Dict[str, object]] = []
    reclass_json: Dict[str, object] = {"schema_version": 1, "trials": []}

    for speed, trial_id in TRIALS:
        trial_dir = baseline_root / speed / trial_id
        truth = read_csv(trial_dir / "ground_truth.csv")
        events = read_csv(trial_dir / "events.csv")
        status = read_json(trial_dir / "trial_status.json")
        samples = validator.load_pose_rows(truth)
        fall = validator.summarize_fall(samples)
        first_legacy_row = next((row for row in truth if float(row["sim_time"]) == fall.first_legacy_fall_time), None)
        first_semantic_row = next((row for row in truth if float(row["sim_time"]) == fall.first_semantic_fall_time), None)
        old_reasons = list(status.get("invalid_reasons", []))
        remaining_reasons = [reason for reason in old_reasons if reason != "FALL_DETECTED"]
        if fall.semantic_fall and "FALL_DETECTED" not in remaining_reasons:
            remaining_reasons.append("FALL_DETECTED")
        remaining_reasons = sorted(set(remaining_reasons))
        reclassified_status = "PASS" if not remaining_reasons else "INVALID"

        pose_rows.append(
            {
                "trial": trial_id,
                "speed": speed,
                "truth_samples": len(samples),
                "min_model_z": fall.min_z,
                "max_body_tilt_deg": fall.max_tilt_deg,
                "first_legacy_fall_time": fall.first_legacy_fall_time,
                "first_semantic_fall_time": fall.first_semantic_fall_time,
                "fixedstand_enter_time": event_time(events, "FIXED_STAND_ENTERED"),
                "trotting_enter_time": event_time(events, "TROTTING_ENTERED"),
                "legacy_height_fall": fall.legacy_height_fall,
                "semantic_fall": fall.semantic_fall,
                "old_invalid_reasons": ";".join(old_reasons),
                "new_invalid_reasons": ";".join(remaining_reasons),
            }
        )
        timeline["trials"][trial_id] = {
            "speed": speed,
            "events": {
                "fixedstand_enter_time": event_time(events, "FIXED_STAND_ENTERED"),
                "trotting_enter_time": event_time(events, "TROTTING_ENTERED"),
                "legacy_fall_time": fall.first_legacy_fall_time,
                "semantic_fall_time": fall.first_semantic_fall_time,
            },
            "first_legacy_fall_pose": first_legacy_row,
            "first_semantic_fall_pose": first_semantic_row,
            "old_invalid_reasons": old_reasons,
            "new_invalid_reasons": remaining_reasons,
        }
        reclass_row = {
            "trial": trial_id,
            "old_fall_result": "FALL_DETECTED" in old_reasons,
            "new_fall_result": fall.semantic_fall,
            "remaining_invalid_reasons": ";".join(remaining_reasons),
            "reclassified_status": reclassified_status,
        }
        reclass_rows.append(reclass_row)
        reclass_json["trials"].append(reclass_row)

    write_csv(output_dir / "existing_trial_pose_summary.csv", pose_rows)
    with (output_dir / "existing_trial_fall_timeline.json").open("w") as handle:
        json.dump(timeline, handle, indent=2, sort_keys=True)
    write_csv(offline_dir / "offline_reclassification.csv", reclass_rows)
    with (offline_dir / "offline_reclassification.json").open("w") as handle:
        json.dump(reclass_json, handle, indent=2, sort_keys=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
