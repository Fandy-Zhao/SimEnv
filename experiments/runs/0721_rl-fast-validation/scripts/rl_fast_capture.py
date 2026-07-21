#!/usr/bin/env python3
"""Capture placeholder for RL fast validation.

Writes metrics.json, verdict.json, timeseries.csv, and summary.md for
preflight-only or blocked runs.  Imports rl_fast_metrics for pure helpers.
TASK_PARTIAL: live ROS subscribers and data collection are not implemented.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional

try:
    import rl_fast_metrics  # type: ignore[import-not-found]
    METRICS_AVAILABLE = True
except ImportError:
    METRICS_AVAILABLE = False
    print("[rl_fast_capture] WARN: rl_fast_metrics module not importable.", file=sys.stderr)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="RL fast validation capture placeholder")
    p.add_argument("--capture-dir", default=os.getcwd(), help="Output directory for capture artifacts")
    p.add_argument("--capture-label", default="rl_fast_smoke", help="Label prefix for artifact filenames")
    p.add_argument("--world-mode", default="earth", help="World mode: earth, competition, native, stairs")
    p.add_argument("--test-profile", default="smoke", help="Test profile: smoke, full, baseline")
    p.add_argument("--policy-path", default="", help="Path to RL policy .pt file")
    p.add_argument("--expected-policy-sha256", default="", help="Expected policy SHA256 hex digest")
    p.add_argument("--task-partial", default="false", help="Set to true to mark output as TASK_PARTIAL")
    return p.parse_args()


def write_timeseries_header(capture_dir: str, label: str, columns: List[str]) -> str:
    """Write an empty timeseries CSV with header and return the path."""
    path = os.path.join(capture_dir, "timeseries.csv")
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(columns)
    return path


def write_metrics_json(capture_dir: str, label: str, metrics: Dict[str, Any]) -> str:
    path = os.path.join(capture_dir, "metrics.json")
    with open(path, "w") as f:
        json.dump(metrics, f, indent=2, default=str)
    return path


def write_verdict_json(capture_dir: str, label: str, verdict: Dict[str, Any]) -> str:
    path = os.path.join(capture_dir, "verdict.json")
    with open(path, "w") as f:
        json.dump(verdict, f, indent=2)
    return path


def write_summary_md(capture_dir: str, label: str, fields: Dict[str, Any]) -> str:
    path = os.path.join(capture_dir, "summary.md")
    with open(path, "w") as f:
        f.write(f"# {label} Summary\n\n")
        f.write(f"**Verdict:** `{fields.get('verdict', 'UNKNOWN')}`\n\n")
        f.write(f"- World mode: {fields.get('world_mode', 'unknown')}\n")
        f.write(f"- Test profile: {fields.get('test_profile', 'unknown')}\n")
        f.write(f"- Timestamp: {fields.get('timestamp', 'unknown')}\n")
        f.write(f"- Policy path: {fields.get('policy_path', 'not set')}\n")
        f.write(f"- TASK_PARTIAL: {fields.get('task_partial', False)}\n")
        f.write(f"- Metrics available: {fields.get('metrics_available', False)}\n\n")
        if fields.get("notes"):
            f.write(f"**Notes:** {fields['notes']}\n")
    return path


def main() -> None:
    args = parse_args()
    os.makedirs(args.capture_dir, exist_ok=True)

    now = datetime.now(timezone.utc).isoformat()
    is_partial = args.task_partial.lower() in ("true", "1", "yes")
    verdict = "TASK_PARTIAL" if is_partial else "BLOCKED"

    timestamp_epoch = datetime.now(timezone.utc).timestamp()

    # Core metrics placeholder
    metrics: Dict[str, Any] = {
        "capture_label": args.capture_label,
        "timestamp": now,
        "timestamp_epoch": timestamp_epoch,
        "world_mode": args.world_mode,
        "test_profile": args.test_profile,
        "policy_path": args.policy_path,
        "task_partial": is_partial,
        "metrics_available": METRICS_AVAILABLE,
        "sample_count": 0,
        "verdict": verdict,
        "notes": "Preflight-only capture. Live ROS data collection is not implemented.",
    }

    # Enrich with pure metric helpers when available
    if METRICS_AVAILABLE and args.policy_path:
        valid, reason = rl_fast_metrics.validate_artifact_path(args.policy_path, require_readable=True)
        metrics["policy_valid"] = valid
        metrics["policy_validation_reason"] = reason
        if valid:
            computed = rl_fast_metrics.compute_sha256(args.policy_path)
            metrics["policy_sha256"] = computed
            if args.expected_policy_sha256:
                metrics["policy_sha256_match"] = computed == args.expected_policy_sha256

    # Write artifacts
    write_timeseries_header(args.capture_dir, args.capture_label, rl_fast_metrics.TSV_COLUMNS if METRICS_AVAILABLE else [])
    write_metrics_json(args.capture_dir, args.capture_label, metrics)

    verdict_data: Dict[str, Any] = {
        "verdict": verdict,
        "timestamp": now,
        "reason": "Live capture not implemented; scaffolding placeholder only." if is_partial else "No runtime data collected.",
        "primary": verdict,
        "secondary": ["LIVE_CAPTURE_NOT_IMPLEMENTED"] if is_partial else [],
    }
    write_verdict_json(args.capture_dir, args.capture_label, verdict_data)

    summary_fields: Dict[str, Any] = {
        "verdict": verdict,
        "world_mode": args.world_mode,
        "test_profile": args.test_profile,
        "timestamp": now,
        "policy_path": args.policy_path or "not set",
        "task_partial": is_partial,
        "metrics_available": METRICS_AVAILABLE,
        "notes": "Scaffold capture placeholder. No ROS runtime data was collected.",
    }
    write_summary_md(args.capture_dir, args.capture_label, summary_fields)

    print(f"[rl_fast_capture] Artifacts written to: {args.capture_dir}")
    print(f"[rl_fast_capture] Verdict: {verdict}")
    sys.exit(0)


if __name__ == "__main__":
    main()
