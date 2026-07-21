#!/usr/bin/env python3
"""Safe offline replay scaffold for RL fast validation.

Provides fixture parsing, metadata validation, policy hash verification,
dimension fields, and explicit OFFLINE_REPLAY_SCAFFOLD_ONLY status when
C++ observation/action reuse is not implemented.

This scaffold does NOT process live ROS bags, C++ replay buffers, or
observation/action tensors — it validates shape only.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import re
import sys
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional, Tuple

try:
    import rl_fast_metrics  # type: ignore[import-not-found]
    METRICS_AVAILABLE = True
except ImportError:
    METRICS_AVAILABLE = False

_STATUS_SCAFFOLD_ONLY = "OFFLINE_REPLAY_SCAFFOLD_ONLY"

# Expected dimension fields for a valid replay fixture
DIMENSION_FIELDS = [
    "observation_dim",
    "action_dim",
    "privileged_dim",
    "history_length",
    "num_envs",
]

FIXTURE_REQUIRED_KEYS = [
    "policy_path",
    "observation_dim",
    "action_dim",
    "timestamp",
    "metadata",
]

SHA256_RE = re.compile(r"^[a-f0-9]{64}$")


def parse_fixture(path: str) -> Dict[str, Any]:
    """Load a replay fixture (JSON) and validate required keys.

    Returns the parsed dict. Raises ValueError on missing keys or parse failure.
    """
    if not os.path.isfile(path):
        raise ValueError(f"Fixture file not found: {path}")
    try:
        with open(path, "r") as f:
            data = json.load(f)
    except json.JSONDecodeError as exc:
        raise ValueError(f"Fixture JSON parse error in {path}: {exc}") from exc

    missing = [k for k in FIXTURE_REQUIRED_KEYS if k not in data]
    if missing:
        raise ValueError(f"Fixture missing required keys: {missing}")

    return data


def validate_fixture_metadata(metadata: Dict[str, Any]) -> Tuple[bool, List[str]]:
    """Validate replay fixture metadata block.

    Returns (valid, warnings).  Required sub-fields include dimension fields.
    """
    warnings: List[str] = []

    for field in DIMENSION_FIELDS:
        if field not in metadata:
            warnings.append(f"Missing dimension field: {field}")

    # Numeric sanity
    for field in DIMENSION_FIELDS:
        val = metadata.get(field)
        if val is not None and not isinstance(val, (int, float)):
            warnings.append(f"Non-numeric {field}: {type(val).__name__}")
        elif field == "privileged_dim" and isinstance(val, (int, float)) and val < 0:
            warnings.append(f"Negative {field}: {val}")
        elif field != "privileged_dim" and isinstance(val, (int, float)) and val <= 0:
            warnings.append(f"Non-positive {field}: {val}")

    return len(warnings) == 0, warnings


def verify_policy_hash(policy_path: str, expected_sha256: str) -> Tuple[bool, str, str]:
    """Verify policy file SHA256 against expected value.

    Returns (match, actual_hash, status_string).
    """
    status = _STATUS_SCAFFOLD_ONLY

    if not METRICS_AVAILABLE:
        return False, "", f"{status}: rl_fast_metrics not available"

    valid, result = rl_fast_metrics.validate_policy_sha256(policy_path, expected_sha256)
    if not valid:
        return False, result, f"{status}: HASH_MISMATCH"
    return True, result, status


def build_replay_status(
    fixture: Dict[str, Any],
    policy_hash_match: bool,
    metadata_valid: bool,
    warnings: List[str],
) -> Dict[str, Any]:
    """Construct the replay status dict with dimension fields and scaffold status."""
    status: Dict[str, Any] = {
        "status": _STATUS_SCAFFOLD_ONLY,
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "fixture_policy_path": fixture.get("policy_path", ""),
        "policy_hash_match": policy_hash_match,
        "metadata_valid": metadata_valid,
        "warnings": warnings,
        "dimensions": {},
        "reason": "C++ observation/action reuse is not implemented.",
    }

    metadata = fixture.get("metadata", {})
    for field in DIMENSION_FIELDS:
        status["dimensions"][field] = metadata.get(field)

    return status


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Offline replay scaffold for RL fast validation")
    p.add_argument("--fixture", required=True, help="Path to replay fixture JSON")
    p.add_argument("--expected-sha256", default="", help="Expected policy SHA256 hex digest")
    p.add_argument("--output", default="replay_status.json", help="Output path for replay status JSON")
    return p.parse_args()


def main() -> None:
    args = parse_args()

    try:
        fixture = parse_fixture(args.fixture)
    except ValueError as exc:
        status = {
            "status": _STATUS_SCAFFOLD_ONLY,
            "error": str(exc),
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }
        with open(args.output, "w") as f:
            json.dump(status, f, indent=2)
        print(f"[replay_rl_state] ERROR: {exc}", file=sys.stderr)
        sys.exit(1)

    metadata = fixture.get("metadata", {})
    metadata_valid, warnings = validate_fixture_metadata(metadata)

    policy_path = fixture.get("policy_path", "")
    policy_hash_match = False
    actual_hash = ""
    if policy_path and args.expected_sha256:
        policy_hash_match, actual_hash, _ = verify_policy_hash(policy_path, args.expected_sha256)
    elif args.expected_sha256 and not rl_fast_metrics.is_valid_sha256_hex(args.expected_sha256) if METRICS_AVAILABLE else True:
        warnings.append("Expected SHA256 is not a valid hex digest.")

    status = build_replay_status(fixture, policy_hash_match, metadata_valid, warnings)

    with open(args.output, "w") as f:
        json.dump(status, f, indent=2)

    print(f"[replay_rl_state] Replay status written to: {args.output}")
    print(f"[replay_rl_state] Status: {_STATUS_SCAFFOLD_ONLY}")

    if warnings:
        print(f"[replay_rl_state] Warnings ({len(warnings)}):")
        for w in warnings:
            print(f"  - {w}")

    sys.exit(0)


if __name__ == "__main__":
    main()
