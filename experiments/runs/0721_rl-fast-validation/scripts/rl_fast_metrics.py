#!/usr/bin/env python3
"""ROS-free pure helpers for RL fast validation metrics."""

from __future__ import annotations

import hashlib
import math
import os
import re
import socket
from typing import Any, Dict, List, Optional, Sequence, Tuple


EPSILON = 1e-9
FALL_HEIGHT_THRESHOLD = 0.12          # m — base height below which the robot is considered fallen
FIXEDSTAND_TILT_MAX_DEG = 45.0        # deg — maximum allowable tilt in FixedStand
FIXEDSTAND_TILT_DURATION_MAX = 0.25   # s — max allowable duration above 30 deg tilt
DEFAULT_GRACE_PERIOD = 1.0            # s — default transition grace period for evaluation windows
TSV_COLUMNS = ["sim_time", "base_x", "base_y", "base_z", "base_qx", "base_qy", "base_qz", "base_qw",
               "base_vx", "base_vy", "base_vz", "base_wx", "base_wy", "base_wz",
               "rtf", "contact_0", "contact_1", "contact_2", "contact_3"]


# ---------------------------------------------------------------------------
# Evaluation window selection (exclude transition grace)
# ---------------------------------------------------------------------------

def select_evaluation_window(
    rows: Sequence[Dict[str, float]],
    grace_period: float = DEFAULT_GRACE_PERIOD,
) -> Sequence[Dict[str, float]]:
    """Return rows whose sim_time exceeds the grace period from start.

    The first row's sim_time is taken as t=0; any row with sim_time <=
    first_sim_time + grace_period is treated as a transition sample and
    excluded from evaluation.
    """
    if len(rows) < 2:
        return []
    t0 = float(rows[0].get("sim_time", 0.0))
    return [r for r in rows if float(r.get("sim_time", 0.0)) > t0 + grace_period]


def win_start_index(rows: Sequence[Dict[str, float]], grace_period: float = DEFAULT_GRACE_PERIOD) -> int:
    """Return first index in rows whose sim_time exceeds t0+grace_period."""
    if not rows:
        return 0
    t0 = float(rows[0].get("sim_time", 0.0))
    for idx, row in enumerate(rows):
        if float(row.get("sim_time", 0.0)) > t0 + grace_period:
            return idx
    return len(rows)


# ---------------------------------------------------------------------------
# Duration below threshold
# ---------------------------------------------------------------------------

def _linear_fraction_below(v0: float, v1: float, threshold: float) -> float:
    if v0 < threshold and v1 < threshold:
        return 1.0
    if v0 >= threshold and v1 >= threshold:
        return 0.0
    if abs(v1 - v0) < EPSILON:
        return 1.0 if v0 < threshold else 0.0
    crossing = max(0.0, min(1.0, (threshold - v0) / (v1 - v0)))
    return crossing if v0 < threshold else 1.0 - crossing


def _linear_fraction_above(v0: float, v1: float, threshold: float) -> float:
    if v0 > threshold and v1 > threshold:
        return 1.0
    if v0 <= threshold and v1 <= threshold:
        return 0.0
    if abs(v1 - v0) < EPSILON:
        return 1.0 if v0 > threshold else 0.0
    crossing = max(0.0, min(1.0, (threshold - v0) / (v1 - v0)))
    return crossing if v0 > threshold else 1.0 - crossing


def duration_below_threshold(
    rows: Sequence[Dict[str, float]],
    key: str,
    threshold: float,
) -> float:
    """Return cumulative sim_time where `key` value is strictly below threshold.

    Linearly interpolates threshold crossings so a transition spike does not
    contaminate steady-state evaluation duration.
    """
    total = 0.0
    for prev, cur in zip(rows, rows[1:]):
        pv = float(prev.get(key, float("inf")))
        cv = float(cur.get(key, float("inf")))
        dt = max(0.0, float(cur.get("sim_time", 0.0)) - float(prev.get("sim_time", 0.0)))
        total += dt * _linear_fraction_below(pv, cv, threshold)
    return total


def duration_above_threshold(
    rows: Sequence[Dict[str, float]],
    key: str,
    threshold: float,
) -> float:
    """Return cumulative sim_time where `key` value exceeds threshold."""
    total = 0.0
    for prev, cur in zip(rows, rows[1:]):
        pv = float(prev.get(key, float("inf")))
        cv = float(cur.get(key, float("inf")))
        dt = max(0.0, float(cur.get("sim_time", 0.0)) - float(prev.get("sim_time", 0.0)))
        total += dt * _linear_fraction_above(pv, cv, threshold)
    return total


# ---------------------------------------------------------------------------
# Verdict priority
# ---------------------------------------------------------------------------

VERDICT_PRIORITY = {
    "FAIL_RUNTIME_ARTIFACT_MISMATCH": 1,
    "FAIL_MASTER_MISMATCH": 2,
    "FAIL_PROCESS_CLEANUP": 3,
    "FAIL_ROS_CLOCK_PUBLISH": 4,
    "FAIL_GAZEBO_SIM_STALL": 5,
    "FAIL_RUNNER_CLOCK_SUBSCRIPTION": 6,
    "FAIL_FSM_ENTRY": 7,
    "FAIL_RL_STATE_ENTRY": 8,
    "FAIL_ACTION_NAN": 9,
    "FAIL_ACTION_INF": 10,
    "FAIL_POLICY_OUTPUT": 11,
    "FAIL_ACTION_SPIKE": 12,
    "FAIL_COMMAND_OUTPUT": 13,
    "FAIL_BODY_CONTACT": 14,
    "FAIL_BASE_HEIGHT": 15,
    "FAIL_TILT": 16,
    "FAIL_STATE_TRANSITION": 17,
    "FAIL_LOW_RTF": 18,
    "BLOCKED_BY_LOW_RTF": 19,
    "TASK_PARTIAL": 20,
    "BLOCKED": 21,
    "PASS": 22,
    "UNKNOWN": 99,
}


def worst_verdict(verdicts: Sequence[str]) -> str:
    """Return the highest-priority (lowest numeric value) verdict from a list."""
    if not verdicts:
        return "UNKNOWN"
    return min(verdicts, key=lambda v: VERDICT_PRIORITY.get(v, 99))


def verdict_priority(verdict: str) -> int:
    """Return the numeric priority of a verdict string. Lower is worse."""
    return VERDICT_PRIORITY.get(verdict, 99)


# ---------------------------------------------------------------------------
# NaN / Inf detection
# ---------------------------------------------------------------------------

def has_nan_or_inf(values: Sequence[float]) -> bool:
    """Return True if any value in the sequence is NaN or Inf."""
    return any(not math.isfinite(float(v)) for v in values)


def strip_nan_inf(values: Sequence[float]) -> List[float]:
    """Return a copy with NaN and Inf values removed."""
    return [float(v) for v in values if math.isfinite(float(v))]


# ---------------------------------------------------------------------------
# Local-frame displacement
# ---------------------------------------------------------------------------

def wrap_angle_rad(angle: float) -> float:
    """Wrap an angle in radians to [-pi, pi)."""
    wrapped = (angle + math.pi) % (2.0 * math.pi) - math.pi
    return math.pi if math.isclose(wrapped, -math.pi, abs_tol=1e-12) else wrapped


def quaternion_tilt_deg(qx: float, qy: float, qz: float, qw: float) -> float:
    """Body z-axis tilt angle in degrees. 0 = upright, 90 = on side."""
    z_axis_z = 1.0 - 2.0 * (qx * qx + qy * qy)
    z_axis_z = max(-1.0, min(1.0, z_axis_z))
    return math.degrees(math.acos(z_axis_z))


def local_frame_displacement(
    start_x: float, start_y: float, start_yaw: float,
    end_x: float, end_y: float, end_yaw: float,
) -> Tuple[float, float, float]:
    """Return (forward, lateral, yaw_change_deg) in the start body frame.

    Forward/lateral are expressed in the coordinate system aligned with
    the start yaw: forward == +x_body, lateral == +y_body.
    """
    dx = end_x - start_x
    dy = end_y - start_y
    c = math.cos(start_yaw)
    s = math.sin(start_yaw)
    forward = c * dx + s * dy
    lateral = -s * dx + c * dy
    yaw_change = math.degrees(wrap_angle_rad(end_yaw - start_yaw))
    return forward, lateral, yaw_change


def world_to_initial_body(dx: float, dy: float, initial_yaw: float) -> Tuple[float, float]:
    """Project (dx, dy) into the body frame defined by initial_yaw."""
    c = math.cos(initial_yaw)
    s = math.sin(initial_yaw)
    return c * dx + s * dy, -s * dx + c * dy


# ---------------------------------------------------------------------------
# Unique port allocation
# ---------------------------------------------------------------------------

_ROS_MASTER_MIN = 12111
_ROS_MASTER_MAX = 12999
_GAZEBO_MASTER_MIN = 12145
_GAZEBO_MASTER_MAX = 13999


def _is_port_available(port: int) -> bool:
    """Check whether a TCP port is available for listening."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind(("", port))
            return True
        except OSError:
            return False


def allocate_unique_ports(start_port: Optional[int] = None) -> Dict[str, int]:
    """Find an available ROS_MASTER_URI and GAZEBO_MASTER_URI port pair.

    Returns dict with keys 'ros_master' and 'gazebo_master'.  Ports are
    chosen from non-overlapping probing windows.
    """
    ros_base = start_port if start_port is not None else _ROS_MASTER_MIN
    gaz_base = ros_base + 34

    for offset in range(0, 256):
        ros_port = ros_base + offset * 2
        gaz_port = gaz_base + offset * 2
        if ros_port > _ROS_MASTER_MAX or gaz_port > _GAZEBO_MASTER_MAX:
            break
        if _is_port_available(ros_port) and _is_port_available(gaz_port):
            return {"ros_master": ros_port, "gazebo_master": gaz_port}

    # Fallback: scan sequentially
    for ros_port in range(_ROS_MASTER_MIN, _ROS_MASTER_MAX + 1):
        if _is_port_available(ros_port):
            for gaz_port in range(_GAZEBO_MASTER_MIN, _GAZEBO_MASTER_MAX + 1):
                if ros_port == gaz_port:
                    continue
                if _is_port_available(gaz_port):
                    return {"ros_master": ros_port, "gazebo_master": gaz_port}

    return {"ros_master": _ROS_MASTER_MIN, "gazebo_master": _GAZEBO_MASTER_MIN}


# ---------------------------------------------------------------------------
# Artifact path validation
# ---------------------------------------------------------------------------

def validate_artifact_path(
    path: str,
    require_executable: bool = False,
    require_readable: bool = True,
) -> Tuple[bool, Optional[str]]:
    """Validate an artifact path exists, optionally executable/readable.

    Returns (valid, reason) where reason is None on success.
    """
    if not path:
        return False, "EMPTY_PATH"
    if not os.path.exists(path):
        return False, "NOT_FOUND"
    if require_readable and not os.access(path, os.R_OK):
        return False, "NOT_READABLE"
    if require_executable and not os.access(path, os.X_OK):
        return False, "NOT_EXECUTABLE"
    if os.path.isdir(path):
        return False, "IS_DIRECTORY"
    return True, None


def validate_artifact_dir(path: str) -> Tuple[bool, Optional[str]]:
    """Validate a directory path exists and is readable."""
    if not path:
        return False, "EMPTY_PATH"
    if not os.path.isdir(path):
        return False, "NOT_DIRECTORY"
    if not os.access(path, os.R_OK):
        return False, "NOT_READABLE"
    return True, None


def validate_artifact_within_worktree(path: str, worktree: str, require_executable: bool = False) -> Tuple[bool, str]:
    """Validate that an artifact exists and resolves inside the expected worktree."""
    valid, reason = validate_artifact_path(path, require_executable=require_executable)
    if not valid:
        return False, reason or "ARTIFACT_INVALID"
    real_path = os.path.realpath(path)
    real_root = os.path.realpath(worktree)
    try:
        common = os.path.commonpath([real_path, real_root])
    except ValueError:
        return False, "FAIL_RUNTIME_ARTIFACT_MISMATCH"
    if common != real_root:
        return False, "FAIL_RUNTIME_ARTIFACT_MISMATCH"
    return True, real_path


# ---------------------------------------------------------------------------
# Policy SHA256 validation
# ---------------------------------------------------------------------------

SHA256_RE = re.compile(r"^[a-f0-9]{64}$")


def compute_sha256(path: str) -> str:
    """Return the lowercase hex SHA256 digest of a file."""
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def validate_policy_sha256(policy_path: str, expected_sha256: str) -> Tuple[bool, str]:
    """Verify a policy file's SHA256 matches the expected value.

    Returns (valid, reason_or_computed_hash).
    """
    valid, reason = validate_artifact_path(policy_path, require_readable=True)
    if not valid:
        return False, reason or "ARTIFACT_INVALID"
    actual = compute_sha256(policy_path)
    if actual != expected_sha256:
        return False, actual
    return True, actual


def is_valid_sha256_hex(s: str) -> bool:
    """Return True if s is a valid-looking SHA256 hex digest."""
    return bool(SHA256_RE.match(s))


# ---------------------------------------------------------------------------
# Clock / master failure classification
# ---------------------------------------------------------------------------

def classify_clock_master_failure(
    clock_rows: Sequence[Dict[str, float]],
    gazebo_sim_advancing: Optional[bool] = None,
    runner_received_clock: Optional[bool] = None,
    master_uri_match: Optional[bool] = None,
    max_clock_gap: float = 2.0,
    min_clock_samples: int = 10,
) -> str:
    """Classify a runtime session for clock or master failure.

    Returns a fine-grained failure verdict or 'OK'.
    """
    if master_uri_match is False:
        return "FAIL_MASTER_MISMATCH"
    if runner_received_clock is False and gazebo_sim_advancing is True:
        return "FAIL_RUNNER_CLOCK_SUBSCRIPTION"
    if not clock_rows:
        return "FAIL_ROS_CLOCK_PUBLISH" if gazebo_sim_advancing else "FAIL_GAZEBO_SIM_STALL"
    if len(clock_rows) < min_clock_samples:
        return "FAIL_ROS_CLOCK_PUBLISH"

    times = [float(r.get("sim_time", 0.0)) for r in clock_rows]
    max_gap = 0.0
    for t0, t1 in zip(times, times[1:]):
        gap = t1 - t0
        if gap > max_gap:
            max_gap = gap

    if max_gap > max_clock_gap:
        return "FAIL_ROS_CLOCK_PUBLISH" if gazebo_sim_advancing else "FAIL_GAZEBO_SIM_STALL"
    return "OK"


def median(values: Sequence[float]) -> Optional[float]:
    vals = sorted(float(v) for v in values if math.isfinite(float(v)))
    if not vals:
        return None
    mid = len(vals) // 2
    if len(vals) % 2:
        return vals[mid]
    return 0.5 * (vals[mid - 1] + vals[mid])


def rtf_gate(rtf_median: Optional[float]) -> Tuple[str, str]:
    if rtf_median is None or not math.isfinite(float(rtf_median)):
        return "BLOCKED_BY_LOW_RTF", "missing_rtf"
    if rtf_median < 0.5:
        return "BLOCKED_BY_LOW_RTF", "rtf_below_0.5"
    if rtf_median < 0.8:
        return "PASS", "limited_smoke_rtf_risk"
    return "PASS", "normal_fast_smoke_rtf"


def summarize_fixedstand_windows(
    rows: Sequence[Dict[str, Any]],
    transition_grace: float = DEFAULT_GRACE_PERIOD,
    height_key: str = "base_z",
    tilt_key: str = "tilt_deg",
) -> Dict[str, Any]:
    if not rows:
        return {"valid": False, "reason": "NO_SAMPLES"}
    eval_rows = list(select_evaluation_window(rows, transition_grace))
    transition_rows = list(rows[:win_start_index(rows, transition_grace)])
    global_min_row = min(rows, key=lambda row: float(row.get(height_key, float("inf"))))
    return {
        "valid": bool(eval_rows),
        "global_min_base_height": min(float(row.get(height_key, float("inf"))) for row in rows),
        "transition_min_base_height": min((float(row.get(height_key, float("inf"))) for row in transition_rows), default=None),
        "evaluation_min_base_height": min((float(row.get(height_key, float("inf"))) for row in eval_rows), default=None),
        "steady_state_median_base_height": median([float(row.get(height_key, float("nan"))) for row in eval_rows]),
        "time_of_global_min": float(global_min_row.get("sim_time", 0.0)),
        "fsm_state_at_global_min": global_min_row.get("fsm_state"),
        "tilt_at_global_min": global_min_row.get(tilt_key),
        "duration_below_threshold_transition": duration_below_threshold(transition_rows, height_key, FALL_HEIGHT_THRESHOLD),
        "duration_below_threshold_evaluation": duration_below_threshold(eval_rows, height_key, FALL_HEIGHT_THRESHOLD),
        "max_tilt_evaluation": max((float(row.get(tilt_key, 0.0)) for row in eval_rows), default=None),
        "duration_tilt_over_30_evaluation": duration_above_threshold(eval_rows, tilt_key, 30.0),
    }


def classify_fixedstand_evaluation(summary: Dict[str, Any]) -> Tuple[str, List[str]]:
    flags: List[str] = []
    if not summary.get("valid"):
        return "FAIL_STATE_TRANSITION", ["NO_EVALUATION_WINDOW"]
    if (summary.get("duration_below_threshold_evaluation") or 0.0) > 0.25:
        flags.append("EVALUATION_BASE_HEIGHT_SUSTAINED_LOW")
        return "FAIL_BASE_HEIGHT", flags
    max_tilt = summary.get("max_tilt_evaluation")
    if max_tilt is not None and float(max_tilt) > FIXEDSTAND_TILT_MAX_DEG:
        flags.append("EVALUATION_TILT_LIMIT")
        return "FAIL_TILT", flags
    if (summary.get("duration_tilt_over_30_evaluation") or 0.0) > FIXEDSTAND_TILT_DURATION_MAX:
        flags.append("EVALUATION_TILT_DURATION")
        return "FAIL_TILT", flags
    return "PASS", flags


class VerdictCollector:
    """Collect and resolve verdicts by priority."""

    def __init__(self) -> None:
        self._verdicts: List[str] = []

    def add(self, verdict: str) -> None:
        self._verdicts.append(verdict)

    def resolve(self) -> str:
        return worst_verdict(self._verdicts) if self._verdicts else "UNKNOWN"

    def __len__(self) -> int:
        return len(self._verdicts)
