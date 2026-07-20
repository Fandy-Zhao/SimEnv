#!/usr/bin/env python3
"""ROS-free helpers for the earth RL motion benchmark."""

from __future__ import annotations

import math
import statistics
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


EPSILON = 1e-9


def quaternion_to_euler(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def wrap_angle_rad(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def world_to_body(vx: float, vy: float, yaw: float) -> Tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * vx + s * vy, -s * vx + c * vy


def finite_values(values: Iterable[float]) -> List[float]:
    return [float(value) for value in values if math.isfinite(float(value))]


def mean(values: Iterable[float]) -> Optional[float]:
    vals = finite_values(values)
    return statistics.fmean(vals) if vals else None


def rows_between(rows: Sequence[Dict[str, float]], start: float, end: float) -> List[Dict[str, float]]:
    return [row for row in rows if start <= float(row["sim_time"]) <= end]


def summarize_motion(rows: Sequence[Dict[str, float]]) -> Dict[str, object]:
    if len(rows) < 2:
        return {
            "sample_count": len(rows),
            "valid": False,
            "reason": "INSUFFICIENT_POSE_SAMPLES",
        }
    start = rows[0]
    end = rows[-1]
    forward, lateral = world_to_body(
        float(end["x"]) - float(start["x"]),
        float(end["y"]) - float(start["y"]),
        float(start["yaw"]),
    )
    duration = max(float(end["sim_time"]) - float(start["sim_time"]), EPSILON)
    return {
        "sample_count": len(rows),
        "valid": True,
        "sim_duration": duration,
        "initial_x": float(start["x"]),
        "initial_y": float(start["y"]),
        "initial_z": float(start["z"]),
        "final_x": float(end["x"]),
        "final_y": float(end["y"]),
        "final_z": float(end["z"]),
        "forward_displacement": forward,
        "lateral_displacement": lateral,
        "yaw_change_deg": math.degrees(wrap_angle_rad(float(end["yaw"]) - float(start["yaw"]))),
        "mean_body_vx": mean(row["body_vx"] for row in rows),
        "mean_body_vy": mean(row["body_vy"] for row in rows),
        "mean_yaw_rate": mean(row["body_wz"] for row in rows),
        "max_roll_deg": math.degrees(max(abs(float(row["roll"])) for row in rows)),
        "max_pitch_deg": math.degrees(max(abs(float(row["pitch"])) for row in rows)),
        "min_z": min(float(row["z"]) for row in rows),
    }


def classify(summary: Dict[str, object], requested_state: int, command_vx: float) -> str:
    if not summary.get("valid"):
        return "INVALID"
    if float(summary["min_z"]) < 0.12:
        return "FAIL_FALL"
    if float(summary["max_roll_deg"]) > 45.0 or float(summary["max_pitch_deg"]) > 45.0:
        return "FAIL_ATTITUDE"
    if requested_state == 6 and command_vx > 0.0 and float(summary["forward_displacement"]) <= 0.0:
        return "FAIL_NO_FORWARD_RESPONSE"
    return "PASS"
