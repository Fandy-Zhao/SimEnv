#!/usr/bin/env python3
"""ROS-free fall validator semantics for G2-D1 Gate V.

The helpers in this file are deliberately small and data-only: they do not
change locomotion, controller state, or runtime scheduling. They allow the
existing G2 trial CSVs to be reclassified offline with explicit pose evidence.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Mapping, Sequence, Tuple


LEGACY_HEIGHT_FALL_M = 0.12
DEFAULT_TILT_FALL_DEG = 60.0


@dataclass(frozen=True)
class PoseSample:
    sim_time: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass(frozen=True)
class FallEvidence:
    legacy_height_fall: bool
    semantic_fall: bool
    min_z: float
    max_tilt_deg: float
    first_legacy_fall_time: float | None
    first_semantic_fall_time: float | None


def _as_float(row: Mapping[str, object], key: str) -> float:
    return float(row[key])


def pose_from_row(row: Mapping[str, object]) -> PoseSample:
    return PoseSample(
        sim_time=_as_float(row, "sim_time"),
        z=_as_float(row, "z"),
        qx=_as_float(row, "qx"),
        qy=_as_float(row, "qy"),
        qz=_as_float(row, "qz"),
        qw=_as_float(row, "qw"),
    )


def quaternion_norm(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)


def normalize_quaternion(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float, float]:
    norm = quaternion_norm(qx, qy, qz, qw)
    if norm <= 0.0 or not math.isfinite(norm):
        raise ValueError("invalid quaternion norm")
    return qx / norm, qy / norm, qz / norm, qw / norm


def quaternion_to_euler(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    qx, qy, qz, qw = normalize_quaternion(qx, qy, qz, qw)
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


def body_up_vector_world(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    """Return the body +Z axis expressed in world coordinates."""
    qx, qy, qz, qw = normalize_quaternion(qx, qy, qz, qw)
    return (
        2.0 * (qx * qz + qw * qy),
        2.0 * (qy * qz - qw * qx),
        1.0 - 2.0 * (qx * qx + qy * qy),
    )


def body_tilt_deg(qx: float, qy: float, qz: float, qw: float) -> float:
    """Angle between body +Z and world +Z."""
    _, _, up_z = body_up_vector_world(qx, qy, qz, qw)
    up_z = max(-1.0, min(1.0, up_z))
    return math.degrees(math.acos(up_z))


def legacy_height_fall(samples: Sequence[PoseSample], height_threshold_m: float = LEGACY_HEIGHT_FALL_M) -> bool:
    return any(sample.z < height_threshold_m for sample in samples)


def semantic_fall(
    samples: Sequence[PoseSample],
    height_threshold_m: float = LEGACY_HEIGHT_FALL_M,
    tilt_threshold_deg: float = DEFAULT_TILT_FALL_DEG,
) -> bool:
    """Candidate explicit fall semantics for offline analysis.

    This is intentionally conservative for Gate V analysis: low model/canonical
    height or high body tilt is fall evidence. It does not weaken the legacy
    height criterion and does not change runtime controller behavior.
    """
    return any(
        sample.z < height_threshold_m
        or body_tilt_deg(sample.qx, sample.qy, sample.qz, sample.qw) > tilt_threshold_deg
        for sample in samples
    )


def first_time_height_fall(samples: Sequence[PoseSample], height_threshold_m: float = LEGACY_HEIGHT_FALL_M) -> float | None:
    for sample in samples:
        if sample.z < height_threshold_m:
            return sample.sim_time
    return None


def first_time_semantic_fall(
    samples: Sequence[PoseSample],
    height_threshold_m: float = LEGACY_HEIGHT_FALL_M,
    tilt_threshold_deg: float = DEFAULT_TILT_FALL_DEG,
) -> float | None:
    for sample in samples:
        if sample.z < height_threshold_m:
            return sample.sim_time
        if body_tilt_deg(sample.qx, sample.qy, sample.qz, sample.qw) > tilt_threshold_deg:
            return sample.sim_time
    return None


def summarize_fall(samples: Sequence[PoseSample]) -> FallEvidence:
    if not samples:
        raise ValueError("no pose samples")
    tilts = [body_tilt_deg(sample.qx, sample.qy, sample.qz, sample.qw) for sample in samples]
    first_legacy = first_time_height_fall(samples)
    first_semantic = first_time_semantic_fall(samples)
    return FallEvidence(
        legacy_height_fall=first_legacy is not None,
        semantic_fall=first_semantic is not None,
        min_z=min(sample.z for sample in samples),
        max_tilt_deg=max(tilts),
        first_legacy_fall_time=first_legacy,
        first_semantic_fall_time=first_semantic,
    )


def load_pose_rows(rows: Iterable[Mapping[str, object]]) -> List[PoseSample]:
    return [pose_from_row(row) for row in rows]
