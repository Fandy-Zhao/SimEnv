"""Shared validation and route metrics for exploration trajectories."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Dict, Iterable, Mapping, Optional


DEFAULT_TARGET_FRAME = "map"
DEFAULT_ROUTE_MAX_SPEED_MPS = 2.0
DEFAULT_ROUTE_MAX_STEP_M = 2.0

REJECT_NON_FINITE = "non_finite"
REJECT_FRAME_MISMATCH = "frame_mismatch"
REJECT_NON_MONOTONIC_TIME = "non_monotonic_time"
REJECT_ZERO_OR_NEGATIVE_DT = "zero_or_negative_dt"
REJECT_SPEED_EXCEEDED = "speed_exceeded"
REJECT_STEP_EXCEEDED = "step_exceeded"
REJECT_REASONS = (
    REJECT_NON_FINITE,
    REJECT_FRAME_MISMATCH,
    REJECT_NON_MONOTONIC_TIME,
    REJECT_ZERO_OR_NEGATIVE_DT,
    REJECT_SPEED_EXCEEDED,
    REJECT_STEP_EXCEEDED,
)


@dataclass(frozen=True)
class RoutePolicy:
    """Policy for accepting a segment into the reported XY route length.

    The 2 m/s default is deliberately above the current 0.8 m/s planner limit
    to tolerate estimator noise while rejecting known 10--1700 m/s divergence.
    The independent 2 m step cap protects against long-gap teleports.
    """

    target_frame: str = DEFAULT_TARGET_FRAME
    max_speed_mps: float = DEFAULT_ROUTE_MAX_SPEED_MPS
    max_step_m: float = DEFAULT_ROUTE_MAX_STEP_M

    def __post_init__(self) -> None:
        if not self.target_frame:
            raise ValueError("target_frame must not be empty")
        if not math.isfinite(self.max_speed_mps) or self.max_speed_mps <= 0:
            raise ValueError("max_speed_mps must be finite and positive")
        if not math.isfinite(self.max_step_m) or self.max_step_m <= 0:
            raise ValueError("max_step_m must be finite and positive")

    def to_dict(self) -> Dict[str, Any]:
        return {
            "target_frame": self.target_frame,
            "route_max_speed_mps": self.max_speed_mps,
            "route_max_step_m": self.max_step_m,
            "route_policy": (
                "finite map-frame poses; strictly increasing sim_time; "
                "speed and step thresholds enforced"
            ),
        }


@dataclass(frozen=True)
class PoseSample:
    x: float
    y: float
    z: float
    sim_time: float
    frame_id: str

    @classmethod
    def from_mapping(cls, sample: Mapping[str, Any]) -> "PoseSample":
        return cls(
            x=_as_float(sample.get("x")),
            y=_as_float(sample.get("y")),
            z=_as_float(sample.get("z", 0.0)),
            sim_time=_as_float(sample.get("sim_time")),
            frame_id=str(sample.get("frame_id", "")).strip(),
        )


@dataclass(frozen=True)
class SegmentEvaluation:
    accepted: bool
    distance_m: float = 0.0
    dt: float = 0.0
    speed_mps: float = 0.0
    reject_reason: Optional[str] = None


@dataclass
class RouteMetrics:
    route_length_m: float = 0.0
    route_total_segments: int = 0
    route_accepted_segments: int = 0
    route_rejected_segments: int = 0
    route_reject_reasons: Dict[str, int] = field(
        default_factory=lambda: {reason: 0 for reason in REJECT_REASONS})

    def add(self, evaluation: SegmentEvaluation) -> None:
        self.route_total_segments += 1
        if evaluation.accepted:
            self.route_accepted_segments += 1
            self.route_length_m += evaluation.distance_m
            return
        self.route_rejected_segments += 1
        reason = evaluation.reject_reason or REJECT_NON_FINITE
        self.route_reject_reasons[reason] = self.route_reject_reasons.get(reason, 0) + 1

    def to_dict(self) -> Dict[str, Any]:
        return {
            "route_length_m": self.route_length_m,
            "route_total_segments": self.route_total_segments,
            "route_accepted_segments": self.route_accepted_segments,
            "route_rejected_segments": self.route_rejected_segments,
            "route_reject_reasons": dict(self.route_reject_reasons),
        }


class RouteAccumulator:
    """Incremental wrapper over the same evaluator used for offline metrics."""

    def __init__(self, policy: RoutePolicy) -> None:
        self.policy = policy
        self.metrics = RouteMetrics()
        self.previous: Optional[PoseSample] = None

    def add(self, sample: Mapping[str, Any] | PoseSample) -> Optional[SegmentEvaluation]:
        current = _coerce_sample(sample)
        if self.previous is None:
            self.previous = current
            return None
        evaluation = evaluate_route_segment(self.previous, current, self.policy)
        self.metrics.add(evaluation)
        self.previous = current
        return evaluation


def _as_float(value: Any) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float("nan")


def _coerce_sample(sample: Mapping[str, Any] | PoseSample) -> PoseSample:
    if isinstance(sample, PoseSample):
        return sample
    return PoseSample.from_mapping(sample)


def validate_pose_sample(
        sample: Mapping[str, Any] | PoseSample,
        target_frame: str = DEFAULT_TARGET_FRAME) -> Optional[str]:
    """Return a reject reason, or ``None`` when a pose sample is valid."""
    pose = _coerce_sample(sample)
    if not all(math.isfinite(value) for value in
               (pose.x, pose.y, pose.z, pose.sim_time)):
        return REJECT_NON_FINITE
    if pose.frame_id != target_frame:
        return REJECT_FRAME_MISMATCH
    return None


def evaluate_route_segment(
        previous: Mapping[str, Any] | PoseSample,
        current: Mapping[str, Any] | PoseSample,
        policy: RoutePolicy) -> SegmentEvaluation:
    """Evaluate one ordered trajectory segment against ``policy``."""
    first = _coerce_sample(previous)
    second = _coerce_sample(current)
    invalid = (validate_pose_sample(first, policy.target_frame)
               or validate_pose_sample(second, policy.target_frame))
    if invalid is not None:
        return SegmentEvaluation(False, reject_reason=invalid)
    if first.frame_id != second.frame_id:
        return SegmentEvaluation(False, reject_reason=REJECT_FRAME_MISMATCH)

    dt = second.sim_time - first.sim_time
    if dt < 0:
        return SegmentEvaluation(False, dt=dt,
                                 reject_reason=REJECT_NON_MONOTONIC_TIME)
    if dt == 0:
        return SegmentEvaluation(False, dt=dt,
                                 reject_reason=REJECT_ZERO_OR_NEGATIVE_DT)

    distance = math.hypot(second.x - first.x, second.y - first.y)
    if not math.isfinite(distance):
        return SegmentEvaluation(False, dt=dt, reject_reason=REJECT_NON_FINITE)
    speed = distance / dt
    if not math.isfinite(speed):
        return SegmentEvaluation(False, distance_m=distance, dt=dt,
                                 reject_reason=REJECT_NON_FINITE)
    if speed > policy.max_speed_mps:
        return SegmentEvaluation(False, distance_m=distance, dt=dt,
                                 speed_mps=speed,
                                 reject_reason=REJECT_SPEED_EXCEEDED)
    if distance > policy.max_step_m:
        return SegmentEvaluation(False, distance_m=distance, dt=dt,
                                 speed_mps=speed,
                                 reject_reason=REJECT_STEP_EXCEEDED)
    return SegmentEvaluation(True, distance_m=distance, dt=dt, speed_mps=speed)


def compute_route_length(
        samples: Iterable[Mapping[str, Any] | PoseSample],
        policy: RoutePolicy) -> RouteMetrics:
    """Compute route distance and reject counters using the shared policy."""
    accumulator = RouteAccumulator(policy)
    for sample in samples:
        accumulator.add(sample)
    return accumulator.metrics
