#!/usr/bin/env python3
"""G2-D1 Pre-WAVE block reason classification.

Defines the first-block-reason enumeration and classification logic.
The first_block_reason values written by the C++ diagnostic layer are:
  0   = PRE_WAVE_BLOCK_NONE (no block detected yet)
  101 = READINESS_HEIGHT_FALSE
  102 = READINESS_STANCE_FALSE
  103 = READINESS_CONTACT_FALSE
  201 = NUMERICAL_GUARD_STATE
  202 = NUMERICAL_GUARD_COMMAND
  203 = NUMERICAL_GUARD_OUTPUT
  301 = SAFETY_GUARD_ATTITUDE
  302 = SAFETY_GUARD_CONTACT

Additional Python-side classifications (from CSV analysis):
  400 = FIXEDSTAND_UNSTABLE (height < 0.12 during FixedStand)
  401 = FALL_BEFORE_TROTTING
  402 = FSM_TROTTING_NOT_ENTERED
  403 = FSM_TROTTING_NOT_SUSTAINED
  404 = READINESS_CONTACT_STALE
  405 = READINESS_HOLD_INCOMPLETE
  406 = WAVE_START_NOT_REQUESTED
  407 = WAVE_START_REQUEST_NOT_CONSUMED
  408 = WAVE_STATUS_NOT_TRANSITIONED
  409 = WAVE_CANCELLED_NUMERICAL
  410 = WAVE_CANCELLED_SAFETY
  411 = WAVE_CANCELLED_FALL
  412 = WAVE_CANCELLED_CONTACT
  413 = WAVE_CANCELLED_UNKNOWN
  414 = NUMERICAL_GUARD_BEFORE_WAVE
  415 = SAFETY_GUARD_BEFORE_WAVE
  416 = PHYSICAL_FALL_BEFORE_WAVE
  500 = UNKNOWN
"""

from __future__ import annotations

import csv
import json
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

# Readiness flags bitmask (from C++ TimingRecord)
FLAG_HEIGHT = 1
FLAG_STANCE = 2
FLAG_CONTACT = 4
FLAG_LINSPEED = 8
FLAG_ANGSPEED = 16
FLAG_TILT = 32
FLAG_MET = 64
FLAG_HOLD = 128

# FSM state enum values
FSM_PASSIVE = 1
FSM_FIXEDSTAND = 2
FSM_FREESTAND = 3
FSM_TROTTING = 4

# WaveStatus enum values
WAVE_STANCE_ALL = 0
WAVE_SWING_ALL = 1
WAVE_ALL = 2

# Pre-WAVE block reasons (C++ layer)
CPP_BLOCK_REASONS = {
    0: "PRE_WAVE_BLOCK_NONE",
    101: "READINESS_HEIGHT_FALSE",
    102: "READINESS_STANCE_FALSE",
    103: "READINESS_CONTACT_FALSE",
    201: "NUMERICAL_GUARD_STATE",
    202: "NUMERICAL_GUARD_COMMAND",
    203: "NUMERICAL_GUARD_OUTPUT",
    301: "SAFETY_GUARD_ATTITUDE",
    302: "SAFETY_GUARD_CONTACT",
}

# Extended block reasons (Python-side analysis)
PY_BLOCK_REASONS = {
    400: "FIXEDSTAND_UNSTABLE",
    401: "FALL_BEFORE_TROTTING",
    402: "FSM_TROTTING_NOT_ENTERED",
    403: "FSM_TROTTING_NOT_SUSTAINED",
    404: "READINESS_CONTACT_STALE",
    405: "READINESS_HOLD_INCOMPLETE",
    406: "WAVE_START_NOT_REQUESTED",
    407: "WAVE_START_REQUEST_NOT_CONSUMED",
    408: "WAVE_STATUS_NOT_TRANSITIONED",
    409: "WAVE_CANCELLED_NUMERICAL",
    410: "WAVE_CANCELLED_SAFETY",
    411: "WAVE_CANCELLED_FALL",
    412: "WAVE_CANCELLED_CONTACT",
    413: "WAVE_CANCELLED_UNKNOWN",
    414: "NUMERICAL_GUARD_BEFORE_WAVE",
    415: "SAFETY_GUARD_BEFORE_WAVE",
    416: "PHYSICAL_FALL_BEFORE_WAVE",
    417: "EXPECTED_NO_STEP_TRIGGER",
    500: "UNKNOWN",
}

ALL_BLOCK_REASONS = {**CPP_BLOCK_REASONS, **PY_BLOCK_REASONS}


def block_reason_name(code: int) -> str:
    """Return the human-readable name for a block reason code."""
    return ALL_BLOCK_REASONS.get(code, f"UNKNOWN_{code}")


@dataclass
class CheckpointResult:
    """Result for a single pre-WAVE checkpoint."""
    name: str
    passed: bool = False
    first_sim_time_us: int = 0
    first_control_sequence: int = 0
    detail: str = ""


@dataclass
class EventRecord:
    """A single structured event in the trial timeline."""
    sim_time_us: int
    control_sequence: int
    event: str
    fsm_state: int = 0
    wave_status: int = 0
    model_height: float = 0.0
    resolved_vx: float = 0.0
    detail: str = ""


@dataclass
class PreWaveTrialSummary:
    """Aggregated pre-WAVE diagnostic summary for one trial."""
    trial_id: str = ""
    command_vx: float = 0.0
    step_required: bool = False
    step_not_required_reason: str = ""
    valid: bool = False
    invalid_reasons: List[str] = field(default_factory=list)

    # FixedStand
    fixedstand_stable: bool = False
    fixedstand_min_height: float = 0.0
    fixedstand_entered_sim_time_us: int = 0

    # Trotting entry
    trotting_entered: bool = False
    trotting_enter_sim_time_us: int = 0

    # Readiness
    height_ready: bool = False
    stance_ready: bool = False
    contact_ready: bool = False
    hold_complete: bool = False
    readiness_achieved_sim_time_us: int = 0

    # Wave
    wave_start_requested: bool = False
    wave_all_entered: bool = False
    wave_all_enter_sim_time_us: int = 0
    wave_cancelled: bool = False
    wave_cancel_sim_time_us: int = 0
    wave_cancel_reason: str = ""

    # Fall
    fall_detected: bool = False
    first_fall_sim_time_us: int = 0
    min_model_height: float = 0.0

    # Guards
    numerical_guard: bool = False
    numerical_guard_stage: int = 0
    safety_guard: bool = False

    # First block
    first_block_reason: int = 0
    first_block_reason_name: str = ""
    first_block_sim_time_us: int = 0

    # Events
    events: List[EventRecord] = field(default_factory=list)
    checkpoints: List[CheckpointResult] = field(default_factory=list)

    # Timeline classification
    timeline_type: str = ""  # A, B, C, D, E, F, UNKNOWN


def classify_readiness(flags: int) -> Dict[str, bool]:
    """Decompose readiness flags bitmask into individual booleans."""
    return {
        "height": bool(flags & FLAG_HEIGHT),
        "stance": bool(flags & FLAG_STANCE),
        "contact": bool(flags & FLAG_CONTACT),
        "linspeed": bool(flags & FLAG_LINSPEED),
        "angspeed": bool(flags & FLAG_ANGSPEED),
        "tilt": bool(flags & FLAG_TILT),
        "met": bool(flags & FLAG_MET),
        "hold": bool(flags & FLAG_HOLD),
    }


def analyze_controller_csv(csv_path: str) -> PreWaveTrialSummary:
    """Analyze a controller_state.csv with prewave_* columns."""
    summary = PreWaveTrialSummary()

    if csv_path is None:
        return summary

    rows: List[Dict] = []
    try:
        with open(csv_path, newline="") as handle:
            rows = list(csv.DictReader(handle))
    except (OSError, csv.Error):
        summary.invalid_reasons.append("CSV_READ_ERROR")
        return summary

    if not rows:
        summary.invalid_reasons.append("CSV_EMPTY")
        return summary

    # Check for prewave columns
    has_prewave = "prewave_readiness_flags" in rows[0] if rows else False

    first_trotting_row = None
    first_wave_all_row = None
    first_fall_row = None
    first_wave_cancel_row = None
    first_non_stance_all_row = None
    first_block_row = None

    min_height = float("inf")
    fixedstand_rows = []
    trotting_rows = []

    for row in rows:
        try:
            fsm = int(float(row.get("fsm_state", 0)))
            wave = int(float(row.get("wave_status", -1)))
            sim_us = int(float(row.get("sim_time_us", 0)))
            seq = int(float(row.get("accepted_state_sequence", 0)))
            height = float(row.get("prewave_model_height", 0.0)) if has_prewave else 0.0
            block_reason = int(float(row.get("prewave_first_block_reason", 0))) if has_prewave else 0
        except (ValueError, KeyError):
            continue

        # Track min height
        if has_prewave and height > 0:
            min_height = min(min_height, height)

        # FixedStand phase
        if fsm == FSM_FIXEDSTAND:
            fixedstand_rows.append(row)
            if height > 0 and height < 0.12:
                if first_fall_row is None:
                    first_fall_row = row

        # Trotting phase
        if fsm == FSM_TROTTING:
            if first_trotting_row is None:
                first_trotting_row = row
            trotting_rows.append(row)

            # Wave ALL entered
            if wave == WAVE_ALL and first_wave_all_row is None:
                first_wave_all_row = row

            # Wave cancel (back to STANCE_ALL from WAVE_ALL or SWING_ALL)
            if wave == WAVE_STANCE_ALL and first_non_stance_all_row is None:
                # Check if previous row had non-STANCE_ALL
                pass

            # First block
            if block_reason != 0 and first_block_row is None:
                first_block_row = row

    # Populate summary
    summary.command_vx = float(rows[-1].get("resolved_vx", 0)) if rows else 0.0

    # Determine if step is required from resolved_vx
    abs_vx = abs(summary.command_vx)
    summary.step_required = abs_vx > 0.03
    if not summary.step_required:
        summary.step_not_required_reason = (
            f"resolved_vx={summary.command_vx:.4f} below threshold 0.03"
        )

    # FixedStand analysis
    if fixedstand_rows:
        summary.fixedstand_entered_sim_time_us = int(float(fixedstand_rows[0].get("sim_time_us", 0)))
        fixed_heights = []
        for r in fixedstand_rows:
            try:
                h = float(r.get("prewave_model_height", 0)) if has_prewave else float(r.get("resolved_vx", 0))
            except (ValueError, KeyError):
                continue
            if has_prewave and h > 0:
                fixed_heights.append(h)
        if fixed_heights:
            summary.fixedstand_min_height = min(fixed_heights)
            summary.fixedstand_stable = min(fixed_heights) >= 0.12

    # Trotting analysis
    if first_trotting_row is not None:
        summary.trotting_entered = True
        summary.trotting_enter_sim_time_us = int(float(first_trotting_row.get("sim_time_us", 0)))

    # Readiness - from prewave fields in the last few trotting rows
    if has_prewave and trotting_rows:
        # Find first row where readiness_hold_complete (flag 128) is set
        for r in trotting_rows:
            try:
                flags = int(float(r.get("prewave_readiness_flags", 0)))
            except (ValueError, KeyError):
                continue
            decomposed = classify_readiness(flags)
            if decomposed["hold"] and not summary.hold_complete:
                summary.hold_complete = True
                summary.readiness_achieved_sim_time_us = int(float(r.get("sim_time_us", 0)))
            if not summary.height_ready and decomposed["height"]:
                summary.height_ready = True
            if not summary.stance_ready and decomposed["stance"]:
                summary.stance_ready = True
            if not summary.contact_ready and decomposed["contact"]:
                summary.contact_ready = True

    # Wave analysis
    if first_wave_all_row is not None:
        summary.wave_all_entered = True
        summary.wave_all_enter_sim_time_us = int(float(first_wave_all_row.get("sim_time_us", 0)))

    # Wave cancel
    if has_prewave:
        for r in trotting_rows:
            try:
                cancel_reason = int(float(r.get("prewave_wave_cancel_reason", 0)))
            except (ValueError, KeyError):
                continue
            if cancel_reason != 0 and not summary.wave_cancelled:
                summary.wave_cancelled = True
                summary.wave_cancel_sim_time_us = int(float(r.get("sim_time_us", 0)))
                summary.wave_cancel_reason = {
                    1: "nonfinite_state", 2: "nonfinite_cmd", 3: "nonfinite_output",
                    4: "unsafe_attitude", 5: "contact_loss", 6: "time_reset", 7: "unknown",
                }.get(cancel_reason, f"unknown_{cancel_reason}")
                break

    # Fall
    summary.min_model_height = min_height if min_height != float("inf") else 0.0
    summary.fall_detected = min_height < 0.12
    if first_fall_row is not None:
        summary.first_fall_sim_time_us = int(float(first_fall_row.get("sim_time_us", 0)))

    # First block
    if first_block_row is not None and has_prewave:
        try:
            summary.first_block_reason = int(float(first_block_row.get("prewave_first_block_reason", 0)))
        except (ValueError, KeyError):
            pass
        summary.first_block_reason_name = block_reason_name(summary.first_block_reason)
        summary.first_block_sim_time_us = int(float(first_block_row.get("sim_time_us", 0)))

    # Numerical guard
    if has_prewave:
        for r in trotting_rows:
            try:
                stage = int(float(r.get("prewave_numerical_guard_stage", 0)))
            except (ValueError, KeyError):
                continue
            if stage != 0:
                summary.numerical_guard = True
                summary.numerical_guard_stage = stage
                break

    # Determine timeline type
    summary.timeline_type = _classify_timeline(summary)
    summary.valid = True
    return summary


def _classify_timeline(s: PreWaveTrialSummary) -> str:
    """Classify the event timeline into type A-F."""
    if not s.trotting_entered:
        if s.fall_detected:
            return "E"  # FixedStand already unstable
        return "UNKNOWN"

    if s.fall_detected and s.first_fall_sim_time_us < s.trotting_enter_sim_time_us:
        return "E"  # Fall before Trotting

    if s.fall_detected and not s.hold_complete:
        return "A"  # Trotting entered, readiness not complete, fall

    if s.hold_complete and not s.wave_all_entered:
        if s.numerical_guard:
            return "F"  # Control output anomaly before wave
        return "B"  # Readiness complete, wave not started, fall

    if s.wave_all_entered and s.wave_cancelled:
        return "D"  # Wave entered then cancelled

    if s.wave_all_entered and not s.wave_cancelled:
        return "COMPLETE"  # Wave entered and sustained

    if s.numerical_guard and not s.wave_all_entered:
        return "F"  # Numerical issue before wave

    return "UNKNOWN"


def determine_first_failing_checkpoint(summary: PreWaveTrialSummary) -> str:
    """Determine the first failing checkpoint from the summary."""
    if not summary.trotting_entered:
        return "P1_FAIL_TROTTING_NOT_ENTERED"

    if not summary.fixedstand_stable:
        return "P0_FAIL_FIXEDSTAND_UNSTABLE"

    if summary.fall_detected and summary.first_fall_sim_time_us < summary.trotting_enter_sim_time_us:
        return "P12_FAIL_PHYSICAL_FALL_BEFORE_TROTTING"

    if not summary.height_ready:
        return "P3_FAIL_HEIGHT_READINESS"

    if not summary.stance_ready:
        return "P4_FAIL_STANCE_READINESS"

    if not summary.contact_ready:
        return "P5_FAIL_CONTACT_READINESS"

    if not summary.hold_complete:
        return "P7_FAIL_HOLD_NOT_COMPLETED"

    if not summary.wave_all_entered:
        if not summary.step_required:
            return "P8_PASS_EXPECTED_NO_STEP_TRIGGER"
        return "P8_FAIL_START_NOT_REQUESTED"

    if summary.wave_cancelled:
        if summary.numerical_guard:
            return "P13_FAIL_NUMERICAL_GUARD"
        return "P15_FAIL_WAVE_CANCELLED"

    if summary.wave_all_entered:
        return "P10_PASS_WAVE_ALL_ENTERED"

    return "UNKNOWN"


def write_summary_json(summary: PreWaveTrialSummary, path: str) -> None:
    """Serialize the trial summary to JSON."""
    data = {
        "trial_id": summary.trial_id,
        "command_vx": summary.command_vx,
        "step_required": summary.step_required,
        "step_not_required_reason": summary.step_not_required_reason,
        "valid": summary.valid,
        "invalid_reasons": summary.invalid_reasons,
        "fixedstand_stable": summary.fixedstand_stable,
        "fixedstand_min_height": summary.fixedstand_min_height,
        "trotting_entered": summary.trotting_entered,
        "trotting_enter_sim_time_us": summary.trotting_enter_sim_time_us,
        "height_ready": summary.height_ready,
        "stance_ready": summary.stance_ready,
        "contact_ready": summary.contact_ready,
        "hold_complete": summary.hold_complete,
        "readiness_achieved_sim_time_us": summary.readiness_achieved_sim_time_us,
        "wave_all_entered": summary.wave_all_entered,
        "wave_all_enter_sim_time_us": summary.wave_all_enter_sim_time_us,
        "wave_cancelled": summary.wave_cancelled,
        "wave_cancel_sim_time_us": summary.wave_cancel_sim_time_us,
        "wave_cancel_reason": summary.wave_cancel_reason,
        "fall_detected": summary.fall_detected,
        "first_fall_sim_time_us": summary.first_fall_sim_time_us,
        "min_model_height": summary.min_model_height,
        "numerical_guard": summary.numerical_guard,
        "numerical_guard_stage": summary.numerical_guard_stage,
        "first_block_reason": summary.first_block_reason,
        "first_block_reason_name": summary.first_block_reason_name,
        "first_block_sim_time_us": summary.first_block_sim_time_us,
        "timeline_type": summary.timeline_type,
        "first_failing_checkpoint": determine_first_failing_checkpoint(summary),
    }
    with open(path, "w") as handle:
        json.dump(data, handle, indent=2, sort_keys=True)


def cross_trial_comparison(summaries: List[PreWaveTrialSummary]) -> Dict:
    """Compare multiple trial summaries to identify common patterns."""
    result = {
        "num_trials": len(summaries),
        "valid_trials": sum(1 for s in summaries if s.valid),
        "timeline_counts": {},
        "first_block_counts": {},
        "first_checkpoint_counts": {},
        "common_pattern": "",
    }
    for s in summaries:
        if not s.valid:
            continue
        tl = s.timeline_type
        result["timeline_counts"][tl] = result["timeline_counts"].get(tl, 0) + 1
        reason = s.first_block_reason_name or "NONE"
        result["first_block_counts"][reason] = result["first_block_counts"].get(reason, 0) + 1
        cp = determine_first_failing_checkpoint(s)
        result["first_checkpoint_counts"][cp] = result["first_checkpoint_counts"].get(cp, 0) + 1

    # Determine most common
    if result["timeline_counts"]:
        result["common_pattern"] = max(result["timeline_counts"], key=result["timeline_counts"].get)
    return result
