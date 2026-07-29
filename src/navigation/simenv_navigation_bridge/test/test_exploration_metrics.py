#!/usr/bin/env python3
"""Unit and offline fixture tests for shared exploration route metrics."""

import csv
import importlib.util
import math
from pathlib import Path
import sys
import tempfile
import unittest

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "src"))

from simenv_navigation_bridge.exploration_metrics import (  # noqa: E402
    REJECT_FRAME_MISMATCH,
    REJECT_NON_FINITE,
    REJECT_NON_MONOTONIC_TIME,
    REJECT_SPEED_EXCEEDED,
    REJECT_STEP_EXCEEDED,
    REJECT_ZERO_OR_NEGATIVE_DT,
    RouteMetrics,
    RoutePolicy,
    compute_route_length,
    evaluate_route_segment,
)


POLICY = RoutePolicy(target_frame="map", max_speed_mps=2.0, max_step_m=2.0)


def sample(time, x, y=0.0, frame="map", z=0.0):
    return {"sim_time": time, "x": x, "y": y, "z": z,
            "frame_id": frame}


def _load_renderer():
    renderer_path = PACKAGE.parents[2] / "tools" / "render_single_floor_exploration.py"
    spec = importlib.util.spec_from_file_location("render_single_floor_exploration",
                                                  renderer_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_fixture(run_dir, rows):
    route_dir = run_dir / "route"
    config_dir = run_dir / "config"
    route_dir.mkdir(parents=True)
    config_dir.mkdir(parents=True)
    fields = ["index", "sim_time", "x", "y", "z", "frame_id"]
    with (route_dir / "trajectory.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for index, row in enumerate(rows):
            writer.writerow({"index": index, **row})
    with (config_dir / "recorder_config.yaml").open("w") as stream:
        yaml.safe_dump({
            "trajectory_target_frame": "map",
            "route_max_speed_mps": 2.0,
            "route_max_step_m": 2.0,
        }, stream)


class ExplorationMetricsTest(unittest.TestCase):
    def test_normal_route_is_one_meter(self):
        metrics = compute_route_length(
            [sample(0.0, 0.0), sample(1.0, 0.5), sample(2.0, 1.0)], POLICY)
        self.assertAlmostEqual(metrics.route_length_m, 1.0)
        self.assertEqual(metrics.route_accepted_segments, 2)
        self.assertEqual(metrics.route_rejected_segments, 0)

    def test_segment_reject_reasons(self):
        cases = [
            (sample(0.0, 0.0), sample(1.0, math.nan), REJECT_NON_FINITE),
            (sample(2.0, 0.0), sample(1.0, 0.1), REJECT_NON_MONOTONIC_TIME),
            (sample(1.0, 0.0), sample(1.0, 0.1), REJECT_ZERO_OR_NEGATIVE_DT),
            (sample(0.0, 0.0), sample(0.1, 1.0), REJECT_SPEED_EXCEEDED),
            (sample(0.0, 0.0), sample(10.0, 3.0), REJECT_STEP_EXCEEDED),
            (sample(0.0, 0.0), sample(1.0, 0.5, frame="camera_init"),
             REJECT_FRAME_MISMATCH),
        ]
        for previous, current, reason in cases:
            with self.subTest(reason=reason):
                result = evaluate_route_segment(previous, current, POLICY)
                self.assertFalse(result.accepted)
                self.assertEqual(result.reject_reason, reason)

    def test_divergent_segment_is_retained_but_not_counted(self):
        samples = [
            sample(0.0, 0.0), sample(1.0, 0.5), sample(2.0, 1.0),
            sample(2.1, 1000.0, 5000.0),
        ]
        metrics = compute_route_length(samples, POLICY)
        self.assertEqual(len(samples), 4)
        self.assertAlmostEqual(metrics.route_length_m, 1.0)
        self.assertEqual(metrics.route_accepted_segments, 2)
        self.assertEqual(metrics.route_rejected_segments, 1)
        self.assertEqual(metrics.route_reject_reasons[REJECT_SPEED_EXCEEDED], 1)

    def test_reject_reason_counters_are_exact(self):
        pairs = [
            (sample(0.0, 0.0), sample(1.0, 0.5)),
            (sample(1.0, 0.0), sample(1.0, 0.1)),
            (sample(2.0, 0.0), sample(1.0, 0.1)),
            (sample(0.0, 0.0), sample(1.0, 0.1, frame="camera_init")),
            (sample(0.0, 0.0), sample(1.0, math.inf)),
        ]
        metrics = RouteMetrics()
        for previous, current in pairs:
            metrics.add(evaluate_route_segment(previous, current, POLICY))
        self.assertEqual(metrics.route_total_segments, 5)
        self.assertEqual(metrics.route_accepted_segments, 1)
        self.assertEqual(metrics.route_rejected_segments, 4)
        self.assertEqual(metrics.route_reject_reasons[
            REJECT_ZERO_OR_NEGATIVE_DT], 1)
        self.assertEqual(metrics.route_reject_reasons[
            REJECT_NON_MONOTONIC_TIME], 1)
        self.assertEqual(metrics.route_reject_reasons[REJECT_FRAME_MISMATCH], 1)
        self.assertEqual(metrics.route_reject_reasons[REJECT_NON_FINITE], 1)

    def test_renderer_uses_shared_metric_for_same_fixture(self):
        rows = [sample(0.0, 0.0), sample(1.0, 0.5), sample(2.0, 1.0),
                sample(2.1, 1000.0, 5000.0)]
        with tempfile.TemporaryDirectory() as temp_dir:
            run_dir = Path(temp_dir)
            _write_fixture(run_dir, rows)
            _, renderer_metrics = _load_renderer().load_trajectory(str(run_dir))
        recorder_metrics = compute_route_length(rows, POLICY)
        self.assertAlmostEqual(renderer_metrics["length_m"],
                               recorder_metrics.route_length_m)
        self.assertEqual(renderer_metrics["route_accepted_segments"], 2)
        self.assertEqual(renderer_metrics["route_rejected_segments"], 1)
        self.assertTrue(renderer_metrics["overlay_allowed"])

    def test_renderer_suppresses_legacy_mixed_frame_overlay(self):
        rows = [sample(0.0, 0.0),
                sample(1.0, 0.5, frame="camera_init")]
        with tempfile.TemporaryDirectory() as temp_dir:
            run_dir = Path(temp_dir)
            _write_fixture(run_dir, rows)
            points, metrics = _load_renderer().load_trajectory(str(run_dir))
        self.assertEqual(len(points), 2)  # retained for diagnostics
        self.assertFalse(metrics["overlay_allowed"])
        self.assertIn("TRAJECTORY_FRAME_MISMATCH", metrics["frame_warning"])
        self.assertEqual(metrics["route_reject_reasons"][
            REJECT_FRAME_MISMATCH], 1)

    def test_renderer_prefers_recorder_authoritative_metrics(self):
        rows = [sample(0.0, 0.0), sample(1.0, 0.5)]
        with tempfile.TemporaryDirectory() as temp_dir:
            run_dir = Path(temp_dir)
            _write_fixture(run_dir, rows)
            authoritative = compute_route_length(rows, POLICY).to_dict()
            with (run_dir / "route" / "metrics.yaml").open("w") as stream:
                yaml.safe_dump(authoritative, stream)
            _, metrics = _load_renderer().load_trajectory(str(run_dir))
        self.assertEqual(metrics["route_metrics_source"], "route/metrics.yaml")
        self.assertAlmostEqual(metrics["length_m"], 0.5)


if __name__ == "__main__":
    unittest.main()
