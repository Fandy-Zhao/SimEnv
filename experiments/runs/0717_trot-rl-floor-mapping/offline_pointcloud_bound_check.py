#!/usr/bin/env python3
"""Offline regression for bounded PointCloud2 extraction."""

import json
import os
import sys
import tempfile
import time

from sensor_msgs import point_cloud2
from std_msgs.msg import Header

sys.path.insert(0, os.path.dirname(__file__))
from capture_mapping_trial import Capture


def run_case(count, raw_limit, saved_limit):
    cloud = point_cloud2.create_cloud_xyz32(
        Header(frame_id="camera_init"),
        [(float(i % 1000) * 0.01, float(i // 1000) * 0.01, 1.0) for i in range(count)])
    with tempfile.TemporaryDirectory(prefix="simenv-cloud-bound-") as output:
        capture = Capture.__new__(Capture)
        capture.last_cloud = cloud
        capture.truth = []
        capture.output_dir = output
        start = time.monotonic()
        result = capture.save_cloud(raw_limit, saved_limit, wall_timeout=3.0)
        elapsed = time.monotonic() - start
        assert result["points"] <= saved_limit, result
        assert elapsed < 8.0, (elapsed, result)
        assert os.path.getsize(result["pcd"]) > 0
        return {"input_points": count, "elapsed_s": elapsed, "result": result}


if __name__ == "__main__":
    metrics = {
        "small": run_case(100, 200000, 50000),
        "large": run_case(250000, 200000, 50000),
    }
    print(json.dumps(metrics, indent=2))
