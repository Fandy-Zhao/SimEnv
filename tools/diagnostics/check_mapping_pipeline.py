#!/usr/bin/env python3
"""Best-effort inspection of the mapping topic pipeline.

Checks for expected ROS topics in the chain:
  /scan (sensor_msgs/PointCloud) -> scan_to_pointcloud2.py ->
  /scan_pointcloud2 (sensor_msgs/PointCloud2) -> FAST-LIO2 ->
  /Odometry (nav_msgs/Odometry) + /cloud_registered (sensor_msgs/PointCloud2)
  -> relayed /state_estimation + /registered_scan.

Outputs JSON and a Markdown summary.  No failures if ROS is unavailable.
"""

import argparse
import json
import subprocess
import sys
from typing import Any, Dict, List, Optional, Tuple


# Expected mapping chain
EXPECTED_TOPICS = [
    "/scan",
    "/scan_pointcloud2",
    "/Odometry",
    "/cloud_registered",
    "/state_estimation",
    "/registered_scan",
]

# Expected node names (substring match)
EXPECTED_NODES = {
    "scan_to_pointcloud2": "converter node for /scan -> /scan_pointcloud2",
    "fastlio_mapping": "FAST-LIO2 odometry and mapping node",
    "laserMapping": "alternative FAST-LIO2 node name",
}


def ros_topic_list() -> Optional[List[str]]:
    try:
        out = subprocess.check_output(
            ["rostopic", "list"], stderr=subprocess.DEVNULL, timeout=5
        )
        return sorted(out.decode().strip().splitlines())
    except Exception:
        return None


def ros_node_list() -> Optional[List[str]]:
    try:
        out = subprocess.check_output(
            ["rosnode", "list"], stderr=subprocess.DEVNULL, timeout=5
        )
        return sorted(out.decode().strip().splitlines())
    except Exception:
        return None


def ros_topic_info(topic: str) -> Optional[str]:
    try:
        out = subprocess.check_output(
            ["rostopic", "info", topic], stderr=subprocess.DEVNULL, timeout=5
        )
        return out.decode().strip()
    except Exception:
        return None


def parse_topic_type(info: Optional[str]) -> Optional[str]:
    if not info:
        return None
    for line in info.splitlines():
        if line.startswith("Type:"):
            return line.split(":", 1)[1].strip()
    return None


def ros_topic_hz(topic: str, window: int = 10) -> Optional[float]:
    try:
        out = subprocess.check_output(
            ["rostopic", "hz", "-w", str(window), topic],
            stderr=subprocess.DEVNULL,
            timeout=window + 5,
        )
        import re

        m = re.search(r"average rate:\s*([0-9.]+)", out.decode())
        return float(m.group(1)) if m else None
    except Exception:
        return None


def check_topics(topics: List[str]) -> Tuple[Dict[str, Any], List[str]]:
    present: Dict[str, Any] = {}
    issues: List[str] = []
    topic_set = set(topics)

    for name in EXPECTED_TOPICS:
        if name in topic_set:
            info = ros_topic_info(name)
            present[name] = {
                "present": True,
                "type": parse_topic_type(info),
                "info": info,
            }
        else:
            present[name] = {"present": False, "type": None, "info": None}
            issues.append(f"Missing topic: {name}")

    return present, issues


def check_nodes(nodes: List[str]) -> Tuple[Dict[str, Any], List[str]]:
    present: Dict[str, Any] = {}
    issues: List[str] = []
    for name, desc in EXPECTED_NODES.items():
        found = any(name in n for n in nodes)
        present[name] = {"present": found, "description": desc}
        if not found:
            issues.append(f"Node not running: {name} ({desc})")
    return present, issues


def render_markdown(
    topic_status: Dict[str, Any],
    node_status: Dict[str, Any],
    topic_rates: Dict[str, Optional[float]],
    issues: List[str],
) -> str:
    lines: List[str] = [
        "## Mapping Pipeline Check",
        "",
    ]

    lines.append("### Expected Topics")
    lines.append("")
    for name in EXPECTED_TOPICS:
        s = topic_status.get(name, {})
        status = ":white_check_mark:" if s.get("present") else ":x:"
        rate = topic_rates.get(name)
        rate_str = f" — {rate:.2f} Hz" if rate else ""
        lines.append(f"- {status} `{name}`{rate_str}")

    lines.append("")
    lines.append("### Expected Nodes")
    lines.append("")
    for name in EXPECTED_NODES:
        s = node_status.get(name, {})
        status = ":white_check_mark:" if s.get("present") else ":x:"
        lines.append(f"- {status} `{name}`")

    if issues:
        lines.append("")
        lines.append("### Issues")
        lines.append("")
        for issue in issues:
            lines.append(f"- {issue}")

    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Check mapping pipeline topics and nodes"
    )
    parser.add_argument(
        "--hz",
        action="store_true",
        help="Also probe topic rates (slower)",
    )
    parser.add_argument(
        "--hz-window", type=int, default=10, help="Window for rostopic hz"
    )
    args = parser.parse_args()

    topics = ros_topic_list()
    nodes = ros_node_list()

    result: Dict[str, Any] = {
        "ros_available": topics is not None,
        "topics": None,
        "nodes": None,
        "topic_status": {},
        "node_status": {},
        "topic_rates": {},
        "issues": [],
    }

    topic_status: Dict[str, Any] = {}
    node_status: Dict[str, Any] = {}
    topic_rates: Dict[str, Optional[float]] = {}
    issues: List[str] = []

    if topics is not None:
        topic_status, topic_issues = check_topics(topics)
        issues.extend(topic_issues)

        if args.hz:
            for name in EXPECTED_TOPICS:
                if topic_status.get(name, {}).get("present"):
                    topic_rates[name] = ros_topic_hz(name, args.hz_window)
                else:
                    topic_rates[name] = None
    else:
        issues.append("ROS not available (rosnode/rostopic not found or master not running)")

    if nodes is not None:
        node_status, node_issues = check_nodes(nodes)
        issues.extend(node_issues)

    result["topics"] = topics
    result["nodes"] = nodes
    result["topic_status"] = topic_status
    result["node_status"] = node_status
    result["topic_rates"] = topic_rates
    result["issues"] = issues

    # JSON output
    json.dump(result, sys.stdout, indent=2, default=str)
    sys.stdout.write("\n")
    sys.stdout.write("\n---\n\n")
    sys.stdout.write(render_markdown(topic_status, node_status, topic_rates, issues))
    sys.stdout.write("\n")


if __name__ == "__main__":
    main()
