#!/usr/bin/env python3
"""Best-effort runtime-metrics snapshot to JSON.  Stdlib only.

Collects: time, git HEAD, load/mem/swap from /proc, vmstat when
available, process stats by name pattern, ROS node/topic list, and
optional topic-hz probes.
"""

import argparse
import json
import os
import re
import subprocess
import sys
import time
from typing import Any, Dict, List, Optional


def git_head(repo_root: str) -> str:
    try:
        out = subprocess.check_output(
            ["git", "-C", repo_root, "rev-parse", "--short", "HEAD"],
            stderr=subprocess.DEVNULL,
        )
        return out.decode().strip()
    except Exception:
        return "unknown"


def load_avg() -> Dict[str, float]:
    try:
        with open("/proc/loadavg") as fh:
            parts = fh.read().strip().split()
            return {
                "load_1m": float(parts[0]),
                "load_5m": float(parts[1]),
                "load_15m": float(parts[2]),
            }
    except Exception:
        return {}


def mem_info() -> Dict[str, int]:
    out: Dict[str, int] = {}
    try:
        with open("/proc/meminfo") as fh:
            for line in fh:
                key, _, rest = line.partition(":")
                parts = rest.strip().split()
                if parts:
                    try:
                        out[key.strip()] = int(parts[0])
                    except ValueError:
                        pass
    except Exception:
        pass
    return out


def vmstat_snapshot() -> Optional[Dict[str, Any]]:
    try:
        out = subprocess.check_output(
            ["vmstat", "-s"], stderr=subprocess.DEVNULL, timeout=5
        )
        result: Dict[str, Any] = {}
        for line in out.decode().strip().splitlines():
            line = line.strip()
            if not line:
                continue
            parts = line.split(None, 1)
            if len(parts) == 2:
                try:
                    result[parts[1].strip()] = int(parts[0])
                except ValueError:
                    result[parts[1].strip()] = parts[0]
        return result
    except Exception:
        return None


def process_stats(patterns: List[str]) -> Dict[str, List[Dict[str, Any]]]:
    result: Dict[str, List[Dict[str, Any]]] = {}
    try:
        out = subprocess.check_output(
            ["ps", "-eo", "pid,pcpu,pmem,rss,vsz,nlwp,psr,etime,comm,args"],
            stderr=subprocess.DEVNULL,
            timeout=5,
        )
    except Exception:
        return result
    lines = out.decode().strip().splitlines()
    if not lines:
        return result
    for line in lines[1:]:
        line = line.strip()
        if not line:
            continue
        parts = line.split(None, 9)
        if len(parts) < 10:
            continue
        comm = parts[8]
        args = parts[9]
        matched = False
        for pat in patterns:
            if re.search(pat, comm) or re.search(pat, args):
                matched = True
                break
        if not matched:
            continue
        try:
            entry = {
                "pid": int(parts[0]),
                "pcpu": float(parts[1]),
                "pmem": float(parts[2]),
                "rss_kb": int(parts[3]),
                "vsz_kb": int(parts[4]),
                "threads": int(parts[5]),
                "psr": parts[6],
                "etime": parts[7],
                "comm": comm,
                "args": args,
            }
        except (ValueError, IndexError):
            continue
        result.setdefault(comm, []).append(entry)
    return result


def ros_node_list() -> Optional[List[str]]:
    try:
        out = subprocess.check_output(
            ["rosnode", "list"], stderr=subprocess.DEVNULL, timeout=5
        )
        return sorted(out.decode().strip().splitlines())
    except Exception:
        return None


def ros_topic_list() -> Optional[List[str]]:
    try:
        out = subprocess.check_output(
            ["rostopic", "list"], stderr=subprocess.DEVNULL, timeout=5
        )
        return sorted(out.decode().strip().splitlines())
    except Exception:
        return None


def topic_hz(topics: List[str], window: int = 10) -> Dict[str, Any]:
    result: Dict[str, Any] = {}
    for topic in topics:
        try:
            out = subprocess.check_output(
                ["rostopic", "hz", "-w", str(window), topic],
                stderr=subprocess.DEVNULL,
                timeout=window + 5,
            )
            text = out.decode().strip()
            m = re.search(r"average rate:\s*([0-9.]+)", text)
            if m:
                result[topic] = float(m.group(1))
            else:
                result[topic] = None
        except Exception:
            result[topic] = None
    return result


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Best-effort runtime-metrics snapshot to JSON"
    )
    parser.add_argument(
        "--repo-root",
        default=os.environ.get(
            "REPO_ROOT",
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
        ),
        help="Repository root for git HEAD",
    )
    parser.add_argument(
        "--pattern",
        action="append",
        default=[
            "gzserver",
            "gzclient",
            "junior_ctrl",
            "fastlio_mapping",
            "laserMapping",
            "scan_to_pointcloud2",
            "ros(master|out)",
            "rviz",
            r"python.*building_generator",
        ],
        help="Process name regex patterns (repeatable)",
    )
    parser.add_argument(
        "--topic",
        action="append",
        default=["/clock"],
        help="ROS topic(s) for hz probe (repeatable)",
    )
    parser.add_argument(
        "--hz-window", type=int, default=5, help="Window size for rostopic hz"
    )
    parser.add_argument(
        "--no-ros", action="store_true", help="Skip all ROS-dependent probes"
    )
    args = parser.parse_args()

    repo_root = os.path.abspath(args.repo_root)

    snapshot: Dict[str, Any] = {
        "timestamp": time.time(),
        "iso_time": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "git_head": git_head(repo_root),
        "load": load_avg(),
        "mem_kb": mem_info(),
        "vmstat": vmstat_snapshot(),
        "processes": process_stats(args.pattern),
    }

    if not args.no_ros:
        snapshot["ros_nodes"] = ros_node_list()
        snapshot["ros_topics"] = ros_topic_list()
        if args.topic:
            snapshot["topic_hz"] = topic_hz(args.topic, args.hz_window)

    json.dump(snapshot, sys.stdout, indent=2, default=str)
    sys.stdout.write("\n")


if __name__ == "__main__":
    main()
