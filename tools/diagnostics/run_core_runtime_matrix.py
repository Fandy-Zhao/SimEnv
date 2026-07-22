#!/usr/bin/env python3
"""Run the core competition RL RTF matrix with scoped process cleanup.

The runner intentionally starts `auto.sh` with `SKIP_GLOBAL_PROCESS_CLEANUP=1`,
`TERMINAL_BACKEND=direct`, and an empty DISPLAY so child processes stay under
the runner-owned process group as much as possible. Cleanup sends SIGKILL only
to that process group.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import re
import signal
import statistics
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


CORE_CASES = {
    "M0": {
        "competition": False,
        "mapping": False,
        "policy_loaded": False,
        "rl_active": False,
        "torch_limit": False,
        "env": {
            "WORLD_MODE": "earth",
            "PHYSICS_PROFILE": "normal",
            "GUI": "false",
        },
    },
    "M1": {
        "competition": True,
        "mapping": False,
        "policy_loaded": False,
        "rl_active": False,
        "torch_limit": False,
        "env": {
            "WORLD_MODE": "competition",
            "PHYSICS_PROFILE": "normal",
            "GUI": "false",
            "START_CONTROLLER": "1",
            "ENABLE_SENSOR_DATA": "0",
            "ENABLE_POINTCLOUD_CONVERTER": "0",
            "ENABLE_FAST_LIO2": "0",
            "ENABLE_RVIZ": "0",
            "ENABLE_REFEREE_ODOM": "0",
            "ENABLE_GROUND_TRUTH": "0",
            "START_BUILDING_CONTROL": "0",
        },
    },
    "M4": {
        "competition": True,
        "mapping": True,
        "policy_loaded": False,
        "rl_active": False,
        "torch_limit": False,
        "env": {
            "WORLD_MODE": "competition",
            "PHYSICS_PROFILE": "normal",
            "GUI": "false",
            "START_CONTROLLER": "1",
            "ENABLE_SENSOR_DATA": "1",
            "ENABLE_POINTCLOUD_CONVERTER": "1",
            "ENABLE_FAST_LIO2": "1",
            "ENABLE_RVIZ": "0",
            "ENABLE_REFEREE_ODOM": "0",
            "ENABLE_GROUND_TRUTH": "0",
            "START_BUILDING_CONTROL": "0",
        },
    },
    "M5": {
        "competition": True,
        "mapping": True,
        "policy_loaded": True,
        "rl_active": False,
        "torch_limit": False,
        "env": {},
    },
    "M6": {
        "competition": True,
        "mapping": False,
        "policy_loaded": True,
        "rl_active": True,
        "torch_limit": False,
        "env": {},
    },
    "M7": {
        "competition": True,
        "mapping": True,
        "policy_loaded": True,
        "rl_active": True,
        "torch_limit": False,
        "env": {},
    },
    "M8": {
        "competition": True,
        "mapping": True,
        "policy_loaded": True,
        "rl_active": True,
        "torch_limit": True,
        "env": {},
    },
}

for case_name in ("M5", "M7", "M8"):
    CORE_CASES[case_name]["env"] = dict(CORE_CASES["M4"]["env"])
for case_name in ("M6",):
    CORE_CASES[case_name]["env"] = dict(CORE_CASES["M1"]["env"])
for case_name in ("M5", "M6", "M7", "M8"):
    CORE_CASES[case_name]["env"]["RL_POLICY_PATH"] = (
        "src/unitree_guide/logs/policy_act_inference_stair.pt"
    )
for key in ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    CORE_CASES["M8"]["env"][key] = "1"

PROCESS_PATTERNS = [
    "gzserver",
    "gzclient",
    "rviz",
    "junior_ctrl",
    "fastlio",
    "laserMapping",
    "scan_to_pointcloud2",
    "rosmaster",
    "rosout",
]

TOPICS = [
    "/clock",
    "/scan",
    "/scan_pointcloud2",
    "/livox/imu",
    "/Odometry",
    "/cloud_registered",
    "/state_estimation",
    "/registered_scan",
]


def run_text(cmd: list[str], timeout: float = 5.0, env: dict[str, str] | None = None) -> str:
    try:
        out = subprocess.check_output(cmd, stderr=subprocess.STDOUT, timeout=timeout, env=env)
        return out.decode(errors="replace")
    except subprocess.CalledProcessError as exc:
        return exc.output.decode(errors="replace")
    except Exception as exc:
        return f"ERROR: {exc}"


def get_clock(env: dict[str, str]) -> float | None:
    text = run_text(["rostopic", "echo", "-n", "1", "/clock"], timeout=4, env=env)
    secs = re.search(r"secs:\s*([0-9]+)", text)
    nsecs = re.search(r"nsecs:\s*([0-9]+)", text)
    if not secs:
        return None
    return float(secs.group(1)) + (float(nsecs.group(1)) / 1_000_000_000.0 if nsecs else 0.0)


def pgrep_snapshot() -> str:
    return run_text(["pgrep", "-af", "|".join(PROCESS_PATTERNS)], timeout=5)


def process_snapshot_rows() -> list[tuple[int, str]]:
    text = run_text(["ps", "-eo", "pid,args"], timeout=5)
    rows: list[tuple[int, str]] = []
    for line in text.splitlines()[1:]:
        parts = line.strip().split(None, 1)
        if len(parts) != 2:
            continue
        try:
            pid = int(parts[0])
        except ValueError:
            continue
        if any(pat in parts[1] for pat in PROCESS_PATTERNS):
            rows.append((pid, parts[1]))
    return rows


def cleanup_scoped_processes(repo: Path, initial_pids: set[int], case_dir: Path) -> None:
    current_pid = os.getpid()
    rows = process_snapshot_rows()
    targets = [
        pid for pid, cmd in rows
        if pid != current_pid and (pid not in initial_pids or str(repo) in cmd)
    ]
    (case_dir / "cleanup_pids.txt").write_text(
        "\n".join(str(pid) for pid in sorted(set(targets))) + ("\n" if targets else "")
    )
    for sig in (signal.SIGTERM, signal.SIGKILL):
        for pid in sorted(set(targets)):
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        time.sleep(2)


def ps_snapshot() -> str:
    return run_text(
        ["ps", "-Leo", "pid,tid,psr,pcpu,pmem,rss,vsz,comm,args", "--sort=-pcpu"],
        timeout=5,
    )


def parse_ps(text: str) -> dict[str, dict[str, float]]:
    totals: dict[str, dict[str, float]] = {}
    for line in text.splitlines()[1:]:
        parts = line.split(None, 8)
        if len(parts) < 9:
            continue
        args = parts[8]
        matched = None
        for pat in ("gzserver", "junior_ctrl", "scan_to_pointcloud2", "fastlio", "laserMapping", "rviz", "gzclient"):
            if pat in args or pat in parts[7]:
                matched = pat
                break
        if not matched:
            continue
        try:
            totals.setdefault(matched, {"cpu": 0.0, "rss_kb": 0.0, "threads": 0.0})
            totals[matched]["cpu"] += float(parts[3])
            totals[matched]["rss_kb"] += float(parts[5])
            totals[matched]["threads"] += 1
        except ValueError:
            pass
    return totals


def topic_hz(topic: str, env: dict[str, str], window: int) -> float | None:
    text = run_text(["rostopic", "hz", "-w", str(window), topic], timeout=window + 5, env=env)
    match = re.search(r"average rate:\s*([0-9.]+)", text)
    return float(match.group(1)) if match else None


def publish_state(state: int, env: dict[str, str]) -> None:
    run_text(["rostopic", "pub", "/fsm/state_cmd", "std_msgs/Int8", f"data: {state}", "-1"], timeout=8, env=env)


def publish_zero_cmd(env: dict[str, str]) -> None:
    msg = "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    run_text(["rostopic", "pub", "/cmd_vel", "geometry_msgs/Twist", msg, "-1"], timeout=8, env=env)


def summarize_rtf(rows: list[dict[str, Any]], warmup_sim: float) -> dict[str, Any]:
    values = [
        float(r["rtf_delta"])
        for r in rows
        if r.get("phase") == "sample" and r.get("rtf_delta") not in ("", None)
    ]
    if not values:
        return {"count": 0}
    ordered = sorted(values)
    p10 = ordered[max(0, int(len(ordered) * 0.10) - 1)]
    p90 = ordered[min(len(ordered) - 1, int(len(ordered) * 0.90))]
    sim_values = [float(r["sim_time"]) for r in rows if r.get("sim_time") not in ("", None)]
    return {
        "count": len(values),
        "mean": statistics.fmean(values),
        "median": statistics.median(values),
        "p10": p10,
        "p90": p90,
        "min": min(values),
        "max": max(values),
        "sample_sim_duration": max(sim_values) - warmup_sim if sim_values else 0.0,
    }


def analyze_timing(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {"exists": False}
    rows = list(csv.DictReader(path.open()))
    action_rows = [r for r in rows if r.get("event") == "ACTION"]
    waits = [r for r in rows if r.get("event") == "POLICY_WAIT"]
    result: dict[str, Any] = {
        "exists": True,
        "rows": len(rows),
        "action_rows": len(action_rows),
        "policy_wait_rows": len(waits),
    }
    wall = [int(r["policy_wall_time_ns"]) for r in action_rows if r.get("policy_wall_time_ns")]
    if len(wall) >= 2:
        intervals = [(b - a) / 1e9 for a, b in zip(wall, wall[1:]) if b > a]
        if intervals:
            result["policy_hz_wall"] = 1.0 / statistics.median(intervals)
            result["policy_interval_median_s"] = statistics.median(intervals)
    wait_us = [int(r["policy_wait_wall_elapsed_us"]) for r in waits if r.get("policy_wait_wall_elapsed_us")]
    if wait_us:
        result["policy_wait_wall_us_median"] = statistics.median(wait_us)
        result["policy_wait_wall_us_p95"] = sorted(wait_us)[min(len(wait_us) - 1, int(len(wait_us) * 0.95))]
        result["policy_wait_wall_us_max"] = max(wait_us)
    return result


def run_case(repo: Path, out_root: Path, case: str, args: argparse.Namespace) -> dict[str, Any]:
    case_cfg = CORE_CASES[case]
    case_dir = out_root / case
    case_dir.mkdir(parents=True, exist_ok=True)
    timing_path = case_dir / "timing.csv"
    rl_diag_path = case_dir / "rl_deploy.csv"
    env = os.environ.copy()
    env.update({
        "PYTHONNOUSERSITE": "1",
        "SKIP_GLOBAL_PROCESS_CLEANUP": "true",
        "TERMINAL_BACKEND": "direct",
        "DISPLAY": "",
        "WAYLAND_DISPLAY": "",
        "TIMING_DIAGNOSTICS_ENABLED": "1" if case_cfg["policy_loaded"] else "0",
        "TIMING_DIAGNOSTICS_PATH": str(timing_path),
        "AUTO_UNPAUSE": "1",
    })
    for key, value in case_cfg["env"].items():
        env[key] = str(repo / value) if key == "RL_POLICY_PATH" and not value.startswith("/") else value
    if case_cfg["policy_loaded"]:
        env["UNITREE_RL_DIAG_PATH"] = str(rl_diag_path)

    (case_dir / "environment.txt").write_text(
        "\n".join(f"{k}={env[k]}" for k in sorted(env) if k in case_cfg["env"] or k in {
            "SKIP_GLOBAL_PROCESS_CLEANUP", "TERMINAL_BACKEND", "TIMING_DIAGNOSTICS_ENABLED",
            "TIMING_DIAGNOSTICS_PATH", "UNITREE_RL_DIAG_PATH", "AUTO_UNPAUSE",
        }) + "\n"
    )
    extra_command_exports = ""
    if case_cfg["policy_loaded"]:
        extra_command_exports = f"export UNITREE_RL_DIAG_PATH={str(rl_diag_path)!r}\n"
    (case_dir / "command.sh").write_text(
        "#!/usr/bin/env bash\nset -euo pipefail\ncd " + str(repo) + "\n" +
        "\n".join(f"export {k}={v!r}" for k, v in case_cfg["env"].items()) +
        "\nexport SKIP_GLOBAL_PROCESS_CLEANUP=true\nexport TERMINAL_BACKEND=direct\n" +
        f"export TIMING_DIAGNOSTICS_ENABLED={env['TIMING_DIAGNOSTICS_ENABLED']!r}\n"
        f"export TIMING_DIAGNOSTICS_PATH={str(timing_path)!r}\n" +
        extra_command_exports +
        "bash ./auto.sh\n"
    )

    initial_pids = {pid for pid, _cmd in process_snapshot_rows()}
    (case_dir / "processes_before.txt").write_text(pgrep_snapshot())
    stdout = (case_dir / "stdout.log").open("wb")
    stderr = (case_dir / "stderr.log").open("wb")
    proc = subprocess.Popen(
        ["bash", str(repo / "auto.sh")],
        cwd=str(repo),
        env=env,
        stdout=stdout,
        stderr=stderr,
        preexec_fn=os.setsid,
    )
    start_wall = time.monotonic()
    rows: list[dict[str, Any]] = []
    previous_wall: float | None = None
    previous_sim: float | None = None
    first_sim: float | None = None
    rl_entered = False
    rl_wall: float | None = None
    rl_sim: float | None = None
    verdict = "CASE_PARTIAL"

    nodes_text = ""
    topics_text = ""
    topic_rates: dict[str, float | None] = {}
    mapping = ""
    processes_text = ""
    try:
        while time.monotonic() - start_wall < args.wall_timeout:
            if proc.poll() is not None:
                verdict = "PROCESS_EXITED"
                break
            sim = get_clock(env)
            wall = time.monotonic()
            if sim is not None and first_sim is None:
                first_sim = sim
            sim_elapsed = sim - first_sim if sim is not None and first_sim is not None else None

            if sim_elapsed is not None and sim_elapsed >= 2.0:
                publish_state(2, env)
            if case_cfg["rl_active"] and not rl_entered and sim_elapsed is not None and sim_elapsed >= args.rl_enter_after_sim:
                publish_zero_cmd(env)
                publish_state(6, env)
                rl_entered = True
                rl_wall = wall - start_wall
                rl_sim = sim

            rtf = ""
            if previous_wall is not None and previous_sim is not None and sim is not None and sim >= previous_sim:
                wall_dt = wall - previous_wall
                rtf = (sim - previous_sim) / wall_dt if wall_dt > 0 else ""
            ps_text = ps_snapshot()
            totals = parse_ps(ps_text)
            phase = "startup"
            if sim_elapsed is not None and sim_elapsed >= args.warmup_sim:
                phase = "sample"
            rows.append({
                "wall_elapsed": wall - start_wall,
                "sim_time": sim if sim is not None else "",
                "sim_elapsed": sim_elapsed if sim_elapsed is not None else "",
                "rtf_delta": rtf,
                "phase": phase,
                "gzserver_cpu": totals.get("gzserver", {}).get("cpu", ""),
                "junior_ctrl_cpu": totals.get("junior_ctrl", {}).get("cpu", ""),
                "fastlio_cpu": totals.get("fastlio", totals.get("laserMapping", {})).get("cpu", ""),
                "scan_adapter_cpu": totals.get("scan_to_pointcloud2", {}).get("cpu", ""),
                "junior_ctrl_threads": totals.get("junior_ctrl", {}).get("threads", ""),
            })
            previous_wall = wall
            previous_sim = sim if sim is not None else previous_sim
            enough_sample = sim_elapsed is not None and sim_elapsed >= args.warmup_sim + args.sample_sim
            enough_rl = not case_cfg["rl_active"] or (
                rl_sim is not None and sim is not None and sim - rl_sim >= args.rl_sample_sim
            )
            if enough_sample and enough_rl:
                verdict = "CASE_COMPLETE"
                break
            time.sleep(args.interval)
        processes_text = ps_snapshot()
        nodes_text = run_text(["rosnode", "list"], env=env)
        topics_text = run_text(["rostopic", "list"], env=env)
        topic_rates = {topic: topic_hz(topic, env, args.hz_window) for topic in TOPICS}
        mapping = run_text([sys.executable, str(repo / "tools/diagnostics/check_mapping_pipeline.py")], timeout=20, env=env)
    finally:
        stdout.close()
        stderr.close()
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        except Exception as exc:
            (case_dir / "cleanup_error.txt").write_text(str(exc) + "\n")
        try:
            proc.wait(timeout=10)
        except Exception:
            pass
        cleanup_scoped_processes(repo, initial_pids, case_dir)

    with (case_dir / "metrics.csv").open("w", newline="") as fh:
        fieldnames = [
            "wall_elapsed", "sim_time", "sim_elapsed", "rtf_delta", "phase",
            "gzserver_cpu", "junior_ctrl_cpu", "fastlio_cpu", "scan_adapter_cpu",
            "junior_ctrl_threads",
        ]
        writer = csv.DictWriter(fh, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)

    (case_dir / "processes.txt").write_text(processes_text)
    (case_dir / "processes_after.txt").write_text(pgrep_snapshot())
    (case_dir / "nodes.txt").write_text(nodes_text)
    (case_dir / "topics.txt").write_text(topics_text)
    (case_dir / "topic_rates.json").write_text(json.dumps(topic_rates, indent=2) + "\n")
    (case_dir / "mapping_pipeline.json").write_text(mapping)
    result = {
        "case": case,
        "verdict": verdict,
        "competition": case_cfg["competition"],
        "mapping": case_cfg["mapping"],
        "policy_loaded": case_cfg["policy_loaded"],
        "rl_active": case_cfg["rl_active"],
        "torch_limit": case_cfg["torch_limit"],
        "rl_enter_wall_elapsed": rl_wall,
        "rl_enter_sim_time": rl_sim,
        "rtf": summarize_rtf(rows, (first_sim or 0.0) + args.warmup_sim),
        "topic_rates": topic_rates,
        "timing": analyze_timing(timing_path),
    }
    (case_dir / "metrics.json").write_text(json.dumps(result, indent=2) + "\n")
    (case_dir / "result.md").write_text(
        f"# {case}\n\nVerdict: `{verdict}`\n\n"
        f"Mean RTF: `{result['rtf'].get('mean', 'NA')}`\n"
        f"p10 RTF: `{result['rtf'].get('p10', 'NA')}`\n"
        f"Policy Hz wall: `{result['timing'].get('policy_hz_wall', 'NA')}`\n"
    )
    return result


def write_matrix(out_root: Path, results: list[dict[str, Any]]) -> None:
    (out_root / "runtime_matrix.json").write_text(json.dumps(results, indent=2) + "\n")
    with (out_root / "runtime_matrix.csv").open("w", newline="") as fh:
        fieldnames = [
            "Case", "Competition", "Mapping", "Policy loaded", "RL active",
            "Torch limit", "Mean RTF", "p10 RTF", "gzserver CPU", "RL CPU",
            "PC2 Hz", "LIO odom Hz", "Verdict",
        ]
        writer = csv.DictWriter(fh, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        for r in results:
            writer.writerow({
                "Case": r["case"],
                "Competition": int(r["competition"]),
                "Mapping": int(r["mapping"]),
                "Policy loaded": int(r["policy_loaded"]),
                "RL active": int(r["rl_active"]),
                "Torch limit": int(r["torch_limit"]),
                "Mean RTF": r["rtf"].get("mean", ""),
                "p10 RTF": r["rtf"].get("p10", ""),
                "gzserver CPU": "",
                "RL CPU": "",
                "PC2 Hz": r["topic_rates"].get("/scan_pointcloud2"),
                "LIO odom Hz": r["topic_rates"].get("/Odometry"),
                "Verdict": r["verdict"],
            })


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", default=str(Path(__file__).resolve().parents[2]))
    parser.add_argument("--out-dir", default="experiments/runs/0723_competition_rl_rtf_collapse")
    parser.add_argument("--cases", nargs="+", default=["M0", "M1", "M4", "M5", "M6", "M7", "M8"])
    parser.add_argument("--warmup-sim", type=float, default=15.0)
    parser.add_argument("--sample-sim", type=float, default=30.0)
    parser.add_argument("--rl-enter-after-sim", type=float, default=5.0)
    parser.add_argument("--rl-sample-sim", type=float, default=20.0)
    parser.add_argument("--wall-timeout", type=float, default=600.0)
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--hz-window", type=int, default=5)
    args = parser.parse_args()

    repo = Path(args.repo_root).resolve()
    out_root = (repo / args.out_dir).resolve()
    results = []
    for case in args.cases:
        if case not in CORE_CASES:
            raise SystemExit(f"unknown case: {case}")
        results.append(run_case(repo, out_root, case, args))
        write_matrix(out_root, results)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
