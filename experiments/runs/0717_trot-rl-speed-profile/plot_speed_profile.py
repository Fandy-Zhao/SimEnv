#!/usr/bin/env python3
"""Render reproducible trajectory and RTF/mobility figures from raw trials."""

import csv
import glob
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


ROOT = os.path.dirname(os.path.abspath(__file__))
RAW = os.path.join(ROOT, "raw")
rows = []
for metrics_path in sorted(glob.glob(os.path.join(RAW, "*", "trial_metrics.json"))):
    with open(metrics_path) as handle:
        metric = json.load(handle)
    metric["run_dir"] = os.path.dirname(metrics_path)
    rows.append(metric)

if len(rows) != 6:
    raise SystemExit("expected six trial_metrics.json files, found %d" % len(rows))

rows.sort(key=lambda row: (row["mode"], row["command_speed_mps"]))
with open(os.path.join(ROOT, "summary.json"), "w") as handle:
    json.dump({"schema_version": 1, "trials": rows}, handle, indent=2, sort_keys=True)
fields = ["mode", "command_speed_mps", "status", "real_time_factor",
          "actual_mean_horizontal_speed_mps", "tracking_ratio", "path_length_m",
          "latter_half_mean_horizontal_speed_mps",
          "net_displacement_m", "active_sim_elapsed_s", "active_wall_elapsed_s",
          "final_stop_mean_speed_mps", "min_base_z_m", "truth_finite"]
with open(os.path.join(ROOT, "summary.csv"), "w", newline="") as handle:
    writer = csv.DictWriter(handle, fieldnames=fields)
    writer.writeheader()
    writer.writerows([{field: row.get(field) for field in fields} for row in rows])

fig, axes = plt.subplots(2, 3, figsize=(14, 8), dpi=160, sharex=False, sharey=False)
for axis, row in zip(axes.flat, rows):
    with open(os.path.join(row["run_dir"], "ground_truth.csv"), newline="") as handle:
        trace = list(csv.DictReader(handle))
    x, y = [float(point["x"]) for point in trace], [float(point["y"]) for point in trace]
    axis.plot(x, y, color="#1976d2" if row["mode"] == "trotting" else "#d32f2f", lw=1.2)
    active = [point for point in trace if row["active"]["sim_start"] <= float(point["ros_time"]) <= row["active"]["sim_end"]]
    if active:
        axis.scatter([float(active[0]["x"])], [float(active[0]["y"])], c="#2e7d32", s=22, label="active start")
        axis.scatter([float(active[-1]["x"])], [float(active[-1]["y"])], c="#ef6c00", s=22, label="active end")
    axis.set_aspect("equal", adjustable="box")
    axis.grid(alpha=.3)
    axis.set_xlabel("world x [m]")
    axis.set_ylabel("world y [m]")
    axis.set_title("%s, cmd %.1f m/s\nactual %.3f m/s, RTF %.3f" %
                   (row["mode"], row["command_speed_mps"], row.get("actual_mean_horizontal_speed_mps", 0), row.get("real_time_factor", 0)))
axes[0, 0].legend(loc="best", fontsize=7)
fig.suptitle("Single-floor Gazebo truth trajectories (full fresh epochs)")
fig.tight_layout()
fig.savefig(os.path.join(ROOT, "trajectory_planar.png"))
plt.close(fig)

fig, axis = plt.subplots(figsize=(8, 5), dpi=160)
for mode, color, marker in (("trotting", "#1976d2", "o"), ("rl", "#d32f2f", "s")):
    selected = [row for row in rows if row["mode"] == mode]
    axis.scatter([row["real_time_factor"] for row in selected],
                 [row["actual_mean_horizontal_speed_mps"] for row in selected],
                 s=65, c=color, marker=marker, label=mode)
    for row in selected:
        axis.annotate("cmd %.1f" % row["command_speed_mps"],
                      (row["real_time_factor"], row["actual_mean_horizontal_speed_mps"]),
                      xytext=(5, 5), textcoords="offset points", fontsize=8)
axis.set_xlabel("real-time factor (sim seconds / wall seconds)")
axis.set_ylabel("actual mean horizontal speed [m/s]")
axis.set_title("Current simulation RTF and measured mobility")
axis.grid(alpha=.3)
axis.legend()
fig.tight_layout()
fig.savefig(os.path.join(ROOT, "rtf_mobility_relation.png"))
