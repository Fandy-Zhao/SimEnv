#!/usr/bin/env python3
"""
render_single_floor_exploration.py

Offline visualization renderer for single-floor exploration results.
Requires NO ROS, NO Gazebo, NO roscore — reads saved artifacts from disk.

Generates:
  plots/final_exploration_overview.png
  plots/final_exploration_overview_meta.yaml

Four core layers:
  1. Ground Truth Layout   — building walls, rooms, lobby, corridor, furniture
  2. Explored Area          — known area from OccupancyGrid (free + occupied)
  3. Robot Trajectory       — full /Odometry path with START/END markers
  4. DSV Goals              — reached (circle) and unreached (X), numbered

Usage:
  python3 tools/render_single_floor_exploration.py --run-dir <RUN_DIR>
  python3 tools/render_single_floor_exploration.py --run-dir <RUN_DIR> \\
      --layout-metadata generated_building/layout_metadata.json
  python3 tools/render_single_floor_exploration.py --help
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from typing import Any, Optional


try:
    from simenv_navigation_bridge.exploration_metrics import (
        DEFAULT_ROUTE_MAX_SPEED_MPS,
        DEFAULT_ROUTE_MAX_STEP_M,
        DEFAULT_TARGET_FRAME,
        RoutePolicy,
        compute_route_length,
    )
except ModuleNotFoundError:
    # Support direct execution from an unbuilt source checkout.
    _REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    _BRIDGE_SRC = os.path.join(
        _REPO_ROOT, "src", "navigation", "simenv_navigation_bridge", "src")
    sys.path.insert(0, _BRIDGE_SRC)
    from simenv_navigation_bridge.exploration_metrics import (
        DEFAULT_ROUTE_MAX_SPEED_MPS,
        DEFAULT_ROUTE_MAX_STEP_M,
        DEFAULT_TARGET_FRAME,
        RoutePolicy,
        compute_route_length,
    )

# ---------------------------------------------------------------------------
# Coordinate transform: world (truth) ⇄ map (odom) frame
# ---------------------------------------------------------------------------
#
# The robot spawns at world position (spawn_x, spawn_y, spawn_yaw) per
# scene_manifest.json.  FAST-LIO2 initialises its odometry at the robot's
# starting pose, so the map-frame origin is the spawn point with identity
# orientation.
#
#   T_truth_to_map:   p_map = R⁻¹(yaw) · (p_world − spawn)
#   T_map_to_truth:   p_world = R(yaw) · p_map + spawn

DEFAULT_SPAWN_X = 0.0
DEFAULT_SPAWN_Y = 2.3
DEFAULT_SPAWN_YAW = math.radians(90)  # 1.5708 rad


def _rot_2d(yaw: float):
    c, s = math.cos(yaw), math.sin(yaw)
    return [[c, -s], [s, c]]


def _vec2_mul(R, v):
    return (R[0][0] * v[0] + R[0][1] * v[1],
            R[1][0] * v[0] + R[1][1] * v[1])


def world_to_map(wx: float, wy: float,
                 spawn_x: float, spawn_y: float, spawn_yaw: float):
    R_inv = _rot_2d(-spawn_yaw)
    return _vec2_mul(R_inv, (wx - spawn_x, wy - spawn_y))


def map_to_world(mx: float, my: float,
                 spawn_x: float, spawn_y: float, spawn_yaw: float):
    R = _rot_2d(spawn_yaw)
    wx, wy = _vec2_mul(R, (mx, my))
    return (wx + spawn_x, wy + spawn_y)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _safe_float(val, default=float("nan")):
    try:
        v = float(val)
        return v if math.isfinite(v) else default
    except (ValueError, TypeError):
        return default


def _load_yaml(path: str):
    import yaml as _yaml
    with open(path, "r") as fh:
        return _yaml.safe_load(fh)


def _rect_corners_map(rect_bounds: dict, sx, sy, syaw):
    """Convert a world-frame bounds dict to map-frame (x,y) polygon corners."""
    x_min, x_max = rect_bounds["x_min"], rect_bounds["x_max"]
    y_min, y_max = rect_bounds["y_min"], rect_bounds["y_max"]
    corners_w = [(x_min, y_min), (x_max, y_min),
                 (x_max, y_max), (x_min, y_max)]
    corners_m = [world_to_map(wx, wy, sx, sy, syaw) for (wx, wy) in corners_w]
    return corners_m


# ---------------------------------------------------------------------------
# Data loaders
# ---------------------------------------------------------------------------

def load_layout_metadata(path: str):
    """Load ground truth layout from layout_metadata.json."""
    return _load_yaml(path)


def load_trajectory(run_dir: str):
    """Load robot trajectory from route/trajectory.csv.

    Returns (points, meta). Non-map legacy rows remain available for diagnostics,
    but ``meta['overlay_allowed']`` is false so they are not plotted on a map.
    """
    csv_path = os.path.join(run_dir, "route", "trajectory.csv")
    points = []
    meta = {"point_count": 0, "valid_point_count": 0,
            "length_m": 0.0, "nan_detected": False,
            "overlay_allowed": False}
    if not os.path.isfile(csv_path):
        meta["error"] = f"trajectory.csv not found: {csv_path}"
        return points, meta

    samples = []
    frames = set()
    with open(csv_path, "r") as fh:
        reader = csv.DictReader(fh)
        meta["columns"] = list(reader.fieldnames or [])
        for row in reader:
            x = _safe_float(row.get("x"))
            y = _safe_float(row.get("y"))
            z = _safe_float(row.get("z"), 0.0)
            t = _safe_float(row.get("sim_time"))
            frame_id = str(row.get("frame_id", "")).strip()
            valid = all(math.isfinite(value) for value in (x, y, z, t))
            meta["point_count"] += 1
            samples.append({
                "x": x, "y": y, "z": z, "sim_time": t,
                "frame_id": frame_id,
            })
            if not valid:
                meta["nan_detected"] = True
                continue
            meta["valid_point_count"] += 1
            frames.add(frame_id or "<missing>")
            points.append((x, y, t, valid))

    config = load_recorder_config(run_dir) or {}
    target_frame = str(config.get(
        "trajectory_target_frame", DEFAULT_TARGET_FRAME)).strip()
    policy = RoutePolicy(
        target_frame=target_frame,
        max_speed_mps=float(config.get(
            "route_max_speed_mps", DEFAULT_ROUTE_MAX_SPEED_MPS)),
        max_step_m=float(config.get(
            "route_max_step_m", DEFAULT_ROUTE_MAX_STEP_M)),
    )
    computed = compute_route_length(samples, policy).to_dict()
    metrics_path = os.path.join(run_dir, "route", "metrics.yaml")
    authoritative = None
    if os.path.isfile(metrics_path):
        loaded = _load_yaml(metrics_path)
        required = {
            "route_length_m", "route_total_segments",
            "route_accepted_segments", "route_rejected_segments",
            "route_reject_reasons",
        }
        if isinstance(loaded, dict) and required.issubset(loaded):
            authoritative = loaded

    route_metrics = authoritative or computed
    meta.update(route_metrics)
    meta["length_m"] = float(route_metrics["route_length_m"])
    meta["route_metrics_source"] = (
        "route/metrics.yaml" if authoritative is not None
        else "shared_policy_recalculation")
    meta["route_max_speed_mps"] = policy.max_speed_mps
    meta["route_max_step_m"] = policy.max_step_m
    meta["target_frame"] = policy.target_frame
    meta["frames"] = sorted(frames)
    meta["overlay_allowed"] = frames == {policy.target_frame}
    if not meta["overlay_allowed"]:
        meta["frame_warning"] = (
            "TRAJECTORY_FRAME_MISMATCH: trajectory frames "
            f"{sorted(frames)} cannot be safely overlaid on "
            f"{policy.target_frame}; trajectory layer suppressed"
        )
        print(f"[render] WARNING: {meta['frame_warning']}")

    # Supplement from trajectory.yaml if present
    yaml_path = os.path.join(run_dir, "route", "trajectory.yaml")
    if os.path.isfile(yaml_path):
        try:
            ty = _load_yaml(yaml_path)
            meta["frame_id"] = ty.get("frame_id", "")
            meta["length_from_yaml"] = ty.get("trajectory_length_m_2d",
                                               meta["length_m"])
        except Exception:
            pass

    return points, meta


def load_goals(run_dir: str):
    """Load DSV goals from goals/goals_unique.csv.

    Returns (goals, meta): goals is list of dicts, meta is summary.
    """
    csv_path = os.path.join(run_dir, "goals", "goals_unique.csv")
    goals = []
    meta = {"total": 0, "reached": 0, "unreached": 0}
    if not os.path.isfile(csv_path):
        meta["error"] = f"goals_unique.csv not found: {csv_path}"
        return goals, meta

    with open(csv_path, "r") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            x = _safe_float(row.get("x"))
            y = _safe_float(row.get("y"))
            reached_str = str(row.get("reached", "")).strip().lower()
            reached = reached_str in ("true", "1", "yes")
            status = row.get("status", "")
            idx = row.get("goal_index_unique", row.get("goal_index", "?"))
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            g = {"x": x, "y": y, "reached": reached, "status": status,
                 "index": str(idx)}
            goals.append(g)
            meta["total"] += 1
            if reached:
                meta["reached"] += 1
            else:
                meta["unreached"] += 1
    return goals, meta


def load_map_data(run_dir: str):
    """Load OccupancyGrid data from map/ directory.

    Returns (pgm_grid, map_meta): grid as 2D numpy array (PGM values:
    0=occupied, 254=free, 205=unknown), metadata dict.
    """
    import numpy as np
    csv_path = os.path.join(run_dir, "map", "occupancy_grid.csv")
    yaml_path = os.path.join(run_dir, "map", "map.yaml")
    meta_path = os.path.join(run_dir, "map", "metadata.yaml")

    meta = {}
    pgm_grid = None

    # Load map.yaml for resolution and origin
    if os.path.isfile(yaml_path):
        try:
            my = _load_yaml(yaml_path)
            meta["resolution"] = float(my.get("resolution", 0.1))
            origin = my.get("origin", [0, 0, 0])
            meta["origin_x"] = float(origin[0])
            meta["origin_y"] = float(origin[1])
        except Exception:
            meta["resolution"] = 0.1
            meta["origin_x"] = -20.0
            meta["origin_y"] = -20.0
    else:
        meta["resolution"] = 0.1
        meta["origin_x"] = -20.0
        meta["origin_y"] = -20.0

    # Load occupancy_grid.csv
    if os.path.isfile(csv_path):
        try:
            pgm_grid = np.loadtxt(csv_path, delimiter=",", dtype=np.float32)
            if pgm_grid.ndim != 2:
                meta["error"] = "occupancy_grid.csv is not 2D"
                pgm_grid = None
            else:
                meta["height"], meta["width"] = pgm_grid.shape
        except Exception as e:
            meta["error"] = f"Cannot read occupancy_grid.csv: {e}"

    # Supplement with metadata.yaml
    if os.path.isfile(meta_path):
        try:
            mm = _load_yaml(meta_path)
            for k in ("width", "height", "resolution", "origin_x", "origin_y",
                      "known_area_m2", "free_area_m2", "occupied_area_m2",
                      "unknown_cell_count", "map_frame"):
                if k in mm and k not in meta:
                    meta[k] = mm[k]
            # Prefer metadata.yaml values for known/free/occupied
            if "known_area_m2" in mm:
                meta["known_area_m2"] = mm["known_area_m2"]
            if "free_area_m2" in mm:
                meta["free_area_m2"] = mm["free_area_m2"]
            if "occupied_area_m2" in mm:
                meta["occupied_area_m2"] = mm["occupied_area_m2"]
        except Exception:
            pass

    return pgm_grid, meta


def load_timing(run_dir: str):
    """Load timing info from timing/timing.yaml."""
    path = os.path.join(run_dir, "timing", "timing.yaml")
    if os.path.isfile(path):
        return _load_yaml(path)
    return {}


def load_summary_fields(run_dir: str):
    """Extract a few key fields from summary.md (best-effort)."""
    info: dict[str, str] = {}
    path = os.path.join(run_dir, "summary.md")
    if not os.path.isfile(path):
        return info
    with open(path, "r") as fh:
        for line in fh:
            line = line.strip()
            if line.startswith("- **Run ID**:"):
                info["run_id"] = line.split(":", 1)[-1].strip()
            elif line.startswith("- **Final verdict**:"):
                info["verdict"] = line.split(":", 1)[-1].strip()
    return info


def load_recorder_config(run_dir: str):
    """Load recorder config for topic names, etc."""
    path = os.path.join(run_dir, "config", "recorder_config.yaml")
    if os.path.isfile(path):
        return _load_yaml(path)
    return {}


# ---------------------------------------------------------------------------
# Layout → truth-json export
# ---------------------------------------------------------------------------

def layout_to_truth_json(layout: dict) -> dict:
    """Extract a simplified truth layout for standalone reuse."""
    floors = layout.get("floors", [])
    floor0 = floors[0] if floors else {}
    rooms = []
    for r in floor0.get("rooms", []):
        rooms.append({
            "id": r.get("id"),
            "bounds": r.get("bounds"),
            "room_type": r.get("room_type"),
            "door_pose": r.get("door_pose"),
        })
    return {
        "footprint": layout.get("footprint"),
        "wall_height": layout.get("wall_height"),
        "floor_0": {
            "lobby_bounds": floor0.get("lobby_bounds"),
            "corridor_bounds": floor0.get("corridor_bounds"),
            "stair_bounds": floor0.get("stair_bounds"),
            "elevator_bounds": floor0.get("elevator_bounds"),
            "rooms": rooms,
        },
        "metadata": layout.get("metadata"),
    }


# ---------------------------------------------------------------------------
# Render
# ---------------------------------------------------------------------------

def render_overview(run_dir: str, layout: dict, spawn: tuple,
                    output_dir_plots: str, meta_out_path: str):
    """Render the 4-layer final exploration overview image and metadata."""
    import matplotlib
    matplotlib.use("Agg")  # noqa — must be before pyplot import
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as MplPolygon
    import numpy as np

    sx, sy, syaw = spawn

    # ── Load data ──
    traj_pts, traj_meta = load_trajectory(run_dir)
    goals, goal_meta = load_goals(run_dir)
    pgm_grid, map_meta = load_map_data(run_dir)
    timing = load_timing(run_dir)
    summary_info = load_summary_fields(run_dir)
    run_id = summary_info.get("run_id", os.path.basename(os.path.abspath(run_dir)))

    seed = ""
    if layout:
        seed = str(layout.get("metadata", {}).get("seed", ""))

    # ── Figure ──
    fig, ax = plt.subplots(figsize=(14, 12))

    # ----------------------------------------------------------------
    # Layer 2: EXPLORED AREA (drawn first to serve as background)
    # ----------------------------------------------------------------
    explored_poly = None
    if pgm_grid is not None and pgm_grid.size > 0:
        height, width = pgm_grid.shape
        res = map_meta.get("resolution", 0.1)
        ox = map_meta.get("origin_x", -20.0)
        oy = map_meta.get("origin_y", -20.0)

        # PGM values: 0=occupied, 254=free, 205=unknown
        # Known = value != 205
        known_mask = (pgm_grid != 205)

        # Show unknown area as light gray
        unknown_rgba = np.zeros((height, width, 4), dtype=np.float32)
        unknown_rgba[~known_mask] = [0.85, 0.85, 0.85, 0.3]  # gray unknown
        unknown_rgba[known_mask] = [1.0, 1.0, 1.0, 0.0]       # transparent

        extent = [ox, ox + width * res, oy, oy + height * res]
        ax.imshow(unknown_rgba, extent=extent, origin='upper',
                  aspect='equal', interpolation='none', zorder=1)

        # Show explored area as light blue-green overlay
        explored_rgba = np.zeros((height, width, 4), dtype=np.float32)
        # free cells → pale cyan
        free_mask = (pgm_grid == 254)
        occupied_mask = (pgm_grid == 0)
        explored_rgba[free_mask] = [0.6, 1.0, 0.6, 0.35]      # green tint, free
        explored_rgba[occupied_mask] = [0.2, 0.2, 0.2, 0.55]  # dark, occupied
        explored_rgba[~known_mask] = [1.0, 1.0, 1.0, 0.0]     # transparent

        ax.imshow(explored_rgba, extent=extent, origin='upper',
                  aspect='equal', interpolation='none', zorder=2)

        # Record explored area stats for metadata
        exp_meta = {
            "known_area_m2": map_meta.get("known_area_m2"),
            "free_area_m2": map_meta.get("free_area_m2"),
            "occupied_area_m2": map_meta.get("occupied_area_m2"),
            "unknown_area_m2": map_meta.get("unknown_cell_count",
                                             0) * res * res if "unknown_cell_count" in map_meta else None,
        }
    else:
        exp_meta = {}
        ax.set_facecolor("whitesmoke")

    # ----------------------------------------------------------------
    # Layer 1: GROUND TRUTH LAYOUT (overlaid on explored area)
    # ----------------------------------------------------------------
    layout_plotted = False
    if layout:
        floors = layout.get("floors", [])
        if floors:
            floor0 = floors[0]

            # --- Outer wall (footprint) ---
            fp = layout.get("footprint", {})
            if fp:
                fp_w = fp.get("width", 20.0)
                fp_l = fp.get("length", 36.0)
                corners_w = [(-fp_w / 2, 0), (fp_w / 2, 0),
                             (fp_w / 2, fp_l), (-fp_w / 2, fp_l)]
                corners_m = [world_to_map(wx, wy, sx, sy, syaw)
                             for (wx, wy) in corners_w]
                poly = MplPolygon(corners_m, closed=True, fill=False,
                                  edgecolor="black", linewidth=2.5, zorder=3,
                                  label="Building footprint")
                ax.add_patch(poly)

            # --- Lobby ---
            lb = floor0.get("lobby_bounds")
            if lb:
                c = _rect_corners_map(lb, sx, sy, syaw)
                poly = MplPolygon(c, closed=True, fill=True, facecolor="lightyellow",
                                  edgecolor="gray", linewidth=0.8, zorder=3,
                                  alpha=0.3, label="Lobby")
                ax.add_patch(poly)

            # --- Corridor ---
            cb = floor0.get("corridor_bounds")
            if cb:
                c = _rect_corners_map(cb, sx, sy, syaw)
                poly = MplPolygon(c, closed=True, fill=True, facecolor="lightblue",
                                  edgecolor="gray", linewidth=0.8, zorder=3,
                                  alpha=0.25, label="Corridor")
                ax.add_patch(poly)

            # --- Rooms ---
            for room in floor0.get("rooms", []):
                rb = room.get("bounds")
                if not rb:
                    continue
                c = _rect_corners_map(rb, sx, sy, syaw)
                room_label = "Room" if room == floor0["rooms"][0] else ""
                poly = MplPolygon(c, closed=True, fill=False,
                                  edgecolor="darkblue", linewidth=1.2,
                                  linestyle="--", zorder=3, label=room_label)
                ax.add_patch(poly)
                # Room name
                cx_m = sum(p[0] for p in c) / 4.0
                cy_m = sum(p[1] for p in c) / 4.0
                rm_id = room.get("id", "").replace("floor_0_", "")
                ax.text(cx_m, cy_m, rm_id, fontsize=6, ha="center", va="center",
                        color="darkblue", alpha=0.7, zorder=4)

            # --- Stair area ---
            sb = floor0.get("stair_bounds")
            if sb:
                c = _rect_corners_map(sb, sx, sy, syaw)
                poly = MplPolygon(c, closed=True, fill=True, facecolor="lightcoral",
                                  edgecolor="darkred", linewidth=1.0, zorder=3,
                                  alpha=0.4, label="Stairs")
                ax.add_patch(poly)

            # --- Elevator shaft ---
            eb = floor0.get("elevator_bounds")
            if eb:
                c = _rect_corners_map(eb, sx, sy, syaw)
                poly = MplPolygon(c, closed=True, fill=True, facecolor="plum",
                                  edgecolor="purple", linewidth=1.0, zorder=3,
                                  alpha=0.5, label="Elevator")
                ax.add_patch(poly)

            # --- Furniture ---
            furniture_plotted = False
            for room in floor0.get("rooms", []):
                for furn in room.get("furniture", []):
                    fpose = furn.get("pose", [0, 0, 0, 0, 0, 0])
                    fsize = furn.get("size", [0.5, 0.5, 0.5])
                    fwx, fwy = fpose[0], fpose[1]
                    fw_hw, fw_hl = fsize[0] / 2.0, fsize[1] / 2.0
                    fcorners_w = [(fwx - fw_hw, fwy - fw_hl),
                                  (fwx + fw_hw, fwy - fw_hl),
                                  (fwx + fw_hw, fwy + fw_hl),
                                  (fwx - fw_hw, fwy + fw_hl)]
                    fcorners_m = [world_to_map(wx, wy, sx, sy, syaw)
                                  for (wx, wy) in fcorners_w]
                    label = "Furniture" if not furniture_plotted else ""
                    poly = MplPolygon(fcorners_m, closed=True, fill=True,
                                      facecolor="darkorange", edgecolor="brown",
                                      linewidth=0.3, zorder=4, alpha=0.5,
                                      label=label)
                    ax.add_patch(poly)
                    furniture_plotted = True

            # --- Doors (mark as gaps) ---
            for door in layout.get("door_specs", []):
                dpose = door.get("pose", [0, 0, 0, 0, 0, 0])
                dwx, dwy, dw_yaw = dpose[0], dpose[1], dpose[5]
                dw = door.get("width", 1.0)
                dmx, dmy = world_to_map(dwx, dwy, sx, sy, syaw)
                # Draw door as a small perpendicular line
                R_door = _rot_2d(dw_yaw)
                door_dx_m, door_dy_m = _vec2_mul(R_door, (dw / 2, 0.3))
                # Approximation: door is a thin rectangle perpendicular to wall
                ax.plot([dmx - door_dx_m, dmx + door_dx_m],
                        [dmy - door_dy_m, dmy + door_dy_m],
                        color="green", linewidth=1.5, zorder=5)

            layout_plotted = True

    if not layout_plotted:
        # If no layout, try to auto-set axes from map or trajectory
        pass

    # ----------------------------------------------------------------
    # Layer 3: ROBOT TRAJECTORY
    # ----------------------------------------------------------------
    if traj_pts and traj_meta.get("overlay_allowed", False):
        xs = [p[0] for p in traj_pts]
        ys = [p[1] for p in traj_pts]
        ax.plot(xs, ys, "-", color="royalblue", linewidth=1.0, alpha=0.8,
                zorder=6, label="Robot trajectory")
        # START
        ax.plot(xs[0], ys[0], "o", color="limegreen", markersize=10,
                markeredgecolor="darkgreen", markeredgewidth=1.5,
                zorder=7, label="START")
        # END
        ax.plot(xs[-1], ys[-1], "s", color="red", markersize=10,
                markeredgecolor="darkred", markeredgewidth=1.5,
                zorder=7, label="END")
    else:
        trajectory_message = traj_meta.get(
            "frame_warning", "NO TRAJECTORY DATA")
        ax.text(0.5, 0.5, trajectory_message, transform=ax.transAxes,
                ha="center", va="center", fontsize=14, color="gray", alpha=0.5)

    # ----------------------------------------------------------------
    # Layer 4: DSV GOALS
    # ----------------------------------------------------------------
    if goals:
        reached_g = [g for g in goals if g["reached"]]
        unreached_g = [g for g in goals if not g["reached"]]

        if reached_g:
            rx = [g["x"] for g in reached_g]
            ry = [g["y"] for g in reached_g]
            ax.scatter(rx, ry, c="green", marker="o", s=55, edgecolors="darkgreen",
                       linewidth=0.8, zorder=8, alpha=0.85,
                       label=f"Reached goals ({len(reached_g)})")

        if unreached_g:
            ux = [g["x"] for g in unreached_g]
            uy = [g["y"] for g in unreached_g]
            ax.scatter(ux, uy, c="red", marker="x", s=65, linewidth=2.0,
                       zorder=8, alpha=0.85,
                       label=f"Unreached goals ({len(unreached_g)})")

        # Goal number labels
        for g in goals:
            ax.annotate(f"G{g['index']}", (g["x"], g["y"]),
                        fontsize=5.5, alpha=0.75, ha="left", va="bottom",
                        zorder=9,
                        textcoords="offset points", xytext=(3, 3))
    else:
        goal_meta_msg = goal_meta.get("error", "No goals recorded")
        print(f"[render] Goals: {goal_meta_msg}")

    # ----------------------------------------------------------------
    # Styling
    # ----------------------------------------------------------------
    # Build title
    title_parts = ["Single-floor Exploration Overview"]
    if run_id:
        title_parts.append(f"run: {run_id}")
    if seed:
        title_parts.append(f"seed: {seed}")
    ax.set_title("\n".join(title_parts), fontsize=13, fontweight="bold")

    # Subtitle with stats
    subtitle_parts = []
    dur = timing.get("exploration_duration_sim_sec")
    if dur is not None:
        subtitle_parts.append(f"Sim time: {dur:.1f}s")
    if traj_meta.get("valid_point_count"):
        subtitle_parts.append(
            f"Route: {traj_meta.get('length_m', 0):.1f}m "
            f"({traj_meta['valid_point_count']} pts)")
        subtitle_parts.append(
            "Segments: "
            f"{traj_meta.get('route_accepted_segments', 0)} accepted / "
            f"{traj_meta.get('route_rejected_segments', 0)} rejected")
    if traj_meta.get("route_rejected_segments", 0) > 0:
        subtitle_parts.append(
            "Trajectory quality warning: "
            f"{traj_meta['route_rejected_segments']} route segments rejected")
    if goal_meta.get("total", 0) > 0:
        subtitle_parts.append(
            f"Goals: {goal_meta['reached']}/{goal_meta['total']} reached")
    if exp_meta.get("known_area_m2") is not None:
        subtitle_parts.append(f"Known area: {exp_meta['known_area_m2']:.1f} m²")
    if subtitle_parts:
        ax.set_xlabel(" | ".join(subtitle_parts), fontsize=8.5)

    ax.set_xlabel(ax.get_xlabel() + "\nMap frame X (m)")
    ax.set_ylabel("Map frame Y (m)")
    ax.legend(loc="upper right", fontsize=7.5, framealpha=0.9, ncol=1)
    ax.grid(True, alpha=0.25, linestyle="--")
    ax.set_aspect("equal")

    # ----------------------------------------------------------------
    # Save
    # ----------------------------------------------------------------
    os.makedirs(output_dir_plots, exist_ok=True)
    png_path = os.path.join(output_dir_plots, "final_exploration_overview.png")
    fig.savefig(png_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"[render] Saved: {png_path}")

    # ----------------------------------------------------------------
    # Metadata YAML
    # ----------------------------------------------------------------
    known_m2 = exp_meta.get("known_area_m2")
    free_m2 = exp_meta.get("free_area_m2")

    # Compute coverage_ratio only if we have reliable truth area
    coverage_ratio = None
    coverage_reason = "denominator_not_reliably_available"
    if layout:
        floors = layout.get("floors", [])
        if floors:
            floor0 = floors[0]
            # Estimate explorable area from lobby + corridor + rooms
            explorable = 0.0
            for region in ["lobby_bounds", "corridor_bounds"]:
                rb = floor0.get(region)
                if rb:
                    w = rb["x_max"] - rb["x_min"]
                    l = rb["y_max"] - rb["y_min"]
                    explorable += w * l
            for room in floor0.get("rooms", []):
                rb = room.get("bounds")
                if rb:
                    w = rb["x_max"] - rb["x_min"]
                    l = rb["y_max"] - rb["y_min"]
                    explorable += w * l
            if explorable > 0 and known_m2 is not None and known_m2 > 0:
                coverage_ratio = min(known_m2 / explorable, 1.0)
                coverage_reason = ""

    meta = {
        "run_id": run_id,
        "seed": seed if seed else None,
        "layout": {
            "source": "layout_metadata.json from building_generator_classic",
            "frame": "world (Gazebo truth)",
            "transform_to_map": {
                "description": "T_truth_to_map = R(-yaw) * (p_world - spawn)",
                "spawn_x": sx,
                "spawn_y": sy,
                "spawn_yaw_rad": syaw,
                "spawn_yaw_deg": math.degrees(syaw),
                "derived_from": "scene_manifest.json robot_start",
            },
        },
        "trajectory": {
            "source": "route/trajectory.csv",
            "point_count": traj_meta.get("point_count", 0),
            "valid_point_count": traj_meta.get("valid_point_count", 0),
            "length_m": round(traj_meta.get("length_m", 0.0), 3),
            "nan_detected": traj_meta.get("nan_detected", False),
            "frames": traj_meta.get("frames", []),
            "target_frame": traj_meta.get("target_frame", DEFAULT_TARGET_FRAME),
            "overlay_allowed": traj_meta.get("overlay_allowed", False),
            "frame_warning": traj_meta.get("frame_warning"),
            "route_metrics_source": traj_meta.get("route_metrics_source"),
            "route_total_segments": traj_meta.get("route_total_segments", 0),
            "route_accepted_segments": traj_meta.get(
                "route_accepted_segments", 0),
            "route_rejected_segments": traj_meta.get(
                "route_rejected_segments", 0),
            "route_reject_reasons": traj_meta.get("route_reject_reasons", {}),
            "route_max_speed_mps": traj_meta.get("route_max_speed_mps"),
            "route_max_step_m": traj_meta.get("route_max_step_m"),
        },
        "goals": {
            "total": goal_meta.get("total", 0),
            "reached": goal_meta.get("reached", 0),
            "unreached": goal_meta.get("unreached", 0),
        },
        "exploration": {
            "sim_time_sec": timing.get("exploration_duration_sim_sec"),
            "map_resolution": map_meta.get("resolution"),
            "map_width": map_meta.get("width"),
            "map_height": map_meta.get("height"),
            "map_origin_x": map_meta.get("origin_x"),
            "map_origin_y": map_meta.get("origin_y"),
            "known_area_m2": known_m2,
            "free_area_m2": exp_meta.get("free_area_m2"),
            "occupied_area_m2": exp_meta.get("occupied_area_m2"),
            "unknown_area_m2": exp_meta.get("unknown_area_m2"),
        },
        "coverage_ratio": coverage_ratio,
    }
    if coverage_reason:
        meta["coverage_ratio_reason"] = coverage_reason

    import yaml as _yaml_lib
    with open(meta_out_path, "w") as fh:
        _yaml_lib.dump(meta, fh, default_flow_style=False, sort_keys=False,
                       allow_unicode=True)
    print(f"[render] Saved: {meta_out_path}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Offline renderer for single-floor exploration results")
    parser.add_argument("--run-dir", required=True,
                        help="Path to the exploration output directory "
                             "(contains route/, goals/, map/, timing/, ...)")
    parser.add_argument("--layout-metadata",
                        help="Path to layout_metadata.json. "
                             "Default: auto-detect from "
                             "generated_building/layout_metadata.json "
                             "or --run-dir/config/layout_metadata.json")
    parser.add_argument("--spawn-x", type=float, default=DEFAULT_SPAWN_X,
                        help=f"Robot spawn X in world frame (default: {DEFAULT_SPAWN_X})")
    parser.add_argument("--spawn-y", type=float, default=DEFAULT_SPAWN_Y,
                        help=f"Robot spawn Y in world frame (default: {DEFAULT_SPAWN_Y})")
    parser.add_argument("--spawn-yaw-deg", type=float,
                        help="Robot spawn yaw in degrees (default: 90)")
    parser.add_argument("--export-layout-json", action="store_true",
                        help="Also export layout/layout_truth.json")
    args = parser.parse_args()

    run_dir = os.path.abspath(args.run_dir)
    if not os.path.isdir(run_dir):
        print(f"ERROR: run_dir not found: {run_dir}", file=sys.stderr)
        sys.exit(1)

    # ── Matplotlib check ──
    try:
        import matplotlib  # noqa: F401
    except ImportError:
        print("ERROR: matplotlib is required. Install with: "
              "pip install matplotlib", file=sys.stderr)
        sys.exit(1)

    # ── Determine spawn params ──
    if args.spawn_yaw_deg is not None:
        spawn_yaw = math.radians(args.spawn_yaw_deg)
    else:
        spawn_yaw = DEFAULT_SPAWN_YAW
    spawn = (args.spawn_x, args.spawn_y, spawn_yaw)

    # ── Load layout ──
    layout = None
    layout_source = "none"

    candidates = []
    if args.layout_metadata:
        candidates.append(args.layout_metadata)
    # Auto-detect
    candidates.append(os.path.join(run_dir, "config", "layout_metadata.json"))
    # Try repo root relative
    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.join(script_dir, "..")
    candidates.append(os.path.join(
        repo_root, "generated_building", "layout_metadata.json"))

    for cand in candidates:
        if cand and os.path.isfile(cand):
            try:
                layout = load_layout_metadata(cand)
                layout_source = cand
                print(f"[render] Loaded layout from: {cand}")
                break
            except Exception as e:
                print(f"[render] WARNING: cannot parse {cand}: {e}")

    if layout is None:
        print("[render] WARNING: No layout metadata found. "
              "Ground truth layout layer will be EMPTY.")
        print("[render] Use --layout-metadata to specify a layout_metadata.json file.")

    # ── Output paths ──
    plots_dir = os.path.join(run_dir, "plots")
    meta_path = os.path.join(plots_dir, "final_exploration_overview_meta.yaml")

    # ── Render ──
    render_overview(run_dir, layout, spawn, plots_dir, meta_path)

    # ── Optional: export truth layout JSON ──
    if args.export_layout_json and layout:
        layout_dir = os.path.join(run_dir, "layout")
        os.makedirs(layout_dir, exist_ok=True)
        truth = layout_to_truth_json(layout)
        import json
        truth_path = os.path.join(layout_dir, "layout_truth.json")
        with open(truth_path, "w") as fh:
            json.dump(truth, fh, indent=2)
        print(f"[render] Exported: {truth_path}")

    print("[render] Done.")


if __name__ == "__main__":
    main()
