#!/usr/bin/env python3
"""
validate_single_floor_exploration_result.py

Validates the output of a single-floor exploration test run.
Checks file existence, content integrity, data quality, and timing validity.

Usage:
  python3 tools/validate_single_floor_exploration_result.py <output_dir>
  python3 tools/validate_single_floor_exploration_result.py <output_dir> --strict
"""

import argparse
import csv
import math
import os
import sys

import yaml


class Colors:
    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    RESET = "\033[0m"
    BOLD = "\033[1m"


def red(s):
    return f"{Colors.RED}{s}{Colors.RESET}"


def green(s):
    return f"{Colors.GREEN}{s}{Colors.RESET}"


def yellow(s):
    return f"{Colors.YELLOW}{s}{Colors.RESET}"


def bold(s):
    return f"{Colors.BOLD}{s}{Colors.RESET}"


class ResultValidator:
    def __init__(self, output_dir, strict=False):
        self.output_dir = output_dir
        self.strict = strict
        self.errors = []
        self.warnings = []
        self.passes = []

    def error(self, msg):
        self.errors.append(msg)
        print(f"  {red('FAIL')} {msg}")

    def warn(self, msg):
        self.warnings.append(msg)
        print(f"  {yellow('WARN')} {msg}")

    def ok(self, msg):
        self.passes.append(msg)
        print(f"  {green('PASS')} {msg}")

    def check(self, condition, ok_msg, fail_msg):
        if condition:
            self.ok(ok_msg)
        else:
            self.error(fail_msg)

    def check_file_exists(self, rel_path):
        fpath = os.path.join(self.output_dir, rel_path)
        exists = os.path.isfile(fpath)
        non_empty = exists and os.path.getsize(fpath) > 0
        self.check(exists and non_empty,
                    f"File exists and non-empty: {rel_path}",
                    f"File missing or empty: {rel_path} ({fpath})")
        return exists and non_empty

    def validate(self):
        print(bold(f"\nValidating exploration results in: {self.output_dir}\n"))

        # ── Directory check ──
        print(bold("--- Directory Structure ---"))
        self.check(os.path.isdir(self.output_dir),
                    f"Output directory exists",
                    f"Output directory not found: {self.output_dir}")

        for sub in ["config", "map", "route", "goals", "plots", "timing", "logs"]:
            sub_path = os.path.join(self.output_dir, sub)
            self.check(os.path.isdir(sub_path),
                        f"Subdirectory exists: {sub}/",
                        f"Missing subdirectory: {sub}/")

        # ── Map files ──
        print(bold("\n--- Map ---"))
        map_pgm = self.check_file_exists("map/map.pgm")
        map_yaml = self.check_file_exists("map/map.yaml")

        if map_yaml:
            try:
                with open(os.path.join(self.output_dir, "map/map.yaml")) as f:
                    map_data = yaml.safe_load(f)
                self.check("image" in map_data,
                            "map.yaml has 'image' field",
                            "map.yaml missing 'image' field")
                self.check("resolution" in map_data,
                            "map.yaml has 'resolution' field",
                            "map.yaml missing 'resolution' field")
                if "resolution" in map_data:
                    self.check(map_data["resolution"] > 0,
                                f"Map resolution positive ({map_data['resolution']})",
                                f"Map resolution invalid ({map_data.get('resolution')})")
                if map_pgm and "image" in map_data:
                    img_path = os.path.join(self.output_dir, "map", map_data["image"])
                    self.check(os.path.isfile(img_path),
                                f"Map image file exists: {map_data['image']}",
                                f"Map image file not found: {map_data['image']}")
            except Exception as e:
                self.error(f"Cannot parse map.yaml: {e}")

        # Map metadata
        meta_yaml = os.path.join(self.output_dir, "map", "metadata.yaml")
        if os.path.isfile(meta_yaml):
            self.ok("Map metadata.yaml exists")
            try:
                with open(meta_yaml) as f:
                    meta = yaml.safe_load(f)
                # ── CRITICAL: Placeholder map detection ──
                placeholder = meta.get("placeholder_map_used", True)
                real_map = meta.get("real_map_received", False)
                map_updates = meta.get("map_update_count", 0)
                known_cells = meta.get("known_area_m2", 0)
                self.check(not placeholder,
                            "Real map received (not placeholder)",
                            "PLACEHOLDER MAP USED — real OccupancyGrid NOT received")
                self.check(real_map,
                            "real_map_received = True",
                            "real_map_received = False")
                self.check(map_updates > 0,
                            f"Map update count > 0 ({map_updates})",
                            "No map updates received (map_update_count=0)")
                self.check(known_cells > 0,
                            f"Known map area > 0 ({known_cells:.2f} m²)",
                            "Known map area is 0 (blank map)")
                self.check(meta.get("width", 0) > 0,
                            f"Map width > 0 ({meta.get('width')})",
                            "Map width invalid")
                self.check(meta.get("height", 0) > 0,
                            f"Map height > 0 ({meta.get('height')})",
                            "Map height invalid")
            except Exception as e:
                self.warn(f"Cannot parse map metadata.yaml: {e}")

        # ── Trajectory ──
        print(bold("\n--- Trajectory ---"))
        traj_csv = self.check_file_exists("route/trajectory.csv")
        traj_yaml = self.check_file_exists("route/trajectory.yaml")

        if traj_csv:
            try:
                with open(os.path.join(self.output_dir, "route/trajectory.csv")) as f:
                    reader = csv.DictReader(f)
                    rows = list(reader)
                self.check(len(rows) > 0,
                            f"Trajectory has {len(rows)} points",
                            "Trajectory CSV is empty")

                if len(rows) >= 2:
                    # Check required fields
                    required = ["sim_time", "x", "y", "z"]
                    for field in required:
                        self.check(field in rows[0],
                                    f"Trajectory has '{field}' field",
                                    f"Trajectory missing '{field}' field")

                    # Check sim_time is monotonic non-decreasing
                    times = [float(r["sim_time"]) for r in rows
                             if r.get("sim_time")]
                    if len(times) >= 2:
                        monotonic = all(times[i] <= times[i+1]
                                       for i in range(len(times)-1))
                        self.check(monotonic,
                                    "Trajectory sim_time is monotonic non-decreasing",
                                    "Trajectory sim_time is NOT monotonic")

                    # Check for NaN/Inf
                    has_nan = False
                    for r in rows:
                        for k in ["x", "y", "z"]:
                            v = r.get(k, "")
                            if v and not math.isfinite(float(v)):
                                has_nan = True
                                break
                    self.check(not has_nan,
                                "Trajectory has no NaN/Inf values",
                                "Trajectory contains NaN/Inf values")

                    # Check trajectory length > 0
                    if traj_yaml:
                        try:
                            with open(os.path.join(self.output_dir, "route/trajectory.yaml")) as f:
                                traj_meta = yaml.safe_load(f)
                            length = traj_meta.get("trajectory_length_m_2d", 0)
                            self.check(length > 0,
                                        f"Trajectory length 2D = {length:.2f}m",
                                        "Trajectory length is 0 or missing")
                        except Exception as e:
                            self.warn(f"Cannot parse trajectory.yaml: {e}")
            except Exception as e:
                self.error(f"Error reading trajectory.csv: {e}")

        # ── Goals ──
        print(bold("\n--- Goals ---"))
        goals_csv = self.check_file_exists("goals/goals.csv")
        goals_yaml = self.check_file_exists("goals/goals.yaml")

        if goals_csv:
            try:
                with open(os.path.join(self.output_dir, "goals/goals.csv")) as f:
                    reader = csv.DictReader(f)
                    rows = list(reader)
                self.check(len(rows) > 0,
                            f"Goals CSV has {len(rows)} entries",
                            "Goals CSV is empty")
                if len(rows) > 0:
                    for field in ["goal_index", "x", "y", "z", "status"]:
                        self.check(field in rows[0],
                                    f"Goals CSV has '{field}' field",
                                    f"Goals CSV missing '{field}' field")
            except Exception as e:
                self.error(f"Error reading goals.csv: {e}")

        if goals_yaml:
            try:
                with open(os.path.join(self.output_dir, "goals/goals.yaml")) as f:
                    goals_meta = yaml.safe_load(f)
                self.check(goals_meta.get("raw_message_count", 0) > 0,
                            f"Raw goal count: {goals_meta.get('raw_message_count')}",
                            "No raw goals recorded")
                self.check(goals_meta.get("unique_goal_count", 0) > 0,
                            f"Unique goal count: {goals_meta.get('unique_goal_count')}",
                            "No unique goals recorded")
            except Exception as e:
                self.warn(f"Cannot parse goals.yaml: {e}")

        # ── Timing ──
        print(bold("\n--- Timing ---"))
        timing_ok = self.check_file_exists("timing/timing.yaml")

        if timing_ok:
            try:
                with open(os.path.join(self.output_dir, "timing/timing.yaml")) as f:
                    timing = yaml.safe_load(f)

                dur_sim = timing.get("exploration_duration_sim_sec", 0)
                self.check(dur_sim > 0,
                            f"Exploration sim duration = {dur_sim:.1f}s",
                            "Exploration sim duration is 0 or negative")

                start_sim = timing.get("exploration_start_sim_time", 0)
                end_sim = timing.get("exploration_end_sim_time", 0)
                self.check(end_sim >= start_sim,
                            f"end_sim ({end_sim:.1f}) >= start_sim ({start_sim:.1f})",
                            "End sim time < start sim time")

                rtf = timing.get("average_rtf", 0)
                self.check(rtf > 0,
                            f"Average RTF = {rtf:.4f}",
                            "Average RTF is 0 or missing")

                reset = timing.get("sim_clock_reset_detected", False)
                self.check(not reset,
                            "No clock reset detected",
                            "SIM CLOCK RESET DETECTED — timing is INVALID")

                timing_valid = timing.get("timing_valid", False)
                self.check(timing_valid,
                            "Timing verdict: VALID",
                            "Timing verdict: INVALID")
            except Exception as e:
                self.error(f"Error parsing timing.yaml: {e}")

        # ── Summary ──
        print(bold("\n--- Summary ---"))
        summary_ok = self.check_file_exists("summary.md")

        # ── Manifest ──
        print(bold("\n--- Manifest ---"))
        manifest_ok = self.check_file_exists("manifest.yaml")

        if manifest_ok:
            try:
                with open(os.path.join(self.output_dir, "manifest.yaml")) as f:
                    manifest = yaml.safe_load(f)
                files_listed = manifest.get("files", {})
                self.check(len(files_listed) > 0,
                            f"Manifest lists {len(files_listed)} files",
                            "Manifest file list is empty")
                # Verify key files are in manifest
                key_files = ["summary.md", "map/map.yaml", "route/trajectory.csv",
                            "goals/goals.csv", "timing/timing.yaml"]
                for kf in key_files:
                    self.check(kf in files_listed,
                                f"Manifest includes {kf}",
                                f"Manifest missing {kf}")
            except Exception as e:
                self.error(f"Error parsing manifest.yaml: {e}")

        # ── Plot ──
        print(bold("\n--- Plot ---"))
        self.check_file_exists("plots/map_route_goals.png")

        # ── Logs ──
        print(bold("\n--- Logs ---"))
        self.check_file_exists("logs/recorder.log")
        self.check_file_exists("logs/health_events.log")

        # ── Config ──
        print(bold("\n--- Config ---"))
        self.check_file_exists("config/runtime_env.txt")
        self.check_file_exists("config/recorder_config.yaml")

        # ── Summary ──
        print(bold("\n" + "=" * 50))
        print(bold("Validation Summary"))
        print("=" * 50)
        print(f"  {green('Passes')}:  {len(self.passes)}")
        print(f"  {yellow('Warnings')}: {len(self.warnings)}")
        print(f"  {red('Errors')}:   {len(self.errors)}")

        if self.errors:
            print(bold(red("\n  VALIDATION FAILED\n")))
            return 1
        elif self.strict and self.warnings:
            print(bold(yellow("\n  VALIDATION PASSED WITH WARNINGS (strict mode = FAIL)\n")))
            return 1
        else:
            print(bold(green("\n  VALIDATION PASSED\n")))
            return 0


def main():
    parser = argparse.ArgumentParser(
        description="Validate single-floor exploration result directory")
    parser.add_argument("output_dir", help="Path to the output directory")
    parser.add_argument("--strict", action="store_true",
                        help="Treat warnings as failures")
    args = parser.parse_args()

    if not os.path.isdir(args.output_dir):
        print(f"ERROR: Directory not found: {args.output_dir}")
        sys.exit(1)

    validator = ResultValidator(args.output_dir, strict=args.strict)
    sys.exit(validator.validate())


if __name__ == "__main__":
    main()
