"""Offline entry point for slide planning and sampled action paths."""
import argparse
import csv
import json
from pathlib import Path
import sys

from .model import CrossingTarget, Design, Pose
from .planner import motion_samples, plan


def main(root, argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("demo", "plan"))
    parser.add_argument("--case", choices=("center", "low", "high_corner"), default="low")
    parser.add_argument("--config", type=Path, default=root / "config/keeper.design.json")
    parser.add_argument("--input", type=Path, help="JSON with crossing_point_m and optional robot_state")
    parser.add_argument("--y-m", type=float)
    parser.add_argument("--z-m", type=float)
    parser.add_argument("--arrival-time-s", type=float)
    parser.add_argument("--elapsed-s", type=float)
    parser.add_argument("--path-csv", type=Path)
    args = parser.parse_args(argv)
    try:
        design = Design.load(args.config)
        raw = json.loads(args.input.read_text(encoding="utf-8")) if args.input else {}
        if raw.get("frame_id", "goal_frame") != "goal_frame":
            raise ValueError("Input must use goal_frame")
        point = raw.get("crossing_point_m", {})
        if (args.y_m is None) != (args.z_m is None):
            raise ValueError("Supply both --y-m and --z-m")
        if args.y_m is not None:
            point = {"y": args.y_m, "z": args.z_m}
        elif args.command == "demo" and not point:
            case = design.scenarios[args.case]
            point = {"y": case["y_m"], "z": case["z_m"]}
        arrival = (args.arrival_time_s if args.arrival_time_s is not None else
                   raw.get("flight_time_from_kick_s", design.timing["flight_time_s"]))
        elapsed = (args.elapsed_s if args.elapsed_s is not None else
                   raw.get("elapsed_since_kick_s", design.timing["processing_and_communication_s"]))
        target = CrossingTarget(point["y"], point["z"], arrival, elapsed, point.get("x", 0))
        start = Pose(**raw["robot_state"]) if "robot_state" in raw else design.neutral
        result = plan(target, design, start)
        if args.path_csv is not None:
            command = result["command"]
            rows = [(0.0, *start.values())]
            if command:
                end = Pose(command["slide_m"])
                rows.extend((elapsed + t, *(p[0] for p in samples))
                            for t, samples in motion_samples(start, end, command["motion_duration_s"], design)
                            if elapsed + t > 0)
                rows.append((arrival, *end.values()))
            else:
                rows.append((arrival, *start.values()))
            args.path_csv.parent.mkdir(parents=True, exist_ok=True)
            with args.path_csv.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(("time_since_kick_s", "slide_m"))
                writer.writerows(rows)
        print(json.dumps({"mode": "offline_model_only", **result}, indent=2, allow_nan=False))
    except (ValueError, TypeError, KeyError, OSError) as exc:
        print(f"Input error: {exc}", file=sys.stderr)
        return 2
    return 0
