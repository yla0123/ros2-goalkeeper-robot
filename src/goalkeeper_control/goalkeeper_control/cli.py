"""Command-line entry point for the local, hardware-disabled planner demo."""
import argparse
import csv
import json
import math
from pathlib import Path
import sys

from .model import CrossingTarget, Design, RobotState
from .planner import plan, trajectory
from .protocol import encode_plan


def _load_json(path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise ValueError(f"Cannot read input file {path}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise ValueError(f"Invalid JSON in {path}: {exc}") from exc


def _shot_from_args(args, design):
    raw = _load_json(args.input) if args.input is not None else {}
    if args.command == "demo" and args.input is None and args.y_m is None and args.z_m is None:
        raw = {"crossing_point_m": {"x": 0.0, "y": 1.3, "z": 1.4}}
    if not isinstance(raw, dict):
        raise ValueError("Shot input must be a JSON object")
    point = raw.get("crossing_point_m", {})
    if not isinstance(point, dict):
        raise ValueError("crossing_point_m must be a JSON object")
    if (args.y_m is None) != (args.z_m is None):
        raise ValueError("--y-m and --z-m must be provided together")
    y_m = args.y_m if args.y_m is not None else point.get("y")
    z_m = args.z_m if args.z_m is not None else point.get("z")
    if y_m is None or z_m is None:
        raise ValueError("Provide both --y-m and --z-m, or an input JSON with crossing_point_m")

    arrival = (args.arrival_time_s if args.arrival_time_s is not None else
               raw.get("flight_time_from_kick_s", design.flight_time_s))
    elapsed = (args.elapsed_s if args.elapsed_s is not None else
               raw.get("elapsed_since_kick_s", design.processing_and_communication_s))
    target = CrossingTarget(
        x_m=point.get("x", 0.0), y_m=y_m, z_m=z_m,
        arrival_time_s=arrival, elapsed_since_kick_s=elapsed,
        frame_id=raw.get("frame_id", "goal_frame"),
    )
    raw_state = raw.get("robot_state", {})
    if not isinstance(raw_state, dict):
        raise ValueError("robot_state must be a JSON object")
    state_data = dict(raw_state)
    for key, value in (("slide_m", args.slide_m), ("tilt_rad", args.tilt_rad)):
        if value is not None:
            state_data[key] = value
    state = RobotState(**state_data)
    target.validate()
    state.validate(design)
    return target, state


def _build_parser(root):
    parser = argparse.ArgumentParser(
        prog="run_keeper.py",
        description="Plan a two-axis goalkeeper pose from a predicted ball crossing coordinate.",
    )
    parser.add_argument("command", choices=("demo", "plan"), help="demo uses a built-in crossing coordinate")
    parser.add_argument("--config", type=Path, default=root / "config" / "keeper.design.json")
    parser.add_argument("--input", type=Path, help="shot JSON with crossing_point_m and timing fields")
    parser.add_argument("--path-csv", type=Path, help="export the complete modeled action path as CSV")
    parser.add_argument("--y-m", type=float, help="predicted ball-center crossing coordinate along the goal line")
    parser.add_argument("--z-m", type=float, help="predicted ball-center height at the goal line")
    parser.add_argument("--arrival-time-s", type=float, help="total ball flight time from kick to goal plane")
    parser.add_argument("--elapsed-s", type=float, help="time already used since the kick")
    parser.add_argument("--slide-m", type=float, help="current carriage offset from center")
    parser.add_argument("--tilt-rad", type=float, help="current panel angle")
    parser.add_argument("--sequence", type=int, default=1, help="dry-run protocol sequence number")
    return parser


def _path_checkpoints(rows, step_s=0.05):
    """Return readable checkpoints plus exact motion and arrival events."""
    final_time = rows[-1]["time_s"]
    selected_times = {0.0, final_time}
    selected_times.update(index * step_s for index in range(int(final_time / step_s) + 1))
    previous_phase = None
    for row in rows:
        if row["phase"] != previous_phase and row["phase"] in ("moving", "settled", "hold"):
            selected_times.add(row["time_s"])
        previous_phase = row["phase"]
    checkpoints = []
    for time_s in sorted(selected_times):
        row = min(rows, key=lambda item: abs(item["time_s"] - time_s))
        if not checkpoints or row["time_s"] != checkpoints[-1]["time_from_kick_s"]:
            checkpoints.append({
                "time_from_kick_s": round(row["time_s"], 6),
                "phase": row["phase"],
                "slide_m": round(row["slide_m"], 4),
                "tilt_deg": round(row["tilt_rad"] * 180.0 / math.pi, 2),
            })
    return checkpoints


def main(root, argv=None):
    parser = _build_parser(root)
    args = parser.parse_args(argv)
    try:
        design = Design.load(args.config)
        target, state = _shot_from_args(args, design)
        result = plan(target, design, state)
        path_rows = trajectory(result, design)
        if args.path_csv is not None:
            args.path_csv.parent.mkdir(parents=True, exist_ok=True)
            with args.path_csv.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=list(path_rows[0]))
                writer.writeheader()
                writer.writerows(path_rows)
        output = {
            "mode": "planning_only",
            "hardware_enabled": False,
            "coordinate_input": {
                "frame_id": target.frame_id,
                "goal_plane_x_m": target.x_m,
                "crossing_y_m": target.y_m,
                "crossing_z_m": target.z_m,
            },
            "timing": {
                "flight_time_from_kick_s": target.arrival_time_s,
                "elapsed_since_kick_s": target.elapsed_since_kick_s,
                "remaining_after_processing_s": result["time_to_goal_s"],
                "settling_reserve_s": design.settling_margin_s,
                "axis_motion_window_s": result["motion_budget_s"],
            },
            "plan": result,
            "action_path_checkpoints": _path_checkpoints(path_rows),
            "serial_preview": encode_plan(result, design, args.sequence) if result["reachable"] else None,
            "warning": "This is a software dry run. It does not open a serial port or move hardware.",
        }
    except (ValueError, TypeError, KeyError, OverflowError, OSError) as exc:
        print(f"Input error: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(output, indent=2, allow_nan=False))
    return 0
