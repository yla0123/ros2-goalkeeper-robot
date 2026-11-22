"""Dry-run host codec for protocol v2. No serial port is opened."""
from dataclasses import dataclass
import math

from .model import finite, RobotState
from .motion import minimum_time
from .planner import swept_clearance


@dataclass(frozen=True)
class Move:
    sequence: int
    slide_m: float
    tilt_rad: float
    motion_ms: int
    ttl_ms: int


def validate_move(move, design, state=None):
    state = state or RobotState()
    state.validate(design)
    if type(move.sequence) is not int or not 1 <= move.sequence <= 4294967295:
        raise ValueError("Sequence must be an integer in [1, 4294967295]")
    if type(move.motion_ms) is not int or type(move.ttl_ms) is not int:
        raise ValueError("Motion duration and TTL must be integers")
    finite(move.slide_m, "slide_m")
    finite(move.tilt_rad, "tilt_rad")
    if not design.slide.minimum <= move.slide_m <= design.slide.maximum:
        raise ValueError("Slide target exceeds limits")
    if not design.tilt.minimum <= move.tilt_rad <= design.tilt.maximum:
        raise ValueError("Tilt target exceeds limits")
    if move.motion_ms <= 0 or move.ttl_ms > design.serial_max_ttl_ms:
        raise ValueError("Invalid motion duration or TTL")
    if move.motion_ms / 1000 + design.settling_margin_s > move.ttl_ms / 1000 + 1e-9:
        raise ValueError("TTL does not include motion and settling")
    if abs(state.slide_velocity_mps) > 1e-6 or abs(state.tilt_velocity_radps) > 1e-6:
        raise ValueError("Protocol demo only accepts stationary initial states")
    required = max(minimum_time(move.slide_m - state.slide_m, design.slide),
                   minimum_time(move.tilt_rad - state.tilt_rad, design.tilt))
    if required > move.motion_ms / 1000 + 1e-8:
        raise ValueError("Move is too fast for configured capabilities")
    if not swept_clearance(state, move.slide_m, move.tilt_rad, design):
        raise ValueError("Move intersects the goal frame")


def decode_move(line, design, state=None):
    if not isinstance(line, str) or not line.endswith("\n") or len(line.encode("utf-8")) > 160:
        raise ValueError("Expected a newline-terminated frame of at most 160 bytes")
    if not line.isascii() or "\n" in line[:-1] or "\r" in line:
        raise ValueError("Expected one ASCII line")
    fields = line[:-1].split(",")
    if len(fields) != 6 or fields[0] != "MOVE":
        raise ValueError("Expected MOVE,sequence,slide_mm,tilt_deg,motion_ms,ttl_ms")
    for index in (1, 4, 5):
        if not fields[index].isdigit():
            raise ValueError("Sequence, motion duration, and TTL must be unsigned integers")
    move = Move(int(fields[1]), float(fields[2]) / 1000,
                math.radians(float(fields[3])), int(fields[4]), int(fields[5]))
    validate_move(move, design, state)
    return move


def encode_plan(result, design, sequence=1):
    if not result["reachable"]:
        raise ValueError("An unreachable target must not produce a MOVE command")
    command = result["command"]
    # Round up so the encoded profile never falls below the planner's minimum.
    duration = int(math.ceil(command["motion_duration_s"] * 1000 - 1e-9))
    ttl = int(math.floor(result["time_to_goal_s"] * 1000 + 1e-6))
    line = (f"MOVE,{sequence},{command['slide_m'] * 1000:.6f},"
            f"{command['tilt_deg']:.6f},{duration},{ttl}\n")
    decode_move(line, design, RobotState(**result["initial_state"]))
    return line
