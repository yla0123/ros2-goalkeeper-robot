"""Validated design inputs. Units are meters, radians, and seconds."""
from dataclasses import dataclass
import json
import math
from pathlib import Path


def finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    if not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


@dataclass(frozen=True)
class Axis:
    minimum: float
    maximum: float
    max_speed: float
    max_acceleration: float

    def validate(self, name):
        for key, value in vars(self).items():
            finite(value, f"{name}.{key}")
        if self.minimum >= self.maximum:
            raise ValueError(f"{name} requires minimum < maximum")
        if self.max_speed <= 0 or self.max_acceleration <= 0:
            raise ValueError(f"{name} speed and acceleration must be positive")


@dataclass(frozen=True)
class FootprintRectangle:
    name: str
    center_y_m: float
    center_z_m: float
    width_m: float
    height_m: float

    def validate(self):
        if not isinstance(self.name, str) or not self.name:
            raise ValueError("Each footprint part needs a name")
        for key in ("center_y_m", "center_z_m", "width_m", "height_m"):
            finite(getattr(self, key), f"{self.name}.{key}")
        if self.width_m <= 0 or self.height_m <= 0:
            raise ValueError("Footprint dimensions must be positive")

    def corners(self):
        return [(self.center_y_m + sy * self.width_m / 2,
                 self.center_z_m + sz * self.height_m / 2)
                for sy in (-1, 1) for sz in (-1, 1)]


@dataclass(frozen=True)
class Design:
    goal_width_m: float
    goal_height_m: float
    goal_plane_world_x_m: float
    penalty_distance_m: float
    post_radius_m: float
    pivot_height_m: float
    ball_radius_m: float
    minimum_overlap_m: float
    frame_clearance_m: float
    keeper_height_m: float
    keeper_arm_span_m: float
    keeper_mass_kg: float
    keeper_depth_m: float
    footprint: tuple
    slide: Axis
    tilt: Axis
    flight_time_s: float
    processing_and_communication_s: float
    settling_margin_s: float
    slide_search_step_m: float
    tilt_search_step_rad: float
    sample_period_s: float
    serial_max_ttl_ms: int

    @classmethod
    def load(cls, filename):
        raw = json.loads(Path(filename).read_text(encoding="utf-8"))
        if raw.get("schema_version") != 3:
            raise ValueError("Expected configuration schema_version 3 (adult keeper silhouette)")
        if raw.get("hardware_enabled") is not False:
            raise ValueError("This planner release supports hardware_enabled=false only")
        geometry = raw["geometry"]
        timing = raw["timing"]
        search = raw["planning"]
        keeper = raw["keeper"]
        result = cls(
            **geometry,
            keeper_height_m=keeper["height_m"],
            keeper_arm_span_m=keeper["arm_span_m"],
            keeper_mass_kg=keeper["mass_kg"],
            keeper_depth_m=keeper["depth_m"],
            footprint=tuple(FootprintRectangle(**part) for part in keeper["footprint"]),
            slide=Axis(**raw["motion"]["slide"]),
            tilt=Axis(**raw["motion"]["tilt"]),
            **timing,
            **search,
            serial_max_ttl_ms=raw["serial"]["max_command_ttl_ms"],
        )
        result.validate()
        return result

    def validate(self):
        self.slide.validate("slide")
        self.tilt.validate("tilt")
        if not self.footprint or len(self.footprint) > 32:
            raise ValueError("Expected between 1 and 32 footprint parts")
        for part in self.footprint:
            part.validate()
        if len({part.name for part in self.footprint}) != len(self.footprint):
            raise ValueError("Footprint names must be unique")
        corners = [point for part in self.footprint for point in part.corners()]
        width = max(p[0] for p in corners) - min(p[0] for p in corners)
        height = max(p[1] for p in corners) - min(p[1] for p in corners)
        if abs(width - self.keeper_arm_span_m) > 1e-8 or abs(height - self.keeper_height_m) > 1e-8:
            raise ValueError("Footprint bounds must match keeper height and arm span")
        for name, value in vars(self).items():
            if isinstance(value, Axis) or name == "footprint":
                continue
            finite(value, name)
            if value <= 0:
                raise ValueError(f"{name} must be positive")
        if self.minimum_overlap_m >= self.ball_radius_m:
            raise ValueError("minimum_overlap_m must be smaller than ball_radius_m")
        if not self.slide.minimum <= 0 <= self.slide.maximum:
            raise ValueError("Slide limits must include the centered position")
        if not self.tilt.minimum <= 0 <= self.tilt.maximum:
            raise ValueError("Tilt limits must include the upright position")
        if self.tilt.minimum < -math.pi / 2 or self.tilt.maximum > math.pi / 2:
            raise ValueError("This design limits tilt to +/-90 degrees")
        if self.slide_search_step_m < 0.001 or self.tilt_search_step_rad < 0.005:
            raise ValueError("Search resolution exceeds this demo's bounded workload")
        if self.sample_period_s < 0.001:
            raise ValueError("sample_period_s must be at least 0.001")
        if self.serial_max_ttl_ms != int(self.serial_max_ttl_ms):
            raise ValueError("serial_max_ttl_ms must be an integer")
        if self.flight_time_s <= self.processing_and_communication_s + self.settling_margin_s:
            raise ValueError("The default timing must leave positive motion time")


@dataclass(frozen=True)
class CrossingTarget:
    y_m: float
    z_m: float
    arrival_time_s: float = 0.55
    elapsed_since_kick_s: float = 0.10
    x_m: float = 0.0
    frame_id: str = "goal_frame"

    def validate(self):
        for name in ("x_m", "y_m", "z_m", "arrival_time_s", "elapsed_since_kick_s"):
            finite(getattr(self, name), name)
        if self.frame_id != "goal_frame":
            raise ValueError("Target must be expressed in goal_frame")
        if abs(self.x_m) > 1e-9:
            raise ValueError("Target is a goal-plane crossing: x_m must be zero")
        if self.arrival_time_s <= 0 or self.elapsed_since_kick_s < 0:
            raise ValueError("Arrival must be positive and elapsed time nonnegative")


@dataclass(frozen=True)
class RobotState:
    slide_m: float = 0.0
    tilt_rad: float = 0.0
    slide_velocity_mps: float = 0.0
    tilt_velocity_radps: float = 0.0

    def validate(self, design):
        for name, value in vars(self).items():
            finite(value, name)
        if not design.slide.minimum <= self.slide_m <= design.slide.maximum:
            raise ValueError("Initial slide position exceeds limits")
        if not design.tilt.minimum <= self.tilt_rad <= design.tilt.maximum:
            raise ValueError("Initial tilt angle exceeds limits")
