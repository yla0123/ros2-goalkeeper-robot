"""Shared geometry for a fixed rectangle and diamond on one sliding base."""
from dataclasses import dataclass
import json
import math
from pathlib import Path

JOINT_NAMES = ("slide_joint",)


def finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


@dataclass(frozen=True)
class Axis:
    minimum: float
    maximum: float
    max_speed: float
    max_acceleration: float

    def validate(self):
        for key, value in vars(self).items():
            finite(value, key)
        if self.maximum <= self.minimum or min(self.max_speed, self.max_acceleration) <= 0:
            raise ValueError("Invalid axis limits")


@dataclass(frozen=True)
class Part:
    name: str
    y_m: float
    z_m: float
    width_m: float
    height_m: float
    depth_m: float
    mass_kg: float
    x_m: float = 0.0
    wheel: bool = False
    diamond: bool = False

    def corners(self):
        """Return counterclockwise projected boundary points."""
        w, h = self.width_m / 2, self.height_m / 2
        offsets = ((0, -h), (w, 0), (0, h), (-w, 0)) if self.diamond else (
            (-w, -h), (w, -h), (w, h), (-w, h))
        return tuple((self.y_m + y, self.z_m + z) for y, z in offsets)


@dataclass(frozen=True)
class Design:
    geometry: dict
    keeper: dict
    timing: dict
    planning: dict
    simulation: dict
    scenarios: dict
    slide: Axis

    @classmethod
    def load(cls, filename):
        raw = json.loads(Path(filename).read_text(encoding="utf-8"))
        if raw.get("schema_version") != 5 or raw.get("hardware_enabled") is not False:
            raise ValueError("Expected schema_version 5 and hardware_enabled=false")
        obj = cls(*(raw[k] for k in ("geometry", "keeper", "timing", "planning", "simulation", "scenarios")),
                  Axis(**raw["motion"]["slide"]))
        obj.validate()
        return obj

    @property
    def goal_width_m(self):
        return self.geometry["goal_width_m"]

    @property
    def goal_height_m(self):
        return self.geometry["goal_height_m"]

    @property
    def outer_width_m(self):
        return self.goal_width_m + 4 * self.geometry["post_radius_m"]

    @property
    def outer_height_m(self):
        return self.goal_height_m + 2 * self.geometry["post_radius_m"]

    @property
    def radius(self):
        return self.geometry["ball_radius_m"]

    @property
    def neutral(self):
        return Pose(0.0)

    def fixed_parts(self):
        b = self.keeper["base"]
        yield Part("chassis", 0, (b["height_m"] + b["shell_bottom_m"]) / 2, b["width_m"],
                   b["height_m"] - b["shell_bottom_m"], b["depth_m"], b["shell_mass_kg"])
        for fore, x in (("front", b["wheel_x_m"]), ("rear", -b["wheel_x_m"])):
            for side, sign in (("left", 1), ("right", -1)):
                yield Part(f"{fore}_{side}_wheel", sign * (b["width_m"] - b["wheel_width_m"]) / 2,
                           b["wheel_radius_m"], b["wheel_width_m"], 2 * b["wheel_radius_m"],
                           2 * b["wheel_radius_m"], b["wheel_mass_kg"], x, True)
        r = self.keeper["rectangle"]
        yield Part("rectangle_panel", 0, r["bottom_m"] + r["height_m"] / 2,
                   r["width_m"], r["height_m"], r["thickness_m"], r["mass_kg"])
        d = self.keeper["diamond"]
        yield Part("diamond_panel", 0, d["center_z_m"], d["horizontal_diagonal_m"], d["vertical_diagonal_m"],
                   d["thickness_m"], d["mass_kg"], x_m=d["center_x_m"], diamond=True)

    def validate(self):
        self.slide.validate()
        self.neutral.validate(self)
        for key, value in {**self.geometry, **self.timing, **self.planning}.items():
            finite(value, key)
            if key != "goal_plane_world_x_m" and value <= 0:
                raise ValueError(f"{key} must be positive")
        if self.geometry["minimum_overlap_m"] >= self.radius or self.geometry["post_radius_m"] > 0.06:
            raise ValueError("Invalid ball overlap margin or frame member radius")
        if min(self.planning["slide_search_step_m"], self.planning["sample_period_s"]) < 0.001:
            raise ValueError("Search and sample resolution must be at least 0.001")
        parts = tuple(self.fixed_parts())
        for part in parts:
            for key in ("y_m", "z_m", "x_m", "width_m", "height_m", "depth_m", "mass_kg"):
                finite(getattr(part, key), f"{part.name}.{key}")
            if min(part.width_m, part.height_m, part.depth_m, part.mass_kg) <= 0:
                raise ValueError(f"Invalid dimensions or mass for {part.name}")
        if not math.isclose(sum(p.mass_kg for p in parts), self.keeper["mass_kg"], abs_tol=1e-8):
            raise ValueError("Moving component masses must sum to keeper.mass_kg")
        top = max(z for part in parts for _, z in part.corners())
        if not math.isclose(top, self.keeper["height_m"], abs_tol=1e-8):
            raise ValueError("Model top must equal the declared assembly height")


@dataclass(frozen=True)
class Pose:
    slide_m: float

    def values(self):
        return (self.slide_m,)

    def validate(self, design):
        finite(self.slide_m, "slide_m")
        if not design.slide.minimum <= self.slide_m <= design.slide.maximum:
            raise ValueError("slide_joint outside limits")


@dataclass(frozen=True)
class CrossingTarget:
    y_m: float
    z_m: float
    arrival_time_s: float
    elapsed_since_kick_s: float
    x_m: float = 0.0

    def validate(self):
        for name, value in vars(self).items():
            finite(value, name)
        if abs(self.x_m) > 1e-9 or self.arrival_time_s <= 0 or self.elapsed_since_kick_s < 0:
            raise ValueError("Expected a goal-plane crossing (x=0) with valid timing")
