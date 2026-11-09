"""Plan a lateral move with exact solid-panel projection and swept clearance."""
from dataclasses import asdict
import math

from .model import Pose
from .motion import Profile, minimum_time

EPS = 1e-9


def footprint(pose, design):
    for part in design.fixed_parts():
        yield part.name, tuple((pose.slide_m + y, z) for y, z in part.corners())


def distance_to_part(y, z, part):
    if not part.diamond:
        return math.hypot(max(0.0, abs(y - part.y_m) - part.width_m / 2),
                          max(0.0, abs(z - part.z_m) - part.height_m / 2))
    if abs(y - part.y_m) / (part.width_m / 2) + abs(z - part.z_m) / (part.height_m / 2) <= 1:
        return 0.0
    points = part.corners()
    distances = []
    for (a, b), (c, d) in zip(points, points[1:] + points[:1]):
        dy, dz = c - a, d - b
        along = min(1.0, max(0.0, ((y-a)*dy + (z-b)*dz) / (dy*dy + dz*dz)))
        distances.append(math.hypot(y-a-along*dy, z-b-along*dz))
    return min(distances)


def covers(target, pose, design):
    y, z = target.y_m - pose.slide_m, target.z_m
    return min(distance_to_part(y, z, p) for p in design.fixed_parts()) <= (
        design.radius - design.geometry["minimum_overlap_m"] + EPS)


def swept_clearance(start, end, design):
    """A single translation keeps the fixed assembly level; extrema occur at its endpoints."""
    y0, y1 = sorted((start.slide_m, end.slide_m))
    margin = design.geometry["frame_clearance_m"]
    edge = design.goal_width_m / 2 - margin
    for part in design.fixed_parts():
        ys, zs = zip(*part.corners())
        floor = 0 if part.wheel else margin
        if min(ys) + y0 < -edge - EPS or max(ys) + y1 > edge + EPS:
            return False
        if min(zs) < floor - EPS or max(zs) > design.goal_height_m - margin + EPS:
            return False
    return True


def move_duration(start, end, design):
    return max(0.001, minimum_time(end.slide_m - start.slide_m, design.slide))


def _candidates(value, axis, step, time):
    ramp = axis.max_speed / axis.max_acceleration
    travel = (axis.max_acceleration * (time / 2) ** 2 if time <= 2 * ramp
              else axis.max_speed * (time - ramp))
    lo, hi = max(axis.minimum, value - travel), min(axis.maximum, value + travel)
    grid = {lo, hi, value}
    grid.update(i * step for i in range(math.ceil(lo / step), math.floor(hi / step) + 1))
    return sorted(grid, key=lambda q: (abs(q - value), q))


def inside_opening(y, z, design):
    r = design.radius
    return abs(y) + r <= design.goal_width_m / 2 + EPS and r - EPS <= z <= design.goal_height_m - r + EPS


def plan(target, design, start=None, extra_delay_s=0.0):
    target.validate()
    start = start or design.neutral
    start.validate(design)
    remaining = target.arrival_time_s - target.elapsed_since_kick_s
    reserve = design.timing["settling_margin_s"] + design.timing["front_contact_reserve_s"]
    budget = remaining - reserve - extra_delay_s
    result = {"target": asdict(target), "initial_state": asdict(start),
              "time_to_goal_s": remaining, "motion_budget_s": budget,
              "reachable": False, "command": None}
    if not inside_opening(target.y_m, target.z_m, design):
        return result | {"reason": "BALL_OUTSIDE_CLEAR_OPENING"}
    if budget < 0.001:
        return result | {"reason": "DEADLINE_TOO_SHORT"}
    if not swept_clearance(start, start, design):
        return result | {"reason": "UNSAFE_INITIAL_POSE"}
    if target.z_m > design.keeper["height_m"] + design.radius - design.geometry["minimum_overlap_m"] + EPS:
        return result | {"reason": "ABOVE_PANEL_COVERAGE"}
    for slide in _candidates(start.slide_m, design.slide, design.planning["slide_search_step_m"], budget):
        end = Pose(slide)
        duration = move_duration(start, end, design)
        if duration <= budget + EPS and covers(target, end, design) and swept_clearance(start, end, design):
            return result | {"reachable": True, "reason": "REACHABLE",
                             "command": asdict(end) | {"motion_duration_s": duration,
                                "finishes_since_kick_s": target.elapsed_since_kick_s + extra_delay_s + duration}}
    return result | {"reason": "NO_SAFE_INTERCEPT_IN_TIME"}


def motion_samples(start, end, duration, design):
    profile = Profile.create(start.slide_m, end.slide_m, duration, design.slide)
    step = design.planning["sample_period_s"]
    times = [i * step for i in range(math.ceil(duration / step)) if i * step < duration - EPS] + [duration]
    for t in times:
        yield t, [profile.sample(t)]
