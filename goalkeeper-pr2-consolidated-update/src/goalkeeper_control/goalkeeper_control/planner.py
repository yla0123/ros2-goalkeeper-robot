"""Plan slide and board tilt using the projected geometry and swept clearance."""
from dataclasses import asdict
from bisect import bisect_left, bisect_right
import math

from .model import Pose
from .motion import Profile, minimum_time

EPS = 1e-9


def footprint(pose, design):
    for part in design.chassis_parts():
        yield part.name, tuple((pose.slide_m + y, z) for y, z in part.corners())
    c, s = math.cos(pose.tilt_rad), math.sin(pose.tilt_rad)
    pivot = design.pivot_height_m
    for part in design.board_parts():
        yield part.name, tuple((pose.slide_m + c*y - s*(z-pivot), pivot + s*y + c*(z-pivot))
                               for y, z in part.corners())


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
    reach = design.radius - design.geometry["minimum_overlap_m"] + EPS
    if any(distance_to_part(y, z, part) <= reach for part in design.chassis_parts()):
        return True
    c, s = math.cos(pose.tilt_rad), math.sin(pose.tilt_rad)
    pivot = design.pivot_height_m
    local_y, local_z = c*y + s*(z-pivot), pivot - s*y + c*(z-pivot)
    return any(distance_to_part(local_y, local_z, part) <= reach for part in design.board_parts())


def _rotation_bounds(a, b, lo, hi):
    candidates = [lo, hi]
    phase = math.atan2(b, a)
    for k in range(math.ceil((lo-phase)/math.pi), math.floor((hi-phase)/math.pi) + 1):
        candidates.append(phase + k*math.pi)
    values = [a*math.cos(t) + b*math.sin(t) for t in candidates]
    return min(values), max(values)


def _overlaps_frame_depth(part, design):
    center_x = part.x_m - design.keeper_offset_from_goal_m
    half_depth = part.depth_m / 2
    frame_front = -2 * design.geometry["post_radius_m"]
    gap = design.geometry["frame_clearance_m"]
    return center_x + half_depth > frame_front - gap and center_x - half_depth < gap


def swept_clearance(start, end, design):
    """Bound the full sweep of each vertex, allowing slide and tilt to run concurrently."""
    if any(not axis.minimum <= q <= axis.maximum for pose in (start, end)
           for q, axis in zip(pose.values(), design.axes)):
        return False
    y0, y1 = sorted((start.slide_m, end.slide_m))
    lo, hi = sorted((start.tilt_rad, end.tilt_rad))
    margin = design.geometry["frame_clearance_m"]
    edge = design.goal_width_m / 2 - margin
    ceiling = design.goal_height_m - margin
    for part in design.chassis_parts():
        ys, zs = zip(*part.corners())
        floor = 0 if part.wheel else margin
        if min(zs) < floor - EPS:
            return False
        if _overlaps_frame_depth(part, design) and (
                min(ys) + y0 < -edge - EPS or max(ys) + y1 > edge + EPS or max(zs) > ceiling + EPS):
            return False
    pivot = design.pivot_height_m
    base_top = design.keeper["base"]["height_m"] + margin
    base_edge = design.keeper["base"]["width_m"] / 2 + margin
    for part in design.board_parts():
        corners = part.corners()
        closest_z = math.inf
        overlaps_frame = _overlaps_frame_depth(part, design)
        for y, z in corners:
            dz0, dz1 = _rotation_bounds(z-pivot, y, lo, hi)
            closest_z = min(closest_z, pivot + dz0)
            if pivot + dz0 < margin - EPS:
                return False
            if overlaps_frame:
                dy0, dy1 = _rotation_bounds(y, pivot-z, lo, hi)
                if y0 + dy0 < -edge - EPS or y1 + dy1 > edge + EPS or pivot + dz1 > ceiling + EPS:
                    return False
        if closest_z >= base_top - EPS:
            continue
        # Near horizontal, the diamond can pass beside the chassis below its roof.
        # Bound the full shape over short angle intervals in the chassis frame.
        segments = max(1, math.ceil((hi-lo) / math.radians(5)))
        for i in range(segments):
            a, b = (lo + (hi-lo)*j/segments for j in (i, i+1))
            if min(pivot + _rotation_bounds(z-pivot, y, a, b)[0] for y, z in corners) >= base_top - EPS:
                continue
            lateral = [_rotation_bounds(y, pivot-z, a, b) for y, z in corners]
            if min(left for left, _ in lateral) < base_edge and max(right for _, right in lateral) > -base_edge:
                return False
    return True


def move_duration(start, end, design):
    return max(0.001, *(minimum_time(q1-q0, axis)
                       for q0, q1, axis in zip(start.values(), end.values(), design.axes)))


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


def _possible_slide_range(target, tilt, design):
    # Clip the projected parts to a horizontal strip around the ball centre.
    # The strip bounds are conservative. covers() checks the exact distance.
    reach = design.radius - design.geometry["minimum_overlap_m"] + EPS
    bottom, top = target.z_m - reach, target.z_m + reach
    ys = []
    for _, corners in footprint(Pose(0.0, tilt), design):
        ys.extend(y for y, z in corners if bottom <= z <= top)
        for (y0, z0), (y1, z1) in zip(corners, corners[1:] + corners[:1]):
            for height in (bottom, top):
                if min(z0, z1) < height < max(z0, z1):
                    ys.append(y0 + (y1-y0) * (height-z0) / (z1-z0))
    if ys:
        return target.y_m - max(ys) - reach, target.y_m - min(ys) + reach
    return None


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
    slides = sorted(_candidates(start.slide_m, design.slide, design.planning["slide_search_step_m"], budget))
    slide_durations = {slide: minimum_time(slide-start.slide_m, design.slide) for slide in slides}
    tilts = _candidates(start.tilt_rad, design.tilt, design.planning["tilt_search_step_rad"], budget)
    candidates = []
    for tilt in tilts:
        bounds = _possible_slide_range(target, tilt, design)
        if bounds is None:
            continue
        lo, hi = bounds
        tilt_duration = minimum_time(tilt-start.tilt_rad, design.tilt)
        for slide in slides[bisect_left(slides, lo):bisect_right(slides, hi)]:
            candidates.append((max(0.001, tilt_duration, slide_durations[slide]), Pose(slide, tilt)))
    for duration, end in sorted(candidates, key=lambda candidate: (candidate[0], abs(candidate[1].tilt_rad))):
        if duration <= budget + EPS and covers(target, end, design) and swept_clearance(start, end, design):
            command = asdict(end) | {"motion_duration_s": duration,
                                     "finishes_since_kick_s": target.elapsed_since_kick_s + extra_delay_s + duration}
            return result | {"reachable": True, "reason": "REACHABLE", "command": command}
    return result | {"reason": "NO_SAFE_INTERCEPT_IN_TIME"}


def motion_samples(start, end, duration, design):
    profiles = [Profile.create(q0, q1, duration, axis)
                for q0, q1, axis in zip(start.values(), end.values(), design.axes)]
    step = design.planning["sample_period_s"]
    times = [i * step for i in range(math.ceil(duration / step)) if i * step < duration - EPS] + [duration]
    for t in times:
        yield t, [profile.sample(t) for profile in profiles]
