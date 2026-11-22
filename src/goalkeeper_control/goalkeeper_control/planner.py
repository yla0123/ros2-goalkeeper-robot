"""Sampled footprint search with conservative swept-clearance checks."""
from dataclasses import asdict
import math

from .model import RobotState
from .motion import minimum_time, Profile


def grid(lower, upper, step):
    count = int(math.ceil((upper - lower) / step))
    return [min(upper, lower + index * step) for index in range(count + 1)]


def keeper_corners(slide, tilt, design):
    c, s = math.cos(tilt), math.sin(tilt)
    return [
        [slide + u * c - v * s, design.pivot_height_m + u * s + v * c]
        for part in design.footprint for u, v in part.corners()
    ]


def signed_keeper_distance(y, z, slide, tilt, design):
    c, s = math.cos(tilt), math.sin(tilt)
    dy, dz = y - slide, z - design.pivot_height_m
    local_y = dy * c + dz * s
    local_z = -dy * s + dz * c
    distances = []
    for part in design.footprint:
        qy = abs(local_y - part.center_y_m) - part.width_m / 2
        qz = abs(local_z - part.center_z_m) - part.height_m / 2
        distances.append(math.hypot(max(qy, 0), max(qz, 0)) + min(max(qy, qz), 0))
    return min(distances)


def covers(target, slide, tilt, design):
    distance = signed_keeper_distance(target.y_m, target.z_m, slide, tilt, design)
    return distance <= design.ball_radius_m - design.minimum_overlap_m + 1e-10


def trig_bounds(a, b, lower, upper):
    angles = [lower, upper]
    stationary = math.atan2(b, a)
    for multiple in range(-2, 3):
        angle = stationary + multiple * math.pi
        if lower <= angle <= upper:
            angles.append(angle)
    values = [a * math.cos(angle) + b * math.sin(angle) for angle in angles]
    return min(values), max(values)


def swept_clearance(start, slide, tilt, design):
    """Enclose every pose between axis endpoints, including internal extrema.

    Treat translation and rotation independently. This is conservative: it
    can reject a synchronized path that would actually fit inside the frame.
    """
    low_angle, high_angle = sorted([start.tilt_rad, tilt])
    low_slide, high_slide = sorted([start.slide_m, slide])
    ys, zs = [], []
    for part in design.footprint:
        for u, v in part.corners():
            ylo, yhi = trig_bounds(u, -v, low_angle, high_angle)
            zlo, zhi = trig_bounds(v, u, low_angle, high_angle)
            ys.extend([low_slide + ylo, high_slide + yhi])
            zs.extend([design.pivot_height_m + zlo, design.pivot_height_m + zhi])
    margin = design.frame_clearance_m
    return (
        min(ys) >= -design.goal_width_m / 2 + margin - 1e-10
        and max(ys) <= design.goal_width_m / 2 - margin + 1e-10
        and min(zs) >= margin - 1e-10
        and max(zs) <= design.goal_height_m - margin + 1e-10
    )


def plan(target, design, state=None):
    target.validate()
    design.validate()
    state = state or RobotState()
    state.validate(design)
    remaining = target.arrival_time_s - target.elapsed_since_kick_s
    budget = remaining - design.settling_margin_s
    result = {
        "schema_version": 3, "mode": "planning_only", "hardware_enabled": False,
        "reachable": False, "reason": None,
        "target": asdict(target), "initial_state": asdict(state),
        "time_to_goal_s": round(remaining, 9),
        "motion_budget_s": round(budget, 9),
        "settling_margin_s": design.settling_margin_s,
        "command": None,
        "model_assumptions": [
            "Known goal-plane ball-center coordinate; no camera processing is implemented.",
            "Both axes initially stationary; capabilities are unverified design assumptions.",
            "Rigid human-shaped union of body parts; 2D contact is not a demonstrated save.",
            "Goal opening follows IFAB Law 1; keeper proportions and motion limits are model assumptions.",
            "Processing and communication are already included in elapsed_since_kick_s.",
        ],
    }
    if abs(state.slide_velocity_mps) > 1e-6 or abs(state.tilt_velocity_radps) > 1e-6:
        result["reason"] = "INITIAL_STATE_MOVING"
        return result
    if budget <= 0:
        result["reason"] = "DEADLINE_TOO_SHORT"
        return result
    if remaining * 1000 > design.serial_max_ttl_ms + 1e-6:
        result["reason"] = "DEADLINE_EXCEEDS_PROTOCOL_LIMIT"
        return result
    if abs(target.y_m) > design.goal_width_m / 2 or not 0 <= target.z_m <= design.goal_height_m:
        result["reason"] = "TARGET_OUTSIDE_GOAL"
        return result
    if not swept_clearance(state, state.slide_m, state.tilt_rad, design):
        result["reason"] = "INITIAL_POSE_COLLIDES_WITH_FRAME"
        return result
    angles = sorted(set(grid(design.tilt.minimum, design.tilt.maximum,
                             design.tilt_search_step_rad) + [0.0, state.tilt_rad]))
    slides = grid(design.slide.minimum, design.slide.maximum, design.slide_search_step_m)
    best, geometry_found, feasible_found = None, False, False
    for angle in angles:
        angle_time = minimum_time(angle - state.tilt_rad, design.tilt)
        # An extra candidate aligns the body centerline with the crossing point.
        extra = target.y_m
        if abs(math.cos(angle)) > 1e-6:
            extra += (target.z_m - design.pivot_height_m) * math.tan(angle)
        candidates = sorted(set(slides + [state.slide_m,
                             max(design.slide.minimum, min(design.slide.maximum, extra))]))
        for slide in candidates:
            if not covers(target, slide, angle, design):
                continue
            geometry_found = True
            if not swept_clearance(state, slide, angle, design):
                continue
            feasible_found = True
            slide_time = minimum_time(slide - state.slide_m, design.slide)
            # A stationary blocking pose still needs a positive codec duration.
            required = max(slide_time, angle_time, 0.001)
            if required > budget + 1e-9:
                continue
            score = (required, abs(slide - state.slide_m) + 0.15 * abs(angle - state.tilt_rad),
                     abs(angle), abs(slide))
            if best is None or score < best[0]:
                best = (score, slide, angle, slide_time, angle_time)
    if best is None:
        result["reason"] = ("DEADLINE_TOO_SHORT" if feasible_found else
                            "SWEPT_FRAME_COLLISION" if geometry_found else
                            "NO_KEEPER_COVERAGE")
        return result
    _, slide, angle, slide_time, angle_time = best
    # Finish as soon as the slower axis can complete. Moving immediately and
    # holding the blocking pose leaves more time before the ball arrives.
    motion_duration = max(slide_time, angle_time, 0.001)
    slide_profile = Profile.create(state.slide_m, slide, motion_duration, design.slide)
    tilt_profile = Profile.create(state.tilt_rad, angle, motion_duration, design.tilt)
    result.update(reachable=True, reason="REACHABLE", command={
        "slide_m": slide, "tilt_rad": angle, "tilt_deg": math.degrees(angle),
        "motion_duration_s": motion_duration,
        "execute_at_s": target.elapsed_since_kick_s,
        "motion_finishes_at_s": target.elapsed_since_kick_s + motion_duration,
        "expires_at_s": target.arrival_time_s,
        "minimum_slide_time_s": slide_time, "minimum_tilt_time_s": angle_time,
        "minimum_parallel_motion_time_s": max(slide_time, angle_time),
        "slide_peak_speed_mps": slide_profile.peak_speed,
        "tilt_peak_speed_radps": tilt_profile.peak_speed,
        "geometric_overlap_m": design.ball_radius_m -
            signed_keeper_distance(target.y_m, target.z_m, slide, angle, design),
    })
    return result


def trajectory(result, design):
    """Model states from kick time to crossing time; unreachable shots hold."""
    target, initial = result["target"], result["initial_state"]
    start_time = target["elapsed_since_kick_s"]
    end_time = target["arrival_time_s"]
    command = result["command"]
    profiles = None
    if result["reachable"]:
        profiles = [
            Profile.create(initial["slide_m"], command["slide_m"],
                           command["motion_duration_s"], design.slide),
            Profile.create(initial["tilt_rad"], command["tilt_rad"],
                           command["motion_duration_s"], design.tilt),
        ]
    count = int(math.ceil(end_time / design.sample_period_s))
    times = set(min(end_time, index * design.sample_period_s) for index in range(count + 1))
    if 0 <= start_time <= end_time:
        times.add(start_time)
    if command:
        times.add(command["motion_finishes_at_s"])
    rows = []
    for time_s in sorted(times):
        if profiles and time_s >= start_time:
            slide, sv, sa = profiles[0].sample(time_s - start_time)
            tilt, tv, ta = profiles[1].sample(time_s - start_time)
        else:
            slide, tilt = initial["slide_m"], initial["tilt_rad"]
            sv = sa = tv = ta = 0.0
        phase = ("processing" if time_s < start_time else
                 "moving" if command and time_s < command["motion_finishes_at_s"] else
                 "settled" if command else "hold")
        rows.append({
            "time_s": round(time_s, 9), "slide_m": slide, "tilt_rad": tilt,
            "slide_velocity_mps": sv, "tilt_velocity_radps": tv,
            "slide_acceleration_mps2": sa, "tilt_acceleration_radps2": ta,
            "phase": phase,
        })
    return rows
