"""Simulation-only score and joint-feedback helpers."""
from .planner import EPS, inside_opening


def goal_crossing(before, after, design):
    """Return the crossing of the whole ball past the back of the goal line, if inside."""
    x0, y0, z0 = before
    x1, y1, z1 = after
    threshold = design.geometry["goal_plane_world_x_m"] + design.radius
    if not x0 <= threshold < x1:
        return None
    fraction = (threshold - x0) / (x1 - x0)
    y, z = y0 + fraction * (y1 - y0), z0 + fraction * (z1 - z0)
    # Permit small physics contact penetration at the pitch only; preserve the
    # measured crossing and the exact width and crossbar limits.
    floor_tolerance = design.simulation["floor_contact_tolerance_m"]
    check_z = design.radius if design.radius - floor_tolerance <= z < design.radius else z
    return (threshold, y, z) if inside_opening(y, check_z, design) else None


def state_pose(names, positions, velocities, joint_names, axes, rest_tolerances):
    """Extract a known, nearly stationary measured pose from Gazebo joint feedback."""
    if not (len(names) == len(positions) == len(velocities)):
        return None
    data = dict(zip(names, zip(positions, velocities)))
    result = []
    for name, axis, tolerance in zip(joint_names, axes, rest_tolerances):
        if name not in data:
            return None
        q, speed = data[name]
        if not (axis.minimum - 0.002 <= q <= axis.maximum + 0.002 and abs(speed) <= tolerance + EPS):
            return None
        result.append(min(axis.maximum, max(axis.minimum, q)))
    return result
