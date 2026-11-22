"""Acceleration-limited, rest-to-rest trapezoidal motion profiles."""
from dataclasses import dataclass
import math


def minimum_time(distance, axis):
    distance = abs(distance)
    if distance == 0:
        return 0.0
    transition_distance = axis.max_speed ** 2 / axis.max_acceleration
    if distance <= transition_distance:
        return 2 * math.sqrt(distance / axis.max_acceleration)
    return distance / axis.max_speed + axis.max_speed / axis.max_acceleration


@dataclass(frozen=True)
class Profile:
    start: float
    end: float
    duration_s: float
    acceleration: float
    peak_speed: float
    acceleration_time_s: float

    @classmethod
    def create(cls, start, end, duration_s, axis):
        if not math.isfinite(duration_s) or duration_s <= 0:
            raise ValueError("Motion duration must be finite and positive")
        distance = abs(end - start)
        if minimum_time(distance, axis) > duration_s + 1e-9:
            raise ValueError("The requested move cannot meet its duration")
        if distance == 0:
            return cls(start, end, duration_s, 0.0, 0.0, 0.0)
        a = axis.max_acceleration
        discriminant = max(0.0, duration_s ** 2 - 4 * distance / a)
        # Stable smaller root of distance = peak_speed * (duration - peak_speed/a).
        peak = 2 * distance / (duration_s + math.sqrt(discriminant))
        if peak > axis.max_speed + 1e-8:
            raise ValueError("Profile would exceed the speed limit")
        return cls(start, end, duration_s, a, peak, peak / a)

    def sample(self, time_s):
        if time_s <= 0 or self.peak_speed == 0:
            return self.start, 0.0, 0.0
        if time_s >= self.duration_s:
            return self.end, 0.0, 0.0
        a, ramp, peak = self.acceleration, self.acceleration_time_s, self.peak_speed
        if time_s < ramp:
            distance = 0.5 * a * time_s ** 2
            velocity, acceleration = a * time_s, a
        elif time_s <= self.duration_s - ramp:
            distance = 0.5 * a * ramp ** 2 + peak * (time_s - ramp)
            velocity, acceleration = peak, 0.0
        else:
            remaining = self.duration_s - time_s
            distance = abs(self.end - self.start) - 0.5 * a * remaining ** 2
            velocity, acceleration = a * remaining, -a
        direction = 1 if self.end >= self.start else -1
        return self.start + direction * distance, direction * velocity, direction * acceleration
