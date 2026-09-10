"""Dependency-free differential-drive limits; SI units throughout."""
from dataclasses import dataclass
import math


@dataclass
class DriveLimiter:
    radius: float = 0.1
    track: float = 0.48
    max_speed: float = 2.0
    max_accel: float = 0.5
    max_yaw_rate: float = 2.0
    max_yaw_accel: float = 1.5
    timeout: float = 0.5
    linear: float = 0.0
    angular: float = 0.0
    requested_linear: float = 0.0
    requested_angular: float = 0.0
    last_sim: float = -math.inf
    last_wall: float = -math.inf
    accepted: int = 0
    rejected: int = 0
    watchdog_stops: int = 0
    _timed_out: bool = True

    def command(self, linear, angular, sim_time, wall_time):
        if not (math.isfinite(linear) and math.isfinite(angular)):
            self.rejected += 1
            self.requested_linear = self.requested_angular = 0.0
            self.last_sim = self.last_wall = -math.inf
            return False
        self.requested_linear = max(-self.max_speed, min(self.max_speed, linear))
        self.requested_angular = max(-self.max_yaw_rate, min(self.max_yaw_rate, angular))
        self.last_sim, self.last_wall = sim_time, wall_time
        self.accepted += 1
        self._timed_out = False
        return True

    def step(self, dt, sim_time, wall_time):
        expired = (sim_time - self.last_sim > self.timeout or
                   wall_time - self.last_wall > self.timeout)
        if expired and not self._timed_out:
            self.watchdog_stops += 1
            self._timed_out = True
        v_goal = 0.0 if expired else self.requested_linear
        w_goal = 0.0 if expired else self.requested_angular
        goal_ratio = max(1.0, abs(v_goal - w_goal * self.track / 2) / self.max_speed,
                         abs(v_goal + w_goal * self.track / 2) / self.max_speed)
        v_goal, w_goal = v_goal / goal_ratio, w_goal / goal_ratio
        dv, dw = v_goal - self.linear, w_goal - self.angular
        # Interpolate both components by one fraction: the wheel-speed feasible
        # region is convex, so this also preserves each acceleration bound near it.
        fraction = min(1.0,
                       self.max_accel * dt / abs(dv) if dv else 1.0,
                       self.max_yaw_accel * dt / abs(dw) if dw else 1.0)
        self.linear += fraction * dv
        self.angular += fraction * dw
        left = (self.linear - self.angular * self.track / 2) / self.radius
        right = (self.linear + self.angular * self.track / 2) / self.radius
        # Preserve curvature while enforcing the individual wheel rim-speed limit.
        ratio = max(1.0, abs(left) * self.radius / self.max_speed,
                    abs(right) * self.radius / self.max_speed)
        left, right = left / ratio, right / ratio
        self.linear = self.radius * (left + right) / 2
        self.angular = self.radius * (right - left) / self.track
        return left, right
