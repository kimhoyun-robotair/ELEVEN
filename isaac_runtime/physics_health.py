"""Flat test-world contact and attitude checks, independent of Isaac imports."""
from dataclasses import dataclass, field
import math
from collections.abc import Mapping, Sequence


def cylinder_bottom(position: Sequence[float], quaternion: Sequence[float], radius: float, width: float) -> float:
    w, x, y, z = map(float, quaternion)
    norm = math.sqrt(w*w + x*x + y*y + z*z)
    if not math.isfinite(norm) or norm == 0:
        raise ValueError("Invalid wheel orientation")
    axis_z = max(-1.0, min(1.0, 2 * (y*z + w*x) / (norm*norm)))
    support_height = radius * math.sqrt(max(0.0, 1 - axis_z*axis_z)) + width / 2 * abs(axis_z)
    return float(position[2]) - support_height


@dataclass
class PhysicsHealth:
    stable_since: float | None = None
    bad_since: float | None = None
    ready: bool = False
    max_tilt_deg: float = 0.0
    min_bottom_m: dict[str, float] = field(default_factory=dict)

    def update(self, seconds: float, tilt_deg: float, bottoms: Mapping[str, float]) -> None:
        if not math.isfinite(tilt_deg) or not all(math.isfinite(z) for z in bottoms.values()):
            raise RuntimeError("Non-finite wheel/attitude measurement")
        stable = tilt_deg < 1.0 and all(z >= -0.001 for z in bottoms.values())
        if stable:
            if self.stable_since is None:
                self.stable_since = seconds
        else:
            self.stable_since = None
        self.ready = self.stable_since is not None and seconds >= 2 and seconds - self.stable_since >= 0.5
        if seconds < 2:
            return
        self.max_tilt_deg = max(self.max_tilt_deg, tilt_deg)
        for name, height in bottoms.items():
            self.min_bottom_m[name] = min(self.min_bottom_m.get(name, height), height)
        issues = [f"{name} penetrated {-z*1000:.2f} mm" for name, z in bottoms.items() if z < -0.002]
        if tilt_deg > 5:
            issues.append(f"base tilt {tilt_deg:.2f} deg")
        if issues:
            if self.bad_since is None:
                self.bad_since = seconds
            if seconds - self.bad_since >= 0.5:
                raise RuntimeError("Physics contact check failed: " + "; ".join(issues))
        else:
            self.bad_since = None
