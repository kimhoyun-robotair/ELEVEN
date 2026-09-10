"""Validate the frame contract of the bundled Isaac Sim 5.1 Camera API."""
from fractions import Fraction
import math
from numbers import Integral

import numpy as np


def camera_sample(frame, width, height, sim_time, physics_dt):
    required = {"rgb", "distance_to_image_plane", "rendering_frame", "rendering_time"}
    missing = required.difference(frame)
    if missing:
        raise ValueError(f"Camera frame missing keys: {sorted(missing)}; available: {sorted(frame)}")
    rgba, depth = frame["rgb"], frame["distance_to_image_plane"]
    if rgba is None or depth is None:
        return None
    rgba, depth = np.asarray(rgba), np.asarray(depth).squeeze()
    if not rgba.size or not depth.size:
        return None
    if rgba.shape != (height, width, 4) or depth.shape != (height, width):
        raise ValueError(f"Camera shape mismatch: rgb={rgba.shape}, depth={depth.shape}, expected={width}x{height}")
    reference = frame["rendering_frame"]
    if not isinstance(reference, dict):
        raise ValueError(f"Expected Camera ReferenceTime mapping, got {type(reference).__name__}")
    numerator = reference.get("referenceTimeNumerator")
    denominator = reference.get("referenceTimeDenominator")
    if not isinstance(numerator, Integral) or not isinstance(denominator, Integral) or denominator <= 0:
        raise ValueError(f"Invalid Camera ReferenceTime: {reference}")
    key = Fraction(int(numerator), int(denominator))
    timestamp = float(frame["rendering_time"])
    if not math.isfinite(timestamp) or timestamp < 0 or timestamp > sim_time + physics_dt:
        raise ValueError(f"Invalid camera simulation timestamp: {timestamp}, current simulation time: {sim_time}")
    return key, timestamp, rgba, depth
