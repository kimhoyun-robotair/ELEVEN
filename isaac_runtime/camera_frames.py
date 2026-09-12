"""Validate the frame contract of the bundled Isaac Sim 5.1 Camera API."""
from fractions import Fraction
import math
from numbers import Integral

import numpy as np


def camera_sample(frame, width, height, sim_time, physics_dt, channels=("color", "depth")):
    required = {"rendering_frame", "rendering_time"}
    if "color" in channels:
        required.add("rgb")
    if "depth" in channels:
        required.add("distance_to_image_plane")
    missing = required.difference(frame)
    if missing:
        raise ValueError(f"Camera frame missing keys: {sorted(missing)}; available: {sorted(frame)}")
    pixels = {}
    for channel, key, shape in (("color", "rgb", (height, width, 4)),
                                ("depth", "distance_to_image_plane", (height, width))):
        if channel not in channels:
            continue
        if frame[key] is None or not np.asarray(frame[key]).size:
            return None
        data = np.asarray(frame[key])
        if channel == "depth":
            data = data.squeeze()
        if data.shape != shape:
            raise ValueError(f"Camera shape mismatch: {key}={data.shape}, expected={shape}")
        pixels[channel] = data
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
    return key, timestamp, pixels.get("color"), pixels.get("depth")
