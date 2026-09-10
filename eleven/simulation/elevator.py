"""Deterministic, dependency-free elevator controller shared with elevator.mjs.

API
---
ElevatorController(floor_heights, initial_floor=0, max_speed=1.8,
                   acceleration=1.0, door_seconds=1.8, dwell_seconds=3.0)
uses metres and seconds. Keyword options are keyword-only. Floors are zero-based
integer indices; supplied heights must be finite and strictly increasing.

request_floor(floor) and request_hall(floor, "up" | "down") return whether a new
request was registered. Duplicate requests never toggle an existing lamp off.
The controller groups calls by landing, visits landings FIFO, and clears the
landing's cabin and hall calls only after its doors have opened completely.
This is a single-car demonstration dispatcher, not an elevator-bank dispatcher:
both hall directions at a serviced landing count as served.

open_doors()/close_doors() return whether that action is safe now. Moving cars
ignore door commands. An obstruction reopens closing doors and holds open doors
at a landing; it does not command opening between floors. press_alarm() provides
a momentary visual alarm signal without changing movement. tick(dt) advances
time and returns snapshot(); snapshot() never mutates state.

Snapshot keys use camelCase in both implementations: state, floor, targetFloor,
position, velocity, doorOpen, cabinLights, hallLights, queue, obstruction,
openButtonLit, closeButtonLit, alarmLit. ``floor`` is the last reached landing
while travelling. ``position`` is the cabin floor's world Z coordinate.
``doorOpen`` is 0 (closed) through 1 (fully open). Lamps are request state, never
inferred from proximity or the selected floor. Controller state is independent
of rendering, wall-clock time, and any physics engine.
"""

from __future__ import annotations

import math
from typing import Iterable


class ElevatorController:
    """Safe door sequencing and an exact triangular/trapezoidal motion profile."""

    def __init__(
        self,
        floor_heights: Iterable[float],
        *,
        initial_floor: int = 0,
        max_speed: float = 1.8,
        acceleration: float = 1.0,
        door_seconds: float = 1.8,
        dwell_seconds: float = 3.0,
    ) -> None:
        self.floor_heights = tuple(self._number(v, "floor height") for v in floor_heights)
        if len(self.floor_heights) < 2:
            raise ValueError("at least two floor heights are required")
        if any(b <= a for a, b in zip(self.floor_heights, self.floor_heights[1:])):
            raise ValueError("floor heights must be strictly increasing")
        self.max_speed = self._positive(max_speed, "max_speed")
        self.acceleration = self._positive(acceleration, "acceleration")
        self.door_seconds = self._positive(door_seconds, "door_seconds")
        self.dwell_seconds = self._number(dwell_seconds, "dwell_seconds")
        if self.dwell_seconds < 0:
            raise ValueError("dwell_seconds must be nonnegative")
        self._validate_floor(initial_floor)
        self.floor = initial_floor
        self.position = self.floor_heights[initial_floor]
        self.velocity = 0.0
        self.door_open = 0.0
        self.state = "idle"
        self.target_floor: int | None = None
        self.obstruction = False
        self._cabin = [False] * len(self.floor_heights)
        self._hall = [{"up": False, "down": False} for _ in self.floor_heights]
        self._queue: list[int] = []
        self._dwell_remaining = 0.0
        self._motion: dict[str, float] | None = None
        self._button_seconds = {"open": 0.0, "close": 0.0, "alarm": 0.0}

    @staticmethod
    def _number(value: float, name: str) -> float:
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
            raise ValueError(f"{name} must be a finite number")
        return float(value)

    @classmethod
    def _positive(cls, value: float, name: str) -> float:
        value = cls._number(value, name)
        if value <= 0:
            raise ValueError(f"{name} must be positive")
        return value

    def _validate_floor(self, floor: int) -> None:
        if isinstance(floor, bool) or not isinstance(floor, int) or not 0 <= floor < len(self.floor_heights):
            raise ValueError("floor must be a valid zero-based integer index")

    def request_floor(self, floor: int) -> bool:
        """Register a cabin button; a duplicate request leaves its lamp lit."""
        self._validate_floor(floor)
        if self._cabin[floor]:
            return False
        self._cabin[floor] = True
        self._enqueue(floor)
        return True

    def request_hall(self, floor: int, direction: str) -> bool:
        """Register one hall direction; impossible terminal directions reject."""
        self._validate_floor(floor)
        if direction not in ("up", "down"):
            raise ValueError("direction must be 'up' or 'down'")
        if (floor == 0 and direction == "down") or (floor == len(self.floor_heights) - 1 and direction == "up"):
            raise ValueError("hall direction leads beyond the building")
        if self._hall[floor][direction]:
            return False
        self._hall[floor][direction] = True
        self._enqueue(floor)
        return True

    def _enqueue(self, floor: int) -> None:
        if floor not in self._queue:
            self._queue.append(floor)
        if self.state != "moving" and floor == self.floor:
            if self.state == "dwell":
                # A call at an already open landing is immediately served.
                self._serve_landing()
                self._dwell_remaining = self.dwell_seconds
            else:
                self._begin_opening()
        elif self.state == "idle":
            self._dispatch()

    def open_doors(self) -> bool:
        """Press door-open; unsafe requests are ignored, with a momentary LED."""
        self._button_seconds["open"] = 0.8
        if self.state == "moving":
            return False
        if self.state == "dwell":
            self._dwell_remaining = self.dwell_seconds
        else:
            self._begin_opening()
        return True

    def close_doors(self) -> bool:
        """Press door-close; obstruction and movement inhibit closing."""
        self._button_seconds["close"] = 0.8
        if self.state == "moving" or self.obstruction:
            return False
        if self.door_open > 0 or self.state == "opening":
            self._begin_closing()
        elif self.state == "idle":
            self._dispatch()
        return True

    def press_alarm(self) -> bool:
        """Signal the alarm button for 1.5 s; this is not an emergency stop."""
        self._button_seconds["alarm"] = 1.5
        return True

    def set_obstruction(self, obstructed: bool) -> None:
        if not isinstance(obstructed, bool):
            raise ValueError("obstructed must be a boolean")
        self.obstruction = obstructed
        if obstructed and self.state == "closing":
            self._begin_opening()
        elif not obstructed and self.state == "idle":
            self._dispatch()

    def _begin_opening(self) -> None:
        # Only callers at a landing reach this method; moving calls are ignored.
        self.state = "opening"
        self.target_floor = self.floor
        self.velocity = 0.0

    def _begin_closing(self) -> None:
        self.state = "closing"
        self.target_floor = self._queue[0] if self._queue else None

    def _serve_landing(self) -> None:
        self._cabin[self.floor] = False
        self._hall[self.floor] = {"up": False, "down": False}
        self._queue = [f for f in self._queue if f != self.floor]

    def _dispatch(self) -> None:
        if self.door_open != 0.0:
            raise RuntimeError("travel interlock: cabin doors must be fully closed")
        if not self._queue:
            self.state = "idle"
            self.target_floor = None
            return
        self.target_floor = self._queue[0]
        if self.target_floor == self.floor:
            self._begin_opening()
        elif self.obstruction:
            self._begin_opening()
        else:
            self._begin_movement()

    def _begin_movement(self) -> None:
        if self.door_open != 0.0 or self.obstruction:
            raise RuntimeError("travel interlock: doors must be closed and clear")
        distance = abs(self.floor_heights[self.target_floor] - self.position)
        ramp = min(self.max_speed / self.acceleration, math.sqrt(distance / self.acceleration))
        peak = self.acceleration * ramp
        cruise = max(0.0, (distance - self.acceleration * ramp * ramp) / peak)
        self._motion = {
            "start": self.position,
            "sign": 1.0 if self.floor_heights[self.target_floor] > self.position else -1.0,
            "distance": distance,
            "ramp": ramp,
            "peak": peak,
            "cruise": cruise,
            "duration": 2.0 * ramp + cruise,
            "elapsed": 0.0,
        }
        self.state = "moving"

    def tick(self, dt: float) -> dict:
        """Advance by finite nonnegative seconds, exactly across state boundaries.

        Motion is an analytic acceleration/cruise/deceleration profile, so the
        resulting travel is independent of the caller's frame rate. Large time
        steps process arrivals, opening, dwell, and closing in the correct order.
        """
        remaining = self._number(dt, "dt")
        if remaining < 0:
            raise ValueError("dt must be nonnegative")
        for button in self._button_seconds:
            self._button_seconds[button] = max(0.0, self._button_seconds[button] - remaining)
        while remaining > 0.0:
            if self.state == "idle":
                break
            if self.state == "moving":
                motion = self._motion
                available = motion["duration"] - motion["elapsed"]
                step = min(remaining, available)
                motion["elapsed"] += step
                remaining = max(0.0, remaining - step)
                t = motion["elapsed"]
                ramp, peak = motion["ramp"], motion["peak"]
                if t < ramp:
                    displacement = 0.5 * self.acceleration * t * t
                    speed = self.acceleration * t
                elif t < ramp + motion["cruise"]:
                    displacement = 0.5 * self.acceleration * ramp * ramp + peak * (t - ramp)
                    speed = peak
                else:
                    until_arrival = max(0.0, motion["duration"] - t)
                    displacement = motion["distance"] - 0.5 * self.acceleration * until_arrival * until_arrival
                    speed = self.acceleration * until_arrival
                self.position = motion["start"] + motion["sign"] * displacement
                self.velocity = motion["sign"] * speed
                if step >= available:
                    self.floor = self.target_floor
                    self.position = self.floor_heights[self.floor]
                    self.velocity = 0.0
                    self._motion = None
                    self._begin_opening()
            elif self.state == "opening":
                available = (1.0 - self.door_open) * self.door_seconds
                step = min(remaining, available)
                self.door_open = min(1.0, self.door_open + step / self.door_seconds)
                remaining = max(0.0, remaining - step)
                if step >= available:
                    self.door_open = 1.0
                    self._serve_landing()
                    self.target_floor = None
                    self.state = "dwell"
                    self._dwell_remaining = self.dwell_seconds
            elif self.state == "dwell":
                if self.obstruction:
                    self._dwell_remaining = self.dwell_seconds
                    break
                step = min(remaining, self._dwell_remaining)
                self._dwell_remaining = max(0.0, self._dwell_remaining - step)
                remaining = max(0.0, remaining - step)
                if self._dwell_remaining == 0.0:
                    self._begin_closing()
            elif self.state == "closing":
                if self.obstruction:
                    self._begin_opening()
                    continue
                available = self.door_open * self.door_seconds
                step = min(remaining, available)
                self.door_open = max(0.0, self.door_open - step / self.door_seconds)
                remaining = max(0.0, remaining - step)
                if step >= available:
                    self.door_open = 0.0
                    self._dispatch()
            else:
                raise RuntimeError(f"unknown elevator state: {self.state}")
        return self.snapshot()

    def snapshot(self) -> dict:
        """Return copies of light/queue state; consumers may safely retain them."""
        return {
            "state": self.state,
            "floor": self.floor,
            "targetFloor": self.target_floor,
            "position": self.position,
            "velocity": self.velocity,
            "doorOpen": self.door_open,
            "cabinLights": self._cabin.copy(),
            "hallLights": [calls.copy() for calls in self._hall],
            "queue": self._queue.copy(),
            "obstruction": self.obstruction,
            "openButtonLit": self._button_seconds["open"] > 0,
            "closeButtonLit": self._button_seconds["close"] > 0,
            "alarmLit": self._button_seconds["alarm"] > 0,
        }
