"""Run with: python -m unittest discover -s tests -p 'test_elevator.py'."""

import math
import unittest

from simulation.elevator import ElevatorController


class ElevatorTests(unittest.TestCase):
    def make_car(self, **options):
        return ElevatorController([0.12, 3.92, 7.72, 11.52], **options)

    def advance_until(self, car, predicate, limit=90):
        for _ in range(int(limit / 0.02)):
            snapshot = car.tick(0.02)
            if predicate(snapshot):
                return snapshot
        self.fail(f"condition not reached: {car.snapshot()}")

    def test_only_requested_cabin_and_hall_buttons_light_until_service(self):
        car = self.make_car()
        self.assertEqual(car.snapshot()["cabinLights"], [False] * 4)
        self.assertTrue(car.request_floor(2))
        self.assertFalse(car.request_floor(2))
        self.assertTrue(car.request_hall(1, "down"))
        self.assertFalse(car.request_hall(1, "down"))
        self.assertEqual(car.snapshot()["queue"], [2, 1])
        self.assertEqual(car.snapshot()["cabinLights"], [False, False, True, False])
        self.assertEqual(car.snapshot()["hallLights"][1], {"up": False, "down": True})
        landed = self.advance_until(car, lambda s: s["floor"] == 2 and s["state"] == "opening")
        self.assertTrue(landed["cabinLights"][2])
        opened = self.advance_until(car, lambda s: s["floor"] == 2 and s["state"] == "dwell")
        self.assertFalse(opened["cabinLights"][2])
        self.assertTrue(opened["hallLights"][1]["down"])
        car.tick(60)
        self.assertEqual(car.snapshot()["state"], "idle")
        self.assertEqual(car.snapshot()["floor"], 1)
        self.assertEqual(car.snapshot()["queue"], [])
        self.assertFalse(any(call["up"] or call["down"] for call in car.snapshot()["hallLights"]))

    def test_floor_grouping_keeps_independent_direction_lamps(self):
        car = self.make_car()
        car.request_hall(1, "up")
        car.request_hall(1, "down")
        car.request_floor(1)
        self.assertEqual(car.snapshot()["queue"], [1])
        self.assertEqual(car.snapshot()["hallLights"][1], {"up": True, "down": True})
        car.tick(60)
        self.assertFalse(car.snapshot()["cabinLights"][1])
        self.assertEqual(car.snapshot()["hallLights"][1], {"up": False, "down": False})

    def test_motion_never_travels_with_open_doors_and_respects_limits(self):
        car = self.make_car(max_speed=1.4, acceleration=0.7)
        car.open_doors()
        car.tick(2)
        car.request_floor(3)
        car.request_floor(0)
        previous = car.snapshot()
        observed_motion = False
        for _ in range(4500):
            current = car.tick(0.02)
            self.assertLessEqual(abs(current["velocity"]), 1.4 + 1e-10)
            self.assertLessEqual(abs(current["velocity"] - previous["velocity"]), 0.7 * 0.02 + 1e-9)
            self.assertGreaterEqual(current["position"], 0.12 - 1e-10)
            self.assertLessEqual(current["position"], 11.52 + 1e-10)
            if current["state"] == "moving" or abs(current["velocity"]) > 1e-10:
                observed_motion = True
                self.assertEqual(current["doorOpen"], 0)
            if current["doorOpen"] > 0:
                self.assertAlmostEqual(current["position"], car.floor_heights[current["floor"]])
                self.assertEqual(current["velocity"], 0)
            previous = current
        self.assertTrue(observed_motion)

    def test_exact_motion_is_invariant_to_frame_partition(self):
        options = dict(max_speed=1.4, acceleration=0.7)
        one_step, many_steps = self.make_car(**options), self.make_car(**options)
        for car in (one_step, many_steps):
            car.request_floor(3)
            car.request_floor(1)
        one_step.tick(17.345)
        for _ in range(3469):
            many_steps.tick(0.005)
        a, b = one_step.snapshot(), many_steps.snapshot()
        self.assertEqual(a["state"], b["state"])
        self.assertEqual(a["queue"], b["queue"])
        for field in ("position", "velocity", "doorOpen"):
            self.assertAlmostEqual(a[field], b[field], places=9)

    def test_obstruction_reopens_closing_doors_and_preserves_destination(self):
        car = self.make_car()
        car.open_doors()
        car.tick(1.8)
        car.request_floor(2)
        car.close_doors()
        car.tick(0.7)
        before = car.snapshot()["doorOpen"]
        car.set_obstruction(True)
        car.tick(0.3)
        self.assertGreater(car.snapshot()["doorOpen"], before)
        car.tick(100)
        held = car.snapshot()
        self.assertEqual((held["state"], held["floor"], held["doorOpen"]), ("dwell", 0, 1))
        self.assertTrue(held["cabinLights"][2])
        self.assertFalse(car.close_doors())
        car.set_obstruction(False)
        car.tick(60)
        self.assertEqual(car.snapshot()["floor"], 2)
        self.assertEqual(car.snapshot()["state"], "idle")

    def test_open_button_and_obstruction_cannot_open_between_floors(self):
        car = self.make_car()
        car.request_floor(3)
        car.tick(1)
        self.assertFalse(car.open_doors())
        self.assertFalse(car.close_doors())
        car.set_obstruction(True)
        snapshot = car.tick(1)
        self.assertEqual(snapshot["state"], "moving")
        self.assertEqual(snapshot["doorOpen"], 0)
        car.tick(60)
        self.assertEqual(car.snapshot()["floor"], 3)
        self.assertEqual(car.snapshot()["doorOpen"], 1)

    def test_current_landing_request_reopens_and_serves_once_open(self):
        car = self.make_car()
        car.request_floor(0)
        self.assertTrue(car.snapshot()["cabinLights"][0])
        car.tick(1.8)
        self.assertFalse(car.snapshot()["cabinLights"][0])
        car.request_floor(0)
        self.assertFalse(car.snapshot()["cabinLights"][0])
        car.close_doors()
        car.tick(0.6)
        car.request_floor(0)
        self.assertEqual(car.snapshot()["state"], "opening")
        car.tick(1)
        self.assertEqual(car.snapshot()["doorOpen"], 1)
        self.assertEqual(car.snapshot()["queue"], [])

    def test_control_leds_only_follow_manual_presses(self):
        car = self.make_car()
        car.request_floor(2)
        car.tick(20)
        self.assertFalse(car.snapshot()["openButtonLit"])
        self.assertFalse(car.snapshot()["closeButtonLit"])
        car.open_doors()
        car.press_alarm()
        self.assertTrue(car.snapshot()["openButtonLit"])
        self.assertTrue(car.snapshot()["alarmLit"])
        car.tick(0.81)
        self.assertFalse(car.snapshot()["openButtonLit"])
        self.assertTrue(car.snapshot()["alarmLit"])
        car.close_doors()
        self.assertTrue(car.snapshot()["closeButtonLit"])
        car.tick(1)
        self.assertFalse(car.snapshot()["closeButtonLit"])
        self.assertFalse(car.snapshot()["alarmLit"])

    def test_snapshot_does_not_expose_mutable_internal_state(self):
        car = self.make_car()
        car.request_floor(2)
        snapshot = car.snapshot()
        snapshot["cabinLights"][2] = False
        snapshot["hallLights"][0]["up"] = True
        snapshot["queue"].clear()
        self.assertEqual(car.snapshot()["queue"], [2])
        self.assertTrue(car.snapshot()["cabinLights"][2])
        self.assertFalse(car.snapshot()["hallLights"][0]["up"])

    def test_input_validation_and_short_triangular_motion(self):
        for heights in ([0], [0, 0], [2, 1], [0, math.inf], [0, True]):
            with self.assertRaises(ValueError):
                ElevatorController(heights)
        car = self.make_car()
        for floor in (-1, 4, 0.5, True):
            with self.assertRaises(ValueError):
                car.request_floor(floor)
        for args in ((0, "down"), (3, "up"), (1, "left")):
            with self.assertRaises(ValueError):
                car.request_hall(*args)
        for dt in (-1, math.nan, math.inf, True):
            with self.assertRaises(ValueError):
                car.tick(dt)
        tiny = ElevatorController([0, 0.01], dwell_seconds=0)
        tiny.request_floor(1)
        tiny.tick(20)
        self.assertEqual(tiny.snapshot()["position"], 0.01)
        self.assertEqual(tiny.snapshot()["state"], "idle")


if __name__ == "__main__":
    unittest.main()
