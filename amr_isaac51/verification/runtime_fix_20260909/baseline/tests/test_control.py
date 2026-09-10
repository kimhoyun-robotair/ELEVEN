"""Offline command-boundary tests; these do not claim a physical simulation pass."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sim.control import DriveLimiter


class DriveLimiterTests(unittest.TestCase):
    def test_forward_and_positive_yaw_wheel_signs(self):
        c = DriveLimiter(max_accel=10, max_yaw_accel=10)
        c.command(1, 0, 0, 0)
        l, r = c.step(.1, .1, .1)
        self.assertAlmostEqual(l, 10)
        self.assertAlmostEqual(r, 10)
        c = DriveLimiter(max_accel=10, max_yaw_accel=10)
        c.command(0, 1, 0, 0)
        l, r = c.step(.1, .1, .1)
        self.assertLess(l, 0)
        self.assertGreater(r, 0)
        self.assertAlmostEqual(.1 * (r - l) / .48, 1)

    def test_linear_and_yaw_acceleration(self):
        c = DriveLimiter(max_accel=.6, max_yaw_accel=1.5)
        c.command(2, 1.5, 0, 0)
        c.step(.01, .01, .01)
        self.assertAlmostEqual(c.linear, .006)
        self.assertLessEqual(abs(c.angular), .015)
        c = DriveLimiter(max_accel=.6, max_yaw_accel=1.5)
        c.command(0, 1.5, 0, 0)
        c.step(.01, .01, .01)
        self.assertAlmostEqual(c.angular, .015)

    def test_saturation_and_requested_curvature(self):
        c = DriveLimiter(max_accel=.6, max_yaw_accel=1.5, max_yaw_rate=1.5)
        for i in range(1200):
            t = i / 120
            c.command(20, 20, t, t)
            l, r = c.step(1 / 120, t, t)
            self.assertLessEqual(max(abs(l), abs(r)), 20 + 1e-10)
        self.assertAlmostEqual(c.angular / c.linear, 1.5 / 2, places=6)
        self.assertAlmostEqual(r, 20, places=6)
        # Leaving a saturated turn must also respect acceleration bounds.
        for i in range(1200,1600):
            t = i / 120
            c.command(2, 0, t, t)
            old_v, old_w = c.linear, c.angular
            c.step(1 / 120, t, t)
            self.assertLessEqual(abs(c.linear - old_v), .6 / 120 + 1e-10)
            self.assertLessEqual(abs(c.angular - old_w), 1.5 / 120 + 1e-10)

    def test_stale_command_causes_controlled_stop(self):
        for expire_by_wall in (False, True):
            c = DriveLimiter(max_accel=1, timeout=.5)
            c.command(1, 0, 0, 0)
            c.step(.1, .1, .1)
            self.assertAlmostEqual(c.linear, .1)
            for i in range(20):
                t = 1 + i / 10
                c.step(.1, .1 if expire_by_wall else t, t if expire_by_wall else .1)
            self.assertEqual(c.watchdog_stops, 1)
            self.assertAlmostEqual(c.linear, 0)
            self.assertAlmostEqual(c.angular, 0)

    def test_nonfinite_commands_are_rejected_and_clear_target(self):
        c = DriveLimiter()
        for v, w in ((math.nan, 0), (math.inf, 0), (0, -math.inf)):
            self.assertFalse(c.command(v, w, 0, 0))
        self.assertEqual(c.rejected, 3)
        self.assertEqual(c.step(.01, .01, .01), (0, 0))


if __name__ == "__main__":
    unittest.main()
