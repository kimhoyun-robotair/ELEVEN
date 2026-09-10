"""Flat-floor penetration checks and rotated cylinder support geometry."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sim.physics_health import PhysicsHealth, cylinder_bottom


class PhysicsHealthTests(unittest.TestCase):
    def test_horizontal_and_vertical_wheel_axis_support(self):
        self.assertAlmostEqual(cylinder_bottom([0, 0, .04], [1, 0, 0, 0], .04, .024), 0)
        q = [math.sqrt(.5), math.sqrt(.5), 0, 0]
        self.assertAlmostEqual(cylinder_bottom([0, 0, .012], q, .04, .024), 0)

    def test_rolling_does_not_change_circular_support(self):
        for angle in range(0, 360, 15):
            t = math.radians(angle) / 2
            self.assertAlmostEqual(cylinder_bottom([0, 0, .04], [math.cos(t), 0, math.sin(t), 0], .04, .024), 0)

    def test_requires_stable_initial_contact(self):
        health = PhysicsHealth()
        for time in [0, 0.5, 1, 1.5]:
            health.update(time, 0, {'caster_fr': 0})
            self.assertFalse(health.ready)
        health.update(2, 0, {'caster_fr': 0})
        self.assertTrue(health.ready)

    def test_sustained_penetration_fails_with_wheel_name(self):
        health = PhysicsHealth()
        health.update(2, 0, {'caster_fr': -.046})
        self.assertFalse(health.ready)
        with self.assertRaisesRegex(RuntimeError, 'caster_fr penetrated 46.00 mm'):
            health.update(2.6, 0, {'caster_fr': -.046})

    def test_transient_contact_error_does_not_trigger_sustained_failure(self):
        health = PhysicsHealth()
        health.update(2, 0, {'caster_fr': -.003})
        health.update(2.1, 0, {'caster_fr': 0})
        health.update(2.7, 0, {'caster_fr': 0})
        self.assertTrue(health.ready)
        self.assertAlmostEqual(health.min_bottom_m['caster_fr'], -.003)

    def test_sustained_tilt_and_nonfinite_measurements_fail(self):
        health = PhysicsHealth()
        health.update(2, 7, {'caster_fr': 0})
        with self.assertRaisesRegex(RuntimeError, 'base tilt'):
            health.update(2.6, 7, {'caster_fr': 0})
        with self.assertRaisesRegex(RuntimeError, 'Non-finite'):
            PhysicsHealth().update(0, 0, {'caster_fr': math.nan})


if __name__ == '__main__':
    unittest.main()
