import math
import unittest
from unittest.mock import patch

from aprl_robot_sim.route import Route


class ScoutPivotControlTests(unittest.TestCase):
    def test_small_scout_heading_error_can_overcome_skid_friction(self):
        for error in (-.07, .07):
            node = Route.__new__(Route)
            node.robot, node.pose = 'scout', (0, 0, .08, -error)

            def observe(predicate, *, control, hold_door):
                self.assertFalse(predicate())
                linear, angular = control()
                self.assertEqual(linear, 0)
                self.assertGreaterEqual(abs(angular), .25)
                self.assertEqual(math.copysign(1, angular), math.copysign(1, error))

            with patch.object(node, 'wait', side_effect=observe):
                node.face(0)

    def test_amr_still_uses_proportional_heading_control(self):
        node = Route.__new__(Route)
        node.robot, node.pose = 'amr', (0, 0, .15, -.07)

        def observe(predicate, *, control, hold_door):
            self.assertFalse(predicate())
            self.assertAlmostEqual(control()[1], .105)

        with patch.object(node, 'wait', side_effect=observe):
            node.face(0)


if __name__ == '__main__':
    unittest.main()
