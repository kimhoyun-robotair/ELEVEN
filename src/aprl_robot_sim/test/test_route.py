import math
import unittest
from unittest.mock import patch

from aprl_robot_sim.route import Route


class PanelApproachControlTests(unittest.TestCase):
    def command(self, current_y, target_y):
        node = Route.__new__(Route)
        node.pose = (-2.92, current_y, .15, -math.pi / 2)
        node.speed = .45
        commands = []

        def observe_wait(predicate, *, control, hold_door):
            self.assertFalse(predicate())
            self.assertTrue(hold_door)
            commands.append(control())

        with patch.object(node, 'wait', side_effect=observe_wait):
            node.straight_to(target_y, hold_door=True)
        return commands[0]

    def test_reverse_toward_panel_while_lidar_faces_exit(self):
        linear, angular = self.command(8.82, 9.22)
        self.assertLess(linear, 0)
        self.assertAlmostEqual(angular, 0)

    def test_drive_forward_away_from_panel_while_lidar_faces_exit(self):
        linear, angular = self.command(9.22, 8.4)
        self.assertGreater(linear, 0)
        self.assertAlmostEqual(angular, 0)

    def test_reverse_diagonal_keeps_front_lidar_pointed_outward(self):
        node = Route.__new__(Route)
        node.pose = (-3.5, 8.4, .15, -3*math.pi/4)
        node.speed, node.phase = .45, 'cabin_button'
        commands = []

        def observe_wait(predicate, *, control, hold_door):
            self.assertFalse(predicate())
            commands.append(control())

        with patch.object(node, 'wait', side_effect=observe_wait):
            node.go([-3.0, 8.9], reverse=True)
        linear, angular = commands[0]
        self.assertLess(linear, 0)
        self.assertAlmostEqual(angular, 0)


if __name__ == '__main__':
    unittest.main()
