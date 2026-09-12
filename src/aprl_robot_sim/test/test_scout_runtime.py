import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

PROJECT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(PROJECT))

from isaac_runtime.camera_frames import camera_sample
from isaac_runtime.control import AMR_DRIVE, SCOUT_DRIVE, DriveLimiter
from isaac_runtime.scout import camera_specs, runtime_config
from isaac_runtime.simulation import arguments


class ScoutDriveTests(unittest.TestCase):
    def test_forward_reaches_all_four_wheels(self):
        limiter = DriveLimiter(radius=.08, track=.49)
        limiter.command(.4, 0, 0, 0)
        self.assertEqual(SCOUT_DRIVE.targets(*limiter.step(1, .1, .1)), (5, 5, 5, 5))

    def test_positive_yaw_counter_rotates_left_and_right(self):
        limiter = DriveLimiter(radius=.08, track=.49)
        limiter.command(0, .4, 0, 0)
        targets = dict(zip(SCOUT_DRIVE.wheels, SCOUT_DRIVE.targets(*limiter.step(1, .1, .1))))
        for name in ('front_left_wheel', 'rear_left_wheel'):
            self.assertAlmostEqual(targets[name], -1.225)
        for name in ('front_right_wheel', 'rear_right_wheel'):
            self.assertAlmostEqual(targets[name], 1.225)

    def test_encoder_averages_each_side_without_doubling_distance(self):
        self.assertEqual(SCOUT_DRIVE.encoder_positions((1, 3, 5, 7)), (2, 6))
        self.assertEqual(AMR_DRIVE.encoder_positions((1, 3)), (1, 3))
        self.assertEqual(AMR_DRIVE.targets(2, 4), (2, 4))
        with self.assertRaises(ValueError):
            SCOUT_DRIVE.encoder_positions((1, 2))

    def test_watchdog_stops_all_wheels(self):
        limiter = DriveLimiter(radius=.08, track=.49)
        limiter.command(.4, .1, 0, 0)
        limiter.step(1, .1, .1)
        self.assertEqual(SCOUT_DRIVE.targets(*limiter.step(1, 1, 1)), (0, 0, 0, 0))


class ScoutSelectionTests(unittest.TestCase):
    def test_selected_robot_uses_its_own_config(self):
        for name, config, radius in (('scout', 'scout.json', .08), ('amr', 'robot.json', .1),
                                     ('locomanipulator', 'robot.json', .1)):
            with self.subTest(robot=name), patch.object(sys, 'argv', ['simulation.py', '--robot', name]):
                args, cfg = arguments()
                self.assertEqual(args.config.name, config)
                self.assertEqual(cfg['drive']['radius_m'], radius)

    def test_scout_retains_original_sensor_layout(self):
        original = json.loads((PROJECT / 'config/scout.json').read_text())
        cfg = runtime_config(original)
        self.assertEqual(cfg['simulation']['physics_hz'], 240)
        self.assertEqual(cfg['imu']['hz'], 200)
        self.assertNotIn('drive', original)
        specs = camera_specs('/World/Robot/base_link', cfg)
        self.assertEqual(len(specs), 6)
        for direction in ('front', 'left', 'right'):
            for channel in ('color', 'depth'):
                spec = specs[f'camera_{direction}/{channel}']
                self.assertEqual(spec['frame'], f'camera_{direction}_{channel}_optical_frame')
                self.assertEqual(spec['topic'], f'/camera_{direction}')
                self.assertEqual(spec['channels'], (channel,))


class CameraSampleTests(unittest.TestCase):
    def frame(self) -> dict[str, object]:
        return {'rendering_time': .5, 'rendering_frame': {
                    'referenceTimeNumerator': 1, 'referenceTimeDenominator': 2},
                'rgb': np.ones((2, 3, 4), dtype=np.uint8),
                'distance_to_image_plane': np.full((2, 3), 2, dtype=np.float32)}

    def test_common_rgbd_contract_is_preserved(self):
        sample = camera_sample(self.frame(), 3, 2, .5, .01)
        assert sample is not None
        self.assertEqual(sample[1], .5)
        assert sample[2] is not None
        self.assertEqual(sample[2].shape, (2, 3, 4))
        self.assertTrue(np.all(sample[3] == 2))

    def test_separate_scout_render_products(self):
        for channel, unused, index in (('color', 'distance_to_image_plane', 3), ('depth', 'rgb', 2)):
            frame = self.frame()
            del frame[unused]
            sample = camera_sample(frame, 3, 2, .5, .01, (channel,))
            assert sample is not None
            self.assertIsNone(sample[index])
            with self.assertRaises(ValueError):
                camera_sample(frame, 3, 2, .5, .01)

    def test_empty_buffer_waits_but_invalid_stamp_fails(self):
        frame = self.frame()
        frame['rgb'] = None
        self.assertIsNone(camera_sample(frame, 3, 2, .5, .01))
        frame = self.frame()
        frame['rendering_time'] = 1.0
        with self.assertRaises(ValueError):
            camera_sample(frame, 3, 2, .5, .01)


if __name__ == '__main__':
    unittest.main()
