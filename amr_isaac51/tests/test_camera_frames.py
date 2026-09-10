"""Regression coverage for the bundled Camera API's RGB and reference-time contract."""
from fractions import Fraction
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sim.camera_frames import camera_sample


class CameraFrameTests(unittest.TestCase):
    def frame(self) -> dict:
        return {"rgb": np.arange(48, dtype=np.uint8).reshape(3, 4, 4),
                "distance_to_image_plane": np.ones((3, 4), dtype=np.float32),
                "rendering_frame": {"referenceTimeNumerator": np.int64(30),
                                    "referenceTimeDenominator": np.int64(30)},
                "rendering_time": 0.5}

    def test_reads_rgb_and_reference_time_without_integer_casting_mapping(self):
        frame = self.frame()
        key, timestamp, rgba, depth = camera_sample(frame, 4, 3, 0.5, 1 / 120)
        self.assertEqual(key, Fraction(1))
        self.assertEqual(timestamp, 0.5)
        np.testing.assert_array_equal(rgba, frame["rgb"])
        np.testing.assert_array_equal(depth, frame["distance_to_image_plane"])

    def test_equivalent_reference_times_identify_same_frame(self):
        first = self.frame()
        second = self.frame()
        second["rendering_frame"] = {"referenceTimeNumerator": 120, "referenceTimeDenominator": 120}
        self.assertEqual(camera_sample(first, 4, 3, 1, .01)[0], camera_sample(second, 4, 3, 1, .01)[0])

    def test_empty_annotators_are_warmup(self):
        for data in (None, np.array([])):
            frame = self.frame()
            frame["rgb"] = data
            self.assertIsNone(camera_sample(frame, 4, 3, 1, .01))

    def test_incompatible_keys_fail_instead_of_silent_timeout(self):
        frame = self.frame()
        frame["rgba"] = frame.pop("rgb")
        with self.assertRaisesRegex(ValueError, "missing keys.*rgb"):
            camera_sample(frame, 4, 3, 1, .01)

    def test_nonempty_wrong_resolution_is_an_error(self):
        with self.assertRaisesRegex(ValueError, "shape mismatch"):
            camera_sample(self.frame(), 8, 6, 1, .01)

    def test_invalid_reference_or_simulation_time_is_rejected(self):
        frame = self.frame()
        frame["rendering_frame"]["referenceTimeDenominator"] = 0
        with self.assertRaisesRegex(ValueError, "ReferenceTime"):
            camera_sample(frame, 4, 3, 1, .01)
        for timestamp in (float("nan"), -1, 2):
            frame = self.frame()
            frame["rendering_time"] = timestamp
            with self.assertRaisesRegex(ValueError, "timestamp"):
                camera_sample(frame, 4, 3, 1, .01)


if __name__ == "__main__":
    unittest.main()
