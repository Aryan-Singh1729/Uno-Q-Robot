import json
import unittest

from local_follower import LocalFollower
from robot_agent import ActionResult


class LocalFollowerTests(unittest.TestCase):
    def test_completed_result_accepts_only_successful_local_motion(self):
        self.assertTrue(LocalFollower._completed(ActionResult(content=json.dumps({"status": "completed"}))))
        self.assertFalse(LocalFollower._completed(ActionResult(content=json.dumps({"status": "obstacle"}))))
        self.assertFalse(LocalFollower._completed(ActionResult(interruption=b"")))

    def test_crop_converts_normalized_box(self):
        try:
            import numpy as np
        except ImportError:
            self.skipTest("numpy unavailable")
        frame = np.zeros((100, 200, 3), dtype=np.uint8)
        crop = LocalFollower._crop(frame, (0.25, 0.20, 0.50, 0.60))
        self.assertEqual(crop.shape, (60, 100, 3))


if __name__ == "__main__":
    unittest.main()
