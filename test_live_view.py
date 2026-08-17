import sys
import threading
import types
import unittest
from unittest.mock import patch

from live_view import LiveCameraView


class FakeWebUI:
    def __init__(self):
        self.handlers = {}
        self.messages = []
        self.frame_seen = threading.Event()
        self.lidar_seen = threading.Event()

    def on_message(self, name, callback):
        self.handlers[name] = callback

    def send_message(self, name, message):
        self.messages.append((name, message))
        if name == "camera_frame":
            self.frame_seen.set()
        if name == "lidar_scan":
            self.lidar_seen.set()


class FakeCamera:
    def data_url(self):
        return "data:image/jpeg;base64,anBlZw=="


class LiveCameraViewTests(unittest.TestCase):
    def make_view(self):
        arduino = types.ModuleType("arduino")
        arduino.__path__ = []
        app_bricks = types.ModuleType("arduino.app_bricks")
        app_bricks.__path__ = []
        web_ui = types.ModuleType("arduino.app_bricks.web_ui")
        web_ui.WebUI = FakeWebUI
        modules = {
            "arduino": arduino,
            "arduino.app_bricks": app_bricks,
            "arduino.app_bricks.web_ui": web_ui,
        }
        with patch.dict(sys.modules, modules):
            return LiveCameraView(
                FakeCamera(),
                lambda _include_points: {
                    "connected": True,
                    "scan_fresh": True,
                    "front_distance_mm": 95,
                    "nearest_distance_mm": 95,
                    "nearest_angle_deg": 0,
                    "stop_distance_mm": 100,
                    "emergency_active": True,
                    "points": [[0, 95]],
                },
                fps=8,
                lidar_fps=8,
            )

    def test_publishes_frames_and_emergency_stop(self):
        view = self.make_view()
        stopped = []
        view.set_emergency_stop(lambda: stopped.append(True))
        view.start()
        self.assertTrue(view.ui.frame_seen.wait(timeout=1))
        self.assertTrue(view.ui.lidar_seen.wait(timeout=1))
        view.ui.handlers["emergency_stop"]("client", {})
        view.close()
        self.assertEqual(stopped, [True])
        frame = next(message for name, message in view.ui.messages if name == "camera_frame")
        self.assertEqual(frame["image"], "anBlZw==")
        scan = next(message for name, message in view.ui.messages if name == "lidar_scan")
        self.assertEqual(scan["stop_distance_mm"], 100)
        self.assertIn(("robot_status", {"state": "stopped", "detail": "Emergency stop requested from live view"}), view.ui.messages)


if __name__ == "__main__":
    unittest.main()
