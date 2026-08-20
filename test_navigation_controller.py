import importlib.util
import json
import sys
import types
import unittest
from pathlib import Path

from robot_agent import MotionCall


def load_controller(execute, read, stop):
    backend = types.ModuleType("execute_motion_bridge")
    backend.execute_motion = execute
    backend.read_sensors = read
    backend.stop_motors = stop
    name = "navigation_controller_under_test"
    path = Path(__file__).with_name("navigation_controller.py")
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    old = sys.modules.get("execute_motion_bridge")
    try:
        sys.modules["execute_motion_bridge"] = backend
        assert spec.loader is not None
        spec.loader.exec_module(module)
    finally:
        if old is None:
            sys.modules.pop("execute_motion_bridge", None)
        else:
            sys.modules["execute_motion_bridge"] = old
    return module.NavigationController


class FakeLidar:
    def __init__(self, snapshots):
        self.snapshots = list(snapshots)
        self.last = self.snapshots[-1]

    def navigation_snapshot(self):
        if self.snapshots:
            self.last = self.snapshots.pop(0)
        return self.last

    def motion_guard_reason(self, _name, _amount):
        return None

    def scan_status(self, include_points=True):
        return {"connected": True, "scan_fresh": True, "points": [] if include_points else None}


def snapshot(front=1000, left=1000, right=1000, rear=1000):
    return {
        "connected": True,
        "scan_fresh": True,
        "sectors_mm": {
            "front": front,
            "front_left": left,
            "left": left,
            "front_right": right,
            "right": right,
            "rear": rear,
        },
    }


class NavigationControllerTests(unittest.TestCase):
    def make_controller(self, scans):
        calls = []
        stops = []

        def execute(call, _stop_event, _guard):
            calls.append(call)
            return json.dumps({"status": "completed", "reason": "duration reached"})

        Controller = load_controller(
            execute,
            lambda: {
                "status": "ok",
                "ultrasonic_mm": 800,
                "navigation_event_sequence": 0,
            },
            stops.append,
        )
        return Controller(FakeLidar(scans)), calls, stops

    def test_clear_forward_command_is_chunked_and_completed(self):
        controller, calls, stops = self.make_controller([snapshot()] * 4)
        result = json.loads(controller.execute(MotionCall("move", 20, 0.8)))
        self.assertEqual(result["status"], "completed")
        self.assertEqual([round(call.amount, 2) for call in calls], [0.35, 0.35, 0.1])
        self.assertEqual(stops, [])
        self.assertEqual(result["navigation_events"], [])

    def test_blocked_path_recovers_toward_clearer_side_then_resumes(self):
        controller, calls, _stops = self.make_controller(
            [snapshot(front=350, left=1200, right=300, rear=900), snapshot()]
        )
        result = json.loads(controller.execute(MotionCall("move", 20, 0.2)))
        self.assertEqual(result["status"], "completed")
        self.assertEqual(calls[0], MotionCall("move", 20, -0.25))
        self.assertEqual(calls[1], MotionCall("spin", 20, 0.35))
        self.assertEqual(calls[2], MotionCall("move", 20, 0.2))
        codes = [event["code"] for event in result["navigation_events"]]
        self.assertEqual(codes[:2], ["OBSTACLE_DETECTED", "OBSTACLE_AVOIDING"])
        # The LLM-facing result contains semantic events, not raw sensor fields.
        self.assertNotIn("ultrasonic_mm", result)
        self.assertNotIn("sectors_mm", result)

    def test_missing_primary_lidar_fails_closed_before_motor_call(self):
        bad = {"connected": False, "scan_fresh": False, "sectors_mm": {}}
        controller, calls, stops = self.make_controller([bad])
        result = json.loads(controller.execute(MotionCall("move", 20, 1.0)))
        self.assertEqual(result["status"], "obstacle")
        self.assertEqual(calls, [])
        self.assertEqual(stops, ["lidar_unavailable"])
        self.assertEqual(result["navigation_events"][0]["code"], "PATH_BLOCKED")


if __name__ == "__main__":
    unittest.main()
