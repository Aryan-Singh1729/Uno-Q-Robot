import importlib.util
import json
import sys
import threading
import types
import unittest
from pathlib import Path

from robot_agent import MotionCall


def load_backend(fake_bridge):
    arduino = types.ModuleType("arduino")
    arduino.__path__ = []
    app_utils = types.ModuleType("arduino.app_utils")
    app_utils.Bridge = fake_bridge
    name = "execute_motion_bridge_under_test"
    path = Path(__file__).with_name("execute_motion_bridge.py")
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    old_arduino = sys.modules.get("arduino")
    old_utils = sys.modules.get("arduino.app_utils")
    try:
        sys.modules["arduino"] = arduino
        sys.modules["arduino.app_utils"] = app_utils
        assert spec.loader is not None
        spec.loader.exec_module(module)
    finally:
        if old_arduino is None:
            sys.modules.pop("arduino", None)
        else:
            sys.modules["arduino"] = old_arduino
        if old_utils is None:
            sys.modules.pop("arduino.app_utils", None)
        else:
            sys.modules["arduino.app_utils"] = old_utils
    return module


class FakeBridge:
    calls = []
    responses = {}

    @classmethod
    def reset(cls, responses):
        cls.calls = []
        cls.responses = {key: list(value) for key, value in responses.items()}

    @classmethod
    def call(cls, method, *args):
        cls.calls.append((method, args))
        response = cls.responses[method].pop(0)
        if isinstance(response, BaseException):
            raise response
        return response


class BridgeMotionTests(unittest.TestCase):
    def setUp(self):
        self.backend = load_backend(FakeBridge)

    def test_waits_for_real_mcu_completion(self):
        FakeBridge.reset(
            {
                "move_robot": [json.dumps({"motion_id": 7, "status": "running", "duration_ms": 50})],
                "robot_status": [json.dumps({"motion_id": 7, "status": "completed", "reason": "duration reached"})],
            }
        )
        result = json.loads(self.backend.execute_motion(MotionCall("move", 30, 2)))
        self.assertEqual(result["status"], "completed")
        self.assertEqual(result["duration_seconds"], 2)

    def test_obstacle_start_is_not_reported_as_completed(self):
        FakeBridge.reset(
            {
                "move_robot": [json.dumps({"motion_id": 2, "status": "obstacle", "reason": "path blocked"})]
            }
        )
        result = json.loads(self.backend.execute_motion(MotionCall("move", 20, 1)))
        self.assertEqual(result["status"], "obstacle")
        self.assertFalse(any(method == "robot_status" for method, _ in FakeBridge.calls))

    def test_speech_before_motion_never_calls_mcu(self):
        FakeBridge.reset({})
        stop = threading.Event()
        stop.set()
        result = json.loads(self.backend.execute_motion(MotionCall("turn", 40, 90), stop))
        self.assertEqual(result["status"], "cancelled")
        self.assertEqual(FakeBridge.calls, [])

    def test_motion_id_mismatch_issues_emergency_stop(self):
        FakeBridge.reset(
            {
                "spin_robot": [json.dumps({"motion_id": 3, "status": "running", "duration_ms": 50})],
                "robot_status": [json.dumps({"motion_id": 4, "status": "running", "reason": ""})],
                "stop_robot": ["{}"],
            }
        )
        result = json.loads(self.backend.execute_motion(MotionCall("turn", 40, -90)))
        self.assertEqual(result["status"], "error")
        self.assertIn(("stop_robot", ("motion_id_mismatch",)), FakeBridge.calls)

    def test_public_timed_turn_uses_fixed_speed_spin_rpc(self):
        FakeBridge.reset(
            {
                "spin_robot": [
                    json.dumps({"motion_id": 6, "status": "running", "duration_ms": 10})
                ],
                "robot_status": [
                    json.dumps({"motion_id": 6, "status": "completed", "reason": "duration reached"})
                ],
            }
        )
        result = json.loads(self.backend.execute_motion(MotionCall("turn", 12, -0.5)))
        self.assertEqual(result["status"], "completed")
        self.assertIn(("spin_robot", (50, -0.5)), FakeBridge.calls)

    def test_wait_for_mcu_requires_ready_flag(self):
        FakeBridge.reset(
            {
                "robot_status": [
                    json.dumps(
                        {
                            "ready": True,
                            "status": "idle",
                            "motion_id": 0,
                            "firmware_version": "lidar-guard-v16.0",
                            "motor_test_mode": False,
                            "sensor_guard_enabled": True,
                            "lidar_stop_distance_mm": 100,
                        }
                    )
                ]
            }
        )
        result = self.backend.wait_for_mcu(timeout=0.1)
        self.assertTrue(result["ready"])

    def test_lidar_preflight_blocks_motion_and_brakes(self):
        FakeBridge.reset(
            {
                "stop_robot": ["{}"],
            }
        )
        result = json.loads(
            self.backend.execute_motion(
                MotionCall("move", 20, 1),
                lidar_guard=lambda: "YDLIDAR X2 emergency stop at 9.5 cm",
            )
        )
        self.assertEqual(result["status"], "obstacle")
        self.assertIn(("stop_robot", ("lidar_preflight_stop",)), FakeBridge.calls)
        self.assertFalse(any(method == "move_robot" for method, _args in FakeBridge.calls))

    def test_lidar_guard_stops_active_motion_before_next_status_poll(self):
        reasons = iter((None, "YDLIDAR X2 emergency stop at 10.0 cm"))
        FakeBridge.reset(
            {
                "move_robot": [
                    json.dumps({"motion_id": 9, "status": "running", "duration_ms": 5000})
                ],
                "stop_robot": ["{}"],
            }
        )
        result = json.loads(
            self.backend.execute_motion(
                MotionCall("move", 20, 5),
                lidar_guard=lambda: next(reasons),
            )
        )
        self.assertEqual(result["status"], "obstacle")
        self.assertIn(("stop_robot", ("lidar_emergency_stop",)), FakeBridge.calls)
        self.assertFalse(any(method == "robot_status" for method, _args in FakeBridge.calls))

    def test_timed_spin_uses_dedicated_rpc(self):
        FakeBridge.reset(
            {
                "spin_robot": [
                    json.dumps({"motion_id": 8, "status": "running", "duration_ms": 10})
                ],
                "robot_status": [
                    json.dumps({"motion_id": 8, "status": "completed", "reason": "duration reached"})
                ],
            }
        )
        result = json.loads(self.backend.execute_motion(MotionCall("spin", 50, -10)))
        self.assertEqual(result["status"], "completed")
        self.assertIn(("spin_robot", (50, -10)), FakeBridge.calls)


if __name__ == "__main__":
    unittest.main()
