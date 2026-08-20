"""Deterministic autonomous navigation between high-level commands and motors."""

from __future__ import annotations

import json
import threading
from collections.abc import Callable
from typing import Any

from execute_motion_bridge import execute_motion, read_sensors, stop_motors
from lidar_x2 import YDLidarX2
from robot_agent import MotionCall

FORWARD_CHUNK_SECONDS = 0.35
LIDAR_AVOID_MM = 450
CORRIDOR_CLOSE_MM = 280
CORRIDOR_ERROR_MM = 140
RECOVERY_REVERSE_SECONDS = 0.25
RECOVERY_TURN_SECONDS = 0.35
MAX_RECOVERIES_PER_COMMAND = 5


class NavigationController:
    """Fuse sensors and execute local avoidance without consulting the LLM."""

    def __init__(self, lidar: YDLidarX2) -> None:
        self.lidar = lidar
        self._lock = threading.RLock()
        self._events: list[dict[str, str]] = []
        self._latest_event = {"code": "NAVIGATION_READY", "detail": "navigation online"}
        self._last_mcu_event_sequence = -1

    def _event(self, code: str, detail: str) -> None:
        event = {"code": code, "detail": detail}
        self._events.append(event)
        self._latest_event = event
        print(f"[NAV] {code}: {detail}")

    def emit_event(self, code: str, detail: str) -> None:
        """Accept a semantic event from another local navigation component."""
        with self._lock:
            self._event(code, detail)

    def _collect_mcu_event(self, status: dict[str, Any]) -> None:
        sequence = status.get("navigation_event_sequence")
        code = str(status.get("navigation_event") or "")
        if not code or sequence == self._last_mcu_event_sequence:
            return
        self._last_mcu_event_sequence = sequence
        safe_details = {
            "OBSTACLE_DETECTED": "a supplemental proximity sensor stopped unsafe motion",
            "TILT_WARNING": "excessive robot tilt was detected",
            "HEADING_CORRECTION": "correcting measured heading drift",
            "ROBOT_STUCK": "commanded movement was not measured",
            "NAVIGATION_READY": "navigation sensors are online",
        }
        self._event(code, safe_details.get(code, "navigation state changed"))

    @staticmethod
    def _public_result(result: dict[str, Any], events: list[dict[str, str]]) -> dict[str, Any]:
        """Build the LLM-facing result without continuous/raw sensor values."""
        hidden = {
            "navigation_event",
            "navigation_event_detail",
            "navigation_event_sequence",
            "ultrasonic_mm",
            "roll_deg",
            "pitch_deg",
            "yaw_deg",
            "sectors_mm",
        }
        public = {key: value for key, value in result.items() if key not in hidden}
        if public.get("status") == "obstacle":
            public["reason"] = "navigation safety override"
        public["navigation_events"] = list(events)
        return public

    @staticmethod
    def _clearance(value: Any) -> float:
        return float(value) if isinstance(value, (int, float)) and value > 0 else 8000.0

    def _guard_for(self, call: MotionCall) -> Callable[[], str | None]:
        return lambda: self.lidar.motion_guard_reason(call.name, call.amount)

    def _primitive(self, call: MotionCall, stop_event: Any | None) -> dict[str, Any]:
        raw = execute_motion(call, stop_event, self._guard_for(call))
        try:
            result = json.loads(raw)
        except (json.JSONDecodeError, TypeError):
            return {"status": "error", "reason": "invalid motor-controller response"}
        if isinstance(result, dict):
            self._collect_mcu_event(result)
            return result
        return {"status": "error", "reason": "invalid motor-controller response"}

    def _recovery_direction(self, sectors: dict[str, Any]) -> int:
        left = self._clearance(sectors.get("front_left")) + self._clearance(sectors.get("left"))
        right = self._clearance(sectors.get("front_right")) + self._clearance(sectors.get("right"))
        # Positive spin is the already verified semantic left direction.
        return 1 if left >= right else -1

    def _recover(
        self,
        sectors: dict[str, Any],
        mcu: dict[str, Any],
        speed: int,
        stop_event: Any | None,
    ) -> bool:
        direction = self._recovery_direction(sectors)
        self._event("OBSTACLE_AVOIDING", "backing up and turning toward the clearer side")
        rear = self._clearance(sectors.get("rear"))
        if rear > LIDAR_AVOID_MM:
            result = self._primitive(
                MotionCall("move", max(15, min(speed, 25)), -RECOVERY_REVERSE_SECONDS),
                stop_event,
            )
            if result.get("status") not in {"completed", "obstacle"}:
                return False
        result = self._primitive(
            MotionCall("spin", max(18, min(speed, 30)), direction * RECOVERY_TURN_SECONDS),
            stop_event,
        )
        return result.get("status") == "completed"

    def execute(self, call: MotionCall, stop_event: Any | None = None) -> str:
        """Execute a high-level motion with LiDAR-primary local navigation."""
        with self._lock:
            self._events = []
            if call.name != "move" or call.amount < 0:
                result = self._primitive(call, stop_event)
                return json.dumps(self._public_result(result, self._events))

            remaining = call.amount
            recoveries = 0
            while remaining > 0:
                if stop_event is not None and stop_event.is_set():
                    stop_motors("explicit_stop")
                    return json.dumps(
                        {
                            "status": "cancelled",
                            "reason": "explicit stop requested",
                            "navigation_events": list(self._events),
                        }
                    )

                snapshot = self.lidar.navigation_snapshot()
                if not snapshot.get("connected") or not snapshot.get("scan_fresh"):
                    self._event("PATH_BLOCKED", "primary LiDAR scan is unavailable")
                    stop_motors("lidar_unavailable")
                    return json.dumps(
                        {
                            "status": "obstacle",
                            "reason": "primary LiDAR scan unavailable",
                            "navigation_events": list(self._events),
                        }
                    )
                sectors = snapshot.get("sectors_mm") or {}
                mcu = read_sensors()
                self._collect_mcu_event(mcu)
                front = min(
                    self._clearance(sectors.get("front")),
                    self._clearance(sectors.get("front_left")),
                    self._clearance(sectors.get("front_right")),
                )

                if front <= LIDAR_AVOID_MM:
                    self._event("OBSTACLE_DETECTED", "LiDAR found the forward path blocked")
                    recoveries += 1
                    if recoveries > MAX_RECOVERIES_PER_COMMAND or not self._recover(
                        sectors, mcu, call.speed, stop_event
                    ):
                        self._event("ROBOT_STUCK", "local recovery could not find a clear route")
                        stop_motors("robot_stuck")
                        return json.dumps(
                            {
                                "status": "obstacle",
                                "reason": "local recovery failed",
                                "navigation_events": list(self._events),
                            }
                        )
                    continue

                left = self._clearance(sectors.get("left"))
                right = self._clearance(sectors.get("right"))
                if min(left, right) <= CORRIDOR_CLOSE_MM:
                    self._event("WALL_TOO_CLOSE", "adjusting away from the nearer wall")
                if abs(left - right) >= CORRIDOR_ERROR_MM and min(left, right) < 700:
                    correction = -0.10 if left < right else 0.10
                    self._event("HEADING_CORRECTION", "centering in the available corridor")
                    corrected = self._primitive(MotionCall("spin", 18, correction), stop_event)
                    if corrected.get("status") != "completed":
                        return json.dumps(self._public_result(corrected, self._events))

                step = min(FORWARD_CHUNK_SECONDS, remaining)
                result = self._primitive(MotionCall("move", call.speed, step), stop_event)
                state = result.get("status")
                if state == "completed":
                    remaining -= step
                    continue
                if state == "obstacle":
                    recoveries += 1
                    snapshot = self.lidar.navigation_snapshot()
                    mcu = read_sensors()
                    self._collect_mcu_event(mcu)
                    if recoveries <= MAX_RECOVERIES_PER_COMMAND and self._recover(
                        snapshot.get("sectors_mm") or {}, mcu, call.speed, stop_event
                    ):
                        continue
                    self._event("ROBOT_STUCK", "supplemental safety repeatedly stopped movement")
                return json.dumps(self._public_result(result, self._events))

            return json.dumps(
                {
                    "status": "completed",
                    "reason": "navigation goal duration reached",
                    **call.arguments(),
                    "navigation_events": list(self._events),
                }
            )

    def live_status(self, include_points: bool = True) -> dict[str, Any]:
        status = self.lidar.scan_status(include_points)
        try:
            supplemental = read_sensors()
            self._collect_mcu_event(supplemental)
        except Exception as exc:
            supplemental = {"status": "error", "reason": str(exc)}
        return {
            **status,
            "navigation_event": dict(self._latest_event),
            "supplemental": supplemental,
        }
