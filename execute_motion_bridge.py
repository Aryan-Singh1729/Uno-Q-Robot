"""Arduino App Lab motion backend for the UNO Q MCU sketch."""

from __future__ import annotations

import json
import time
from typing import Any

from arduino.app_utils import Bridge
from robot_agent import MotionCall

POLL_INTERVAL_SECONDS = 0.05
BRIDGE_READY_TIMEOUT_SECONDS = 20.0
MOTION_TIMEOUT_GRACE_SECONDS = 2.0
TERMINAL_STATES = {"completed", "cancelled", "obstacle", "error", "idle"}


def _sensor_summary(status: dict[str, Any]) -> str:
    def value(key: str, unit: str) -> str:
        reading = status.get(key)
        return f"{reading}{unit}" if reading is not None else "unknown"

    return (
        f"ultrasonic={value('ultrasonic_cm', 'cm')}, "
        f"left_tof={value('tof_left_mm', 'mm')}, "
        f"right_tof={value('tof_right_mm', 'mm')}"
    )


def _decode_response(raw: Any, operation: str) -> dict[str, Any]:
    if isinstance(raw, dict):
        result = raw
    elif isinstance(raw, str):
        try:
            result = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"{operation} returned invalid JSON: {raw!r}") from exc
    else:
        raise RuntimeError(f"{operation} returned unsupported data: {raw!r}")
    if not isinstance(result, dict):
        raise RuntimeError(f"{operation} response must be an object")
    return result


def wait_for_mcu(timeout: float = BRIDGE_READY_TIMEOUT_SECONDS) -> dict[str, Any]:
    """Wait until the sketch has registered its App Lab Bridge functions."""
    deadline = time.monotonic() + timeout
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        try:
            status = _decode_response(Bridge.call("robot_status"), "robot_status")
            if status.get("ready") is True:
                if status.get("firmware_version") != "motor-map-v15.4":
                    last_error = RuntimeError(
                        "stale MCU sketch detected; App Lab did not flash motor-map-v15.4"
                    )
                    time.sleep(0.25)
                    continue
                print("[MOTOR] UNO Q MCU bridge is ready")
                if status.get("motor_test_mode") is True:
                    print("[MOTOR] MOTOR-ONLY TEST MODE: all sensor guards are disabled")
                missing = [
                    label
                    for key, label in (
                        ("tof_left_ready", "left ToF"),
                        ("tof_right_ready", "right ToF"),
                        ("mpu_ready", "MPU6050"),
                    )
                    if status.get(key) is False
                ]
                if missing:
                    print(f"[SENSOR] unavailable at startup: {', '.join(missing)}")
                return status
            last_error = RuntimeError(f"MCU is not ready: {status}")
        except Exception as exc:
            last_error = exc
        time.sleep(0.25)
    raise RuntimeError(
        "UNO Q motor sketch did not become ready in Arduino App Lab"
        + (f": {last_error}" if last_error else "")
    )


def _emergency_stop(reason: str) -> None:
    try:
        Bridge.call("stop_robot", reason)
    except Exception as exc:
        print(f"[MOTOR] emergency stop RPC failed: {exc}")


def execute_motion(
    call: MotionCall,
    stop_event: Any | None = None,
) -> str:
    """Start a validated MCU motion and poll its real completion state."""
    if stop_event is not None and stop_event.is_set():
        return json.dumps({"status": "cancelled", "reason": "explicit stop requested"})
    method = {
        "move": "move_robot",
        # The public turn tool is duration-based; reuse the MCU's timed spin RPC.
        "turn": "spin_robot",
        "spin": "spin_robot",
    }[call.name]
    speed = 50 if call.name == "turn" else call.speed
    try:
        started = _decode_response(
            Bridge.call(method, speed, call.amount),
            method,
        )
    except Exception as exc:
        _emergency_stop("bridge_error")
        return json.dumps({"status": "error", "reason": f"bridge call failed: {exc}"})

    if started.get("status") != "running":
        print(
            f"[MOTOR] {call.name} rejected: status={started.get('status')}, "
            f"reason={started.get('reason')}; "
            f"{_sensor_summary(started)}"
        )
        return json.dumps(started)

    motion_id = started.get("motion_id")
    duration_ms = started.get("duration_ms")
    if not isinstance(motion_id, int) or not isinstance(duration_ms, (int, float)):
        _emergency_stop("invalid_start_response")
        return json.dumps({"status": "error", "reason": "invalid MCU start response"})

    timeout = max(0.1, float(duration_ms) / 1000.0) + MOTION_TIMEOUT_GRACE_SECONDS
    deadline = time.monotonic() + timeout
    print(f"[MOTOR] {call.name} started (id={motion_id}, timeout={timeout:.2f}s)")

    while time.monotonic() < deadline:
        if stop_event is not None and stop_event.is_set():
            _emergency_stop("explicit_stop")
            return json.dumps(
                {"status": "cancelled", "reason": "explicit stop requested", **call.arguments()}
            )
        try:
            status = _decode_response(Bridge.call("robot_status"), "robot_status")
        except Exception as exc:
            _emergency_stop("bridge_error")
            return json.dumps({"status": "error", "reason": f"status polling failed: {exc}"})

        if status.get("motion_id") != motion_id:
            _emergency_stop("motion_id_mismatch")
            return json.dumps({"status": "error", "reason": "MCU motion state changed unexpectedly"})
        state = status.get("status")
        if state in TERMINAL_STATES:
            print(
                f"[MOTOR] motion {motion_id} finished: {state}; "
                f"reason={status.get('reason')}; "
                f"{_sensor_summary(status)}"
            )
            return json.dumps({**status, **call.arguments()})
        time.sleep(POLL_INTERVAL_SECONDS)

    _emergency_stop("python_timeout")
    return json.dumps({"status": "error", "reason": "MCU motion timed out", **call.arguments()})


def read_sensors() -> dict[str, Any]:
    try:
        return _decode_response(Bridge.call("read_sensors"), "read_sensors")
    except Exception as exc:
        print(f"[SENSOR] read_sensors failed: {exc}")
        return {"status": "error", "reason": str(exc)}


def stop_motors(reason: str = "python_shutdown") -> None:
    _emergency_stop(reason)
    print(f"[MOTOR] stop issued: {reason}")
