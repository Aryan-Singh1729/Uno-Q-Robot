"""Arduino App Lab WebUI camera, LiDAR, and robot status bridge."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from typing import Any


class LiveCameraView:
    """Publish camera, fused navigation telemetry, and task state through WebUI."""

    def __init__(
        self,
        camera: Any,
        lidar_reader: Callable[[bool], dict[str, Any]] | None = None,
        fps: float = 3.0,
        lidar_fps: float = 4.0,
    ) -> None:
        try:
            from arduino.app_bricks.web_ui import WebUI
        except ImportError as exc:
            raise RuntimeError("Arduino WebUI brick is unavailable") from exc
        self.camera = camera
        self.lidar_reader = lidar_reader
        self.frame_period = 1.0 / max(0.5, min(fps, 8.0))
        self.lidar_period = 1.0 / max(1.0, min(lidar_fps, 8.0))
        self.ui = WebUI()
        self._stop = threading.Event()
        self._camera_thread: threading.Thread | None = None
        self._lidar_thread: threading.Thread | None = None
        self._emergency_stop: Callable[[], None] | None = None
        self._last_error = ""
        self.ui.on_message("emergency_stop", self._on_emergency_stop)

    def set_emergency_stop(self, callback: Callable[[], None]) -> None:
        self._emergency_stop = callback

    def _on_emergency_stop(self, _sid: str, _payload: Any) -> None:
        if self._emergency_stop is not None:
            self._emergency_stop()
        self.publish_status("stopped", "Emergency stop requested from live view")

    def start(self) -> None:
        if self._camera_thread is not None and self._camera_thread.is_alive():
            return
        self._stop.clear()
        self._camera_thread = threading.Thread(
            target=self._stream_camera,
            name="live-camera",
            daemon=True,
        )
        self._camera_thread.start()
        if self.lidar_reader is not None:
            self._lidar_thread = threading.Thread(
                target=self._stream_lidar,
                name="live-lidar",
                daemon=True,
            )
            self._lidar_thread.start()
        print(
            "[WEB] live camera + LiDAR view: http://arduinoq.local:7000 "
            "(use the Network URL printed by WebUI if .local is unavailable)"
        )

    def publish_status(self, state: str, detail: str = "") -> None:
        try:
            self.ui.send_message("robot_status", message={"state": state, "detail": detail})
        except Exception as exc:
            if str(exc) != self._last_error:
                print(f"[WEB] status update failed: {exc}")
                self._last_error = str(exc)

    def _stream_camera(self) -> None:
        while not self._stop.is_set():
            started = time.monotonic()
            try:
                data_url = self.camera.data_url()
                _, payload = data_url.split(",", 1)
                self.ui.send_message(
                    "camera_frame",
                    message={"image": payload, "image_type": "image/jpeg"},
                )
                self._last_error = ""
            except Exception as exc:
                message = str(exc)
                if message != self._last_error:
                    print(f"[WEB] camera stream unavailable: {message}")
                    self._last_error = message
                self.publish_status("camera_error", message)
                self._stop.wait(1.0)
                continue
            self._stop.wait(max(0.0, self.frame_period - (time.monotonic() - started)))

    def _stream_lidar(self) -> None:
        last_error = ""
        while not self._stop.is_set():
            started = time.monotonic()
            try:
                assert self.lidar_reader is not None
                status = self.lidar_reader(True)
                self.ui.send_message("lidar_scan", message=status)
                error = str(status.get("last_error") or "")
                if error and error != last_error:
                    print(f"[WEB] LiDAR stream unavailable: {error}")
                last_error = error
            except Exception as exc:
                message = str(exc)
                if message != last_error:
                    print(f"[WEB] LiDAR stream failed: {message}")
                last_error = message
                self.ui.send_message(
                    "lidar_scan",
                    message={
                        "connected": False,
                        "scan_fresh": False,
                        "front_distance_mm": 0,
                        "nearest_distance_mm": 0,
                        "nearest_angle_deg": -1,
                        "stop_distance_mm": 100,
                        "emergency_latched": False,
                        "emergency_active": False,
                        "last_error": message,
                        "points": [],
                    },
                )
            self._stop.wait(max(0.0, self.lidar_period - (time.monotonic() - started)))

    def close(self) -> None:
        self._stop.set()
        for thread in (self._camera_thread, self._lidar_thread):
            if thread is not None:
                thread.join(timeout=2.0)
        self._camera_thread = None
        self._lidar_thread = None
