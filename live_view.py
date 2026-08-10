"""Arduino App Lab WebUI camera feed and robot status bridge."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from typing import Any


class LiveCameraView:
    """Publish shared camera frames and task state through App Lab's WebUI brick."""

    def __init__(self, camera: Any, fps: float = 3.0) -> None:
        try:
            from arduino.app_bricks.web_ui import WebUI
        except ImportError as exc:
            raise RuntimeError("Arduino WebUI brick is unavailable") from exc
        self.camera = camera
        self.frame_period = 1.0 / max(0.5, min(fps, 8.0))
        self.ui = WebUI()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
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
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._stream, name="live-camera", daemon=True)
        self._thread.start()
        print("[WEB] live camera view: http://arduinoq.local:7000 (USB fallback: http://127.0.0.1:7000)")

    def publish_status(self, state: str, detail: str = "") -> None:
        try:
            self.ui.send_message("robot_status", message={"state": state, "detail": detail})
        except Exception as exc:
            if str(exc) != self._last_error:
                print(f"[WEB] status update failed: {exc}")
                self._last_error = str(exc)

    def _stream(self) -> None:
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

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
