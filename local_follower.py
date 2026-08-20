"""Continuous local camera/LiDAR follower; no real-time LLM dependency."""

from __future__ import annotations

import json
import threading
import time
from collections.abc import Callable
from typing import Any

from robot_agent import ActionResult, MotionCall

FOLLOW_SPEED_PERCENT = 18
FOLLOW_STEP_SECONDS = 0.25
FOLLOW_TURN_SECONDS = 0.12
FOLLOW_LOOP_SECONDS = 0.15
FOLLOW_MIN_DISTANCE_MM = 700
FOLLOW_MAX_DISTANCE_MM = 1300
FOLLOW_CENTER_TOLERANCE = 0.10
TRACK_CONFIDENCE_MIN = 0.52
CAMERA_HORIZONTAL_FOV_DEG = 70.0


class LocalFollower:
    """Track one initialized target locally and delegate safe motion to navigation."""

    def __init__(
        self,
        camera: Any,
        lidar: Any,
        on_action: Callable[[MotionCall], ActionResult],
        on_event: Callable[[str, str], None],
        on_status: Callable[[str, str], None] | None = None,
    ) -> None:
        self.camera = camera
        self.lidar = lidar
        self.on_action = on_action
        self.on_event = on_event
        self.on_status = on_status
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._target = ""
        self._bbox: tuple[float, float, float, float] | None = None
        self._template: Any = None
        self._lock = threading.Lock()

    @property
    def active(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self, target: str, bbox: tuple[float, float, float, float]) -> None:
        self.stop("replaced")
        frame = self.camera.frame()
        template = self._crop(frame, bbox)
        if template is None:
            raise RuntimeError("follow target box is outside the camera frame")
        with self._lock:
            self._target = target
            self._bbox = bbox
            self._template = self._gray(template)
        stop_event = threading.Event()
        self._stop = stop_event
        self._thread = threading.Thread(
            target=self._run,
            args=(stop_event,),
            name="local-follower",
            daemon=True,
        )
        self._thread.start()
        self.on_event("FOLLOWER_STARTED", f"locally tracking {target}")

    def stop(self, reason: str = "requested") -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        was_active = thread is not None
        self._thread = None
        if was_active:
            self.on_event("FOLLOWER_STOPPED", reason)

    def _run(self, stop_event: threading.Event) -> None:
        lost_reported = False
        while not stop_event.is_set():
            started = time.monotonic()
            try:
                frame = self.camera.frame()
                tracked = self._track(frame)
                if tracked is None:
                    if not lost_reported:
                        self.on_event("FOLLOW_TARGET_LOST", "camera lost the selected target")
                        lost_reported = True
                    self._status("follower_waiting", "target temporarily out of view")
                    stop_event.wait(0.35)
                    continue
                lost_reported = False
                bbox, confidence = tracked
                center_x = bbox[0] + bbox[2] / 2.0
                bearing = (center_x - 0.5) * CAMERA_HORIZONTAL_FOV_DEG
                target_mm = self.lidar.sector_clearance(bearing % 360.0, 8.0)
                detail = (
                    f"target={self._target}, bearing={bearing:.1f}deg, "
                    f"range={round(target_mm) if target_mm is not None else 'unknown'}mm, "
                    f"track={confidence:.2f}"
                )
                self._status("following", detail)

                if center_x < 0.5 - FOLLOW_CENTER_TOLERANCE:
                    result = self.on_action(MotionCall("spin", FOLLOW_SPEED_PERCENT, FOLLOW_TURN_SECONDS))
                elif center_x > 0.5 + FOLLOW_CENTER_TOLERANCE:
                    result = self.on_action(MotionCall("spin", FOLLOW_SPEED_PERCENT, -FOLLOW_TURN_SECONDS))
                elif target_mm is None:
                    self.on_event("PATH_BLOCKED", "no LiDAR range aligned with the camera target")
                    stop_event.wait(0.25)
                    continue
                elif target_mm > FOLLOW_MAX_DISTANCE_MM:
                    result = self.on_action(MotionCall("move", FOLLOW_SPEED_PERCENT, FOLLOW_STEP_SECONDS))
                else:
                    if target_mm < FOLLOW_MIN_DISTANCE_MM:
                        self.on_event("WALL_TOO_CLOSE", "holding follower clearance")
                    stop_event.wait(0.25)
                    continue

                if not self._completed(result):
                    self.on_event("PATH_BLOCKED", "navigation stopped the follower motion")
                    stop_event.wait(0.35)
            except Exception as exc:
                self.on_event("FOLLOWER_ERROR", str(exc)[:180])
                self._status("follower_error", str(exc))
                stop_event.wait(0.75)
            stop_event.wait(max(0.0, FOLLOW_LOOP_SECONDS - (time.monotonic() - started)))

    def _track(self, frame: Any) -> tuple[tuple[float, float, float, float], float] | None:
        import cv2

        gray = self._gray(frame)
        with self._lock:
            template = self._template
        if template is None:
            return None
        best: tuple[float, tuple[int, int, int, int]] | None = None
        for scale in (0.82, 1.0, 1.18):
            width = max(12, int(template.shape[1] * scale))
            height = max(12, int(template.shape[0] * scale))
            if width >= gray.shape[1] or height >= gray.shape[0]:
                continue
            candidate = cv2.resize(template, (width, height))
            scores = cv2.matchTemplate(gray, candidate, cv2.TM_CCOEFF_NORMED)
            _, score, _, location = cv2.minMaxLoc(scores)
            if best is None or score > best[0]:
                best = (float(score), (location[0], location[1], width, height))
        if best is None or best[0] < TRACK_CONFIDENCE_MIN:
            return None
        score, (x, y, width, height) = best
        normalized = (
            x / frame.shape[1],
            y / frame.shape[0],
            width / frame.shape[1],
            height / frame.shape[0],
        )
        # Slow appearance adaptation preserves identity while handling scale/light changes.
        crop = frame[y : y + height, x : x + width]
        updated = cv2.resize(self._gray(crop), (template.shape[1], template.shape[0]))
        with self._lock:
            self._template = cv2.addWeighted(template, 0.90, updated, 0.10, 0)
            self._bbox = normalized
        return normalized, score

    @staticmethod
    def _gray(frame: Any) -> Any:
        import cv2

        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    @staticmethod
    def _crop(frame: Any, bbox: tuple[float, float, float, float]) -> Any | None:
        x, y, width, height = bbox
        x1 = max(0, min(frame.shape[1] - 1, round(x * frame.shape[1])))
        y1 = max(0, min(frame.shape[0] - 1, round(y * frame.shape[0])))
        x2 = max(x1 + 1, min(frame.shape[1], round((x + width) * frame.shape[1])))
        y2 = max(y1 + 1, min(frame.shape[0], round((y + height) * frame.shape[0])))
        crop = frame[y1:y2, x1:x2]
        return crop if crop.size and crop.shape[0] >= 12 and crop.shape[1] >= 12 else None

    def _status(self, state: str, detail: str) -> None:
        if self.on_status is not None:
            self.on_status(state, detail)

    @staticmethod
    def _completed(result: ActionResult) -> bool:
        if result.interruption is not None or not result.content:
            return False
        try:
            return json.loads(result.content).get("status") in {"completed", "simulated"}
        except (json.JSONDecodeError, AttributeError):
            return False
