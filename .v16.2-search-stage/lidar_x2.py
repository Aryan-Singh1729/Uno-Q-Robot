"""Minimal YDLIDAR X2 serial reader and forward-sector safety guard."""

from __future__ import annotations

import glob
import math
import threading
import time
from typing import Any


class YDLidarX2:
    BAUDRATE = 115_200
    HEADER = b"\xaa\x55"

    def __init__(
        self,
        port: str | None = None,
        *,
        front_angle_deg: float = 0.0,
        half_width_deg: float = 25.0,
        stop_distance_mm: int = 100,
        required: bool = True,
    ) -> None:
        self.requested_port = port.strip() if port else None
        self.port: str | None = None
        self.front_angle_deg = front_angle_deg % 360.0
        self.half_width_deg = max(5.0, min(90.0, half_width_deg))
        # X2's specified near limit and this build's fixed emergency boundary.
        # Never allow a configuration value to expand the requested 10 cm zone.
        self.stop_distance_mm = min(100, max(1, stop_distance_mm))
        self.required = required
        self.serial: Any = None
        self._serial_module: Any = None
        self.running = False
        self.thread: threading.Thread | None = None
        self.lock = threading.Lock()
        self.points: dict[int, tuple[float, float]] = {}
        self.last_packet_at = 0.0
        self.packet_count = 0
        self.emergency_latched = False
        self.emergency_distance_mm = 0
        self.last_error = "not started"

    @staticmethod
    def candidate_ports() -> list[str]:
        preferred = ["/dev/ydlidar"]
        discovered = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
        return preferred + [port for port in discovered if port not in preferred]

    def start(self) -> None:
        try:
            import serial
        except ImportError as exc:
            self.last_error = "pyserial is not installed"
            print(f"[LIDAR] unavailable: {self.last_error}")
            return

        self._serial_module = serial
        self.running = True
        self._open_serial()
        self.thread = threading.Thread(target=self._reader_loop, name="ydlidar-x2", daemon=True)
        self.thread.start()

    def _open_serial(self) -> bool:
        ports = [self.requested_port] if self.requested_port else self.candidate_ports()
        print(f"[LIDAR] candidate serial ports: {ports or 'none'}")
        for port in ports:
            if not port:
                continue
            try:
                connection = self._serial_module.Serial(port, self.BAUDRATE, timeout=0.2)
                connection.reset_input_buffer()
                # X2 adapters support motor control through DTR. Start with
                # asserted DTR and automatically toggle it if no stream arrives.
                try:
                    connection.dtr = True
                except Exception as exc:
                    print(f"[LIDAR] adapter DTR control unavailable on {port}: {exc}")
                self.serial = connection
                self.port = port
                self.last_error = "waiting for scan packets"
                print(f"[LIDAR] opened YDLIDAR X2 serial port {self.port} at {self.BAUDRATE} baud")
                return True
            except Exception as exc:
                self.last_error = f"cannot open {port}: {exc}"
        if self.serial is None:
            print(f"[LIDAR] unavailable: {self.last_error}")
        return False

    def _disconnect_serial(self) -> None:
        connection = self.serial
        self.serial = None
        self.port = None
        if connection is not None:
            try:
                connection.dtr = False
                connection.close()
            except Exception:
                pass

    def _reader_loop(self) -> None:
        buffer = bytearray()
        opened_at = time.monotonic()
        dtr_toggled = False
        while self.running:
            if self.serial is None:
                if self._open_serial():
                    buffer.clear()
                    opened_at = time.monotonic()
                    dtr_toggled = False
                else:
                    time.sleep(1.0)
                    continue
            try:
                chunk = self.serial.read(4096)
                if chunk:
                    buffer.extend(chunk)
                    self._consume(buffer)
                elif not dtr_toggled and time.monotonic() - opened_at >= 2:
                    dtr_toggled = True
                    try:
                        self.serial.dtr = not bool(self.serial.dtr)
                        print("[LIDAR] no packets yet; toggled adapter DTR motor control")
                    except Exception as exc:
                        print(f"[LIDAR] cannot toggle adapter DTR: {exc}")
            except Exception as exc:
                self.last_error = f"serial read failed: {exc}"
                print(f"[LIDAR] {self.last_error}")
                self._disconnect_serial()
                time.sleep(0.5)

    def _consume(self, buffer: bytearray) -> None:
        while True:
            header = buffer.find(self.HEADER)
            if header < 0:
                if len(buffer) > 1:
                    del buffer[:-1]
                return
            if header:
                del buffer[:header]
            if len(buffer) < 10:
                return
            sample_count = buffer[3]
            if sample_count < 1 or sample_count > 100:
                del buffer[0]
                continue
            packet_size = 10 + sample_count * 2
            if len(buffer) < packet_size:
                return
            packet = bytes(buffer[:packet_size])
            del buffer[:packet_size]
            points = self.decode_packet(packet)
            if not points:
                continue
            now = time.monotonic()
            with self.lock:
                for angle, distance in points:
                    self.points[round(angle) % 360] = (distance, now)
                self.last_packet_at = now
                self.packet_count += 1
                self.last_error = ""

    @classmethod
    def decode_packet(cls, packet: bytes) -> list[tuple[float, float]]:
        """Decode one official no-intensity triangle-LiDAR scan packet."""
        if len(packet) < 12 or packet[:2] != cls.HEADER:
            return []
        ct = packet[2]
        count = packet[3]
        if count < 1 or len(packet) != 10 + count * 2:
            return []
        fsa = int.from_bytes(packet[4:6], "little")
        lsa = int.from_bytes(packet[6:8], "little")
        expected = int.from_bytes(packet[8:10], "little")
        checksum = 0x55AA ^ fsa ^ ((count << 8) | ct) ^ lsa
        raw_distances: list[int] = []
        for offset in range(10, len(packet), 2):
            raw = int.from_bytes(packet[offset : offset + 2], "little")
            checksum ^= raw
            raw_distances.append(raw)
        if checksum & 0xFFFF != expected:
            return []

        start = (fsa >> 1) / 64.0
        end = (lsa >> 1) / 64.0
        difference = (end - start) % 360.0
        points: list[tuple[float, float]] = []
        for index, raw in enumerate(raw_distances):
            distance = raw / 4.0
            if distance <= 0:
                continue
            angle = start if count == 1 else start + difference * index / (count - 1)
            correction = math.degrees(
                math.atan(21.8 * (155.3 - distance) / (155.3 * distance))
            )
            points.append(((angle + correction) % 360.0, distance))
        return points

    def _front_readings(self) -> list[float]:
        now = time.monotonic()
        with self.lock:
            readings = [
                distance
                for angle, (distance, timestamp) in self.points.items()
                if now - timestamp <= 1.0
                and abs((angle - self.front_angle_deg + 180) % 360 - 180)
                <= self.half_width_deg
                and 0 < distance <= 8_000
            ]
        return sorted(readings)

    def _fresh_points(self) -> list[tuple[int, float]]:
        now = time.monotonic()
        with self.lock:
            return sorted(
                (angle, distance)
                for angle, (distance, timestamp) in self.points.items()
                if now - timestamp <= 1.0 and 0 < distance <= 8_000
            )

    def status(self) -> dict[str, Any]:
        readings = self._front_readings()
        points = self._fresh_points()
        fresh = time.monotonic() - self.last_packet_at <= 1.0
        front_mm = readings[0] if readings else None
        nearest_angle, nearest_mm = min(points, key=lambda item: item[1]) if points else (-1, None)
        emergency_active = nearest_mm is not None and nearest_mm <= self.stop_distance_mm
        return {
            "model": "YDLIDAR X2",
            "source": "Linux USB serial adapter",
            "required": self.required,
            "port": self.port,
            "connected": self.serial is not None and self.running,
            "scan_fresh": fresh,
            "front_distance_mm": round(front_mm) if front_mm is not None else None,
            "front_sample_count": len(readings),
            "nearest_distance_mm": round(nearest_mm) if nearest_mm is not None else 0,
            "nearest_angle_deg": nearest_angle,
            "stop_distance_mm": self.stop_distance_mm,
            "front_angle_deg": self.front_angle_deg,
            "front_half_width_deg": self.half_width_deg,
            "packet_count": self.packet_count,
            "emergency_latched": self.emergency_latched,
            "emergency_active": emergency_active,
            "emergency_distance_mm": self.emergency_distance_mm,
            "last_error": self.last_error or None,
        }

    def scan_status(self, include_points: bool = True) -> dict[str, Any]:
        status = self.status()
        if include_points:
            status["points"] = [
                [angle, round(distance)]
                for angle, distance in self._fresh_points()
                if angle % 2 == 0
            ]
        return status

    def guard_reason(self) -> str | None:
        status = self.status()
        if not status["connected"] or not status["scan_fresh"]:
            return "YDLIDAR X2 scan unavailable" if self.required else None
        points = self._fresh_points()
        distance = min((item[1] for item in points), default=0.0)
        if 0 < distance <= self.stop_distance_mm:
            self.emergency_latched = True
            self.emergency_distance_mm = round(distance)
            return f"YDLIDAR X2 emergency stop at {distance / 10:.1f} cm"
        return None

    def clear_emergency_latch(self) -> None:
        self.emergency_latched = False
        self.emergency_distance_mm = 0

    def close(self) -> None:
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1)
        self._disconnect_serial()
