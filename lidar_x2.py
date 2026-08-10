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
        stop_distance_mm: int = 300,
        required: bool = True,
    ) -> None:
        self.requested_port = port.strip() if port else None
        self.port: str | None = None
        self.front_angle_deg = front_angle_deg % 360.0
        self.half_width_deg = max(5.0, min(90.0, half_width_deg))
        self.stop_distance_mm = max(100, stop_distance_mm)
        self.required = required
        self.serial: Any = None
        self.running = False
        self.thread: threading.Thread | None = None
        self.lock = threading.Lock()
        self.points: dict[int, tuple[float, float]] = {}
        self.last_packet_at = 0.0
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

        ports = [self.requested_port] if self.requested_port else self.candidate_ports()
        print(f"[LIDAR] candidate serial ports: {ports or 'none'}")
        for port in ports:
            if not port:
                continue
            try:
                connection = serial.Serial(port, self.BAUDRATE, timeout=0.2)
                connection.reset_input_buffer()
                # X2 adapters support motor control through DTR. Start with
                # asserted DTR and automatically toggle it if no stream arrives.
                connection.dtr = True
                self.serial = connection
                self.port = port
                self.last_error = "waiting for scan packets"
                break
            except Exception as exc:
                self.last_error = f"cannot open {port}: {exc}"
        if self.serial is None:
            print(f"[LIDAR] unavailable: {self.last_error}")
            return

        self.running = True
        self.thread = threading.Thread(target=self._reader_loop, name="ydlidar-x2", daemon=True)
        self.thread.start()
        print(f"[LIDAR] opened YDLIDAR X2 serial port {self.port} at {self.BAUDRATE} baud")

    def _reader_loop(self) -> None:
        buffer = bytearray()
        opened_at = time.monotonic()
        dtr_toggled = False
        while self.running:
            try:
                chunk = self.serial.read(4096)
                if chunk:
                    buffer.extend(chunk)
                    self._consume(buffer)
                elif not dtr_toggled and time.monotonic() - opened_at >= 2:
                    self.serial.dtr = not bool(self.serial.dtr)
                    dtr_toggled = True
                    print("[LIDAR] no packets yet; toggled adapter DTR motor control")
            except Exception as exc:
                self.last_error = f"serial read failed: {exc}"
                print(f"[LIDAR] {self.last_error}")
                self.running = False

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
                and 100 <= distance <= 8_000
            ]
        return sorted(readings)

    def status(self) -> dict[str, Any]:
        readings = self._front_readings()
        fresh = time.monotonic() - self.last_packet_at <= 1.0
        # Use the second-nearest point to reject a single isolated reflection.
        front_mm = readings[1] if len(readings) >= 2 else None
        return {
            "model": "YDLIDAR X2",
            "required": self.required,
            "port": self.port,
            "connected": self.serial is not None and self.running,
            "scan_fresh": fresh,
            "front_distance_mm": round(front_mm) if front_mm is not None else None,
            "front_sample_count": len(readings),
            "stop_distance_mm": self.stop_distance_mm,
            "front_angle_deg": self.front_angle_deg,
            "last_error": self.last_error or None,
        }

    def guard_reason(self) -> str | None:
        status = self.status()
        if not status["connected"] or not status["scan_fresh"]:
            return "YDLIDAR X2 scan unavailable" if self.required else None
        distance = status["front_distance_mm"]
        if isinstance(distance, int) and distance <= self.stop_distance_mm:
            return f"YDLIDAR X2 obstacle at {distance / 10:.1f} cm"
        return None

    def close(self) -> None:
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1)
        if self.serial is not None:
            try:
                self.serial.dtr = False
                self.serial.close()
            except Exception:
                pass
            self.serial = None

