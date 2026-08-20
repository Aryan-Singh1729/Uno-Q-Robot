import time
import unittest

from lidar_x2 import YDLidarX2


def make_packet(start_deg, end_deg, distances_mm, ct=0):
    count = len(distances_mm)
    fsa = (round(start_deg * 64) << 1) | 1
    lsa = (round(end_deg * 64) << 1) | 1
    raw = [round(distance * 4) for distance in distances_mm]
    checksum = 0x55AA ^ fsa ^ ((count << 8) | ct) ^ lsa
    for sample in raw:
        checksum ^= sample
    return b"".join(
        (
            b"\xaa\x55",
            bytes((ct, count)),
            fsa.to_bytes(2, "little"),
            lsa.to_bytes(2, "little"),
            (checksum & 0xFFFF).to_bytes(2, "little"),
            b"".join(sample.to_bytes(2, "little") for sample in raw),
        )
    )


class YDLidarX2Tests(unittest.TestCase):
    def test_official_triangle_packet_distance_and_angles_decode(self):
        points = YDLidarX2.decode_packet(make_packet(0, 10, [155, 155]))
        self.assertEqual([distance for _angle, distance in points], [155, 155])
        self.assertLess(points[0][0], 1)
        self.assertGreater(points[1][0], 9)

    def test_bad_checksum_is_rejected(self):
        packet = bytearray(make_packet(0, 5, [1000, 1200]))
        packet[-1] ^= 1
        self.assertEqual(YDLidarX2.decode_packet(bytes(packet)), [])

    def test_front_guard_uses_multiple_recent_points(self):
        lidar = YDLidarX2(required=True, stop_distance_mm=100)
        lidar.serial = object()
        lidar.running = True
        now = time.monotonic()
        lidar.last_packet_at = now
        lidar.points = {0: (95, now), 1: (240, now), 2: (260, now)}
        self.assertEqual(lidar.guard_reason(), "YDLIDAR X2 emergency stop at 9.5 cm")

    def test_stop_distance_cannot_expand_beyond_ten_centimetres(self):
        self.assertEqual(YDLidarX2(stop_distance_mm=300).stop_distance_mm, 100)

    def test_reading_just_over_ten_centimetres_does_not_stop(self):
        lidar = YDLidarX2(required=True)
        lidar.serial = object()
        lidar.running = True
        now = time.monotonic()
        lidar.last_packet_at = now
        lidar.points = {0: (100.4, now)}
        self.assertIsNone(lidar.guard_reason())

    def test_required_lidar_fails_closed_without_fresh_scan(self):
        lidar = YDLidarX2(required=True)
        self.assertEqual(lidar.guard_reason(), "YDLIDAR X2 scan unavailable")

    def test_navigation_snapshot_separates_left_front_and_right(self):
        lidar = YDLidarX2(required=True)
        lidar.serial = object()
        lidar.running = True
        now = time.monotonic()
        lidar.last_packet_at = now
        lidar.points = {
            0: (500, now),
            5: (520, now),
            90: (1400, now),
            270: (300, now),
            180: (1700, now),
        }
        sectors = lidar.navigation_snapshot()["sectors_mm"]
        self.assertEqual(sectors["front"], 500)
        self.assertEqual(sectors["left"], 300)
        self.assertEqual(sectors["right"], 1400)
        self.assertEqual(sectors["rear"], 1700)

    def test_escape_guard_checks_commanded_direction(self):
        lidar = YDLidarX2(required=True)
        lidar.serial = object()
        lidar.running = True
        now = time.monotonic()
        lidar.last_packet_at = now
        lidar.points = {0: (90, now), 180: (900, now)}
        self.assertIsNotNone(lidar.motion_guard_reason("move", 1.0))
        self.assertIsNone(lidar.motion_guard_reason("move", -1.0))


if __name__ == "__main__":
    unittest.main()
