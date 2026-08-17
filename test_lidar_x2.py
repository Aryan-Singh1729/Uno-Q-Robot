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


if __name__ == "__main__":
    unittest.main()
