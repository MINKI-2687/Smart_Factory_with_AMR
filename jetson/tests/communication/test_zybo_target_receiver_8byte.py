#!/usr/bin/env python3
"""Unit tests for the Zybo 8-byte shape/coordinate packet."""

import unittest

from zybo_target_receiver_8byte import (
    TargetPacket,
    calculate_checksum,
    parse_packet,
    validate_frame,
)


class ZyboTargetReceiver8ByteTests(unittest.TestCase):
    def test_fpga_xor_packet(self):
        # CIRCLE at centroid (640, 360), reserved=0.
        payload = bytes((1, 0x02, 0x80, 0x01, 0x68, 0x00))
        checksum = calculate_checksum(payload)
        frame = bytes((0xAA,)) + payload + bytes((checksum,))
        self.assertEqual(
            parse_packet(frame),
            TargetPacket(1, 640, 360, 0, checksum),
        )

    def test_xor_includes_reserved_byte(self):
        base = bytes((3, 0x04, 0x00, 0x02, 0x00, 0x00))
        changed_reserved = bytes((3, 0x04, 0x00, 0x02, 0x00, 0xA5))
        self.assertEqual(
            calculate_checksum(changed_reserved),
            calculate_checksum(base) ^ 0xA5,
        )

    def test_rejects_bad_checksum(self):
        self.assertIsNone(parse_packet(bytes.fromhex("AA 01 02 80 01 68 00 00")))

    def test_rejects_bad_header_and_shape(self):
        self.assertIsNone(parse_packet(bytes.fromhex("55 01 00 00 00 00 00 01")))
        self.assertIsNone(parse_packet(bytes.fromhex("AA 04 00 00 00 00 00 04")))

    def test_none_mode_skips_checksum_only(self):
        frame = bytes.fromhex("AA 02 01 02 03 04 00 FF")
        self.assertIsNotNone(parse_packet(frame, "none"))

    def test_accepts_documented_maximum_coordinates(self):
        payload = bytes((2, 0x07, 0x80, 0x04, 0x38, 0x00))
        frame = bytes((0xAA,)) + payload + bytes((calculate_checksum(payload),))
        packet = parse_packet(frame)
        self.assertIsNotNone(packet)
        self.assertEqual((packet.center_x, packet.center_y), (1920, 1080))

    def test_rejects_out_of_range_coordinates(self):
        payload = bytes((1, 0x07, 0x81, 0x00, 0x00, 0x00))
        frame = bytes((0xAA,)) + payload + bytes((calculate_checksum(payload),))
        self.assertEqual(validate_frame(frame), "X_RANGE:1921")

    def test_rejects_nonzero_reserved(self):
        payload = bytes((1, 0x00, 0x10, 0x00, 0x20, 0x01))
        frame = bytes((0xAA,)) + payload + bytes((calculate_checksum(payload),))
        self.assertEqual(validate_frame(frame), "RESERVED:0x01")

    def test_none_requires_zero_coordinates(self):
        payload = bytes((0, 0x00, 0x01, 0x00, 0x00, 0x00))
        frame = bytes((0xAA,)) + payload + bytes((calculate_checksum(payload),))
        self.assertEqual(validate_frame(frame), "NONE_COORDINATES:(1,0)")


if __name__ == "__main__":
    unittest.main()
