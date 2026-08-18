#!/usr/bin/env python3
"""Unit tests for the frame-to-event gate in zybo_slot_receiver.py."""

import unittest

from zybo_slot_receiver import CONFIRMATION_COUNT, ShapeEventGate, parse_packet


class ShapeEventGateTests(unittest.TestCase):
    def test_default_ack_transaction_mode_accepts_one_packet(self):
        self.assertEqual(CONFIRMATION_COUNT, 1)
        gate = ShapeEventGate()
        self.assertEqual(gate.observe(1), ("event", 1))
        self.assertEqual(gate.observe(1)[0], "ignored")
        self.assertEqual(gate.observe(0)[0], "rearmed")
        self.assertEqual(gate.observe(1), ("event", 2))

    def test_valid_packets(self):
        self.assertEqual(parse_packet(bytes.fromhex("AA 00 FF")), 0)
        self.assertEqual(parse_packet(bytes.fromhex("AA 01 FE")), 1)
        self.assertEqual(parse_packet(bytes.fromhex("AA 02 FD")), 2)
        self.assertEqual(parse_packet(bytes.fromhex("AA 03 FC")), 3)
        self.assertIsNone(parse_packet(bytes.fromhex("AA 01 FF")))

    def test_repeated_none_is_logged_once(self):
        gate = ShapeEventGate(required_count=3)
        actions = [gate.observe(0)[0] for _ in range(6)]
        self.assertEqual(
            actions,
            ["silent", "silent", "idle", "silent", "silent", "silent"],
        )

    def test_one_event_then_ignore_until_three_none(self):
        gate = ShapeEventGate(required_count=3)
        actions = [gate.observe(1)[0] for _ in range(6)]
        self.assertEqual(
            actions,
            ["candidate", "candidate", "event", "ignored", "silent", "silent"],
        )
        self.assertFalse(gate.armed)

        self.assertEqual(gate.observe(0)[0], "silent")
        self.assertEqual(gate.observe(0)[0], "silent")
        self.assertEqual(gate.observe(0)[0], "rearmed")
        self.assertTrue(gate.armed)

        self.assertEqual(gate.observe(2)[0], "candidate")
        self.assertEqual(gate.observe(2)[0], "candidate")
        self.assertEqual(gate.observe(2), ("event", 2))

    def test_shape_change_restarts_consecutive_count(self):
        gate = ShapeEventGate(required_count=3)
        self.assertEqual(gate.observe(1), ("candidate", 1))
        self.assertEqual(gate.observe(1), ("candidate", 2))
        self.assertEqual(gate.observe(3), ("candidate", 1))
        self.assertEqual(gate.observe(3), ("candidate", 2))
        self.assertEqual(gate.observe(3), ("event", 1))

    def test_bad_packet_breaks_consecutive_count(self):
        gate = ShapeEventGate(required_count=3)
        self.assertEqual(gate.observe(1), ("candidate", 1))
        self.assertEqual(gate.observe(1), ("candidate", 2))
        gate.break_sequence()
        self.assertEqual(gate.observe(1), ("candidate", 1))


if __name__ == "__main__":
    unittest.main()
