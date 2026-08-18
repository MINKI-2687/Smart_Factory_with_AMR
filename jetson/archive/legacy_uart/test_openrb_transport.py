#!/usr/bin/env python3
"""Unit tests for the Jetson/OpenRB-150 5-byte protocol."""

import unittest

from openrb_transport import (
    STATUS_ACK,
    STATUS_DONE,
    OpenRbTransport,
    build_command,
    build_response,
    crc8,
    parse_response,
)


class OpenRbProtocolTests(unittest.TestCase):
    def test_crc8_known_vector(self):
        self.assertEqual(crc8(bytes((1, 3))), 0x1C)

    def test_build_command(self):
        self.assertEqual(build_command(1, 3), bytes.fromhex("AA 55 01 03 1C"))

    def test_parse_ack(self):
        self.assertEqual(parse_response(build_response(7, STATUS_ACK), 7), STATUS_ACK)

    def test_parse_done(self):
        self.assertEqual(parse_response(build_response(7, STATUS_DONE), 7), STATUS_DONE)

    def test_bad_crc_is_rejected(self):
        frame = bytearray(build_response(7, STATUS_ACK))
        frame[-1] ^= 0x01
        with self.assertRaisesRegex(ValueError, "CRC mismatch"):
            parse_response(bytes(frame), 7)

    def test_invalid_slot_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "slot"):
            build_command(1, 10)

    def test_combined_ack_and_done_chunk_is_preserved(self):
        class FakeUart:
            def __init__(self, received):
                self.received = bytearray(received)
                self.written = bytearray()

            @property
            def in_waiting(self):
                return len(self.received)

            def read(self, size):
                data = bytes(self.received[:size])
                del self.received[:size]
                return data

            def write(self, data):
                self.written.extend(data)
                return len(data)

            def flush(self):
                pass

        command_id = 9
        uart = FakeUart(
            build_response(command_id, STATUS_ACK)
            + build_response(command_id, STATUS_DONE)
        )
        transport = object.__new__(OpenRbTransport)
        transport.uart = uart
        transport.ack_timeout = 0.1
        transport.done_timeout = 0.1
        transport.retries = 0
        transport.bytes_sent = 0
        transport.bytes_received = 0
        transport._receive_buffer = bytearray()

        result = transport.send_slot_command(command_id, 3)

        self.assertTrue(result.success)
        self.assertEqual(result.reason, "DONE")
        self.assertEqual(transport.bytes_sent, 5)
        self.assertEqual(transport.bytes_received, 10)


if __name__ == "__main__":
    unittest.main()
