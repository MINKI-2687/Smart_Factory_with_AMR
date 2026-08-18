#!/usr/bin/env python3
"""Unit tests for the Jetson/OpenRB 9-byte target command protocol."""

import unittest

from openrb_target_transport import (
    OpenRbTargetTransport,
    build_target_command,
)
from openrb_transport import STATUS_ACK, STATUS_DONE, build_response


class OpenRbTargetProtocolTests(unittest.TestCase):
    def test_build_target_command(self):
        self.assertEqual(
            build_target_command(1, 3, 640, 360),
            bytes.fromhex("AA 55 01 03 02 80 01 68 A2"),
        )

    def test_rejects_invalid_fields(self):
        for values in (
            (256, 1, 0, 0),
            (1, 0, 0, 0),
            (1, 10, 0, 0),
            (1, 1, 1920, 0),
            (1, 1, 0, 1080),
        ):
            with self.subTest(values=values), self.assertRaises(ValueError):
                build_target_command(*values)

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
        transport = object.__new__(OpenRbTargetTransport)
        transport.uart = uart
        transport.ack_timeout = 0.1
        transport.done_timeout = 0.1
        transport.retries = 0
        transport.bytes_sent = 0
        transport.bytes_received = 0
        transport._receive_buffer = bytearray()

        result = transport.send_target_command(command_id, 3, 640, 360)

        self.assertTrue(result.success)
        self.assertEqual(result.reason, "DONE")
        self.assertEqual(transport.bytes_sent, 9)
        self.assertEqual(transport.bytes_received, 10)


if __name__ == "__main__":
    unittest.main()
