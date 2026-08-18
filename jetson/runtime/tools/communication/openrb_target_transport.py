#!/usr/bin/env python3
"""Reliable Jetson -> OpenRB target-coordinate and rack-slot transport.

Command (Jetson -> OpenRB), 9 bytes::

    AA 55 command_id slot x_hi x_lo y_hi y_lo crc8(payload)

``payload`` is the six bytes from ``command_id`` through ``y_lo``.  X and Y
are unsigned 16-bit big-endian Pcam pixel coordinates.  The valid pixel
ranges are X=0..1919 and Y=0..1079.

Response (OpenRB -> Jetson), unchanged 5-byte protocol::

    AA 55 command_id status crc8(command_id, status)

Retries reuse the complete command, including the same command_id, slot and
coordinates.  OpenRB must not execute an identical retry twice.
"""

from __future__ import annotations

from dataclasses import dataclass
import time

import serial

from openrb_transport import (
    FRAME_SIZE as RESPONSE_SIZE,
    SOF,
    STATUS_ACK,
    STATUS_DONE,
    STATUS_NAMES,
    crc8,
    parse_response,
)


COMMAND_SIZE = 9
CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080


def build_target_command(
    command_id: int,
    slot: int,
    center_x: int,
    center_y: int,
) -> bytes:
    """Build one coordinate+slot command using the agreed 9-byte format."""
    if not 0 <= command_id <= 0xFF:
        raise ValueError("command_id must be 0..255")
    if not 1 <= slot <= 9:
        raise ValueError("slot must be 1..9")
    if not 0 <= center_x < CAMERA_WIDTH:
        raise ValueError(f"center_x must be 0..{CAMERA_WIDTH - 1}")
    if not 0 <= center_y < CAMERA_HEIGHT:
        raise ValueError(f"center_y must be 0..{CAMERA_HEIGHT - 1}")

    payload = bytes((command_id, slot))
    payload += center_x.to_bytes(2, byteorder="big")
    payload += center_y.to_bytes(2, byteorder="big")
    return SOF + payload + bytes((crc8(payload),))


@dataclass(frozen=True)
class TargetCommandResult:
    success: bool
    command_id: int
    slot: int
    center_x: int
    center_y: int
    reason: str
    attempts: int


class OpenRbTargetTransport:
    """Send one target command and require ACK followed by DONE."""

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        ack_timeout: float = 1.0,
        done_timeout: float = 60.0,
        retries: int = 2,
    ) -> None:
        if baudrate <= 0 or ack_timeout <= 0 or done_timeout <= 0 or retries < 0:
            raise ValueError("invalid OpenRB serial/timeout/retry setting")
        self.uart = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        self.port = port
        self.baudrate = baudrate
        self.ack_timeout = ack_timeout
        self.done_timeout = done_timeout
        self.retries = retries
        self.bytes_sent = 0
        self.bytes_received = 0
        self._receive_buffer = bytearray()
        self.uart.reset_input_buffer()
        self.uart.reset_output_buffer()

    def close(self) -> None:
        self.uart.close()

    def _read_response(self, command_id: int, timeout: float) -> int | None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            while True:
                header_index = self._receive_buffer.find(SOF)
                if header_index < 0:
                    self._receive_buffer[:] = (
                        self._receive_buffer[-1:]
                        if self._receive_buffer[-1:] == b"\xAA"
                        else b""
                    )
                    break
                if header_index > 0:
                    del self._receive_buffer[:header_index]
                if len(self._receive_buffer) < RESPONSE_SIZE:
                    break

                frame = bytes(self._receive_buffer[:RESPONSE_SIZE])
                del self._receive_buffer[:RESPONSE_SIZE]
                try:
                    return parse_response(frame, command_id)
                except ValueError as exc:
                    print(
                        f"[OPENRB DROP] {exc} "
                        f"frame={frame.hex(' ').upper()}"
                    )

            chunk = self.uart.read(self.uart.in_waiting or 1)
            if not chunk:
                continue
            self.bytes_received += len(chunk)
            self._receive_buffer.extend(chunk)
        return None

    def _result(
        self,
        success: bool,
        command_id: int,
        slot: int,
        center_x: int,
        center_y: int,
        reason: str,
        attempts: int,
    ) -> TargetCommandResult:
        return TargetCommandResult(
            success,
            command_id,
            slot,
            center_x,
            center_y,
            reason,
            attempts,
        )

    def send_target_command(
        self,
        command_id: int,
        slot: int,
        center_x: int,
        center_y: int,
    ) -> TargetCommandResult:
        frame = build_target_command(command_id, slot, center_x, center_y)
        total_attempts = self.retries + 1
        command_started = time.monotonic()

        for attempt in range(1, total_attempts + 1):
            self.uart.write(frame)
            self.uart.flush()
            self.bytes_sent += len(frame)
            print(
                f"[OPENRB TARGET TX] attempt={attempt}/{total_attempts} "
                f"command_id={command_id} slot={slot} "
                f"center=({center_x},{center_y}) "
                f"frame={frame.hex(' ').upper()}"
            )

            status = self._read_response(command_id, self.ack_timeout)
            if status is None:
                print(f"[OPENRB TIMEOUT] ACK command_id={command_id}")
                continue
            print(
                f"[OPENRB RX] command_id={command_id} "
                f"status={STATUS_NAMES[status]} "
                f"elapsed={time.monotonic() - command_started:.2f}s"
            )

            if status == STATUS_DONE:
                return self._result(
                    True, command_id, slot, center_x, center_y, "DONE", attempt
                )
            if status != STATUS_ACK:
                return self._result(
                    False,
                    command_id,
                    slot,
                    center_x,
                    center_y,
                    STATUS_NAMES[status],
                    attempt,
                )

            status = self._read_response(command_id, self.done_timeout)
            if status is None:
                print(f"[OPENRB TIMEOUT] DONE command_id={command_id}")
                continue
            print(
                f"[OPENRB RX] command_id={command_id} "
                f"status={STATUS_NAMES[status]} "
                f"elapsed={time.monotonic() - command_started:.2f}s"
            )
            if status == STATUS_DONE:
                return self._result(
                    True, command_id, slot, center_x, center_y, "DONE", attempt
                )
            return self._result(
                False,
                command_id,
                slot,
                center_x,
                center_y,
                STATUS_NAMES[status],
                attempt,
            )

        return self._result(
            False,
            command_id,
            slot,
            center_x,
            center_y,
            "RESPONSE_TIMEOUT",
            total_attempts,
        )
