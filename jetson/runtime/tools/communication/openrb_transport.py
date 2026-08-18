#!/usr/bin/env python3
"""Reliable framed Jetson <-> OpenRB-150 slot-command transport.

Command (Jetson -> OpenRB), 5 bytes::

    AA 55 command_id slot crc8(command_id, slot)

Response (OpenRB -> Jetson), 5 bytes::

    AA 55 command_id status crc8(command_id, status)

Retries reuse the same command_id.  The OpenRB firmware must return ACK/DONE
again for a duplicate command without executing the robot motion twice.
"""

from __future__ import annotations

from dataclasses import dataclass
import time

import serial


SOF = bytes((0xAA, 0x55))
FRAME_SIZE = 5

STATUS_ACK = 0x06
STATUS_DONE = 0x07
STATUS_NAK = 0x15
STATUS_BUSY = 0xE0
STATUS_FAULT = 0xE1

STATUS_NAMES = {
    STATUS_ACK: "ACK",
    STATUS_DONE: "DONE",
    STATUS_NAK: "NAK",
    STATUS_BUSY: "BUSY",
    STATUS_FAULT: "FAULT",
}


def crc8(data: bytes) -> int:
    """CRC-8/ATM: polynomial 0x07, initial value 0x00."""
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def build_command(command_id: int, slot: int) -> bytes:
    if not 0 <= command_id <= 0xFF:
        raise ValueError("command_id must be 0..255")
    if not 1 <= slot <= 9:
        raise ValueError("slot must be 1..9")
    payload = bytes((command_id, slot))
    return SOF + payload + bytes((crc8(payload),))


def build_response(command_id: int, status: int) -> bytes:
    if not 0 <= command_id <= 0xFF:
        raise ValueError("command_id must be 0..255")
    if status not in STATUS_NAMES:
        raise ValueError(f"unsupported status: 0x{status:02X}")
    payload = bytes((command_id, status))
    return SOF + payload + bytes((crc8(payload),))


def parse_response(frame: bytes, expected_command_id: int) -> int:
    if len(frame) != FRAME_SIZE:
        raise ValueError(f"response length must be {FRAME_SIZE}")
    if frame[:2] != SOF:
        raise ValueError("invalid response header")
    if frame[2] != expected_command_id:
        raise ValueError(
            f"unexpected command_id={frame[2]}, expected={expected_command_id}"
        )
    expected_crc = crc8(frame[2:4])
    if frame[4] != expected_crc:
        raise ValueError(
            f"response CRC mismatch got=0x{frame[4]:02X} expected=0x{expected_crc:02X}"
        )
    if frame[3] not in STATUS_NAMES:
        raise ValueError(f"unknown response status=0x{frame[3]:02X}")
    return frame[3]


@dataclass(frozen=True)
class CommandResult:
    success: bool
    command_id: int
    slot: int
    reason: str
    attempts: int


class OpenRbTransport:
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
                if len(self._receive_buffer) < FRAME_SIZE:
                    break

                frame = bytes(self._receive_buffer[:FRAME_SIZE])
                del self._receive_buffer[:FRAME_SIZE]
                try:
                    return parse_response(frame, command_id)
                except ValueError as exc:
                    print(f"[OPENRB DROP] {exc} frame={frame.hex(' ').upper()}")

            chunk = self.uart.read(self.uart.in_waiting or 1)
            if not chunk:
                continue
            self.bytes_received += len(chunk)
            self._receive_buffer.extend(chunk)
        return None

    def send_slot_command(self, command_id: int, slot: int) -> CommandResult:
        frame = build_command(command_id, slot)
        total_attempts = self.retries + 1
        command_started = time.monotonic()

        for attempt in range(1, total_attempts + 1):
            self.uart.write(frame)
            self.uart.flush()
            self.bytes_sent += len(frame)
            print(
                f"[OPENRB TX] attempt={attempt}/{total_attempts} "
                f"command_id={command_id} slot={slot} "
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
                return CommandResult(True, command_id, slot, "DONE", attempt)
            if status != STATUS_ACK:
                return CommandResult(
                    False, command_id, slot, STATUS_NAMES[status], attempt
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
                return CommandResult(True, command_id, slot, "DONE", attempt)
            return CommandResult(False, command_id, slot, STATUS_NAMES[status], attempt)

        return CommandResult(
            False, command_id, slot, "RESPONSE_TIMEOUT", total_attempts
        )
