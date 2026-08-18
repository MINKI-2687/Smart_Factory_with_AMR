#!/usr/bin/env python3
"""Receive the actual 8-byte Zybo shape/centroid UART packet.

Packet (Zybo -> Jetson)::

    [0] 0xAA       header
    [1] shape      0=NONE, 1=CIRCLE, 2=SQUARE, 3=TRIANGLE
    [2] cx high    centroid X high byte
    [3] cx low     centroid X low byte
    [4] cy high    centroid Y high byte
    [5] cy low     centroid Y low byte
    [6] reserved   currently 0
    [7] checksum   XOR of bytes [1:7]

Coordinates are unsigned 16-bit big-endian values.  For a NONE packet the
FPGA sends zero for both coordinates.  The current FPGA fills ``reserved``
with zero but it remains part of the XOR checksum.

Optional response (Jetson -> Zybo, standalone receiver에서는 ``--ack`` 사용)::

    0x06 ACK       valid packet
    0x15 NAK       invalid packet

This file can be run alone for UART bring-up, and its parser is also imported
by ``robot_dispatch_controller.py``.  The integrated controller always sends
ACK/NAK immediately so the FPGA handshake remains responsive while camera and
OpenRB work runs in a separate worker thread.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime
import sys

import serial

from zybo_slot_receiver import ShapeEventGate


HEADER = 0xAA
PACKET_SIZE = 8
MAX_CENTER_X = 1920
MAX_CENTER_Y = 1080
ACK_BYTE = 0x06
NAK_BYTE = 0x15
SHAPE_NAMES = {0: "NONE", 1: "CIRCLE", 2: "SQUARE", 3: "TRIANGLE"}
CHECKSUM_MODES = ("xor", "none")


@dataclass(frozen=True)
class TargetPacket:
    shape: int
    center_x: int
    center_y: int
    reserved: int
    checksum: int


def calculate_checksum(payload: bytes, mode: str = "xor") -> int:
    """Calculate the FPGA XOR checksum for bytes [shape..reserved]."""
    if mode == "xor":
        value = 0
        for byte in payload:
            value ^= byte
        return value
    if mode == "none":
        return 0
    raise ValueError(f"unsupported checksum mode: {mode}")


def validate_frame(frame: bytes, checksum_mode: str = "xor") -> str | None:
    """Return an error description, or ``None`` for a valid FPGA frame."""
    if len(frame) != PACKET_SIZE:
        return f"LENGTH:{len(frame)}"
    if frame[0] != HEADER:
        return f"HEADER:0x{frame[0]:02X}"
    if frame[1] not in SHAPE_NAMES:
        return f"SHAPE:{frame[1]}"
    if frame[6] != 0:
        return f"RESERVED:0x{frame[6]:02X}"

    center_x = (frame[2] << 8) | frame[3]
    center_y = (frame[4] << 8) | frame[5]
    if center_x > MAX_CENTER_X:
        return f"X_RANGE:{center_x}"
    if center_y > MAX_CENTER_Y:
        return f"Y_RANGE:{center_y}"
    if frame[1] == 0 and (center_x != 0 or center_y != 0):
        return f"NONE_COORDINATES:({center_x},{center_y})"

    if checksum_mode != "none":
        expected = calculate_checksum(frame[1:7], checksum_mode)
        if frame[7] != expected:
            return f"CHECKSUM:0x{frame[7]:02X}/0x{expected:02X}"
    return None


def parse_packet(frame: bytes, checksum_mode: str = "xor") -> TargetPacket | None:
    """Return a decoded packet, or ``None`` if any field is invalid."""
    if validate_frame(frame, checksum_mode) is not None:
        return None

    return TargetPacket(
        shape=frame[1],
        center_x=(frame[2] << 8) | frame[3],
        center_y=(frame[4] << 8) | frame[5],
        reserved=frame[6],
        checksum=frame[7],
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Receive Zybo 8-byte shape/coordinate packets over UART"
    )
    parser.add_argument("port", nargs="?", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--checksum",
        choices=CHECKSUM_MODES,
        default="xor",
        help="FPGA checksum rule (default: xor)",
    )
    parser.add_argument(
        "--raw",
        action="store_true",
        help="Print every UART read chunk before packet parsing",
    )
    parser.add_argument(
        "--ack",
        action="store_true",
        help="Valid packet에 ACK, invalid packet에 NAK 송신",
    )
    return parser.parse_args()


def format_raw(data: bytes) -> str:
    return data.hex(" ").upper()


def main() -> int:
    args = parse_args()
    if args.baud <= 0:
        print("[ERR] baud는 양수여야 합니다.", file=sys.stderr)
        return 2

    try:
        uart = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
            write_timeout=1.0,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
    except (ValueError, serial.SerialException, OSError) as exc:
        print(f"[ERR] UART 포트 열기 실패: {exc}", file=sys.stderr)
        return 2

    uart.reset_input_buffer()
    uart.reset_output_buffer()
    receive_buffer = bytearray()
    gate = ShapeEventGate(required_count=1)
    valid_count = 0
    invalid_count = 0
    ack_count = 0
    nak_count = 0

    print(f"[OK] {args.port} @ {args.baud} 8-N-1")
    print(
        "[FORMAT] AA SHAPE CX_H CX_L CY_H CY_L RESERVED CHECKSUM "
        f"(checksum={args.checksum})"
    )
    print(f"[INFO] ACK/NAK transmission: {'ON' if args.ack else 'OFF'}")
    print("[INFO] 8바이트 패킷 대기 중...\n")

    try:
        while True:
            chunk = uart.read(uart.in_waiting or 1)
            if not chunk:
                continue
            if args.raw:
                print(f"[RAW RX] {format_raw(chunk)}")
            receive_buffer.extend(chunk)

            while True:
                header_index = receive_buffer.find(bytes((HEADER,)))
                if header_index < 0:
                    receive_buffer.clear()
                    break
                if header_index > 0:
                    noise = bytes(receive_buffer[:header_index])
                    print(f"[DROP] header 이전 바이트: {format_raw(noise)}")
                    del receive_buffer[:header_index]
                if len(receive_buffer) < PACKET_SIZE:
                    break

                frame = bytes(receive_buffer[:PACKET_SIZE])
                packet = parse_packet(frame, args.checksum)
                if packet is None:
                    invalid_count += 1
                    gate.break_sequence()
                    reason = validate_frame(frame, args.checksum)
                    print(
                        f"[BAD] frame={format_raw(frame)} "
                        f"reason={reason}"
                    )
                    if args.ack:
                        uart.write(bytes((NAK_BYTE,)))
                        uart.flush()
                        nak_count += 1
                    # Drop only the current candidate header so a later 0xAA
                    # already in the buffer can be used for resynchronization.
                    del receive_buffer[0]
                    continue

                del receive_buffer[:PACKET_SIZE]
                valid_count += 1
                if args.ack:
                    uart.write(bytes((ACK_BYTE,)))
                    uart.flush()
                    ack_count += 1

                timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                action, event_value = gate.observe(packet.shape)
                packet_text = (
                    f"shape={SHAPE_NAMES[packet.shape]} "
                    f"centroid=({packet.center_x},{packet.center_y}) "
                    f"reserved=0x{packet.reserved:02X} "
                    f"raw={format_raw(frame)}"
                )
                if action == "idle":
                    print(f"[{timestamp}] [IDLE] {packet_text}")
                elif action == "event":
                    print(f"[{timestamp}] [EVENT] #{event_value} {packet_text}")
                elif action == "ignored":
                    print("[IGNORE] NONE 수신 전까지 추가 물체 이벤트를 차단합니다.")
                elif action == "rearmed":
                    print("[REARM] NONE 확인; 다음 물체를 받을 준비가 됐습니다.")

    except KeyboardInterrupt:
        print("\n[STOP] 사용자 종료")
    except (serial.SerialException, OSError) as exc:
        print(f"\n[ERR] UART 오류: {exc}", file=sys.stderr)
        return 2
    finally:
        uart.close()

    print(
        f"[END] valid={valid_count} invalid={invalid_count} "
        f"ACK={ack_count} NAK={nak_count} events={gate.event_count}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
