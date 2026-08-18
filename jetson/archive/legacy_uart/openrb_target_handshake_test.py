#!/usr/bin/env python3
"""Send one 9-byte coordinate+slot command and verify ACK then DONE."""

from __future__ import annotations

import argparse
import sys
import time

import serial

from openrb_target_transport import (
    CAMERA_HEIGHT,
    CAMERA_WIDTH,
    OpenRbTargetTransport,
)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Jetson -> OpenRB coordinate+slot UART handshake test"
    )
    parser.add_argument("port", help="OpenRB USB-UART port, e.g. /dev/ttyUSB1")
    parser.add_argument("--slot", type=int, required=True, help="rack slot 1..9")
    parser.add_argument(
        "--x", type=int, required=True, help=f"Pcam center X 0..{CAMERA_WIDTH - 1}"
    )
    parser.add_argument(
        "--y", type=int, required=True, help=f"Pcam center Y 0..{CAMERA_HEIGHT - 1}"
    )
    parser.add_argument("--command-id", type=int, help="optional command ID 0..255")
    parser.add_argument("--ack-timeout", type=float, default=1.0)
    parser.add_argument("--done-timeout", type=float, default=60.0)
    parser.add_argument("--retries", type=int, default=0)
    args = parser.parse_args()

    if not 1 <= args.slot <= 9:
        parser.error("--slot must be 1..9")
    if not 0 <= args.x < CAMERA_WIDTH:
        parser.error(f"--x must be 0..{CAMERA_WIDTH - 1}")
    if not 0 <= args.y < CAMERA_HEIGHT:
        parser.error(f"--y must be 0..{CAMERA_HEIGHT - 1}")
    if args.command_id is not None and not 0 <= args.command_id <= 255:
        parser.error("--command-id must be 0..255")

    command_id = (
        args.command_id if args.command_id is not None else int(time.time()) & 0xFF
    )
    try:
        transport = OpenRbTargetTransport(
            port=args.port,
            ack_timeout=args.ack_timeout,
            done_timeout=args.done_timeout,
            retries=args.retries,
        )
    except (ValueError, serial.SerialException, OSError) as exc:
        print(f"[FAIL] OpenRB 포트 열기 실패: {exc}", file=sys.stderr)
        return 2

    print(f"[OK] OpenRB target UART: {args.port} @ 115200 8-N-1")
    try:
        result = transport.send_target_command(
            command_id,
            args.slot,
            args.x,
            args.y,
        )
    except (ValueError, serial.SerialException, OSError) as exc:
        print(f"[FAIL] UART 오류: {exc}", file=sys.stderr)
        return 2
    finally:
        transport.close()

    if not result.success:
        print(
            f"[FAIL] command_id={command_id} slot={args.slot} "
            f"center=({args.x},{args.y}) reason={result.reason}"
        )
        return 1
    print(
        f"[PASS] ACK + DONE command_id={command_id} slot={args.slot} "
        f"center=({args.x},{args.y})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
