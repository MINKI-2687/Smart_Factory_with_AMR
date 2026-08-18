#!/usr/bin/env python3
"""Send one slot command to a NUCLEO-F411RE and verify ACK then DONE."""

from __future__ import annotations

import argparse
import sys
import time

import serial

from stm32_transport import Stm32Transport


def main() -> int:
    parser = argparse.ArgumentParser(description="STM32 UART LED handshake test")
    parser.add_argument("port", help="STM32 USB-UART port, e.g. /dev/ttyUSB1")
    parser.add_argument("--slot", type=int, default=1, choices=range(1, 10))
    parser.add_argument("--command-id", type=int, choices=range(0, 256))
    args = parser.parse_args()

    command_id = (
        args.command_id if args.command_id is not None else int(time.time()) & 0xFF
    )
    try:
        transport = Stm32Transport(port=args.port, retries=0)
    except (ValueError, serial.SerialException) as exc:
        print(f"[FAIL] STM32 포트 열기 실패: {exc}", file=sys.stderr)
        return 2

    print(f"[OK] STM32 UART: {args.port} @ 115200 8-N-1")
    try:
        result = transport.send_slot_command(command_id, args.slot)
    except serial.SerialException as exc:
        print(f"[FAIL] UART 오류: {exc}", file=sys.stderr)
        return 2
    finally:
        transport.close()

    if not result.success:
        print(
            f"[FAIL] command_id={command_id} slot={args.slot} "
            f"reason={result.reason}"
        )
        return 1
    print(f"[PASS] ACK + DONE command_id={command_id} slot={args.slot}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
