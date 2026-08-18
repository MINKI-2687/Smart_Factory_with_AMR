# #!/usr/bin/env python3
# """FPGA 8-byte target + live rack state -> OpenRB 9-byte controller."""

# from __future__ import annotations

# import argparse
# import json
# import sys
# import time
# from urllib.error import HTTPError, URLError
# from urllib.request import Request, urlopen

# import serial

# from openrb_target_transport import (
#     CAMERA_HEIGHT,
#     CAMERA_WIDTH,
#     OpenRbTargetTransport,
# )
# from slot_selector import select_slot
# from zybo_slot_receiver import ShapeEventGate
# from zybo_target_receiver_8byte import (
#     ACK_BYTE,
#     HEADER,
#     NAK_BYTE,
#     PACKET_SIZE,
#     SHAPE_NAMES,
#     parse_packet,
#     validate_frame,
# )


# DEFAULT_STATUS_URL = "http://127.0.0.1:8080/status.json"
# DEFAULT_MONITOR_EVENT_URL = "http://127.0.0.1:8080/controller-event"
# OPENRB_DEFAULT_BAUD = 115200
# FPGA_DEFAULT_BAUD = 115200
# SHAPE_KO = {1: "동그라미", 2: "네모", 3: "세모"}


# def parse_args() -> argparse.Namespace:
#     parser = argparse.ArgumentParser(
#         description=(
#             "Receive an FPGA 8-byte shape/centroid packet, select a "
#             "camera-verified slot, and require OpenRB ACK/DONE"
#         )
#     )
#     parser.add_argument("--fpga-port", default="/dev/ttyUSB0")
#     parser.add_argument("--baud", default=FPGA_DEFAULT_BAUD, type=int)
#     parser.add_argument("--camera-status-url", default=DEFAULT_STATUS_URL)
#     parser.add_argument(
#         "--monitor-event-url",
#         default=DEFAULT_MONITOR_EVENT_URL,
#         help="웹 UI에 컨트롤러 이벤트를 보낼 로컬 URL",
#     )
#     parser.add_argument("--http-timeout", default=1.0, type=float)

#     mode = parser.add_mutually_exclusive_group(required=True)
#     mode.add_argument(
#         "--dry-run",
#         action="store_true",
#         help="Select and print the slot without opening the OpenRB UART port.",
#     )
#     mode.add_argument(
#         "--openrb-port",
#         help="USB-UART connected to OpenRB Serial3, e.g. /dev/ttyUSB1",
#     )
#     parser.add_argument("--openrb-baud", default=OPENRB_DEFAULT_BAUD, type=int)
#     parser.add_argument("--openrb-ack-timeout", default=1.0, type=float)
#     parser.add_argument(
#         "--openrb-done-timeout",
#         default=60.0,
#         type=float,
#         help="ACK 후 OpenRB DONE을 기다릴 최대 시간(초, 기본 60)",
#     )
#     parser.add_argument("--openrb-retries", default=2, type=int)
#     return parser.parse_args()


# def fetch_camera_status(url: str, timeout: float) -> dict:
#     request = Request(url, headers={"Cache-Control": "no-store"})
#     with urlopen(request, timeout=timeout) as response:
#         if response.status != 200:
#             raise RuntimeError(f"camera HTTP status={response.status}")
#         document = json.load(response)
#     if not isinstance(document, dict):
#         raise ValueError("camera status JSON must be an object")
#     return document


# def camera_summary(document: dict) -> str:
#     pose = document.get("pose")
#     if isinstance(pose, dict):
#         pose_text = (
#             f"aligned={pose.get('aligned')} score={pose.get('score')} "
#             f"shift=({pose.get('shift_x')},{pose.get('shift_y')})"
#         )
#     else:
#         pose_text = "pose=None"
#     return (
#         f"status={document.get('status')} frame={document.get('frame')} "
#         f"{pose_text}"
#     )


# def publish_monitor_event(
#     url: str,
#     *,
#     kind: str,
#     tag: str,
#     message: str,
#     timeout: float,
# ) -> None:
#     body = json.dumps(
#         {"kind": kind, "tag": tag, "message": message},
#         ensure_ascii=False,
#     ).encode("utf-8")
#     request = Request(
#         url,
#         data=body,
#         headers={"Content-Type": "application/json; charset=utf-8"},
#         method="POST",
#     )
#     try:
#         with urlopen(request, timeout=timeout) as response:
#             if response.status != 204:
#                 raise RuntimeError(f"monitor event HTTP status={response.status}")
#     except (HTTPError, URLError, TimeoutError, OSError, RuntimeError) as exc:
#         # 웹 UI 로그 실패는 로봇 안전 판정에 영향을 주지 않는다.
#         print(f"[UI EVENT WARN] {exc}")


# def publish_column_full_event(
#     args: argparse.Namespace,
#     *,
#     shape: int,
#     column: str,
#     camera_status: dict,
# ) -> None:
#     stable_states = camera_status.get("stable_states")
#     rack_full = bool(
#         isinstance(stable_states, dict)
#         and len(stable_states) == 9
#         and all(state == "OCCUPIED" for state in stable_states.values())
#     )
#     shape_name = SHAPE_KO.get(shape, SHAPE_NAMES.get(shape, str(shape)))
#     if rack_full:
#         message = (
#             f"적재함 9칸이 모두 찼습니다. "
#             f"새 {shape_name} 물체는 적재하지 않았습니다."
#         )
#     else:
#         message = (
#             f"{shape_name} 적재 구역({column})이 모두 찼습니다. "
#             "로봇팔 명령을 보내지 않았습니다."
#         )
#     publish_monitor_event(
#         args.monitor_event_url,
#         kind="full",
#         tag="적재 차단",
#         message=message,
#         timeout=args.http_timeout,
#     )


# def process_shape_event(
#     shape: int,
#     target_x: int,
#     target_y: int,
#     event_number: int,
#     args: argparse.Namespace,
#     openrb: OpenRbTargetTransport | None,
#     command_id: int,
# ) -> tuple[bool, bool]:
#     """Return (accepted, OpenRB transmission attempted)."""
#     print(
#         f"[FPGA EVENT] #{event_number} shape={SHAPE_NAMES[shape]} "
#         f"centroid=({target_x},{target_y})"
#     )
#     if not 0 <= target_x < CAMERA_WIDTH or not 0 <= target_y < CAMERA_HEIGHT:
#         print(
#             f"[BLOCK] reason=FPGA_TARGET_OUT_OF_RANGE "
#             f"center=({target_x},{target_y}) expected="
#             f"X:0..{CAMERA_WIDTH - 1},Y:0..{CAMERA_HEIGHT - 1}"
#         )
#         return False, False
#     try:
#         camera_status = fetch_camera_status(
#             args.camera_status_url,
#             args.http_timeout,
#         )
#     except (HTTPError, URLError, TimeoutError, OSError, ValueError, RuntimeError) as exc:
#         print(f"[BLOCK] reason=CAMERA_STATUS_UNAVAILABLE detail={exc}")
#         return False, False

#     print(f"[CAMERA] {camera_summary(camera_status)}")
#     selection = select_slot(shape, camera_status)
#     if not selection.accepted:
#         print(
#             f"[BLOCK] shape={selection.shape_name} column={selection.column} "
#             f"reason={selection.reason}"
#         )
#         if selection.reason == "COLUMN_FULL":
#             publish_column_full_event(
#                 args,
#                 shape=shape,
#                 column=selection.column,
#                 camera_status=camera_status,
#             )
#         return False, False

#     slot = int(selection.slot_index)
#     print(
#         f"[SELECT] shape={selection.shape_name} column={selection.column} "
#         f"slot_name={selection.slot_name} slot={slot}"
#     )

#     print(f"[TARGET] object_center=({target_x},{target_y}) slot={slot}")
#     if openrb is None:
#         print(
#             f"[DRY-RUN] OpenRB 전송 생략: command_id={command_id} "
#             f"slot={slot} center=({target_x},{target_y})"
#         )
#         return True, False

#     try:
#         result = openrb.send_target_command(
#             command_id,
#             slot,
#             target_x,
#             target_y,
#         )
#     except (ValueError, serial.SerialException, OSError) as exc:
#         print(
#             f"[BLOCK] reason=OPENRB_UART_FAILED command_id={command_id} "
#             f"slot={slot} center=({target_x},{target_y}) detail={exc}"
#         )
#         return False, True

#     if result.success:
#         print(
#             f"[DISPATCH DONE] OpenRB-150 command_id={command_id} slot={slot} "
#             f"center=({target_x},{target_y}) attempts={result.attempts}"
#         )
#         return True, True
#     print(
#         f"[BLOCK] reason=OPENRB_{result.reason} command_id={command_id} "
#         f"slot={slot} center=({target_x},{target_y}) attempts={result.attempts}"
#     )
#     return False, True


# def open_fpga_uart(port: str, baudrate: int) -> serial.Serial:
#     return serial.Serial(
#         port=port,
#         baudrate=baudrate,
#         bytesize=serial.EIGHTBITS,
#         parity=serial.PARITY_NONE,
#         stopbits=serial.STOPBITS_ONE,
#         timeout=0.05,
#         write_timeout=1.0,
#         xonxoff=False,
#         rtscts=False,
#         dsrdtr=False,
#     )


# def main() -> int:
#     args = parse_args()
#     if args.openrb_port and args.openrb_port == args.fpga_port:
#         print(
#             "[ERR] FPGA와 OpenRB-150에 같은 UART 포트를 지정할 수 없습니다.",
#             file=sys.stderr,
#         )
#         return 2
#     if (
#         args.baud <= 0
#         or args.http_timeout <= 0
#         or args.openrb_baud <= 0
#         or args.openrb_ack_timeout <= 0
#         or args.openrb_done_timeout <= 0
#         or args.openrb_retries < 0
#     ):
#         print("[ERR] baud/timeout/retry 설정값을 확인하세요.", file=sys.stderr)
#         return 2

#     try:
#         fpga_uart = open_fpga_uart(args.fpga_port, args.baud)
#     except (ValueError, serial.SerialException, OSError) as exc:
#         print(f"[ERR] FPGA 포트 열기 실패: {exc}", file=sys.stderr)
#         return 2

#     openrb = None
#     if args.openrb_port:
#         try:
#             openrb = OpenRbTargetTransport(
#                 port=args.openrb_port,
#                 baudrate=args.openrb_baud,
#                 ack_timeout=args.openrb_ack_timeout,
#                 done_timeout=args.openrb_done_timeout,
#                 retries=args.openrb_retries,
#             )
#         except (ValueError, serial.SerialException, OSError) as exc:
#             fpga_uart.close()
#             print(f"[ERR] OpenRB-150 포트 열기 실패: {exc}", file=sys.stderr)
#             return 2

#     print(f"[OK] FPGA UART: {args.fpga_port} @ {args.baud} 8-N-1")
#     print(f"[OK] Camera status: {args.camera_status_url}")
#     if openrb is None:
#         print("[SAFETY] DRY-RUN: OpenRB-150 포트를 열지 않고 전송하지 않습니다.")
#     else:
#         print(
#             f"[OK] OpenRB-150 UART: {args.openrb_port} "
#             f"@ {args.openrb_baud} 8-N-1"
#         )
#         print(
#             "[SAFETY] 9바이트 좌표+슬롯 명령을 보내고 "
#             "5바이트 ACK와 DONE을 모두 확인합니다."
#         )
#         print(
#             "[INFO] OpenRB 연결: Jetson TX->D13(Serial3 RX), "
#             "Jetson RX<-D14(Serial3 TX), GND 공통"
#         )
#     print(
#         "[SAFETY] zybo_target_receiver_8byte.py 또는 다른 FPGA UART "
#         "수신기를 동시에 실행하지 마세요.\n"
#     )

#     try:
#         initial_status = fetch_camera_status(args.camera_status_url, args.http_timeout)
#         print(f"[CAMERA READY] {camera_summary(initial_status)}\n")
#     except (HTTPError, URLError, TimeoutError, OSError, ValueError, RuntimeError) as exc:
#         print(f"[WARN] 카메라 상태를 아직 읽을 수 없습니다: {exc}")
#         print("       shape 이벤트는 ACK하지만 슬롯 선택은 차단됩니다.\n")

#     fpga_uart.reset_input_buffer()
#     fpga_uart.reset_output_buffer()
#     gate = ShapeEventGate()
#     receive_buffer = bytearray()
#     valid_count = 0
#     ack_count = 0
#     error_count = 0
#     selected_count = 0
#     blocked_count = 0
#     command_id = int(time.time()) & 0xFF

#     try:
#         while True:
#             chunk = fpga_uart.read(fpga_uart.in_waiting or 1)
#             if not chunk:
#                 continue
#             receive_buffer.extend(chunk)

#             while True:
#                 header_index = receive_buffer.find(bytes([HEADER]))
#                 if header_index < 0:
#                     receive_buffer.clear()
#                     break
#                 if header_index > 0:
#                     del receive_buffer[:header_index]
#                 if len(receive_buffer) < PACKET_SIZE:
#                     break

#                 frame = bytes(receive_buffer[:PACKET_SIZE])
#                 packet = parse_packet(frame)
#                 if packet is None:
#                     error_count += 1
#                     gate.break_sequence()
#                     fpga_uart.write(bytes([NAK_BYTE]))
#                     fpga_uart.flush()
#                     print(
#                         f"[NAK] invalid_frame={frame.hex(' ').upper()} "
#                         f"reason={validate_frame(frame)}"
#                     )
#                     del receive_buffer[0]
#                     continue

#                 del receive_buffer[:PACKET_SIZE]
#                 valid_count += 1
#                 # FPGA ACK timeout(현재 150ms) 안에 먼저 응답한 뒤 카메라 조회와
#                 # OpenRB 동작을 수행한다.
#                 fpga_uart.write(bytes([ACK_BYTE]))
#                 fpga_uart.flush()
#                 ack_count += 1

#                 shape = packet.shape
#                 action, value = gate.observe(shape)
#                 if action == "idle":
#                     print("[IDLE] NONE 확인; 반복 NONE 로그를 생략합니다.")
#                 elif action == "event":
#                     accepted, attempted = process_shape_event(
#                         shape,
#                         packet.center_x,
#                         packet.center_y,
#                         int(value),
#                         args,
#                         openrb,
#                         command_id,
#                     )
#                     if attempted:
#                         command_id = (command_id + 1) & 0xFF
#                     if accepted:
#                         selected_count += 1
#                     else:
#                         blocked_count += 1
#                 elif action == "ignored":
#                     print("[IGNORE] NONE 확인 전까지 추가 non-NONE을 무시합니다.")
#                 elif action == "rearmed":
#                     print("[REARM] NONE 확인; 다음 물체를 받을 준비가 됐습니다.")
#                 elif action == "candidate":
#                     print(f"[CANDIDATE] shape={SHAPE_NAMES[shape]} count={value}")

#     except KeyboardInterrupt:
#         print("\n[STOP] 사용자 종료")
#     except (serial.SerialException, OSError) as exc:
#         print(f"\n[ERR] UART 오류: {exc}", file=sys.stderr)
#         return 2
#     finally:
#         fpga_uart.close()
#         if openrb is not None:
#             openrb.close()

#     openrb_tx = openrb.bytes_sent if openrb is not None else 0
#     openrb_rx = openrb.bytes_received if openrb is not None else 0
#     print(
#         f"[END] valid={valid_count} ACK={ack_count} invalid={error_count} "
#         f"events={gate.event_count} selected={selected_count} blocked={blocked_count} "
#         f"openrb_tx_bytes={openrb_tx} openrb_rx_bytes={openrb_rx}"
#     )
#     return 0


# if __name__ == "__main__":
#     raise SystemExit(main())

#!/usr/bin/env python3
"""FPGA 8-byte target + live rack state -> OpenRB 9-byte controller."""

from __future__ import annotations

import argparse
import json
import sys
import time
import threading
import queue
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import serial

from openrb_target_transport import (
    CAMERA_HEIGHT,
    CAMERA_WIDTH,
    OpenRbTargetTransport,
)
from slot_selector import select_slot
from zybo_slot_receiver import ShapeEventGate
from zybo_target_receiver_8byte import (
    ACK_BYTE,
    HEADER,
    NAK_BYTE,
    PACKET_SIZE,
    SHAPE_NAMES,
    parse_packet,
    validate_frame,
)


DEFAULT_STATUS_URL = "http://127.0.0.1:8080/status.json"
DEFAULT_MONITOR_EVENT_URL = "http://127.0.0.1:8080/controller-event"
OPENRB_DEFAULT_BAUD = 115200
FPGA_DEFAULT_BAUD = 115200
SHAPE_KO = {1: "동그라미", 2: "네모", 3: "세모"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Receive an FPGA 8-byte shape/centroid packet, select a "
            "camera-verified slot, and require OpenRB ACK/DONE"
        )
    )
    parser.add_argument("--fpga-port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", default=FPGA_DEFAULT_BAUD, type=int)
    parser.add_argument("--camera-status-url", default=DEFAULT_STATUS_URL)
    parser.add_argument(
        "--monitor-event-url",
        default=DEFAULT_MONITOR_EVENT_URL,
        help="웹 UI에 컨트롤러 이벤트를 보낼 로컬 URL",
    )
    parser.add_argument("--http-timeout", default=1.0, type=float)

    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument(
        "--dry-run",
        action="store_true",
        help="Select and print the slot without opening the OpenRB UART port.",
    )
    mode.add_argument(
        "--openrb-port",
        help="USB-UART connected to OpenRB Serial3, e.g. /dev/ttyUSB1",
    )
    parser.add_argument("--openrb-baud", default=OPENRB_DEFAULT_BAUD, type=int)
    parser.add_argument("--openrb-ack-timeout", default=1.0, type=float)
    parser.add_argument(
        "--openrb-done-timeout",
        default=60.0,
        type=float,
        help="ACK 후 OpenRB DONE을 기다릴 최대 시간(초, 기본 60)",
    )
    parser.add_argument("--openrb-retries", default=2, type=int)
    return parser.parse_args()


def fetch_camera_status(url: str, timeout: float) -> dict:
    request = Request(url, headers={"Cache-Control": "no-store"})
    with urlopen(request, timeout=timeout) as response:
        if response.status != 200:
            raise RuntimeError(f"camera HTTP status={response.status}")
        document = json.load(response)
    if not isinstance(document, dict):
        raise ValueError("camera status JSON must be an object")
    return document


def camera_summary(document: dict) -> str:
    pose = document.get("pose")
    if isinstance(pose, dict):
        pose_text = (
            f"aligned={pose.get('aligned')} score={pose.get('score')} "
            f"shift=({pose.get('shift_x')},{pose.get('shift_y')})"
        )
    else:
        pose_text = "pose=None"
    return (
        f"status={document.get('status')} frame={document.get('frame')} "
        f"{pose_text}"
    )


def publish_monitor_event(
    url: str,
    *,
    kind: str,
    tag: str,
    message: str,
    timeout: float,
) -> None:
    body = json.dumps(
        {"kind": kind, "tag": tag, "message": message},
        ensure_ascii=False,
    ).encode("utf-8")
    request = Request(
        url,
        data=body,
        headers={"Content-Type": "application/json; charset=utf-8"},
        method="POST",
    )
    try:
        with urlopen(request, timeout=timeout) as response:
            if response.status != 204:
                raise RuntimeError(f"monitor event HTTP status={response.status}")
    except (HTTPError, URLError, TimeoutError, OSError, RuntimeError) as exc:
        # 웹 UI 로그 실패는 로봇 안전 판정에 영향을 주지 않는다.
        print(f"[UI EVENT WARN] {exc}")


def publish_column_full_event(
    args: argparse.Namespace,
    *,
    shape: int,
    column: str,
    camera_status: dict,
) -> None:
    stable_states = camera_status.get("stable_states")
    rack_full = bool(
        isinstance(stable_states, dict)
        and len(stable_states) == 9
        and all(state == "OCCUPIED" for state in stable_states.values())
    )
    shape_name = SHAPE_KO.get(shape, SHAPE_NAMES.get(shape, str(shape)))
    if rack_full:
        message = (
            f"적재함 9칸이 모두 찼습니다. "
            f"새 {shape_name} 물체는 적재하지 않았습니다."
        )
    else:
        message = (
            f"{shape_name} 적재 구역({column})이 모두 찼습니다. "
            "로봇팔 명령을 보내지 않았습니다."
        )
    publish_monitor_event(
        args.monitor_event_url,
        kind="full",
        tag="적재 차단",
        message=message,
        timeout=args.http_timeout,
    )


def process_shape_event(
    shape: int,
    target_x: int,
    target_y: int,
    event_number: int,
    args: argparse.Namespace,
    openrb: OpenRbTargetTransport | None,
    command_id: int,
) -> tuple[bool, bool]:
    """Return (accepted, OpenRB transmission attempted)."""
    print(
        f"[FPGA EVENT] #{event_number} shape={SHAPE_NAMES[shape]} "
        f"centroid=({target_x},{target_y})"
    )
    if not 0 <= target_x < CAMERA_WIDTH or not 0 <= target_y < CAMERA_HEIGHT:
        print(
            f"[BLOCK] reason=FPGA_TARGET_OUT_OF_RANGE "
            f"center=({target_x},{target_y}) expected="
            f"X:0..{CAMERA_WIDTH - 1},Y:0..{CAMERA_HEIGHT - 1}"
        )
        return False, False
    try:
        camera_status = fetch_camera_status(
            args.camera_status_url,
            args.http_timeout,
        )
    except (HTTPError, URLError, TimeoutError, OSError, ValueError, RuntimeError) as exc:
        print(f"[BLOCK] reason=CAMERA_STATUS_UNAVAILABLE detail={exc}")
        return False, False

    print(f"[CAMERA] {camera_summary(camera_status)}")
    selection = select_slot(shape, camera_status)
    if not selection.accepted:
        print(
            f"[BLOCK] shape={selection.shape_name} column={selection.column} "
            f"reason={selection.reason}"
        )
        if selection.reason == "COLUMN_FULL":
            publish_column_full_event(
                args,
                shape=shape,
                column=selection.column,
                camera_status=camera_status,
            )
        return False, False

    slot = int(selection.slot_index)
    print(
        f"[SELECT] shape={selection.shape_name} column={selection.column} "
        f"slot_name={selection.slot_name} slot={slot}"
    )

    print(f"[TARGET] object_center=({target_x},{target_y}) slot={slot}")
    if openrb is None:
        print(
            f"[DRY-RUN] OpenRB 전송 생략: command_id={command_id} "
            f"slot={slot} center=({target_x},{target_y})"
        )
        return True, False

    try:
        result = openrb.send_target_command(
            command_id,
            slot,
            target_x,
            target_y,
        )
    except (ValueError, serial.SerialException, OSError) as exc:
        print(
            f"[BLOCK] reason=OPENRB_UART_FAILED command_id={command_id} "
            f"slot={slot} center=({target_x},{target_y}) detail={exc}"
        )
        return False, True

    if result.success:
        print(
            f"[DISPATCH DONE] OpenRB-150 command_id={command_id} slot={slot} "
            f"center=({target_x},{target_y}) attempts={result.attempts}"
        )
        return True, True
    print(
        f"[BLOCK] reason=OPENRB_{result.reason} command_id={command_id} "
        f"slot={slot} center=({target_x},{target_y}) attempts={result.attempts}"
    )
    return False, True


class ArmWorker:
    """카메라 조회 + OpenRB 전송을 메인 UART 수신 루프와 분리해서 처리한다.

    메인 스레드는 enqueue()만 호출하고 즉시 리턴한다. 실제 느린 작업
    (HTTP 조회, OpenRB ACK/DONE 대기, 최악의 경우 (ack_timeout+done_timeout)
    * (retries+1) 까지 블로킹)은 run()이 도는 별도 스레드에서 순차 처리된다.
    이렇게 해야 메인 루프가 언제나 fpga_uart.read()로 즉시 돌아갈 수 있고,
    Zybo의 150ms x 3회 ACK 타임아웃 안에 항상 응답할 수 있다.
    """

    def __init__(self, args: argparse.Namespace,
                openrb: "OpenRbTargetTransport | None") -> None:
        self.args = args
        self.openrb = openrb
        self.queue: "queue.Queue[tuple[int, int, int, int] | None]" = queue.Queue()
        self.command_id = int(time.time()) & 0xFF
        self.selected_count = 0
        self.blocked_count = 0
        self._lock = threading.Lock()
        self._thread = threading.Thread(
            target=self._run, name="arm-worker", daemon=True
        )

    def start(self) -> None:
        self._thread.start()

    def enqueue(self, shape: int, target_x: int, target_y: int,
               event_number: int) -> None:
        self.queue.put((shape, target_x, target_y, event_number))

    def stop_and_join(self, timeout: float = 5.0) -> None:
        self.queue.put(None)  # 종료 신호
        self._thread.join(timeout=timeout)

    def _run(self) -> None:
        while True:
            job = self.queue.get()
            if job is None:
                return
            shape, target_x, target_y, event_number = job

            with self._lock:
                command_id = self.command_id

            accepted, attempted = process_shape_event(
                shape,
                target_x,
                target_y,
                event_number,
                self.args,
                self.openrb,
                command_id,
            )

            with self._lock:
                if attempted:
                    self.command_id = (self.command_id + 1) & 0xFF
                if accepted:
                    self.selected_count += 1
                else:
                    self.blocked_count += 1


def open_fpga_uart(port: str, baudrate: int) -> serial.Serial:
    return serial.Serial(
        port=port,
        baudrate=baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.05,
        write_timeout=1.0,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
    )


def main() -> int:
    args = parse_args()
    if args.openrb_port and args.openrb_port == args.fpga_port:
        print(
            "[ERR] FPGA와 OpenRB-150에 같은 UART 포트를 지정할 수 없습니다.",
            file=sys.stderr,
        )
        return 2
    if (
        args.baud <= 0
        or args.http_timeout <= 0
        or args.openrb_baud <= 0
        or args.openrb_ack_timeout <= 0
        or args.openrb_done_timeout <= 0
        or args.openrb_retries < 0
    ):
        print("[ERR] baud/timeout/retry 설정값을 확인하세요.", file=sys.stderr)
        return 2

    try:
        fpga_uart = open_fpga_uart(args.fpga_port, args.baud)
    except (ValueError, serial.SerialException, OSError) as exc:
        print(f"[ERR] FPGA 포트 열기 실패: {exc}", file=sys.stderr)
        return 2

    openrb = None
    if args.openrb_port:
        try:
            openrb = OpenRbTargetTransport(
                port=args.openrb_port,
                baudrate=args.openrb_baud,
                ack_timeout=args.openrb_ack_timeout,
                done_timeout=args.openrb_done_timeout,
                retries=args.openrb_retries,
            )
        except (ValueError, serial.SerialException, OSError) as exc:
            fpga_uart.close()
            print(f"[ERR] OpenRB-150 포트 열기 실패: {exc}", file=sys.stderr)
            return 2

    worker = ArmWorker(args, openrb)
    worker.start()

    print(f"[OK] FPGA UART: {args.fpga_port} @ {args.baud} 8-N-1")
    print(f"[OK] Camera status: {args.camera_status_url}")
    if openrb is None:
        print("[SAFETY] DRY-RUN: OpenRB-150 포트를 열지 않고 전송하지 않습니다.")
    else:
        print(
            f"[OK] OpenRB-150 UART: {args.openrb_port} "
            f"@ {args.openrb_baud} 8-N-1"
        )
        print(
            "[SAFETY] 9바이트 좌표+슬롯 명령을 보내고 "
            "5바이트 ACK와 DONE을 모두 확인합니다."
        )
        print(
            "[INFO] OpenRB 연결: Jetson TX->D13(Serial3 RX), "
            "Jetson RX<-D14(Serial3 TX), GND 공통"
        )
    print(
        "[SAFETY] zybo_target_receiver_8byte.py 또는 다른 FPGA UART "
        "수신기를 동시에 실행하지 마세요.\n"
    )

    try:
        initial_status = fetch_camera_status(args.camera_status_url, args.http_timeout)
        print(f"[CAMERA READY] {camera_summary(initial_status)}\n")
    except (HTTPError, URLError, TimeoutError, OSError, ValueError, RuntimeError) as exc:
        print(f"[WARN] 카메라 상태를 아직 읽을 수 없습니다: {exc}")
        print("       shape 이벤트는 ACK하지만 슬롯 선택은 차단됩니다.\n")

    fpga_uart.reset_input_buffer()
    fpga_uart.reset_output_buffer()
    gate = ShapeEventGate()
    receive_buffer = bytearray()
    valid_count = 0
    ack_count = 0
    error_count = 0

    try:
        while True:
            chunk = fpga_uart.read(fpga_uart.in_waiting or 1)
            if not chunk:
                continue
            receive_buffer.extend(chunk)

            while True:
                header_index = receive_buffer.find(bytes([HEADER]))
                if header_index < 0:
                    receive_buffer.clear()
                    break
                if header_index > 0:
                    del receive_buffer[:header_index]
                if len(receive_buffer) < PACKET_SIZE:
                    break

                frame = bytes(receive_buffer[:PACKET_SIZE])
                packet = parse_packet(frame)
                if packet is None:
                    error_count += 1
                    gate.break_sequence()
                    fpga_uart.write(bytes([NAK_BYTE]))
                    fpga_uart.flush()
                    print(
                        f"[NAK] invalid_frame={frame.hex(' ').upper()} "
                        f"reason={validate_frame(frame)}"
                    )
                    del receive_buffer[0]
                    continue

                del receive_buffer[:PACKET_SIZE]
                valid_count += 1
                # FPGA ACK timeout(현재 150ms) 안에 먼저 응답한 뒤 카메라 조회와
                # OpenRB 동작을 수행한다.
                fpga_uart.write(bytes([ACK_BYTE]))
                fpga_uart.flush()
                ack_count += 1

                shape = packet.shape
                action, value = gate.observe(shape)
                if action == "idle":
                    print("[IDLE] NONE 확인; 반복 NONE 로그를 생략합니다.")
                elif action == "event":
                    # [PATCH] 느린 처리(카메라 조회 + OpenRB 전송)를 워커
                    # 스레드로 넘기고 즉시 리턴한다. 메인 루프는 곧바로
                    # fpga_uart.read()로 돌아가 다음 Zybo 패킷을 ACK할 수
                    # 있어야 하므로, 여기서 절대 블로킹하면 안 된다.
                    worker.enqueue(
                        shape, packet.center_x, packet.center_y, int(value)
                    )
                    print(
                        f"[QUEUED] event={value} shape={SHAPE_NAMES[shape]} "
                        f"center=({packet.center_x},{packet.center_y}) "
                        f"qsize={worker.queue.qsize()}"
                    )
                elif action == "ignored":
                    print("[IGNORE] NONE 확인 전까지 추가 non-NONE을 무시합니다.")
                elif action == "rearmed":
                    print("[REARM] NONE 확인; 다음 물체를 받을 준비가 됐습니다.")
                elif action == "candidate":
                    print(f"[CANDIDATE] shape={SHAPE_NAMES[shape]} count={value}")

    except KeyboardInterrupt:
        print("\n[STOP] 사용자 종료")
    except (serial.SerialException, OSError) as exc:
        print(f"\n[ERR] UART 오류: {exc}", file=sys.stderr)
        return 2
    finally:
        fpga_uart.close()
        worker.stop_and_join()
        if openrb is not None:
            openrb.close()

    openrb_tx = openrb.bytes_sent if openrb is not None else 0
    openrb_rx = openrb.bytes_received if openrb is not None else 0
    print(
        f"[END] valid={valid_count} ACK={ack_count} invalid={error_count} "
        f"events={gate.event_count} selected={worker.selected_count} "
        f"blocked={worker.blocked_count} "
        f"openrb_tx_bytes={openrb_tx} openrb_rx_bytes={openrb_rx}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())