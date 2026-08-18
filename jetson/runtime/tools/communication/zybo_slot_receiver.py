#!/usr/bin/env python3
"""
Zybo Z7-20 -> Jetson Orin Nano 도형 패킷 수신기 (3바이트 버전)
main_shape_ack.cc 와 짝을 이루는 스크립트입니다.

패킷 포맷 (Zybo -> Jetson, 3바이트)
    [0] 0xAA        헤더
    [1] shape       0=NONE 1=CIRCLE 2=SQUARE 3=TRIANGLE
    [2] ~shape      shape 의 비트 반전 (검증용)

    예:  NONE     -> AA 00 FF
         CIRCLE   -> AA 01 FE
         SQUARE   -> AA 02 FD
         TRIANGLE -> AA 03 FC

응답 (Jetson -> Zybo, 1바이트)
    0x06 (ACK)  검증 통과
    0x15 (NAK)  검증 실패

배선 (PL2303 USB-UART, 양방향 필수 + 공통 GND)
    Zybo Pmod JE UART0 TX  ->  PL2303 흰색 RXD
    Zybo Pmod JE UART0 RX  <-  PL2303 초록색 TXD
    Zybo GND               --- PL2303 검정색 GND
    PL2303 빨간색 +5V 선은 연결하지 않음

실행 후 아무 로그도 안 뜨면 배선/포트/보레이트를 의심하세요.
"""

import sys
import serial

# ---------------- 설정 ----------------
PORT     = '/dev/ttyUSB0'   # PL2303 USB-UART 장치
BAUD     = 115200
HEADER   = 0xAA
ACK_BYTE = 0x06
NAK_BYTE = 0x15
PKT_LEN  = 3
# FPGA가 동일 패킷을 ACK 수신 시까지 재전송하고, ACK 후에는 중단한다.
# 따라서 Jetson은 첫 정상 패킷을 즉시 확정해야 한다. 3으로 설정하면 FPGA가
# 첫 ACK에서 멈추므로 영원히 3회 확인에 도달하지 못한다.
CONFIRMATION_COUNT = 1

SHAPE_NAMES = {0: 'NONE', 1: 'CIRCLE', 2: 'SQUARE', 3: 'TRIANGLE'}

# 헤더가 아닌 바이트가 이만큼 연속으로 들어오면 경고를 띄웁니다.
# (배선/보레이트 문제로 쓰레기 값만 계속 들어오는 상황을 빨리 알아채기 위함)
STALE_WARN_THRESHOLD = 200


def parse_packet(buf):
    """검증 후 shape 반환. 검증 실패면 None."""
    shape = buf[1]
    if buf[2] != ((~shape) & 0xFF):
        return None
    if shape not in SHAPE_NAMES:
        return None
    return shape


class ShapeEventGate:
    """Convert repeated frame-level shapes into one object-level event.

    Every valid UART packet is still ACKed by ``main``.  This class only
    decides whether the packet should produce a log or one downstream event.
    After an event, every non-NONE shape is ignored until NONE is observed
    ``required_count`` times consecutively.  Ignoring every non-NONE shape,
    rather than only the previously accepted shape, prevents classification
    jitter for one physical object from creating a second robot command.
    """

    def __init__(self, required_count=CONFIRMATION_COUNT):
        if required_count < 1:
            raise ValueError("required_count must be positive")
        self.required_count = required_count
        self.armed = True
        self.candidate_shape = None
        self.candidate_count = 0
        self.none_count = 0
        self.idle_logged = False
        self.ignore_logged = False
        self.event_count = 0

    def break_sequence(self):
        """Break an in-progress consecutive sequence after a bad packet."""
        self.candidate_shape = None
        self.candidate_count = 0
        self.none_count = 0

    def observe(self, shape):
        """Return ``(action, value)`` for one validated shape packet.

        Actions are ``silent``, ``idle``, ``candidate``, ``event``,
        ``ignored`` and ``rearmed``.  ``value`` is the candidate count for a
        candidate, or the event number for an event.
        """
        if shape not in SHAPE_NAMES:
            raise ValueError(f"unsupported shape: {shape}")

        if self.armed:
            if shape == 0:
                self.candidate_shape = None
                self.candidate_count = 0
                self.none_count = min(self.none_count + 1, self.required_count)
                if self.none_count == self.required_count and not self.idle_logged:
                    self.idle_logged = True
                    return "idle", self.none_count
                return "silent", None

            self.none_count = 0
            self.idle_logged = False
            if shape == self.candidate_shape:
                self.candidate_count += 1
            else:
                self.candidate_shape = shape
                self.candidate_count = 1

            if self.candidate_count < self.required_count:
                return "candidate", self.candidate_count

            self.event_count += 1
            self.armed = False
            self.candidate_shape = None
            self.candidate_count = 0
            self.none_count = 0
            self.ignore_logged = False
            return "event", self.event_count

        if shape == 0:
            self.none_count = min(self.none_count + 1, self.required_count)
            if self.none_count == self.required_count:
                self.armed = True
                self.candidate_shape = None
                self.candidate_count = 0
                self.idle_logged = True
                self.ignore_logged = False
                return "rearmed", self.none_count
            return "silent", None

        self.none_count = 0
        if not self.ignore_logged:
            self.ignore_logged = True
            return "ignored", None
        return "silent", None


def handle_detection(shape):
    """안정화된 도형 이벤트를 W-280 슬롯 선택기로 연결할 자리입니다."""
    # FPGA가 안정화한 도형의 첫 정상 패킷에서 물체 한 개당 한 번 호출됩니다.
    # 다음 단계에서 select_slot(shape, stable_states)를 연결합니다.
    pass


def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.05)
    except serial.SerialException as e:
        print(f"[ERR] 포트 열기 실패: {e}")
        print("      권한 문제라면: sudo usermod -aG dialout $USER  (재로그인 필요)")
        sys.exit(1)

    print(f"[OK ] {PORT} @ {BAUD}bps 열림. 패킷 대기 중...")
    print("      FPGA ACK-재전송 방식: 첫 정상 도형 패킷을 즉시 이벤트로 인정합니다.")
    print("      이벤트 후 NONE 패킷을 받을 때까지 재동작을 차단합니다.\n")

    ser.reset_input_buffer()
    ser.reset_output_buffer()

    state = 'IDLE'
    buf = bytearray()
    rx_count = 0
    err_count = 0
    ack_count = 0
    stale_count = 0        # HEADER를 못 찾고 버려진 바이트 수 (진단용)
    stale_warned = False

    gate = ShapeEventGate()

    try:
        while True:
            data = ser.read(1)
            if not data:
                continue
            b = data[0]

            if state == 'IDLE':
                if b == HEADER:
                    buf = bytearray([b])
                    state = 'COLLECT'
                    stale_count = 0
                    stale_warned = False
                else:
                    stale_count += 1
                    if stale_count >= STALE_WARN_THRESHOLD and not stale_warned:
                        print(f"[WARN] 헤더(0x{HEADER:02X}) 없이 이상한 바이트만 "
                              f"{stale_count}개 이상 들어오고 있습니다. "
                              f"마지막 값: 0x{b:02X}")
                        print("       -> 배선(TX/RX 교차, GND 공통), 보레이트, "
                              "포트 번호를 확인하세요.")
                        stale_warned = True

            elif state == 'COLLECT':
                buf.append(b)
                if len(buf) < PKT_LEN:
                    continue

                shape = parse_packet(buf)

                if shape is not None:
                    rx_count += 1
                    ser.write(bytes([ACK_BYTE]))
                    ser.flush()          # OS 버퍼에 머무르지 않도록 즉시 송신
                    ack_count += 1

                    action, value = gate.observe(shape)
                    raw = ' '.join(f'{v:02X}' for v in buf)
                    if action == "idle":
                        print("[IDLE] NONE 확인 완료; 반복 NONE 로그를 생략합니다.")
                    elif action == "candidate":
                        print(f"[CANDIDATE] {SHAPE_NAMES[shape]} "
                              f"{value}/{CONFIRMATION_COUNT} raw={raw}")
                    elif action == "event":
                        print(f"[EVENT] #{value} {SHAPE_NAMES[shape]} "
                              f"확정 ({CONFIRMATION_COUNT}/{CONFIRMATION_COUNT}) raw={raw}")
                        handle_detection(shape)
                    elif action == "ignored":
                        print("[IGNORE] 명령 잠금 상태; NONE 확인 전까지 "
                              "non-NONE 패킷을 무시합니다.")
                    elif action == "rearmed":
                        print(f"[REARM] NONE {CONFIRMATION_COUNT}회 확인; "
                              "다음 물체를 받을 준비가 됐습니다.")

                else:
                    err_count += 1
                    gate.break_sequence()
                    expected = (~buf[1]) & 0xFF
                    print(f"[BAD] check byte: got 0x{buf[2]:02X}, expected 0x{expected:02X}  "
                          f"raw={' '.join(f'{v:02X}' for v in buf)}")
                    ser.write(bytes([NAK_BYTE]))
                    ser.flush()

                state = 'IDLE'

    except KeyboardInterrupt:
        print(f"\n[END] 정상 수신 {rx_count}건, ACK {ack_count}건, "
              f"이벤트 {gate.event_count}건, 오류 {err_count}건")
    finally:
        ser.close()


if __name__ == '__main__':
    main()
