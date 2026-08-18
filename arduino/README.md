# 🦾 Robot Arm Controller — OpenRB-150 로봇팔 제어

> **OpenRB-150 (ARM Cortex-M0+) + Dynamixel 서보 모터** 기반의 물류 파지 및 슬롯 적재(Pick-and-Place) 매니퓰레이터 시스템  
> Jetson에서 9바이트 UART로 물체 좌표와 빈 슬롯 번호를 수신하여, 실시간 역기구학(IK) 연산과 삼각 보간 좌표 보정을 거쳐 물체를 집어 지정된 슬롯에 안전하게 적재합니다.

---

## ⚡ 빠른 시작 (Setup & Upload)

1. **Arduino IDE 설정**:
   - 보드 매니저: `OpenRB-150` (ROBOTIS)
   - 라이브러리 매니저: `Dynamixel2Arduino` 설치
2. **포트 연결 & 업로드**:
   - 스케치 파일 열기: `arduino/openrb_robot_arm/openrb_robot_arm.ino`
   - 보드 선택: `OpenRB-150` ➔ 컴파일 및 업로드

---

## 🔄 제어 파이프라인

```text
[Jetson Orin Nano]
       │  9-Byte TARGET 패킷 (CMD_ID, Slot 1~9, X, Y, CRC8)
       ▼
[OpenRB-150 (Serial3)] ──> CRC8 검증 및 파싱 ──> 5-Byte ACK 응답
       │
       ▼
[좌표 보정 & IK]       ──> 8점 실측 삼각형 보간 ──> 목표 각도 계산
       │
       ▼
[물체 파지 (Pick)]      ──> Init/Home ➔ Approach ➔ Grip ➔ Lift
       │
       ▼
[슬롯 이동 (Move)]      ──> 관절공간 선형보간 (부드러운 가속도 프로파일)
       │
       ▼
[정밀 적재 (Place)]     ──> 슬롯 진입 (PLACE_SPEED) ➔ Settle 안정 대기 ➔ Release
       │
       ▼
[완료 보고]             ──> Home 복귀 ➔ Jetson으로 5-Byte DONE 패킷 전송
```

---

## 📡 통신 프로토콜 (UART 115200 8-N-1)

### 1. Jetson → OpenRB 명령 (9 Bytes)
```text
[AA] [55] [CMD_ID] [SLOT] [X_H] [X_L] [Y_H] [Y_L] [CRC8]
```
- `CMD_ID`: 명령 고유 식별 번호
- `SLOT`: 목표 적재함 슬롯 번호 (`1` ~ `9`)
- `X_H / X_L`, `Y_H / Y_L`: Big-endian 픽셀 좌표
- `CRC8`: Byte[2] ~ Byte[7] 6바이트 다항식 `0x07` 검증

### 2. OpenRB → Jetson 응답 (5 Bytes)
```text
[AA] [55] [CMD_ID] [STATUS] [CRC8]
```
- `STATUS`:
  - `0x06` : **ACK** (명령 수신 성공 및 동작 개시)
  - `0x07` : **DONE** (적재 완료 및 안전 홈 복귀 완료)
  - `0x15` : **NAK** (CRC8 오류 또는 비정상 패킷)
  - `0xE0` : **BUSY** (현재 이전 동작 수행 중)
  - `0xE1` : **FAULT** (모터 이상 또는 도달 실패)

---

## 🧩 핵심 기능 및 제어 알고리즘

| 기능 | 설명 |
|---|---|
| **삼각형 보간 좌표 보정** | 실측 8개 기준점 기반 삼각 보간을 적용하여 비전 좌표계 ➔ 로봇 물리 좌표계 오차 보정 |
| **Settle 연속 안정 판정** | `SETTLE_STABLE_COUNT` 연속 충족 시 통과하도록 하여 팔 끝의 관성/진동 감쇠 후 안전 파지 |
| **관절공간 선형보간** | 큰 각도 이동 시 각 축의 이동량을 스텝 단위로 쪼개어 이동하여 말단 튐 및 충돌 방지 |
| **저속 적재 프로파일** | 슬롯 진입 구간 전용 저속/저가속도 프로파일(`PLACE_SPEED`, `PLACE_ACCEL`) 적용 |
| **자동 복구 (RETRY)** | 그리퍼 해제 실패 또는 파지 실패 시 안전 HOME 복귀 후 RETRY 패킷 재전송 |

---

## 🔌 하드웨어 인터페이스

| 포트 | 연결 대상 | 통신 속도 / 핀 |
|---|---|---|
| `Serial` | PC USB 디버깅 | 115200 bps |
| `Serial1` | Dynamixel TTL 버스 | 1 Mbps (내장 자동 방향전환) |
| `Serial3` | Jetson USB-UART | 115200 bps (`D13=RX`, `D14=TX`) |
