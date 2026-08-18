# ⚡ FPGA — 영상처리 파이프라인 & RC Car 모터 제어

> **Zybo Z7-20** 보드를 활용한 2가지 하드웨어 서브시스템

---

## 📂 서브모듈 안내

| 서브모듈 | 역할 | 주요 기술 | 상세 링크 |
|---|---|---|:---:|
| **`vision-pipeline/`** | Pcam 5C 기반 물체 인식 및 좌표 추적 | AXI4-Stream, HSV Classifier, Vitis PS (ARM) | [바로가기 ➔](vision-pipeline/README.md) |
| **`motor-control/`** | RC Car 모터 구동 및 엔코더 오도메트리 | PWM Controller, Quadrature Decoder, UART | 아래 내용 참조 |

---

## 🚗 Motor Control (`motor-control/`)

라즈베리파이 주행 제어기와 UART로 통신하며 양쪽 바퀴 모터를 구동하고 휠 엔코더를 계측합니다.

### 📁 디렉토리 구조
- `rtl/`: SystemVerilog RTL 소스 (최상위: `rc_car_top.sv`)
- `constraints/`: 보드 핀 제약 파일 (`rc_car.xdc`)
- `sim/`: ModelSim/Vivado 시뮬레이션용 테스트벤치 5종
- `docs/`: 하드웨어 블록 다이어그램

### 🧩 RTL 모듈 요약
| 모듈명 | 주요 기능 |
|---|---|
| `rc_car_top` | 모터 제어 탑 모듈 (UART, 모터, 엔코더 통합) |
| `UART` | 115200bps 8-N-1 시리얼 송수신기 |
| `cmd_rx` | 속도 및 방향 명령 패킷 디코더 |
| `motor_controller` | PWM 신호 생성 및 모터 드라이버 방향 제어 |
| `encoder_counter` | A/B 직교 위상 엔코더 펄스 카운터 |
| `encoder_tx` | 엔코더 펄스 데이터 UART 송신 인코더 |
| `watchdog` | 통신 단절 시 모터 자동 차단 안전 타이머 |
