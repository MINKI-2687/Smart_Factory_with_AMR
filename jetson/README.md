# 🧠 Jetson — AI 비전 및 통합 디스패치 컨트롤러

> **Jetson Orin Nano** 기반의 적재함 상태 실시간 AI 추론 및 시스템 통합 디스패처  
> TensorRT 기반 9슬롯 모니터링 → FPGA 물체 좌표 수신 → OpenRB 로봇팔 명령 전송을 총괄합니다.

---

## ⚡ 빠른 시작 (Execution)

터미널 2개를 열어 각각 실행합니다:

```bash
# [Terminal 1] 카메라 정렬 & AI 적재함 모니터링 (Web UI :8080)
./start_vision.sh --max-allowed-shift 24 --minimum-alignment-score 0.25

# [Terminal 2] FPGA - OpenRB 통합 디스패치 컨트롤러
./start_controller.sh --fpga-port /dev/ttyUSB1 --openrb-port /dev/ttyUSB0

# (옵션) 환경 점검 및 단위 검증
./preflight.sh     # 하드웨어/패키지 사전 점검
./verify.sh        # 파이썬 문법 및 안전 게이트 단위 테스트
```

---

## 🔄 시스템 데이터 흐름

```text
[Logitech C270] ──> live_rack_monitor.py (TensorRT FP16) ──> 9개 슬롯 빈자리(EMPTY) 판정
                                                                      │
[Zybo FPGA]     ──> 8B UART (도형 ID + X,Y 좌표) ─────────────────────┼─> robot_dispatch_controller.py
                                                                      │          │
[OpenRB 로봇팔]  <── 9B UART (좌표 + 목표 슬롯 번호) ───────────────────┘          │
[OpenRB 로봇팔]  ──> 5B UART (ACK / DONE / FAULT) ───────────────────────────────┘
```

---

## 📊 AI 모델 및 슬롯 판정 규칙

- **AI 모델**: MobileNetV3-Small (TensorRT FP16 최적화)
- **판정 기준**: 
  - $P(\text{occupied}) \le 0.005 \rightarrow \mathbf{EMPTY}$ (빈 슬롯)
  - $P(\text{occupied}) \ge 0.995 \rightarrow \mathbf{OCCUPIED}$ (적재됨)
- **안전 게이트**: 5프레임 중 4프레임 이상 일치 시 `STABLE` 상태 인정 (Fail-Closed)
- **슬롯 번호 및 우선순위**:
  ```text
  상단 (L3): [ 7 ]  [ 8 ]  [ 9 ]
  중단 (L2): [ 4 ]  [ 5 ]  [ 6 ]
  하단 (L1): [ 1 ]  [ 2 ]  [ 3 ]
             C1(원) C2(사각) C3(삼각)
  ```
  *(각 도형별로 하단 `L1 → L2 → L3` 순서로 최우선 적재)*

---

## 📡 UART 통신 프로토콜 요약

| 경로 | 크기 | 패킷 구조 | 주요 필드 |
|---|:---:|---|---|
| **FPGA → Jetson** | 8B | `[AA] [SHAPE] [X_H] [X_L] [Y_H] [Y_L] [00] [XOR]` | `01`=원, `02`=사각, `03`=삼각 |
| **Jetson → OpenRB** | 9B | `[AA] [55] [CMD_ID] [SLOT] [X_H] [X_L] [Y_H] [Y_L] [CRC8]` | `SLOT`=1~9, CRC8 검증 |
| **OpenRB → Jetson** | 5B | `[AA] [55] [CMD_ID] [STATUS] [CRC8]` | `06`=ACK, `07`=DONE, `15`=NAK |

---

## 📂 디렉토리 구성

| 폴더 | 설명 |
|---|---|
| `runtime/` | 실기 배포용 런타임 (통신 도구, TensorRT 엔진, V4L2 설정) |
| `ai/` | Colab 학습 스크립트, ONNX 모델, 데이터셋 메타데이터 |
| `tests/` | 통신 프레임, 슬롯 선택, 안전 게이트 모의 테스트 |
| `evidence/` | 30분 롱런 로그 및 벤치마크 검증 데이터 |
| `docs/` | Jetson 런타임 가이드 PDF |
