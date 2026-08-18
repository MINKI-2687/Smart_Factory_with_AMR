<div align="center">

# 🤖 스마트 물류 자율주행 로봇 시스템

**RC Shuttle × FPGA Vision × Jetson AI × Robot Arm**

자율주행 셔틀이 물류 구역을 왕복하고, FPGA 영상처리로 물체를 인식하며,
AI 비전이 적재함 상태를 모니터링하고, 로봇팔이 물체를 집어 빈 슬롯에 적재하는
엔드투엔드 스마트 물류 시스템

<br>

![C](https://img.shields.io/badge/C-A8B9CC?style=flat-square&logo=c&logoColor=white)
![Python](https://img.shields.io/badge/Python-3776AB?style=flat-square&logo=python&logoColor=white)
![SystemVerilog](https://img.shields.io/badge/SystemVerilog-CAD09D?style=flat-square)
![C++](https://img.shields.io/badge/C++-00599C?style=flat-square&logo=cplusplus&logoColor=white)
![Arduino](https://img.shields.io/badge/Arduino-00979D?style=flat-square&logo=arduino&logoColor=white)
![TensorRT](https://img.shields.io/badge/TensorRT-76B900?style=flat-square&logo=nvidia&logoColor=white)
![Vivado](https://img.shields.io/badge/Xilinx_Vivado-E01F27?style=flat-square)
![Zybo Z7](https://img.shields.io/badge/Zybo_Z7--20-7D3C98?style=flat-square)
![Jetson](https://img.shields.io/badge/Jetson_Orin_Nano-76B900?style=flat-square&logo=nvidia&logoColor=white)
![Raspberry Pi](https://img.shields.io/badge/Raspberry_Pi-C51A4A?style=flat-square&logo=raspberrypi&logoColor=white)

</div>

---

## 📋 목차

- [시스템 아키텍처](#-시스템-아키텍처)
- [서브시스템 개요](#-서브시스템-개요)
- [디렉토리 구조](#-디렉토리-구조)
- [하드웨어 구성](#-하드웨어-구성)
- [통신 프로토콜](#-통신-프로토콜)
- [실행 방법](#-실행-방법)
- [기술 스택](#-기술-스택)

---

## 🏗 시스템 아키텍처
<img width="2874" height="1086" alt="image" src="https://github.com/user-attachments/assets/ad4e3d0a-a30d-4bee-9f4d-1f3db6973dd1" />



---

## 📦 서브시스템 개요

### 1. 자율주행 셔틀 (`rc_shuttle/`)

| 항목 | 내용 |
|---|---|
| **플랫폼** | Raspberry Pi + RPLidar C1 |
| **언어** | C11 (43개 소스 모듈, 47개 헤더) |
| **기능** | 고정맵 LiDAR SLAM → A* 경로계획 → PID 추종 → 슬롯 주차 |
| **통신** | FPGA 모터보드 ↔ UART (속도·방향 명령 / 엔코더 피드백) |
| **빌드** | `make` (GCC, `-flto` LTO 최적화) |
| **뷰어** | `live_view.py` — matplotlib 실시간 시각화 |

👉 자세한 내용: [`rc_shuttle/README.md`](rc_shuttle/README.md)

### 2. Jetson AI 비전 & 통합 제어 (`jetson/`)

| 항목 | 내용 |
|---|---|
| **플랫폼** | Jetson Orin Nano |
| **언어** | Python |
| **AI** | MobileNetV3-Small (TensorRT FP16), 9슬롯 EMPTY/OCCUPIED 이진분류 |
| **기능** | 적재함 실시간 모니터링 (웹 UI) + FPGA·OpenRB 간 통합 디스패치 |
| **안전** | 5프레임 다수결 STABLE 판정, fail-closed 방식 |
| **통신** | FPGA → 8B UART (도형+좌표), Jetson → OpenRB 9B UART (좌표+슬롯) |

👉 자세한 내용: [`jetson/README.md`](jetson/README.md)

### 3. FPGA 영상처리 & 모터 제어 (`fpga/`)

| 서브모듈 | 내용 |
|---|---|
| **vision-pipeline** | Pcam 5C → MIPI → HSV 물체 추적 → HDMI 출력 + UART 좌표 전송 (Zybo Z7-20, Verilog/SV + Vitis C++) |
| **motor-control** | RC Car 모터 PWM 제어 + 엔코더 오도메트리 (Zybo Z7-20, SystemVerilog) |

👉 자세한 내용: [`fpga/README.md`](fpga/README.md)

### 4. 로봇팔 제어 (`arduino/`)

| 항목 | 내용 |
|---|---|
| **플랫폼** | OpenRB-150 + Dynamixel 서보 모터 |
| **언어** | Arduino C++ |
| **기능** | Jetson 좌표 수신 → 역기구학 보간 → 물체 집기 → 빈 슬롯 적재 |
| **특징** | 삼각형 보간 좌표 보정, 관절공간 선형보간, settle 안정 판정, RETRY 복구 |

---

## 📂 디렉토리 구조

```
.
├── rc_shuttle/                  # 자율주행 셔틀 (C / Raspberry Pi)
│   ├── app/                     #   main_shuttle.c (CLI 엔트리)
│   ├── include/                 #   헤더 47개 (선언)
│   ├── src/                     #   구현 43개 (실행 코드)
│   ├── viewer/                  #   live_view.py (실시간 시각화)
│   ├── maps/                    #   고정맵 (150×90, 10mm 격자)
│   ├── sim/                     #   오프라인 시뮬레이터 (Python + ctypes)
│   ├── tests/                   #   C 검증 + Python 스모크 테스트
│   └── tools/                   #   하드웨어 브링업 도구
│
├── jetson/                      # Jetson AI 비전 & 통합 제어 (Python)
│   ├── runtime/                 #   시연 런타임
│   │   ├── tools/communication/ #     FPGA 수신, 슬롯 선택, OpenRB 전송
│   │   ├── tools/deployment/    #     카메라·TensorRT·정렬·안전 게이트
│   │   ├── config/              #     C270 V4L2 설정
│   │   └── deployment/          #     TensorRT 엔진·메타데이터
│   ├── ai/                      #   학습·재현 자료
│   │   ├── training/            #     Colab 노트북/스크립트
│   │   ├── models/              #     ONNX 모델
│   │   └── datasets/            #     데이터셋 요약
│   ├── tests/                   #   단위 테스트
│   ├── evidence/                #   검증 보고서·로그
│   └── docs/                    #   런타임 가이드 PDF
│
├── fpga/                        # FPGA 설계 (Verilog / SystemVerilog)
│   ├── vision-pipeline/         #   Pcam 영상처리 + Vitis PS
│   │   ├── vivado/              #     Block Design, RTL, 제약, Tcl
│   │   ├── vitis/               #     ARM PS 코드 (main.cc)
│   │   ├── arduino/             #     OpenRB IK 코드 (vision용)
│   │   ├── artifacts/           #     BIT, XSA, ELF
│   │   └── docs/                #     블록 다이어그램
│   └── motor-control/           #   RC Car 모터 제어
│       ├── rtl/                 #     SystemVerilog 모듈 7개
│       ├── constraints/         #     FPGA 핀 매핑 (.xdc)
│       └── sim/                 #     테스트벤치 5개
│
├── arduino/                     # 로봇팔 제어 (OpenRB-150)
│   └── openrb_robot_arm/
│       └── openrb_robot_arm.ino #     좌표 수신 → IK → 집기/적재
│
└── docs/                        # 프로젝트 문서
    └── poster.pptx              #   포스터 양식
```

---

## 🔧 하드웨어 구성

| 장치 | 역할 | 인터페이스 |
|---|---|---|
| **Zybo Z7-20** (×2) | FPGA 영상처리 / 모터 제어 | HDMI, Pcam MIPI, UART, GPIO |
| **Jetson Orin Nano** | AI 추론, 통합 디스패치 | USB (카메라, UART×2) |
| **Raspberry Pi** | 자율주행 셔틀 제어 | USB (LiDAR, UART) |
| **RPLidar C1** | 360° LiDAR 스캐너 | USB-Serial |
| **Pcam 5C** | OV5640 MIPI 카메라 | MIPI CSI-2 |
| **Logitech C270** | 적재함 모니터링 카메라 | USB |
| **OpenRB-150** | 로봇팔 서보 컨트롤러 | TTL (Dynamixel), UART |
| **Dynamixel 서보** | 로봇팔 관절 구동 | TTL bus |
| **HC-SR04** | 초음파 안전 가드 | GPIO (Trig/Echo) |

---

## 📡 통신 프로토콜

### FPGA → Jetson (8바이트, 115200 8-N-1)

```
[AA] [SHAPE] [X_H] [X_L] [Y_H] [Y_L] [RESERVED] [XOR]
```

- `SHAPE`: `01`=원, `02`=사각, `03`=삼각
- `X/Y`: Big-endian 픽셀 좌표
- `XOR`: `SHAPE`~`RESERVED` XOR 체크섬

### Jetson → OpenRB (9바이트)

```
[AA] [55] [CMD_ID] [SLOT] [X_H] [X_L] [Y_H] [Y_L] [CRC8]
```

- `SLOT`: 빈 슬롯 번호 (1~9)
- `CRC8`: poly=0x07, init=0x00, 범위=`CMD_ID`~`Y_L` 6바이트

### OpenRB → Jetson (5바이트)

```
[AA] [55] [CMD_ID] [STATUS] [CRC8]
```

- `STATUS`: `06`=ACK, `07`=DONE, `15`=NAK, `E0`=BUSY, `E1`=FAULT

### 슬롯 매핑

```
     C1(원) C2(사각) C3(삼각)
L3:   7      8       9
L2:   4      5       6
L1:   1      2       3
```

도형 열 매핑: `CIRCLE→C1`, `SQUARE→C2`, `TRIANGLE→C3` — 각 열에서 `L1→L2→L3` 순으로 가장 낮은 빈 슬롯 선택.

---

## 🚀 실행 방법

### 자율주행 셔틀 (Raspberry Pi)

```bash
cd rc_shuttle/
make                    # main_shuttle 빌드
make run                # live_view.py로 실기 실행
make all                # 도구 + 테스트 + 시뮬용 .so 포함
```

### Jetson AI 비전 & 통합 제어

```bash
cd jetson/

# 1. 사전 점검
./preflight.sh

# 2. 카메라 & 웹 UI 실행
./start_vision.sh --max-allowed-shift 24 --minimum-alignment-score 0.25

# 3. 통합 디스패치 컨트롤러 실행 (별도 터미널)
./start_controller.sh --fpga-port /dev/ttyUSB1 --openrb-port /dev/ttyUSB0

# 테스트
./verify.sh
```

### FPGA

1. **Vision Pipeline**: Vivado에서 `fpga/vision-pipeline/vivado/block_design/hw.xpr` 열기
2. **Motor Control**: Vivado에서 `fpga/motor-control/rtl/rc_car_top.sv`를 최상위 모듈로 합성
3. 미리 빌드된 산출물: `fpga/vision-pipeline/artifacts/` (BIT, XSA, ELF)

### 로봇팔 (OpenRB-150)

Arduino IDE에서 `arduino/openrb_robot_arm/openrb_robot_arm.ino`를 OpenRB-150에 업로드.

---

## 🛠 기술 스택

| 영역 | 기술 |
|---|---|
| **자율주행** | C11, LiDAR SLAM (스캔매칭), A* 경로계획, PID 제어, 슬롯 주차 |
| **AI 추론** | MobileNetV3-Small, PyTorch → ONNX → TensorRT FP16 |
| **FPGA 설계** | Vivado 2024.1, SystemVerilog, Verilog, Vitis (Zynq PS) |
| **영상처리** | HSV 색상 분류, MIPI CSI-2, AXI4-Stream, VDMA |
| **로봇팔** | Dynamixel 서보, 역기구학(IK), 삼각형 보간 좌표 보정 |
| **통신** | UART (8-N-1 115200bps), CRC8 체크섬, 재시도 프로토콜 |
| **웹 UI** | Flask/HTTP (적재함 상태 모니터링, :8080) |
| **시뮬레이션** | Python ctypes + matplotlib, C 유닛 테스트 |

---

## 📄 License

이 프로젝트는 교육 목적으로 제작되었습니다.
