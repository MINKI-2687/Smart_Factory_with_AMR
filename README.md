<div align="center">

# 🤖 스마트 물류 자율주행 로봇 시스템
### Smart Factory Logistics with Autonomous AMR & Vision AI

**RC Shuttle × FPGA Vision × Jetson AI × Robot Arm**

자율주행 셔틀이 물류 구역을 왕복하고, FPGA 영상처리로 물체를 인식하며,  
AI 비전이 적재함 상태를 모니터링하고, 로봇팔이 물체를 집어 빈 슬롯에 적재하는  
**End-to-End 스마트 팩토리 물류 자동화 시스템**

<br>

<p align="center">
  <b>팀명: 들었다 놓조</b>
</p>

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

## 🎬 시스템 시연 (Demonstration)

| 🚗 자율주행 셔틀 (AMR) | 📷 FPGA 영상처리 & 도형 인식 |
|:---:|:---:|
| ![AMR Shuttle](docs/images/amr_shuttle_demo.gif) | ![FPGA Shape Tracking](docs/images/fpga_shape_detect_demo.gif) |
| **LiDAR SLAM 기반 주행 및 슬롯 도킹** | **Pcam 5C 기반 실시간 HSV 추적 & 도형 판별** |

| 🧠 Jetson AI 적재함 모니터링 | 🦾 로봇팔 물류 파지 및 슬롯 적재 |
|:---:|:---:|
| ![AI Rack Monitor](docs/images/ai_rack_monitor_demo.gif) | ![Robot Arm](docs/images/robot_arm_demo.gif) |
| **MobileNetV3 (TensorRT FP16) 9슬롯 모니터링** | **OpenRB-150 기반 IK 역기구학 픽앤플레이스** |

---

## 📋 목차

- [시스템 아키텍처](#-시스템-아키텍처)
- [서브시스템 개요](#-서브시스템-개요)
- [디렉토리 구조](#-디렉토리-구조)
- [하드웨어 구성](#-하드웨어-구성)
- [통신 프로토콜](#-통신-프로토콜)
- [트러블슈팅 및 개선](#-트러블슈팅-및-개선)
- [실행 방법](#-실행-방법)
- [기술 스택](#-기술-스택)

---

## 🏗 시스템 아키텍처

<p align="center">
  <img src="docs/images/system_architecture.png" alt="System Architecture" width="90%">
</p>

```mermaid
graph TB
    subgraph SHUTTLE["🚗 자율주행 셔틀 (Raspberry Pi)"]
        LIDAR["RPLidar C1"]
        SLAM["LiDAR SLAM<br/>스캔매칭 위치추정"]
        ASTAR["A* 경로계획"]
        PID["PID 추종"]
        SLOT["슬롯 정렬/진입/착석"]
    end

    subgraph FPGA_VISION["📷 FPGA 영상처리 (Zybo Z7-20)"]
        PCAM["Pcam 5C"]
        HSV["HSV 물체 추적<br/>(도형·색상 분류)"]
        SR04["SR04 초음파<br/>안전 가드"]
        VITIS["Vitis PS<br/>(ARM Cortex-A9)"]
    end

    subgraph FPGA_MOTOR["⚡ FPGA 모터 제어 (Zybo Z7-20)"]
        MOTOR_CTRL["PWM 모터 제어"]
        ENCODER["엔코더 피드백"]
        WATCHDOG["와치독 타이머"]
    end

    subgraph JETSON["🧠 Jetson AI (Orin Nano)"]
        CAM["Logitech C270"]
        TRT["MobileNetV3-Small<br/>TensorRT FP16"]
        MONITOR["적재함 모니터링<br/>(9슬롯 EMPTY/OCCUPIED)"]
        DISPATCH["통합 디스패치<br/>컨트롤러"]
        WEB["웹 UI :8080"]
    end

    subgraph ARM["🦾 로봇팔 (OpenRB-150)"]
        DXL["Dynamixel 서보"]
        IK["역기구학(IK)<br/>좌표→관절각"]
        GRIPPER["그리퍼<br/>집기/놓기"]
    end

    LIDAR --> SLAM --> ASTAR --> PID --> SLOT
    PID -.->|UART| FPGA_MOTOR
    FPGA_MOTOR -.->|엔코더 데이터| SHUTTLE

    PCAM --> HSV --> VITIS
    SR04 --> VITIS
    VITIS -->|"UART 8B<br/>도형+좌표"| DISPATCH

    CAM --> TRT --> MONITOR --> DISPATCH
    DISPATCH -->|"UART 9B<br/>좌표+슬롯"| ARM
    ARM -->|"UART 5B<br/>ACK/DONE"| DISPATCH
    MONITOR --> WEB

    IK --> DXL --> GRIPPER

    style SHUTTLE fill:#2ecc71,color:#fff
    style FPGA_VISION fill:#3498db,color:#fff
    style FPGA_MOTOR fill:#e67e22,color:#fff
    style JETSON fill:#9b59b6,color:#fff
    style ARM fill:#e74c3c,color:#fff
```

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

---

### 2. Jetson AI 비전 & 통합 제어 (`jetson/`)

<p align="center">
  <img src="docs/images/ai_rack_roi_cnn.png" alt="AI Rack ROI & CNN" width="75%">
</p>

| 항목 | 내용 |
|---|---|
| **플랫폼** | Jetson Orin Nano |
| **언어** | Python |
| **AI** | MobileNetV3-Small (TensorRT FP16), 9슬롯 EMPTY/OCCUPIED 이진분류 |
| **기능** | 적재함 실시간 모니터링 (웹 UI :8080) + FPGA·OpenRB 간 통합 디스패치 |
| **안전** | 5프레임 다수결 STABLE 판정, fail-closed 방식 |
| **통신** | FPGA → 8B UART (도형+좌표), Jetson → OpenRB 9B UART (좌표+슬롯) |

👉 자세한 내용: [`jetson/README.md`](jetson/README.md)

---

### 3. FPGA 영상처리 & 모터 제어 (`fpga/`)

| 서브모듈 | 내용 |
|---|---|
| **vision-pipeline** | Pcam 5C → MIPI → HSV 물체 추적 → HDMI 출력 + UART 좌표 전송 (Zybo Z7-20, Verilog/SV + Vitis C++) |
| **motor-control** | RC Car 모터 PWM 제어 + 엔코더 오도메트리 (Zybo Z7-20, SystemVerilog) |

👉 자세한 내용: [`fpga/README.md`](fpga/README.md)

---

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
│   ├── app/                     #   CLI 메인 엔트리 (main_shuttle.c)
│   ├── include/                 #   헤더 47개 (선언)
│   ├── src/                     #   실행 구현 모듈 43개
│   ├── viewer/                  #   live_view.py (실시간 시각화)
│   ├── maps/                    #   고정맵 (150×90, 10mm 격자)
│   ├── sim/                     #   오프라인 시뮬레이터 (Python ctypes)
│   ├── tests/                   #   단위 테스트
│   └── tools/                   #   하드웨어 브링업 도구
│
├── jetson/                      # Jetson AI 비전 & 통합 제어 (Python)
│   ├── runtime/                 #   시연 런타임 (TensorRT, V4L2, 통신)
│   ├── ai/                      #   학습 Colab 노트북, ONNX 모델
│   ├── tests/                   #   단위 테스트
│   ├── evidence/                #   검증 보고서 및 로그
│   └── docs/                    #   런타임 가이드 PDF
│
├── fpga/                        # FPGA 설계 (Verilog / SystemVerilog)
│   ├── vision-pipeline/         #   Pcam 영상처리 + Vitis PS
│   │   ├── vivado/              #     Block Design, RTL, 제약, Tcl
│   │   ├── vitis/               #     ARM PS 코드 (main.cc)
│   │   ├── artifacts/           #     BIT, XSA, ELF 산출물
│   │   └── docs/                #     블록 다이어그램
│   └── motor-control/           #   RC Car 모터 제어
│       ├── rtl/                 #     SystemVerilog 모듈 7개
│       ├── constraints/         #     FPGA 핀 매핑 (.xdc)
│       └── sim/                 #     테스트벤치 5개
│
├── arduino/                     # 로봇팔 제어 (OpenRB-150)
│   └── openrb_robot_arm/
│       └── openrb_robot_arm.ino #   좌표 수신 → IK → 집기/적재
│
└── docs/                        # 프로젝트 문서 & 발표 자료
    ├── Smart_Factory_with_AMR.pptx  # 팀 프로젝트 발표 PPT
    └── images/                  # 시연 GIF 및 다이어그램
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

<p align="center">
  <img src="docs/images/protocol_diagram.png" alt="Communication Protocol" width="80%">
</p>

### FPGA → Jetson (8바이트, 115200 8-N-1)
```
[AA] [SHAPE] [X_H] [X_L] [Y_H] [Y_L] [RESERVED] [XOR]
```
- `SHAPE`: `01`=원, `02`=사각, `03`=삼각
- `X/Y`: Big-endian 픽셀 좌표 (중심점)
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

---

## 🛠 트러블슈팅 및 개선 (Troubleshooting)

<p align="center">
  <img src="docs/images/troubleshoot_shape_detect.png" alt="Troubleshooting Shape Detection" width="85%">
</p>

1. **조명 민감도 및 도형 인식 오류 개선**
   - 조명 변화에 취약한 단순 RGB 임계값을 **HSV 색공간 변환 및 모폴로지 필터링**으로 대체하여 인식 안정성 확보
2. **로봇팔 도달 안정성 (Settle Time) 확보**
   - 단일 허용 오차 통과 방식에서 `SETTLE_STABLE_COUNT` 연속 충족 방식으로 변경하여 떨림/관성으로 인한 파지 실패 방지
3. **엔코더 오도메트리 슬립 보정**
   - LiDAR Scan Matching 위치 보정과 엔코더 카운터 동기화를 통해 누적 슬립 오차 최소화

---

## 🚀 실행 방법

### 1. 자율주행 셔틀 (Raspberry Pi)
```bash
cd rc_shuttle/
make          # 빌드
make run      # 실시간 UI와 함께 실행
```

### 2. Jetson AI 비전 & 통합 제어 (Orin Nano)
```bash
cd jetson/

# Terminal 1: 비전 모니터링 & 웹 UI
./start_vision.sh --max-allowed-shift 24 --minimum-alignment-score 0.25

# Terminal 2: 통합 디스패처
./start_controller.sh --fpga-port /dev/ttyUSB1 --openrb-port /dev/ttyUSB0
```

---

## 📄 License & Presentation

- **발표 자료**: [`docs/Smart_Factory_with_AMR.pptx`](docs/Smart_Factory_with_AMR.pptx)
- 본 프로젝트는 교육 및 스마트 팩토리 연구 목적으로 제작되었습니다.
