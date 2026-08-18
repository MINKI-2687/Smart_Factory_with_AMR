# 📷 FPGA Vision Pipeline — 영상처리 & 물체 좌표 추적

> **Zybo Z7-20 (Zynq-7000)** 기반의 실시간 하드웨어 영상처리 및 Zynq PS 제어 파이프라인  
> Pcam 5C 카메라 영상 입력 → HSV 색상/도형 인식 → HDMI 출력 및 UART 좌표 전송을 수행합니다.

---

## 🔄 비전 데이터 파이프라인

```text
[Pcam 5C (MIPI CSI-2)]
         │
         ▼
[Digilent MIPI D-PHY / CSI-2 RX] ──> [AXI Bayer to RGB] ──> [AXI Gamma]
                                                                  │
                                                                  ▼
[HDMI 디스플레이 출력] <── [AXI4-Stream / Video Out] <── [HSV Blue Tracker (Custom RTL)]
                                                                  │ (추적 좌표 패킹)
                                                                  ▼
[Jetson / OpenRB] <── [UART TARGET 패킷 전송] <── [Vitis PS (ARM Cortex-A9)]
```

---

## 📂 디렉토리 구성

| 폴더 | 설명 |
|---|---|
| `vivado/block_design/` | Vivado Block Design (`system.bd`, `hw.xpr`) |
| `vivado/rtl/` | 커스텀 영상처리 RTL (`basic_axis_blue_tracker_1080.sv`, HSV 분류기 등) |
| `vivado/constraints/` | Zybo Z7 보드, Pcam, HDMI, UART 핀 제약 (`.xdc`) |
| `vivado/scripts/` | Vivado 합성 및 타이밍 검증 자동화 Tcl 스크립트 |
| `vitis/` | Zynq PS ARM 프로세서 실행 소스 (`main.cc`) |
| `artifacts/` | 미리 생성된 하드웨어 산출물 (`.bit`, `.xsa`, `.elf`) |
| `docs/` | 시스템 블록 다이어그램 (Mermaid, Draw.io) |

---

## 🧩 주요 RTL 모듈

| 모듈명 | 유형 | 역할 |
|---|---|---|
| `basic_pcam_1080_axis` | Custom Top | AXI4-Stream 영상 파이프라인 통합 모듈 |
| `basic_axis_blue_tracker_1080` | Custom RTL | 영상 스트림 내 파란색 영역 Bounding Box & 중심 좌표 계산 |
| `basic_rgb_hsv_blue_classifier` | Custom RTL | RGB → HSV 실시간 변환 및 파란색 픽셀 분류 |
| `basic_sr04_guard_wrapper` | Custom RTL | HC-SR04 초음파 센서 거리 측정 및 비상 정지 가드 |
