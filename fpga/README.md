# FPGA — 영상처리 파이프라인 & RC Car 모터 제어

Zybo Z7-20 기반 FPGA 설계를 두 파트로 나누어 관리합니다.

---

## 디렉토리 구조

```
fpga/
├── vision-pipeline/         # Pcam 5C 영상처리 + Vitis PS + OpenRB UART
│   ├── vivado/
│   │   ├── block_design/    # Vivado Block Design (.bd, .xpr)
│   │   ├── rtl/             # 사용자 RTL (HSV 추적기, SR04 가드)
│   │   ├── constraints/     # 보드·카메라·UART·SR04 핀 제약
│   │   └── scripts/         # 빌드·타이밍 검증 Tcl
│   ├── vitis/               # ARM PS 애플리케이션 (main.cc)
│   ├── arduino/             # OpenRB-150 IK·UART 코드 (.ino)
│   ├── artifacts/           # 생성된 BIT, XSA, ELF
│   ├── docs/                # 블록 다이어그램 (Mermaid, drawio)
│   └── README.md
│
└── motor-control/           # RC Car FPGA 모터 제어 모듈
    ├── rtl/                 # SystemVerilog RTL
    │   ├── rc_car_top.sv    # 최상위 모듈
    │   ├── UART.sv          # UART 통신
    │   ├── cmd_rx.sv        # 명령 수신
    │   ├── motor_controller.sv
    │   ├── encoder_counter.sv
    │   ├── encoder_tx.sv    # 엔코더 데이터 송신
    │   └── watchdog.sv      # 와치독 타이머
    ├── constraints/
    │   └── rc_car.xdc       # FPGA 핀 매핑
    ├── sim/                 # 테스트벤치
    │   ├── tb_rc_car_top.sv
    │   ├── tb_uart.sv
    │   ├── tb_motor_controller.sv
    │   ├── tb_motor_and_encoder.sv
    │   └── tb_encoder_counter.sv
    └── docs/                # 블록 다이어그램 XML (프레젠테이션용)
```

---

## vision-pipeline

Pcam 5C → MIPI D-PHY → BayerToRGB → Gamma → VDMA → HSV 물체 추적 → HDMI 출력.
추적기가 안정 좌표를 `coord_word`로 AXI GPIO에 노출하면, Vitis PS(ARM)가 읽어
UART TARGET 패킷으로 Jetson/OpenRB에 전송합니다.

자세한 내용은 [vision-pipeline/README.md](vision-pipeline/README.md) 참조.

## motor-control

라즈베리파이 ↔ FPGA 간 UART 명령으로 RC Car 모터를 제어합니다.
엔코더 피드백을 읽어 오도메트리 데이터를 호스트에 반환합니다.

### 모듈 요약

| 모듈 | 설명 |
|---|---|
| `rc_car_top` | 최상위. UART RX/TX, 모터 2개, 엔코더 2개 연결 |
| `UART` | 115200 8-N-1 송수신기 |
| `cmd_rx` | 수신 바이트 → 명령 파싱 (좌/우 모터 속도·방향) |
| `motor_controller` | PWM 생성 + 방향 제어 |
| `encoder_counter` | A/B 상 엔코더 카운터 |
| `encoder_tx` | 엔코더 값 패킹 → UART 송신 |
| `watchdog` | 명령 타임아웃 시 모터 정지 |
