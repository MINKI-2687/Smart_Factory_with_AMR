# BLOCK DIAGRAM CODE

## Snapshot (2026-08-12)

이 묶음은 2026-08-11에 수정한 최신 Vitis 및 OpenRB 코드를 포함한다.

- Vitis: OpenRB의 `RETRY` 패킷(타입 `0x03`) 수신 후 새로운 좌표 선택 및 TARGET 재전송
- OpenRB: 그리퍼 해제 실패 복구, 안전 HOME 복귀, HOME에서 RETRY 반복 송신
- ELF: 위 Vitis 소스로 2026-08-11에 다시 빌드한 실행 파일
- Vivado RTL/BIT/XSA: 변경 없음(기존 HSV, 좌표 추적, SR04 하드웨어 유지)

`docs/`의 기존 그림은 이전 배치 참고용이다. 새 블록 다이어그램에는
OpenRB에서 Vitis로 돌아가는 `RETRY [AA 55 03 Seq CRC8]` 경로를 추가해야 한다.

이 폴더는 현재 사용하는 Zybo Z7-20, Pcam 5C, HSV 물체 추적,
SR04, Vitis UART, OpenRB-150 코드를 블록 다이어그램 작성용으로
정리한 복사본이다. 원본 프로젝트는 변경하지 않는다.

## Folder Structure

- `artifacts/`: 실제 생성된 BIT, XSA, ELF 실행 산출물
- `vivado/block_design/`: Vivado Block Design과 프로젝트 파일
- `vivado/rtl/`: 현재 사용하는 사용자 RTL과 HSV 테스트벤치
- `vivado/constraints/`: 보드, 카메라, UART, SR04 핀 제약
- `vivado/scripts/`: HSV 빌드 및 타이밍 확인 Tcl
- `vitis/`: PS에서 실행되는 Vitis 애플리케이션 소스
- `arduino/`: OpenRB-150 IK, UART, Pick-and-Place 코드
- `docs/BLOCK_DIAGRAM.mmd`: Mermaid 블록 다이어그램 코드

## Read These Files First

1. `docs/BLOCK_DIAGRAM.mmd`
2. `vivado/block_design/system.bd`
3. `vivado/rtl/basic_pcam_1080_axis.v`
4. `vivado/rtl/basic_axis_blue_tracker_1080.sv`
5. `vivado/rtl/basic_rgb_hsv_blue_classifier.sv`
6. `vivado/rtl/basic_sr04_guard_wrapper.v`
7. `vivado/rtl/basic_sr04_guard.sv`
8. `vitis/main.cc`
9. `arduino/omx_ik_area_test.ino`

## Block Types

- Xilinx IP: Processing System 7, AXI VDMA, AXI GPIO,
  Video Timing Controller, AXI4-Stream to Video Out, clocks and resets.
- Digilent IP: MIPI D-PHY RX, MIPI CSI-2 RX, AXI BayerToRGB,
  AXI Gamma Correction, rgb2dvi.
- Custom RTL Module Reference: `basic_pcam_1080_axis` and
  `basic_sr04_guard_wrapper`.
- Nested custom RTL: the blue tracker and HSV classifier are instantiated
  inside `basic_pcam_1080_axis`; they are not separate top-level BD blocks.
- PS software: Vitis `main.cc`; this is software, not an FPGA IP block.

## Current Data Flow

Pcam -> MIPI D-PHY RX -> MIPI CSI-2 RX -> BayerToRGB -> Gamma ->
VDMA S2MM -> DDR -> VDMA MM2S -> custom blue tracker -> HDMI.

The tracker also packs stable object coordinates into `coord_word`.
AXI GPIO exposes that word to the ARM. Vitis reads up to six objects,
selects the smallest Y and then the smallest X, checks SW0 and SR04,
and sends a UART TARGET packet.

## Important Boundary

The BIT file configures PL hardware. The ELF runs on the PS ARM processor.
The XSA describes the hardware platform used by Vitis. Arduino behavior is
included only to document the UART boundary and the complete system flow.
