# 스마트 물류 로봇팔 2 — Jetson 통합 릴리스

이 폴더는 다른 Jetson에 그대로 복사해 실행할 수 있도록 정리한 **실행용
릴리스**입니다. 원본 개발 폴더 `rack_dataset_v1`은 변경하지 않았고, 예전
STM32/3바이트 UART·초기 학습 자료는 `archive/`에 보관했습니다.

## 먼저 확인할 것

이 릴리스는 Jetson Orin Nano에서 다음 장치를 사용하는 최종 흐름을 기준으로
합니다.

```text
Logitech C270
  -> live_rack_monitor.py (TensorRT, 9개 ROI EMPTY/OCCUPIED, 웹 UI :8080)
Zybo Z7-20 FPGA
  -> 8바이트 UART (도형 + 중심좌표) -> robot_dispatch_controller.py
Jetson
  -> 9바이트 UART (좌표 + 슬롯) -> OpenRB-150
OpenRB-150
  -> 5바이트 ACK/DONE/FAULT 응답 -> Jetson
```

카메라 모니터와 통합 컨트롤러는 **서로 다른 터미널에서 하나씩** 실행합니다.
같은 FPGA 포트를 단독 수신기와 통합 컨트롤러가 동시에 열면 안 됩니다.

## 실행 순서

### 1. 사전 점검

```bash
cd /path/to/robot_rack_project_20260811
./preflight.sh
```

`/dev/video0`, `/dev/ttyUSB*`, Python 패키지와 TensorRT 엔진 파일을 점검합니다.
하드웨어가 연결되지 않은 PC에서 실행해도 누락 항목을 `WARN`으로 표시하고
점검 결과를 보여줍니다.

### 2. 카메라·웹 UI

```bash
./start_vision.sh \
  --max-allowed-shift 24 \
  --minimum-alignment-score 0.25 \
  --alignment-interval 0.5
```

브라우저에서 `http://JETSON_IP:8080/`을 열고 `STABLE`, `Aligned=YES`, 9개
슬롯 상태가 정상인지 확인합니다. 미리보기 밝기 보정은 화면 표시 전용이며
TensorRT 입력에는 적용되지 않습니다.

### 3. 통합 제어

먼저 `ls -l /dev/ttyUSB*`로 FPGA와 OpenRB 포트 번호를 확인합니다. 포트 번호는
USB 연결 순서에 따라 바뀔 수 있습니다.

```bash
./start_controller.sh \
  --fpga-port /dev/ttyUSB1 \
  --openrb-port /dev/ttyUSB0 \
  --openrb-done-timeout 60
```

로봇을 연결하지 않고 슬롯 선택만 확인하려면 다음처럼 실행합니다.

```bash
./start_controller.sh --fpga-port /dev/ttyUSB1 --dry-run
```

종료는 두 터미널에서 `Ctrl+C`입니다. 실행 중인 모니터의 상태는 다음으로
확인할 수 있습니다.

```bash
curl -fsS http://127.0.0.1:8080/status.json
```

## 프로젝트 구성

```text
jetson/
├── runtime/                 # 시연 시 필요한 런타임
│   ├── tools/communication/ # FPGA 수신, 슬롯 선택, OpenRB 전송
│   ├── tools/deployment/    # 카메라·TensorRT·정렬·안전 게이트
│   ├── config/              # C270 V4L2 설정
│   ├── deployment/...       # TensorRT 엔진·메타데이터·임계값
│   ├── raw/                 # 적재함 정렬 기준 이미지
│   └── openrb/              # OpenRB에 업로드할 최종 .ino
├── ai/                      # 학습·재현용 자료
│   ├── training/             # v2 Colab notebook/script
│   ├── models/               # ONNX 원본
│   ├── datasets/             # v2 요약과 Colab ZIP
│   ├── config/               # 데이터셋·ROI 설정
│   └── tools/                # 데이터/노트북 감사·패키징 도구
├── tests/                   # 통신·슬롯·안전 게이트 단위 테스트
├── evidence/                # 검증 보고서·30분 로그·카메라 결정 자료
├── docs/                    # 기존 런타임 가이드 PDF
├── optional_drivers/        # Jetson L4T용 PL2303 소스/모듈
└── archive/                 # 사용하지 않는 초기 코드·환경·자료
```

`__pycache__`, Colab 임시 파일, 실행 중 생성되는 로그는 릴리스에 넣지
않았습니다. 실행 중 필요한 로그는 각 프로그램의 표준 출력과 기존 원본
프로젝트의 로그 위치를 사용합니다.

## 데이터와 판정 규칙

- MobileNetV3-Small 이진 분류: `EMPTY` / `OCCUPIED`
- 입력: 224×224, ImageNet mean/std 전처리
- TensorRT 엔진: FP16 연산, FP32 입력/출력, TensorRT 10.3
- `p_occupied <= 0.005` → `EMPTY`, `>= 0.995` → `OCCUPIED`, 그 사이는 `UNKNOWN`
- 5프레임 중 최소 4프레임이 일치해야 `STABLE`; 카메라 fault·unknown이면 명령을
  보내지 않는 fail-closed 방식
- 도형 열 매핑: `CIRCLE→C1`, `SQUARE→C2`, `TRIANGLE→C3`
- 각 열에서 `L1→L2→L3` 순서로 가장 낮은 빈 슬롯 선택

슬롯 번호는 다음과 같습니다.

```text
상단 L3:  7  8  9
중단 L2:  4  5  6
하단 L1:  1  2  3
          C1 C2 C3
```

## UART 계약

### FPGA → Jetson (8바이트, 115200 8-N-1)

```text
AA SHAPE X_H X_L Y_H Y_L RESERVED XOR
```

`X/Y`는 big-endian 픽셀 좌표이며, 마지막 XOR는 `SHAPE`부터 `RESERVED`까지의
XOR입니다. Jetson은 유효 패킷마다 `0x06` ACK, 오류 프레임에는 `0x15` NAK를
즉시 반환합니다. 현재 FPGA가 ACK를 읽지 않고 한 번만 보내는 모드라면 단독
수신기의 `--ack`는 통신 확인용일 뿐이며, 이벤트 게이트 요구 횟수와 FPGA 송신
정책을 맞춰야 합니다.

### Jetson → OpenRB (9바이트)

```text
AA 55 COMMAND_ID SLOT X_H X_L Y_H Y_L CRC8
```

CRC8은 polynomial `0x07`, initial `0x00`, 범위는 `COMMAND_ID`부터 `Y_L`까지
6바이트입니다. 재시도는 같은 `COMMAND_ID`와 같은 프레임을 사용합니다.

### OpenRB → Jetson (5바이트)

```text
AA 55 COMMAND_ID STATUS CRC8
```

`06=ACK`, `07=DONE`, `15=NAK`, `E0=BUSY`, `E1=FAULT`. Jetson은 ACK만으로 성공
처리하지 않고 DONE까지 기다립니다. 실제 OpenRB 스케치는
`runtime/openrb/final_openrb_20260810.ino`입니다.

## 하드웨어 연결 요약

FPGA와 OpenRB 각각 USB-TTL 하나가 필요합니다. 두 장치 모두 TX/RX는 교차하고
GND는 공통으로 연결합니다. OpenRB 쪽 USB-TTL의 빨간색 +5V 선은 연결하지
않습니다. OpenRB Serial3는 `D13=RX`, `D14=TX`, 115200bps입니다.

## 검증

```bash
./verify.sh
```

런타임 Python 문법, 통신 프레임, 슬롯 선택, 5프레임 안전 게이트 테스트를
실행합니다. TensorRT·카메라·UART가 없는 환경에서도 소프트웨어 테스트까지는
실행할 수 있습니다.

## 현재 알고 있는 제한

1. `final_openrb_20260810.ino`의 `USE_FIXED_PICK_POSE=true`이면 FPGA 좌표를
   수신·전달하더라도 실제 집기 위치는 OpenRB의 고정 pick pose를 사용할 수
   있습니다. 좌표 기반 집기를 쓰려면 OpenRB 쪽에서 해당 옵션과 좌표 변환을
   별도로 구현해야 합니다.
2. OpenRB가 DONE을 보내지 않으면 Jetson은 기본 60초 후 재시도하고 차단합니다.
   실제 모터 동작 종료 직후 DONE이 정확히 송신되는지 확인해야 합니다.
3. 카메라·적재함의 위치와 조명은 기준 촬영 조건을 유지해야 합니다. 정렬 완화
   옵션은 오차를 허용하지만 잘못된 ROI를 정상으로 볼 가능성도 키웁니다.

## 원본과 보관 자료

- 원본 개발 폴더: `../rack_dataset_v1` (보존, 자동 변경하지 않음)
- 초기 3바이트/STM32/OpenRB 시험 코드: `archive/legacy_uart/`
- 초기 학습·데이터셋 자료: `archive/legacy_training/`
- 캡처용 보조 코드: `archive/legacy_capture/`
- Jetson 전용 가상환경과 드라이버 빌드 자료: `archive/legacy_environment/`,
  `optional_drivers/`

다운로드한 뒤 이 폴더의 `README.md`, `preflight.sh`, `start_vision.sh`,
`start_controller.sh`만 따라가면 됩니다. 경로를 하드코딩하지 않았으므로 폴더
이름이나 위치를 바꿔도 실행 스크립트가 자기 위치를 기준으로 동작합니다.
