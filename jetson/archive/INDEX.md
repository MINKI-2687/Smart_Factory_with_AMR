# 보관 자료 안내

이 폴더는 현재 최종 시연 흐름에서 직접 실행하지 않는 자료를 한곳에 모은
곳입니다. 새 기능을 추가할 때 먼저 `runtime/`을 수정하고, 이 보관 자료는
프로토콜 비교나 과거 작업 확인에만 사용합니다.

## legacy_uart

- `zybo_uart_receiver_3byte_legacy.py`: 초기 `AA SHAPE ~SHAPE` 수신기
- `stm32_*`, `STM32_CUBEIDE_*`: Nucleo-F411RE 시험용 5바이트 프로토콜
- `openrb_handshake_test.py`, `openrb_uart_handshake/`: OpenRB 슬롯 전용 초기 시험
- `openrb_target_handshake_9byte/`: 좌표+슬롯 프로토콜 초기 스케치
- `openrb_robot_arm_integrated/`: 초기 로봇팔 통합 스케치

현재 사용하는 것은 `runtime/tools/communication/`의 8바이트 FPGA 수신기와
9바이트 OpenRB 전송기입니다. 위 레거시 스크립트를 통합 실행과 동시에 같은
UART 포트에서 실행하면 안 됩니다.

## legacy_training

v1 데이터셋·ROI 설정과 초기 MobileNetV3-Small Colab notebook입니다. 현재
재현 기준은 `ai/training/02_train_mobilenetv3_small_colab_v2.*`와
`ai/config/*v2*`입니다.

## legacy_capture

초기 카메라 캡처·ROI 확인 보조 코드입니다. 시연 시에는
`runtime/tools/deployment/live_rack_monitor.py`만 사용합니다.

## legacy_environment

이 Jetson에서 사용하던 Python 가상환경입니다. CPU 아키텍처·JetPack 버전에
종속되므로 다른 컴퓨터에서 그대로 활성화하지 말고, 새 Jetson에서는 시스템
패키지와 JetPack에 맞춰 다시 설치해야 합니다.

`optional_drivers/pl2303_l4t_r36_5/`는 PL2303 드라이버 빌드 산출물과 소스입니다.
이미 커널에 설치된 드라이버를 자동으로 다시 설치하지 않으며, USB-TTL이
인식되지 않을 때만 참고합니다.
