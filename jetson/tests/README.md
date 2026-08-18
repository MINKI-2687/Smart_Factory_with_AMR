# 릴리스 테스트

상위 폴더에서 `./verify.sh`를 실행하면 런타임 Python 문법과 다음 동작을
검증합니다.

- FPGA 8바이트 패킷의 XOR 체크섬·좌표 범위·NONE 검증
- OpenRB 9바이트 명령과 ACK/DONE 응답 파서
- 도형→열→가장 낮은 빈 슬롯 선택
- 카메라 5프레임 합의와 fail-closed 안전 게이트

테스트는 실제 `/dev/ttyUSB*`, 카메라, TensorRT 엔진을 사용하지 않습니다.
