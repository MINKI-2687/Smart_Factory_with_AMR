#!/usr/bin/env bash
# run_wander.sh
# wander_avoid 실행을 도와주는 스크립트. 시리얼 장치가 몇 번인지 매번 헷갈리기 쉬워서,
# 실행 전에 /dev/ttyUSB*, /dev/ttyACM*를 먼저 보여주고 확인 후 실행함.
#
# 사용법:
#   ./run_wander.sh                                   # 장치 자동탐색 + 확인 후 실행
#   ./run_wander.sh /dev/ttyUSB0 115200 /dev/ttyUSB1 460800   # 직접 지정해서 바로 실행

set -e

BIN="$(dirname "$0")/wander_avoid"

if [ ! -x "$BIN" ]; then
    echo "[run_wander] wander_avoid 실행파일이 없습니다. 먼저 빌드하세요:"
    echo "  make wander   (또는)  gcc -std=c11 -O2 -o wander_avoid wander_avoid.c -lm -lpthread"
    exit 1
fi

if [ "$#" -eq 4 ]; then
    FPGA_DEV="$1"; FPGA_BAUD="$2"; LIDAR_DEV="$3"; LIDAR_BAUD="$4"
else
    echo "=== 연결된 시리얼 장치 목록 ==="
    ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "  (USB/ACM 시리얼 장치가 안 보입니다 - 케이블/전원 확인해주세요)"
    echo ""
    echo "FPGA와 라이다 중 어느 게 어느 장치인지 확인 안 됐으면,"
    echo "케이블을 하나씩 뽑았다 꽂아보면서 위 목록에서 사라졌다 나타나는 걸 보면 구분됩니다."
    echo ""
    read -rp "FPGA 장치 경로 (예: /dev/ttyUSB0): " FPGA_DEV
    read -rp "FPGA 보드레이트 (기본 115200, 실측 필요): " FPGA_BAUD
    FPGA_BAUD="${FPGA_BAUD:-115200}"
    read -rp "라이다 장치 경로 (예: /dev/ttyUSB1): " LIDAR_DEV
    read -rp "라이다 보드레이트 (기본 460800, 실측 필요): " LIDAR_BAUD
    LIDAR_BAUD="${LIDAR_BAUD:-460800}"
fi

echo ""
echo "=== 실행 ==="
echo "$BIN $FPGA_DEV $FPGA_BAUD $LIDAR_DEV $LIDAR_BAUD"
echo "(Ctrl+C로 안전하게 정지+종료됩니다)"
echo ""
exec "$BIN" "$FPGA_DEV" "$FPGA_BAUD" "$LIDAR_DEV" "$LIDAR_BAUD"
