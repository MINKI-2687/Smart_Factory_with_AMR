"""
lidar_diagnostic.py
라이다 통신이 안 될 때 원인을 좁히기 위한 최소 진단 스크립트.
main.py 전체를 안 돌리고, 라이다 하나만 떼어내서 확인.

사용법: python3 lidar_diagnostic.py [포트이름]
  예: python3 lidar_diagnostic.py /dev/ttyUSB1
  인자 없이 실행하면 사용 가능한 포트 목록만 보여줌.
"""
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("pyserial이 설치되어 있지 않습니다. 'pip install pyserial' 먼저 실행하세요.")
    sys.exit(1)


def list_ports():
    print("=== 현재 연결된 시리얼 포트 목록 ===")
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("  (아무 포트도 안 잡힘 - USB 케이블/드라이버 자체를 확인해야 함)")
        return []
    for p in ports:
        print(f"  {p.device}  -  {p.description}  (hwid: {p.hwid})")
    return [p.device for p in ports]


def raw_dump(port_name, baudrate=460800, duration=3.0):
    print(f"\n=== {port_name} @ {baudrate}bps 로 {duration}초간 raw 수신 확인 ===")
    print("(아무것도 안 보내고, 그냥 라이다가 뭘 흘려보내는지만 관찰)")
    try:
        ser = serial.Serial(port_name, baudrate=baudrate, timeout=0.5)
    except Exception as e:
        print(f"  포트 열기 실패: {type(e).__name__}: {e}")
        print("  -> 포트 이름이 틀렸거나, 권한 문제(dialout 그룹)이거나, 다른 프로그램이 이미 점유 중일 수 있음")
        return

    total = b""
    start = time.monotonic()
    while time.monotonic() - start < duration:
        chunk = ser.read(64)
        if chunk:
            total += chunk

    print(f"  {duration}초 동안 총 {len(total)}바이트 수신됨")
    if len(total) == 0:
        print("  >> 단 1바이트도 안 옴. 다음을 확인하세요:")
        print("     1) 라이다에 전원(5V)이 실제로 들어가고 있는지 (모터 도는 소리/LED로 확인)")
        print("     2) TX/RX 배선이 서로 뒤바뀌어 있지 않은지 (라이다TX->어댑터RX, 라이다RX->어댑터TX)")
        print("     3) 포트 이름이 정말 이 장치가 맞는지 (list_ports 결과와 대조)")
        print("     4) baudrate가 460800이 맞는지 (모델에 따라 다를 수 있음, 박스/라벨 재확인)")
    else:
        print(f"  처음 32바이트(hex): {total[:32].hex().upper()}")
        print("  >> 뭔가 오긴 오는 상태. 프로토콜 정렬 문제일 가능성이 높음(아래 GET_HEALTH 테스트로 확인)")

    ser.close()


def test_get_health(port_name, baudrate=460800):
    print(f"\n=== {port_name} 에 실제로 GET_HEALTH 요청을 보내고 응답 확인 ===")
    sys.path.insert(0, '.')
    try:
        from rplidar_c1 import build_get_health, parse_response_descriptor, parse_health_response
    except ImportError:
        print("  rplidar_c1.py를 같은 폴더에서 못 찾음 - 이 스크립트를 프로젝트 폴더 안에서 실행하세요")
        return

    ser = serial.Serial(port_name, baudrate=baudrate, timeout=0.5)
    ser.reset_input_buffer()

    request = build_get_health()
    print(f"  보내는 바이트: {request.hex().upper()}")
    ser.write(request)

    time.sleep(0.2)
    raw = ser.read(64)  # 응답서술자(7) + 헬스응답(3) = 10바이트 예상, 넉넉히 64바이트 읽어봄
    print(f"  받은 바이트({len(raw)}개): {raw.hex().upper() if raw else '(없음)'}")

    if len(raw) >= 7:
        descriptor = parse_response_descriptor(raw[:7])
        print(f"  응답서술자 파싱: {descriptor}")
        if descriptor and len(raw) >= 10:
            health = parse_health_response(raw[7:10])
            print(f"  헬스 응답 파싱: {health}")
            if health:
                print("  >> GET_HEALTH 성공! 라이다 통신 자체는 정상입니다.")
    else:
        print("  >> 응답이 너무 짧거나 없음 - 여전히 통신 문제")

    ser.close()


if __name__ == "__main__":
    available = list_ports()

    if len(sys.argv) < 2:
        print("\n사용법: python3 lidar_diagnostic.py <포트이름>")
        print(f"예: python3 lidar_diagnostic.py {available[0] if available else '/dev/ttyUSB1'}")
        sys.exit(0)

    port = sys.argv[1]
    raw_dump(port, duration=3.0)
    test_get_health(port)
