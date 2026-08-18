#include "serial_port.h"


/* ============================================================
 * serial_port_open: 리눅스(라즈베리파이) 시리얼포트를 열어서 fd 반환.
 * fpga_serial.h/rplidar_reader.h는 "이미 열려있는 fd"만 받게 설계돼 있어서(테스트용
 * socketpair도 그대로 넣을 수 있게), 실기에서 실제 /dev/ttyUSB0 같은 포트를 여는
 * 코드가 여기 하나만 있으면 됨 - main.c류 실행파일들이 공유해서 씀.
 * 실패 시 -1 반환.
 * ============================================================ */
int serial_port_open(const char *device, int baud) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        fprintf(stderr, "[serial_port] failed to open %s: %s\n", device, strerror(errno));
        return -1;
    }

    /* PATCH (2026-08-08): 같은 포트를 두 프로세스가 동시에 여는 것을 막음.
     *
     * 리눅스는 시리얼 장치를 여러 프로세스가 동시에 열 수 있게 허용함. 그러면
     *  - 라이다: 바이트 스트림을 두 프로세스가 나눠 가져가서 양쪽 다 찢어진 스캔을
     *            받음 -> 스캔 모양이 뭉개지고 자세추정이 발산함
     *  - FPGA:  두 프로세스가 서로 다른 모터 명령을 번갈아 보내서 로봇이 제자리에서
     *            앞뒤로 진동함
     * 실기에서 이전 실행이 안 죽고 남아 있는 상태로 재실행했을 때 정확히 이 증상이
     * 나왔음(한 스텝에 각도가 136도 튀는 등, 단일 프로세스로는 불가능한 로그).
     * flock은 프로세스가 죽으면 자동 해제되므로 정리 부담도 없음. */
    if (flock(fd, LOCK_EX | LOCK_NB) < 0) {
        fprintf(stderr,
            "[serial_port] %s 를 다른 프로세스가 이미 쓰고 있습니다.\n"
            "              이전 실행이 안 죽고 남아 있을 가능성이 큽니다. 확인:\n"
            "                  ps aux | grep main_shuttle\n"
            "                  pkill -f main_shuttle\n"
            "              (두 프로세스가 같은 포트를 열면 라이다 데이터가 반씩 찢기고\n"
            "               모터 명령이 서로 덮어써서 로봇이 제자리에서 진동합니다.)\n",
            device);
        close(fd);
        return -1;
    }
    ioctl(fd, TIOCEXCL);   /* 이후의 open()도 거부하게 함 */

    /* NONBLOCK으로 열었으니 이후 read()가 블로킹 없이 즉시 리턴 -
     * fpga_serial.h/rplidar_reader.h가 자체적으로 폴링+짧은 슬립으로 재시도하는
     * 구조라 이렇게 맞춰야 함(그 파일들 안 고치고 그대로 재사용하려는 목적) */

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        fprintf(stderr, "[serial_port] tcgetattr failed on %s: %s\n", device, strerror(errno));
        close(fd);
        return -1;
    }

    speed_t speed;
    switch (baud) {
        case 9600:    speed = B9600;    break;
        case 19200:   speed = B19200;   break;
        case 38400:   speed = B38400;   break;
        case 57600:   speed = B57600;   break;
        case 115200:  speed = B115200;  break;
        case 230400:  speed = B230400;  break;
        case 460800:  speed = B460800;  break;
        case 921600:  speed = B921600;  break;
        default:
            fprintf(stderr, "[serial_port] unsupported baud rate %d "
                             "(9600/19200/38400/57600/115200/230400/460800/921600 중 하나 쓰세요)\n", baud);
            close(fd);
            return -1;
    }
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    /* raw 모드를 직접 설정(cfmakeraw는 배포판에 따라 feature-test 매크로가 더 필요해서
     * _POSIX_C_SOURCE만 정의된 환경에서 선언이 안 보일 수 있음 - 그냥 직접 플래그를 끔) */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_cflag &= ~(CSIZE | PARENB);
    tty.c_cflag |= CS8;

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSTOPB;    /* 스탑비트 1개 */
#ifdef CRTSCTS
    tty.c_cflag &= ~CRTSCTS;   /* 하드웨어 흐름제어 없음 (지원 안 하는 환경이면 그냥 생략) */
#endif

    tty.c_cc[VMIN] = 0;        /* NONBLOCK과 함께 쓰므로 이 값 자체는 큰 의미 없음 */
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "[serial_port] tcsetattr failed on %s: %s\n", device, strerror(errno));
        close(fd);
        return -1;
    }

    tcflush(fd, TCIOFLUSH);    /* 열자마자 남아있을 수 있는 쓰레기 데이터 비움 */
    return fd;
}


void serial_port_close(int fd) {
    if (fd >= 0) close(fd);
}
