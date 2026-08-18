/* motor_test.c
 * 10진수 M패킷으로 PWM 200과 207(최대)을 각각 보내보는 테스트.
 * FPGALink(송신스레드) 안 거치고, 여기서 직접 바이트를 만들어서 write()하고
 * 그 내용을 화면에 그대로 찍음 - 뭘 보내는지 100% 투명하게 보려는 목적.
 *
 * 빌드: gcc -std=c11 -O2 -o motor_test motor_test.c -lm
 * 실행: ./motor_test <fpga장치> <fpga보드레이트>
 *   예: ./motor_test /dev/ttyUSB2 115200
 *
 * 순서: [1] "M +170 +170\n"을 3초간 반복 전송 (새 데드존 하한값)
 *       [2] 2초 정지
 *       [3] "M +255 +255\n"을 3초간 반복 전송 (최대PWM)
 *       [4] 정지
 */
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "serial_port.h"

static void sleep_ms(int ms) {
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (long)(ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

static void send_and_print(int fd, const char *label, const char *cmd) {
    ssize_t w = write(fd, cmd, strlen(cmd));
    fprintf(stderr, "[motor_test][%s] 전송(%zd바이트): \"", label, w);
    for (const char *p = cmd; *p; p++) {
        if (*p == '\n') fprintf(stderr, "\\n");
        else fputc(*p, stderr);
    }
    fprintf(stderr, "\"\n");
}

static void send_stop(int fd) {
    const char *stop_cmd = "M +000 +000\n";
    ssize_t w = write(fd, stop_cmd, strlen(stop_cmd));
    (void)w;
}

int main(int argc, char **argv) {
    if (argc != 3) {
        fprintf(stderr, "사용법: %s <fpga장치> <fpga보드레이트>\n", argv[0]);
        fprintf(stderr, "  예: %s /dev/ttyUSB2 115200\n", argv[0]);
        return 1;
    }
    const char *device = argv[1];
    int baud = atoi(argv[2]);

    int fd = serial_port_open(device, baud);
    if (fd < 0) {
        fprintf(stderr, "[motor_test] 포트 열기 실패\n");
        return 1;
    }

    printf("[motor_test] 20Hz로 3초간 반복 전송하며 실제 바퀴 움직임을 눈으로 확인해주세요.\n");
    printf("[motor_test] 로봇이 갑자기 움직일 수 있으니 바퀴가 허공에 뜨게 들어두거나 충분한 공간을 확보하세요.\n\n");

    /* ---- 1단계: PWM 200 (요청하신 최소기준값) ---- */
    printf("=== [1단계] \"M +180 +180\\n\" (새 데드존 하한) - 3초간 전송 ===\n");
    const char *cmd_170 = "M +180 +180\n";
    for (int i = 0; i < 60; i++) {  /* 20Hz * 3초 */
        send_and_print(fd, "(+) PWM180", cmd_170);
        sleep_ms(50);
    }
    send_stop(fd);
    printf("=== [1단계] 종료, 정지 명령 전송함 ===\n\n");

    sleep_ms(2000);

    /* ---- 2단계: 최대 PWM(207) ---- */
    printf("=== [2단계] \"M +200 -180\\n\" (최대) - 3초간 전송 ===\n");
    const char *cmd_max = "M +200 -180\n";
    for (int i = 0; i < 60; i++) {
        send_and_print(fd, "(-) PWM180", cmd_max);
        sleep_ms(50);
    }
    send_stop(fd);
    printf("=== [2단계] 종료, 정지 명령 전송함 ===\n\n");

    serial_port_close(fd);
    printf("[motor_test] 완료. 170에서도 잘 돌았는지, 255랑 체감차이 있는지 알려주세요.\n");
    return 0;
}

