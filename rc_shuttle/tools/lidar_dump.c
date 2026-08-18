/* lidar_dump.c
 * 라이다만 읽어서 live_view.py가 알아먹는 SCAN 줄을 계속 출력함.
 * FPGA도 안 열고 모터도 절대 안 건드림 - 로봇을 손으로 들고 돌려보면서 "라이다가
 * 제대로 도는가 / 방이 어떻게 보이는가"만 확인하는 용도(RoboStudio로 보던 것과 같은
 * 것을 우리 코드 경로로 보는 것). 라이다 읽기 코드가 정상인지 격리해서 확인할 수
 * 있으므로, 문제가 라이다인지 아닌지 가르는 데 씀.
 *
 * 빌드: gcc -std=c11 -O2 -o lidar_dump lidar_dump.c -lm -lpthread
 * 실행: ./lidar_dump /dev/ttyUSB1 460800
 *       ./lidar_dump /dev/ttyUSB1 460800 | python3 live_view.py --polar
 *
 * 출력 형식(live_view.py와 동일):
 *   SCAN <점개수> <x1> <y1> <x2> <y2> ...      (로봇 원점 기준 미터 단위)
 *   STATE navigate <스텝> 0 0 0 0 0 0          (자세는 항상 원점/0도 - 위치추정 안 함)
 */
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include <math.h>

#include "serial_port.h"
#include "rplidar_reader.h"
#include "lidar_thread.h"
#include "angle_utils.h"
#include "lidar_types.h"

static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int sig) { (void)sig; g_stop = 1; }

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "사용법: %s <lidar장치> [보드레이트, 기본 460800]\n", argv[0]);
        fprintf(stderr, "  예: %s /dev/ttyUSB1 460800\n", argv[0]);
        fprintf(stderr, "  예: %s /dev/ttyUSB1 | python3 live_view.py --polar\n", argv[0]);
        return 1;
    }
    const char *dev = argv[1];
    int baud = (argc >= 3) ? atoi(argv[2]) : 460800;

    signal(SIGINT, on_sigint);

    fprintf(stderr, "[lidar_dump] 포트 여는 중: %s @ %d baud (모터는 건드리지 않음)\n", dev, baud);
    int fd = serial_port_open(dev, baud);
    if (fd < 0) {
        fprintf(stderr, "[lidar_dump] 포트 열기 실패\n");
        return 1;
    }

    RplidarReader reader;
    rplidar_reader_init(&reader, fd, 2.0, 3);
    LidarThread lidar;
    lidar_thread_start(&lidar, &reader);
    fprintf(stderr, "[lidar_dump] 스캔 시작 (Ctrl+C로 종료)\n");

    long last_count = -1;
    long printed = 0;
    while (!g_stop) {
        pthread_mutex_lock(&lidar.lock);
        long c = lidar.scan_count;
        LidarScan scan = lidar.latest_scan;
        long errs = lidar.error_count;
        pthread_mutex_unlock(&lidar.lock);

        if (c != last_count && scan.count > 0) {
            last_count = c;
            /* 자세는 원점/0도 고정 - 이 프로그램은 위치추정을 하지 않음 */
            printf("STATE navigate %ld 0.0000 0.0000 0.0000 0 0 0\n", printed);
            printf("SCAN %d", scan.count);
            for (int i = 0; i < scan.count; i++) {
                double a = scan.readings[i].angle_deg * ANGLE_PI / 180.0;
                double d = scan.readings[i].dist_mm / 1000.0;
                printf(" %.3f %.3f", d * cos(a), d * sin(a));
            }
            printf("\n");
            fflush(stdout);

            if (printed % 10 == 0) {
                fprintf(stderr, "[lidar_dump] 스캔 %ld회, 이번 %d점, 오류 %ld회\n",
                        c, scan.count, errs);
            }
            printed++;
        }

        struct timespec ts = {0, 20 * 1000000L};
        nanosleep(&ts, NULL);
    }

    fprintf(stderr, "\n[lidar_dump] 종료 중...\n");
    lidar_thread_stop(&lidar);
    serial_port_close(fd);
    fprintf(stderr, "[lidar_dump] 종료 완료\n");
    return 0;
}
