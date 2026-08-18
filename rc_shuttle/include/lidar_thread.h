#ifndef LIDAR_THREAD_H
#define LIDAR_THREAD_H
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif

#include <pthread.h>
#include <time.h>
#include <stdbool.h>
#include "rplidar_reader.h"
#include "lidar_types.h"

/* ============================================================
 * LidarThread: 백그라운드에서 계속 라이다를 읽어서 "최신 스캔"을 들고 있는 스레드.
 * robot_runner.h(전체 SLAM+내비게이션 파이프라인)와 이 라이다만 쓰는 간단한 테스트
 * 실행파일들이 전부 이 로직을 공유해야 해서 별도 헤더로 뺌 - 안 그러면 파일마다
 * 복붙하다가 한쪽만 고쳐지는 사고가 남(이 프로젝트에서 실제로 몇 번 겪었던 패턴).
 * ============================================================ */
typedef struct {
    RplidarReader *reader;
    pthread_mutex_t lock;
    LidarScan latest_scan;
    volatile bool running;
    pthread_t thread;
    long scan_count, error_count;
} LidarThread;
bool lidar_thread_should_continue(void *ctx);
void *lidar_thread_loop(void *arg);
void lidar_thread_start(LidarThread *lt, RplidarReader *reader);
void lidar_thread_get_latest(LidarThread *lt, LidarScan *out);
void lidar_thread_stop(LidarThread *lt);

#endif /* LIDAR_THREAD_H */
