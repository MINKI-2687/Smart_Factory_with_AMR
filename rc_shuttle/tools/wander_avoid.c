/* wander_avoid.c
 * 엔코더/오도메트리를 전혀 안 쓰고, 라이다만으로 장애물을 피하며 계속 돌아다니는
 * 순수 반응형(reactive) 테스트. 목적지도 없고 위치추정도 안 함 - "라이다+FPGA+모터가
 * 하드웨어적으로 잘 붙어있는지"만 빠르게 확인하기 위한 최소 구성.
 *
 * PATCH (2026-08-04): 원래는 obstacle_avoidance.h의 compute_repulsion()을 그대로
 * 썼는데, 그건 danger_distance=0.1m로 반응거리가 너무 짧고(다른 곳(도킹/A*국소회피)
 * 에서 쓰는 값이라 여기서 임의로 못 바꿈), 정면에 대칭으로 장애물이 있으면 좌우 반발력이
 * 서로 상쇄돼서 "제자리에서 좌우로 흔들리기만 하고 못 지나가는" 문제가 실측으로 확인됨.
 *
 * 대신 이 파일 안에서 직접 좌/우 창(윈도우)의 평균 개방거리를 비교해서 "어느 쪽이 더
 * 트였는지" 명시적으로 판단하고, 한 번 회피방향을 정하면(SEEK->AVOID 전환) 최소
 * AVOID_MIN_STEPS 스텝 + 전방이 충분히 트일 때까지(히스테리시스) 그 방향을 계속
 * 유지함 - 애매하게 대칭이라 방향이 계속 뒤집히는 걸 막음.
 *
 * 빌드: gcc -std=c11 -O2 -o wander_avoid wander_avoid.c -lm -lpthread
 * 실행: ./wander_avoid <fpga장치> <fpga보드레이트> <lidar장치> <lidar보드레이트>
 *   예: ./wander_avoid /dev/ttyUSB0 115200 /dev/ttyUSB1 460800
 * (보드레이트는 실측 확인 안 된 추정값입니다 - 실제 하드웨어 스펙에 맞게 바꿔서 쓰세요)
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
#include "fpga_serial.h"
#include "rplidar_reader.h"
#include "lidar_thread.h"
#include "angle_utils.h"
#include "lidar_types.h"

typedef struct {
    int controller_max_speed;
    int pwm_safety_cap;
    double wheel_diameter, wheel_separation;
    int ticks_per_rev;
    int control_hz;

    double base_forward_speed;
    double max_angular_speed;
    double avoid_angular_speed;   /* 회피모드에서 확실하게 도는 각속도(최대치 근처) */

    /* 반응 거리(m) - 실측으로 더 늘리거나 줄여도 됨. 라이다 자체 감지거리(하드웨어
     * 스펙상 훨씬 멂, RPLidar C1은 보통 최대 12m급)와는 다른 개념 - 이건 "몇 m 앞에서부터
     * 반응할지"를 우리가 정하는 소프트웨어 임계값임. */
    double front_half_width_deg;  /* 정면 판정 각도범위(중심 기준 절반폭) */
    double side_center_deg;       /* 좌/우 판정창 중심각(좌=+side_center, 우=-side_center) */
    double side_half_width_deg;   /* 좌/우 판정창 폭(절반) */
    double avoid_enter_dist;      /* 전방이 이보다 가까우면 회피모드 진입 */
    double avoid_exit_dist;       /* 전방이 이보다 멀어지고 최소유지스텝 지나면 seek 복귀 */
    double caution_dist;          /* seek 모드에서도 이 거리 안쪽이면 미리 살짝 트인쪽으로 꺾기 시작 */
    int avoid_min_steps;          /* 회피모드 최소 유지 스텝(진동 방지) */
    double max_range_m;           /* 판정 윈도우에 아무 리딩도 없으면(=그 방향엔 아무 것도
                                    * 안 걸림) "이만큼은 트였다"고 가정할 기본값 */
} WanderConfig;

static void wander_config_defaults(WanderConfig *cfg) {
    cfg->controller_max_speed = 80;
    cfg->pwm_safety_cap = 255;
    cfg->wheel_diameter = 0.066;
    cfg->wheel_separation = 0.160;
    cfg->ticks_per_rev = 20;
    cfg->control_hz = 20;

    cfg->base_forward_speed = 40.0;
    cfg->max_angular_speed = 30.0;
    cfg->avoid_angular_speed = 28.0;

    cfg->front_half_width_deg = 25.0;
    cfg->side_center_deg = 60.0;
    cfg->side_half_width_deg = 45.0;
    cfg->avoid_enter_dist = 0.6;
    cfg->avoid_exit_dist = 0.9;
    cfg->caution_dist = 1.3;
    cfg->avoid_min_steps = 20;   /* 20Hz 기준 1초 */
    cfg->max_range_m = 3.0;
}

typedef enum { WANDER_SEEK, WANDER_AVOID } WanderState;

static double angle_diff_deg(double a, double b) {
    double d = a - b;
    while (d > 180.0) d -= 360.0;
    while (d < -180.0) d += 360.0;
    return d;
}

/* 윈도우(center_deg ± half_width_deg) 안에 있는 리딩 중 최솟값(m). 리딩이 하나도
 * 없으면(=그 방향엔 아무것도 안 걸림) max_range_m을 그대로 반환(=트여있다고 봄). */
static double window_min_dist_m(const LidarScan *scan, double center_deg, double half_width_deg,
                                 double max_range_m) {
    double best = max_range_m;
    for (int i = 0; i < scan->count; i++) {
        if (fabs(angle_diff_deg(scan->readings[i].angle_deg, center_deg)) > half_width_deg) continue;
        double d = scan->readings[i].dist_mm / 1000.0;
        if (d < best) best = d;
    }
    return best;
}

/* 윈도우 안 리딩들의 평균거리(m). 리딩이 하나도 없으면 max_range_m 반환. */
static double window_avg_dist_m(const LidarScan *scan, double center_deg, double half_width_deg,
                                 double max_range_m) {
    double sum = 0.0;
    int n = 0;
    for (int i = 0; i < scan->count; i++) {
        if (fabs(angle_diff_deg(scan->readings[i].angle_deg, center_deg)) > half_width_deg) continue;
        sum += scan->readings[i].dist_mm / 1000.0;
        n++;
    }
    if (n == 0) return max_range_m;
    return sum / n;
}

static volatile sig_atomic_t g_stop_requested = 0;
static void handle_sigint(int sig) { (void)sig; g_stop_requested = 1; }

int main(int argc, char **argv) {
    if (argc != 5) {
        fprintf(stderr, "사용법: %s <fpga장치> <fpga보드레이트> <lidar장치> <lidar보드레이트>\n", argv[0]);
        fprintf(stderr, "  예: %s /dev/ttyUSB0 115200 /dev/ttyUSB1 460800\n", argv[0]);
        fprintf(stderr, "  (보드레이트는 실측 확인 안 된 추정값입니다 - 실제 스펙에 맞게 바꿔서 쓰세요)\n");
        return 1;
    }
    const char *fpga_device = argv[1];
    int fpga_baud = atoi(argv[2]);
    const char *lidar_device = argv[3];
    int lidar_baud = atoi(argv[4]);

    WanderConfig cfg;
    wander_config_defaults(&cfg);

    signal(SIGINT, handle_sigint);

    printf("[wander] FPGA 포트 여는 중: %s @ %d baud\n", fpga_device, fpga_baud);
    int fpga_fd = serial_port_open(fpga_device, fpga_baud);
    if (fpga_fd < 0) {
        fprintf(stderr, "[wander] FPGA 포트 열기 실패, 종료\n");
        return 1;
    }

    printf("[wander] 라이다 포트 여는 중: %s @ %d baud\n", lidar_device, lidar_baud);
    int lidar_fd = serial_port_open(lidar_device, lidar_baud);
    if (lidar_fd < 0) {
        fprintf(stderr, "[wander] 라이다 포트 열기 실패, 종료\n");
        serial_port_close(fpga_fd);
        return 1;
    }

    FPGALink fpga;
    fpga_link_init(&fpga, fpga_fd, cfg.controller_max_speed, cfg.pwm_safety_cap,
                    cfg.wheel_diameter, cfg.wheel_separation, cfg.ticks_per_rev, cfg.control_hz);
    fpga_link_start(&fpga);
    printf("[wander] FPGA 링크 시작됨\n");

    RplidarReader reader;
    rplidar_reader_init(&reader, lidar_fd, 2.0, 3);
    LidarThread lidar;
    lidar_thread_start(&lidar, &reader);
    printf("[wander] 라이다 스캔 시작됨\n");

    printf("[wander] 반응형 회피주행 시작 (Ctrl+C로 종료)\n");
    printf("[wander] 설정: 정면판정=+-%.0fdeg, 좌우판정중심=+-%.0fdeg(폭+-%.0fdeg), "
           "회피진입=%.2fm, 회피해제=%.2fm, 주의거리=%.2fm\n",
           cfg.front_half_width_deg, cfg.side_center_deg, cfg.side_half_width_deg,
           cfg.avoid_enter_dist, cfg.avoid_exit_dist, cfg.caution_dist);

    WanderState state = WANDER_SEEK;
    int avoid_turn_dir = 1;      /* +1: 좌회전(양의 각속도값), -1: 우회전 */
    int avoid_steps_committed = 0;

    double control_period = 1.0 / cfg.control_hz;
    long step = 0;

    while (!g_stop_requested) {
        LidarScan scan;
        lidar_thread_get_latest(&lidar, &scan);

        double front = window_min_dist_m(&scan, 0.0, cfg.front_half_width_deg, cfg.max_range_m);
        double left = window_avg_dist_m(&scan, cfg.side_center_deg, cfg.side_half_width_deg, cfg.max_range_m);
        double right = window_avg_dist_m(&scan, -cfg.side_center_deg, cfg.side_half_width_deg, cfg.max_range_m);

        if (state == WANDER_SEEK) {
            if (front < cfg.avoid_enter_dist) {
                state = WANDER_AVOID;
                avoid_turn_dir = (left > right) ? 1 : -1;
                avoid_steps_committed = 0;
            }
        } else { /* WANDER_AVOID */
            avoid_steps_committed++;
            if (front > cfg.avoid_exit_dist && avoid_steps_committed >= cfg.avoid_min_steps) {
                state = WANDER_SEEK;
            }
        }

        int left_cmd, right_cmd;
        if (state == WANDER_AVOID) {
            /* 확실하게 트인 쪽으로 제자리 회전(전진 없음) - "빙글 돌아서 빈 곳으로" */
            double w = avoid_turn_dir * cfg.avoid_angular_speed;
            left_cmd = (int)lround(-w);
            right_cmd = (int)lround(w);
        } else {
            /* SEEK: 기본 전진 + 가까워지면 트인 쪽으로 미리 살짝 꺾기("회피해서 지나감") */
            double steer = 0.0;
            if (front < cfg.caution_dist) {
                double bias = (left > right) ? 1.0 : -1.0;
                double span = cfg.caution_dist - cfg.avoid_enter_dist;
                double closeness = (span > 1e-6) ? (cfg.caution_dist - front) / span : 1.0;
                if (closeness > 1.0) closeness = 1.0;
                if (closeness < 0.0) closeness = 0.0;
                steer = bias * closeness;
            }
            double angular = steer * cfg.max_angular_speed;
            double linear = cfg.base_forward_speed * (1.0 - 0.5 * fabs(steer));
            left_cmd = (int)lround(linear - angular);
            right_cmd = (int)lround(linear + angular);
        }

        fpga_link_set_speed(&fpga, left_cmd, right_cmd);

        if (step % (cfg.control_hz / 2) == 0) {  /* 0.5초마다 상태 출력 */
            long sent, recv, fail;
            fpga_link_get_stats(&fpga, &sent, &recv, &fail);
            fprintf(stderr,
                    "[wander] step=%ld state=%s scan_pts=%d front=%.2fm left=%.2fm right=%.2fm "
                    "L=%d R=%d lidar_scans=%ld lidar_err=%ld fpga_fail=%ld\n",
                    step, (state == WANDER_AVOID) ? "AVOID" : "SEEK ", scan.count, front, left, right,
                    left_cmd, right_cmd, lidar.scan_count, lidar.error_count, fail);
            (void)sent; (void)recv;
        }

        step++;
        struct timespec ts;
        ts.tv_sec = (time_t)control_period;
        ts.tv_nsec = (long)((control_period - ts.tv_sec) * 1e9);
        nanosleep(&ts, NULL);
    }

    printf("[wander] 종료 신호 받음, 정리 중...\n");
    fpga_link_set_speed(&fpga, 0, 0);
    struct timespec ts = {0, 100 * 1000000L};
    nanosleep(&ts, NULL);
    fpga_link_close(&fpga);
    lidar_thread_stop(&lidar);
    serial_port_close(fpga_fd);
    serial_port_close(lidar_fd);
    printf("[wander] 종료 완료\n");
    return 0;
}
