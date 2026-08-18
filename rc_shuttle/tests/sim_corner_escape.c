#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif
/* ============================================================
 * sim_corner_escape.c  (2026-08-10b)
 *
 * 실기 없이 robot_runner__corner_escape() / corner_backoff() 를 '진짜 그대로'
 * 돌려보는 폐루프 시뮬레이터.
 *
 * 흉내내는 것은 두 가지뿐이다.
 *   - 라이다: set_map.txt 에 광선투사해서 LidarThread.latest_scan 을 직접 채운다.
 *   - 바퀴  : FPGALink.current_left/right 를 읽어 미분구동으로 자세를 적분한다.
 * 탈출 로직 자체는 실기와 완전히 같은 코드가 돈다(복제본이 아님).
 *
 * 빌드: gcc -std=c11 -O2 -Wall -Wextra -I. -o sim_corner_escape \
 *              sim_corner_escape.c -lm -lpthread
 * ============================================================ */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <pthread.h>
#include "robot_runner.h"

/* ---------------- 지도 ---------------- */
static unsigned char *g_grid = NULL;
static int g_rows = 0, g_cols = 0;
static double g_res = 0.01;

static bool load_map(const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) return false;
    if (fscanf(f, "%d %d %lf", &g_rows, &g_cols, &g_res) != 3) { fclose(f); return false; }
    g_grid = (unsigned char *)calloc((size_t)g_rows * g_cols, 1);
    char *line = (char *)malloc((size_t)g_cols + 8);
    for (int r = 0; r < g_rows; r++) {
        if (fscanf(f, "%s", line) != 1) { fclose(f); free(line); return false; }
        for (int c = 0; c < g_cols; c++) g_grid[r * g_cols + c] = (line[c] == '1');
    }
    free(line); fclose(f);
    return true;
}

static bool occupied(double x, double y) {
    int c = (int)floor(x / g_res), r = (int)floor(y / g_res);
    if (r < 0 || r >= g_rows || c < 0 || c >= g_cols) return true;
    return g_grid[r * g_cols + c] != 0;
}

/* ---------------- 시뮬레이터 상태 ---------------- */
static pthread_mutex_t g_pose_lock = PTHREAD_MUTEX_INITIALIZER;
static double g_x, g_y, g_th;              /* 참값 자세 */
static volatile bool g_sim_running = false;
static FPGALink *g_link = NULL;
static LidarThread *g_lt = NULL;
static double g_path_len = 0.0;            /* 총 이동거리(직진 성분) */
static double g_turn_total = 0.0;          /* 총 회전량 */
static bool   g_hit_wall = false;          /* 차체가 벽을 파고들었는가 */

/* 차체 사각형이 벽과 겹치는지 - '더 박았는지'를 감시하기 위한 것 */
static bool body_overlaps(double x, double y, double th, double hl, double hw) {
    for (double u = -hl; u <= hl + 1e-9; u += 0.02)
        for (double v = -hw; v <= hw + 1e-9; v += 0.02) {
            double px = x + u * cos(th) - v * sin(th);
            double py = y + u * sin(th) + v * cos(th);
            if (occupied(px, py)) return true;
        }
    return false;
}

/* speed(1~80) -> 바퀴 선속도(m/s). robot_runner__linear_speed 와 같은 모델을 쓰되,
 * '실제는 모델보다 2배 빠르다'는 실기 관측(선속도 배율 주석)을 반영한다. */
static double wheel_mps(int sp, const RobotConfig *cfg) {
    if (sp == 0) return 0.0;
    int a = abs(sp);
    char sign; int pwm;
    speed_to_pwm(a, cfg->controller_max_speed, cfg->pwm_safety_cap, &sign, &pwm);
    double v = ((double)pwm / cfg->pwm_safety_cap) * ROBOT_MAX_WHEEL_MPS * 2.0;
    return (sp > 0) ? v : -v;
}

static void raycast_into(LidarScan *s, double x, double y, double th) {
    const int N = 240;
    s->count = 0;
    for (int i = 0; i < N && s->count < LIDAR_MAX_READINGS; i++) {
        double ab = -180.0 + 360.0 * i / N;
        double a = th + ab * ANGLE_PI / 180.0;
        double d;
        for (d = 0.02; d < 3.0; d += 0.004)
            if (occupied(x + d * cos(a), y + d * sin(a))) break;
        if (d >= 3.0) continue;
        s->readings[s->count].angle_deg = ab;
        s->readings[s->count].dist_mm = d * 1000.0;
        s->count++;
    }
}

static RobotConfig g_cfg_copy;

static void *sim_loop(void *arg) {
    (void)arg;
    const double DT = 0.010;
    double scan_acc = 0.0;
    const double HL = 0.5 * g_cfg_copy.body_length, HW = 0.5 * g_cfg_copy.body_width;
    while (g_sim_running) {
        int l, r;
        pthread_mutex_lock(&g_link->lock);
        l = g_link->current_left; r = g_link->current_right;
        pthread_mutex_unlock(&g_link->lock);

        double vl = wheel_mps(l, &g_cfg_copy), vr = wheel_mps(r, &g_cfg_copy);
        double v = 0.5 * (vl + vr);
        double w = (vr - vl) / g_cfg_copy.wheel_separation;

        pthread_mutex_lock(&g_pose_lock);
        double nx = g_x + v * cos(g_th) * DT;
        double ny = g_y + v * sin(g_th) * DT;
        double nth = normalize_angle(g_th + w * DT);
        /* 벽을 뚫고 지나가지는 못한다 - 겹치면 그 성분은 막힌다(실기와 비슷하게) */
        if (!body_overlaps(nx, ny, nth, HL, HW)) {
            g_path_len += fabs(v) * DT;
            g_turn_total += fabs(w) * DT;
            g_x = nx; g_y = ny; g_th = nth;
        } else if (!body_overlaps(g_x, g_y, nth, HL, HW)) {
            g_turn_total += fabs(w) * DT;      /* 회전만 허용 */
            g_th = nth;
        } else {
            g_hit_wall = true;                  /* 밀어붙이고 있음 */
        }
        double cx = g_x, cy = g_y, cth = g_th;
        pthread_mutex_unlock(&g_pose_lock);

        scan_acc += DT;
        if (scan_acc >= 0.05) {                 /* 20Hz 로 새 스캔 */
            scan_acc = 0.0;
            LidarScan s;
            raycast_into(&s, cx, cy, cth);
            pthread_mutex_lock(&g_lt->lock);
            g_lt->latest_scan = s;
            g_lt->scan_count++;
            pthread_mutex_unlock(&g_lt->lock);
        }
        struct timespec ts = {0, (long)(DT * 1e9)};
        nanosleep(&ts, NULL);
    }
    return NULL;
}

static void run_case(const char *name, RobotConfig *cfg,
                     double x0, double y0, double th0_deg, double target_deg) {
    const double HL = 0.5 * cfg->body_length, HW = 0.5 * cfg->body_width;
    g_x = x0; g_y = y0; g_th = th0_deg * ANGLE_PI / 180.0;
    g_path_len = 0.0; g_turn_total = 0.0; g_hit_wall = false;

    FPGALink link;
    fpga_link_init(&link, -1, cfg->controller_max_speed, cfg->pwm_safety_cap,
                    cfg->wheel_diameter, cfg->wheel_separation, cfg->ticks_per_rev,
                    cfg->fpga_send_hz);
    LidarThread lt;
    memset(&lt, 0, sizeof(lt));
    pthread_mutex_init(&lt.lock, NULL);
    lt.latest_scan.count = 0;
    lt.scan_count = 0;

    g_link = &link; g_lt = &lt; g_cfg_copy = *cfg;
    g_sim_running = true;
    pthread_t th;
    pthread_create(&th, NULL, sim_loop, NULL);
    robot_runner__sleep_sec(0.20);          /* 첫 스캔이 들어올 때까지 */

    double want = normalize_angle(target_deg * ANGLE_PI / 180.0 - g_th);
    printf("\n================ %s ================\n", name);
    printf("시작 자세 (%.3f, %.3f, %+.0fdeg), 목표 방향 %+.0fdeg -> 돌아야 할 각 %+.0fdeg\n",
           x0, y0, th0_deg, target_deg, want * 180.0 / ANGLE_PI);
    printf("시작 시 차체-벽 겹침: %s\n",
           body_overlaps(g_x, g_y, g_th, HL, HW) ? "있음(이미 물림)" : "없음");

    double moved = 0.0;
    bool ok = robot_runner__corner_escape(&link, &lt, cfg, want, 0.008,
                                           "시뮬", &moved);

    g_sim_running = false;
    pthread_join(th, NULL);

    LidarScan fin;
    raycast_into(&fin, g_x, g_y, g_th);
    double rem = normalize_angle(target_deg * ANGLE_PI / 180.0 - g_th);
    double gap = robot_runner__rot_gap_shifted(&fin, 0.0, rem, HL, HW, NULL);
    printf("---- 결과: %s\n", ok ? "탈출 성공" : "탈출 실패");
    printf("   끝난 자세 (%.3f, %.3f, %+.0fdeg)  [이동 %.0fmm, 회전 %.0fdeg]\n",
           g_x, g_y, g_th * 180.0 / ANGLE_PI, g_path_len * 1000.0,
           g_turn_total * 180.0 / ANGLE_PI);
    printf("   남은회전 %+.0fdeg 여유 %+.1fmm, 직진 가능 앞 %.0f / 뒤 %.0f mm\n",
           rem * 180.0 / ANGLE_PI, gap * 1000.0,
           robot_runner__axis_free_dist(&fin, +1, HL, HW, 0.010) * 1000.0,
           robot_runner__axis_free_dist(&fin, -1, HL, HW, 0.010) * 1000.0);
    printf("   탈출 도중 벽에 밀어붙인 적: %s\n", g_hit_wall ? "있음" : "없음");

    fpga_link_close(&link);
    pthread_mutex_destroy(&lt.lock);
}

int main(void) {
    if (!load_map("set_map.txt")) { fprintf(stderr, "set_map.txt 를 못 읽었습니다\n"); return 1; }

    RobotConfig cfg;
    robot_config_defaults(&cfg);
    cfg.body_length = 0.255; cfg.body_width = 0.130;
    cfg.robot_radius = 0.14;
    cfg.slot_parking = true;
    cfg.slot_staging_y = 0.6475;
    cfg.sg_settle_sec = 0.12;
    cfg.has_encoder = false;
    cfg.lidar_offset_forward_m = 0.0;   /* 스캔을 이미 차체중심에서 쏘고 있으므로 0 */
    cfg.lidar_mirror = false;

    printf("지도 %.2f x %.2f m, 차체 %.0fx%.0fmm (반대각선 %.1fmm)\n",
           g_cols * g_res, g_rows * g_res,
           cfg.body_length * 1000.0, cfg.body_width * 1000.0,
           500.0 * hypot(cfg.body_length, cfg.body_width));

    run_case("실기 끼임 자세 (구석)",      &cfg, 1.320, 0.710, -43.0, -90.0);
    run_case("B대기점 +30mm 넘침",        &cfg, 1.305, 0.6475,  0.0, -90.0);
    run_case("통로 아래벽에 붙은 자세",    &cfg, 1.275, 0.560,   0.0, -90.0);

    free(g_grid);
    return 0;
}
