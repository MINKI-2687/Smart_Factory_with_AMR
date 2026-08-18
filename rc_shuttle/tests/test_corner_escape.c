#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif
/* ============================================================
 * test_corner_escape.c  (2026-08-10b)
 *
 * 실기 없이 '구석 끼임' 관련 기하 판정만 검증하는 오프라인 테스트.
 * set_map.txt 를 읽어 광선투사(raycast)로 가짜 라이다 스캔을 만들고,
 *   (1) 예전 ensure_rotation_room 의 방향 선택이 왜 벽으로 밀었는지
 *   (2) 새 best_axis_shift 가 반대로(벽에서 멀어지게) 고르는지
 *   (3) axis_free_dist 가 부채꼴보다 얼마나 정확한지
 * 를 숫자로 확인한다.
 *
 * 빌드: gcc -std=c11 -O2 -Wall -Wextra -I. -o test_corner_escape \
 *              test_corner_escape.c -lm -lpthread
 * ============================================================ */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "robot_runner.h"

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
    free(line);
    fclose(f);
    return true;
}

static bool occupied(double x, double y) {
    int c = (int)floor(x / g_res), r = (int)floor(y / g_res);
    if (r < 0 || r >= g_rows || c < 0 || c >= g_cols) return true;
    return g_grid[r * g_cols + c] != 0;
}

/* 자세 (x,y,th) 에서 360도 라이다 스캔을 만든다. 각도는 이미 '차체 기준'이다
 * (apply_lidar_frame_fix 를 통과한 뒤와 같은 규약: 0도=정면, +90도=왼쪽). */
static void make_scan(LidarScan *s, double x, double y, double th, int n) {
    s->count = 0;
    for (int i = 0; i < n && s->count < LIDAR_MAX_READINGS; i++) {
        double a_body = -180.0 + 360.0 * i / n;
        double a = th + a_body * ANGLE_PI / 180.0;
        double d = 0.0;
        for (d = 0.02; d < 3.0; d += 0.002) {
            if (occupied(x + d * cos(a), y + d * sin(a))) break;
        }
        if (d >= 3.0) continue;
        s->readings[s->count].angle_deg = a_body;
        s->readings[s->count].dist_mm = d * 1000.0;
        s->count++;
    }
}

/* 예전 ensure_rotation_room 의 방향 선택 규칙 (비교용으로 그대로 옮겨 적음) */
static int old_dir_rule(double th, double out_sign_y) {
    double fwd_y = sin(th) * out_sign_y;
    return (fwd_y >= 0.0) ? +1 : -1;
}

static void report(const char *name, RobotConfig *cfg, double x, double y, double th_deg) {
    const double HL = 0.5 * cfg->body_length, HW = 0.5 * cfg->body_width;
    double th = th_deg * ANGLE_PI / 180.0;
    LidarScan scan;
    make_scan(&scan, x, y, th, 460);

    double turn = normalize_angle(-90.0 * ANGLE_PI / 180.0 - th);   /* 슬롯(-90도)으로 */
    double gdir = 0.0;
    double gap0 = robot_runner__rot_gap_shifted(&scan, 0.0, turn, HL, HW, &gdir);

    double cone_f = robot_runner__sector_min(&scan, 0.0, 25.0);
    double cone_r = robot_runner__sector_min(&scan, 180.0, 25.0);
    double rect_f = robot_runner__axis_free_dist(&scan, +1, HL, HW, 0.010);
    double rect_r = robot_runner__axis_free_dist(&scan, -1, HL, HW, 0.010);

    int od = old_dir_rule(th, +1.0);
    double sgap = gap0;
    double back_lim = rect_r - 0.012, fwd_lim = rect_f - 0.012;
    if (back_lim > 0.060) back_lim = 0.060;
    if (fwd_lim  > 0.060) fwd_lim  = 0.060;
    double s = robot_runner__best_axis_shift(&scan, turn, HL, HW,
                                              back_lim, fwd_lim, 0.015, &sgap);

    printf("---- %s : pose=(%.3f, %.3f, %+.0fdeg), 돌 각도 %+.0fdeg\n",
           name, x, y, th_deg, turn * 180.0 / ANGLE_PI);
    printf("   회전 여유            : %+.1f mm  (막힌 방향 %+.0fdeg)\n", gap0 * 1000.0, gdir);
    printf("   부채꼴(+-25도) 여유  : 앞 %.1f / 뒤 %.1f mm  (벽까지 거리)\n",
           cone_f * 1000.0, cone_r * 1000.0);
    printf("   직사각형 스윕 가능   : 앞 %.1f / 뒤 %.1f mm  (차체 앞끝부터)\n",
           rect_f * 1000.0, rect_r * 1000.0);
    printf("   예전 규칙이 고르는 쪽: %s\n", od > 0 ? "전진" : "후진");
    printf("   새 규칙이 고르는 쪽  : %s %.0f mm -> 여유 %+.1f mm\n",
           (s > 0) ? "전진" : (s < 0 ? "후진" : "이동없음"), fabs(s) * 1000.0, sgap * 1000.0);

    /* 예전 규칙대로 갔다면 여유가 어떻게 되는지도 같이 보여준다 */
    double old_step = (od > 0 ? +1.0 : -1.0) * 0.030;
    double old_gap = robot_runner__rot_gap_shifted(&scan, old_step, turn, HL, HW, NULL);
    printf("   예전 규칙대로 30mm %s -> 여유 %+.1f mm  (%s)\n\n",
           od > 0 ? "전진" : "후진", old_gap * 1000.0,
           (old_gap < gap0) ? "나빠짐" : "좋아짐");
}

int main(void) {
    if (!load_map("set_map.txt")) { fprintf(stderr, "set_map.txt 를 못 읽었습니다\n"); return 1; }
    printf("지도: %d x %d, %.0f mm/칸  ->  실제 %.2f x %.2f m\n\n",
           g_rows, g_cols, g_res * 1000.0, g_cols * g_res, g_rows * g_res);

    RobotConfig cfg;
    robot_config_defaults(&cfg);
    cfg.body_length = 0.255; cfg.body_width = 0.130;
    cfg.robot_radius = 0.14;

    printf("차체 %.0f x %.0f mm -> 반길이 %.1f, 반폭 %.1f, 반대각선 %.1f mm\n",
           cfg.body_length * 1000.0, cfg.body_width * 1000.0,
           500.0 * cfg.body_length, 500.0 * cfg.body_width,
           500.0 * hypot(cfg.body_length, cfg.body_width));
    printf("=> 90도 제자리 회전에 필요한 사방 여유 = %.1f mm + 안전여유\n\n",
           500.0 * hypot(cfg.body_length, cfg.body_width));

    /* B 대기점 정위치 (통로를 따라 +x 로 달려온 직후라 heading 0도) */
    report("B대기점 정위치", &cfg, 1.275, 0.6475, 0.0);
    /* 접근 상자(+-30mm) 끝까지 넘어간 경우 - 실기에서 자주 나오는 자세 */
    report("B대기점 +30mm 넘침", &cfg, 1.305, 0.6475, 0.0);
    /* 로그/화면에서 실제로 물렸던 자세 */
    report("실기 끼임 자세", &cfg, 1.320, 0.710, -43.0);
    /* A 대기점 (반대쪽 벽) - 대칭인지 확인 */
    report("A대기점 정위치", &cfg, 0.225, 0.6475, 180.0);

    free(g_grid);
    return 0;
}
