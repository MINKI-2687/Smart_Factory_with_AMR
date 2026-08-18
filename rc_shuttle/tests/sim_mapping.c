#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "slam.h"
#include "frontier_exploration.h"
#include "goal_pid.h"
#include "obstacle_avoidance.h"
#include "safe_navigator.h"

/* ---- 가상 환경: 3x3m 방, 벽 + 원형 장애물 3개(이미지 속 배치와 비슷하게) ---- */
typedef struct { double x, y, r; } Circle;
static Circle g_obstacles[3] = {
    {1.00, 1.00, 0.20},
    {1.75, 2.00, 0.20},
    {2.00, 0.90, 0.20},
};
#define N_OBST 3
#define ROOM_SIZE 3.0

/* 한 방향 레이가 벽/장애물에 부딪히는 거리(m)를 계산. 못 맞으면 max_range 반환(음수 아님, 그냥 max) */
static double raycast(double rx, double ry, double angle, double max_range) {
    double best = max_range;
    double dirx = cos(angle), diry = sin(angle);

    /* 벽 4개 (world 0..3, 0..3) */
    if (dirx > 1e-9) { double t = (ROOM_SIZE - rx) / dirx; if (t > 0 && t < best) best = t; }
    if (dirx < -1e-9) { double t = (0.0 - rx) / dirx; if (t > 0 && t < best) best = t; }
    if (diry > 1e-9) { double t = (ROOM_SIZE - ry) / diry; if (t > 0 && t < best) best = t; }
    if (diry < -1e-9) { double t = (0.0 - ry) / diry; if (t > 0 && t < best) best = t; }

    /* 원형 장애물들 (레이-원 교차, 가장 가까운 양의 해) */
    for (int i = 0; i < N_OBST; i++) {
        double cx = g_obstacles[i].x - rx, cy = g_obstacles[i].y - ry;
        double proj = cx * dirx + cy * diry;
        if (proj < 0) continue;
        double perp2 = (cx * cx + cy * cy) - proj * proj;
        double r2 = g_obstacles[i].r * g_obstacles[i].r;
        if (perp2 > r2) continue;
        double t = proj - sqrt(r2 - perp2);
        if (t > 0 && t < best) best = t;
    }
    return best;
}

/* 가상 라이다 스캔 생성 (2도 간격, ±180도) */
static void simulate_lidar(double rx, double ry, double rtheta, LidarScan *out) {
    out->count = 0;
    for (double a_deg = -180.0; a_deg < 180.0; a_deg += 2.0) {
        double world_angle = rtheta + a_deg * ANGLE_PI / 180.0;
        double dist = raycast(rx, ry, world_angle, 3.0);
        if (dist >= 2.999) continue;  /* 범위 밖: 리딩 없음(실제 라이다처럼) */
        if (out->count < LIDAR_MAX_READINGS) {
            out->readings[out->count].angle_deg = a_deg;
            out->readings[out->count].dist_mm = dist * 1000.0;
            out->count++;
        }
    }
}

int main(void) {
    OccupancyGridSLAM slam;
    slam_init(&slam, ROOM_SIZE, ROOM_SIZE, 0.05, 0.0, 0.0, 0.3, 0.3, 0.0, 0.15, 12.0, 0.03, 3.0);

    /* SLAM 보정 없이 "완벽한 오도메트리"를 준다고 가정(SLAM 드리프트와 프론티어버그를
     * 분리해서 프론티어/내비게이션 로직 자체만 검증하기 위함) */
    double true_x = 0.3, true_y = 0.3, true_theta = 0.0;

    GoalPIDController goal_ctrl;
    goal_pid_init_default(&goal_ctrl, 0.4);
    ObstacleAvoidance avoider;
    obstacle_avoidance_init_default(&avoider);
    SafeNavigator local_ctrl;
    safe_navigator_init(&local_ctrl, &goal_ctrl, &avoider);

    FrontierExplorer fe;
    frontier_explorer_init(&fe, 3);
    fe.stuck_timeout_steps = 20 * 10; /* 10초 */

    const double dt = 0.05; /* 20Hz */
    const double K_V = 0.01, K_W = 0.01; /* PID출력(단위없음) -> 실제 m/s, rad/s 환산 스케일 (프로젝트 노트의 "느린배속" 컨셉과 동일 취지) */

    int target_change_count = 0;
    int max_steps = 20 * 60 * 5; /* 5분 분량 */
    int step;
    double min_dist_to_far_corner = 1e9;

    for (step = 0; step < max_steps; step++) {
        LidarScan scan;
        simulate_lidar(true_x, true_y, true_theta, &scan);

        /* 지도만 갱신(완벽한 오도메트리 가정이라 스캔매칭 보정 불필요) */
        slam_update_map(&slam, true_x, true_y, true_theta, &scan);

        double dist_to_target = fe.has_target ? hypot(true_x - fe.target_x, true_y - fe.target_y) : 1e300;
        bool reached_current = fe.has_target && dist_to_target < 0.4;

        bool changed = frontier_explorer_step(&fe, &slam, true_x, true_y, reached_current);
        if (changed) {
            target_change_count++;
            if (target_change_count <= 30) {
                printf("[step %5d] NEW target=(%.3f,%.3f) prev_dist=%.3f avoid_hist=%d miss=%d\n",
                       step, fe.target_x, fe.target_y, dist_to_target, fe.avoid_history_count, fe.miss_count);
            }
        }

        if (frontier_explorer_done(&fe)) {
            printf("\n[step %5d] EXPLORATION DONE (miss_count=%d >= max_miss=%d)\n", step, fe.miss_count, fe.max_miss);
            break;
        }

        if (fe.has_target) {
            WheelCmd cmd = safe_navigator_compute(&local_ctrl, true_x, true_y, true_theta,
                                                   fe.target_x, fe.target_y, &scan);
            double linear = (cmd.left + cmd.right) / 2.0;
            double angular = (cmd.right - cmd.left) / 2.0;
            double v = linear * K_V, w = angular * K_W;
            true_x += v * cos(true_theta) * dt;
            true_y += v * sin(true_theta) * dt;
            true_theta = normalize_angle(true_theta + w * dt);
        }

        double d_far = hypot(true_x - 2.7, true_y - 2.7);
        if (d_far < min_dist_to_far_corner) min_dist_to_far_corner = d_far;
    }

    if (step >= max_steps) printf("\n[step %5d] TIMED OUT (5분 동안 탐사 안 끝남)\n", step);

    printf("\n=== 요약 ===\n");
    printf("총 목표 전환 횟수: %d\n", target_change_count);
    printf("최종 로봇 위치: (%.3f, %.3f)\n", true_x, true_y);
    printf("반대편 코너(2.7,2.7)에 가장 가까이 간 거리: %.3f m (0에 가까울수록 그쪽을 탐사한 것)\n", min_dist_to_far_corner);

    slam_free(&slam);
    return 0;
}
