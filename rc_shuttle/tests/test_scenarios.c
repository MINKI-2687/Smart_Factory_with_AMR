#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "slam.h"
#include "frontier_exploration.h"
#include "goal_pid.h"
#include "obstacle_avoidance.h"
#include "safe_navigator.h"
#include "docking.h"
#include "obstacle_list.h"
#include "astar.h"
#include "planning_pipeline.h"
#include "dynamic_navigator.h"
#include "lidar_clustering.h"
#include "detected_obstacle.h"

/* ============================================================
 * 공용 가상환경: 원형 장애물 + 사각 벽. 시나리오마다 다른 배치를 줌.
 * ============================================================ */
typedef struct { double x, y, r; } Circle;

#define MAX_OBST 16
typedef struct {
    const char *name;
    double room_size;
    Circle obstacles[MAX_OBST];      /* "실제 세계" 장애물 (라이다가 실제로 보는 것) */
    int n_obstacles;
    Circle known_obstacles[MAX_OBST]; /* 고정맵 모드에서 A*가 "미리 아는" 장애물
                                          (SLAM모드에서는 안 씀) - 실제와 다르면
                                          "예상치 못한 장애물" 테스트가 됨 */
    int n_known_obstacles;
    double start_x, start_y, start_theta;
    double goal_x, goal_y, goal_theta_deg;  /* has_goal_theta는 프로젝트 규칙상 항상 true */
    bool use_slam;                    /* true: 프론티어 매핑부터. false: 고정맵 + A* + 재탐색 */
} Scenario;

static double raycast(const Circle *obstacles, int n_obst, double room_size,
                       double rx, double ry, double angle, double max_range) {
    double best = max_range;
    double dirx = cos(angle), diry = sin(angle);

    if (dirx > 1e-9) { double t = (room_size - rx) / dirx; if (t > 0 && t < best) best = t; }
    if (dirx < -1e-9) { double t = (0.0 - rx) / dirx; if (t > 0 && t < best) best = t; }
    if (diry > 1e-9) { double t = (room_size - ry) / diry; if (t > 0 && t < best) best = t; }
    if (diry < -1e-9) { double t = (0.0 - ry) / diry; if (t > 0 && t < best) best = t; }

    for (int i = 0; i < n_obst; i++) {
        double cx = obstacles[i].x - rx, cy = obstacles[i].y - ry;
        double proj = cx * dirx + cy * diry;
        if (proj < 0) continue;
        double perp2 = (cx * cx + cy * cy) - proj * proj;
        double r2 = obstacles[i].r * obstacles[i].r;
        if (perp2 > r2) continue;
        double t = proj - sqrt(r2 - perp2);
        if (t > 0 && t < best) best = t;
    }
    return best;
}

static void simulate_lidar(const Circle *obstacles, int n_obst, double room_size,
                            double rx, double ry, double rtheta, LidarScan *out) {
    out->count = 0;
    for (double a_deg = -180.0; a_deg < 180.0; a_deg += 2.0) {
        double world_angle = rtheta + a_deg * ANGLE_PI / 180.0;
        double dist = raycast(obstacles, n_obst, room_size, rx, ry, world_angle, 3.0);
        if (dist >= 2.999) continue;
        if (out->count < LIDAR_MAX_READINGS) {
            out->readings[out->count].angle_deg = a_deg;
            out->readings[out->count].dist_mm = dist * 1000.0;
            out->count++;
        }
    }
}

/* ---------------- 결과 구조체 ---------------- */
typedef struct {
    bool mapping_or_plan_ok;
    bool nav_done;
    double final_x, final_y, final_theta;
    double pos_error, heading_error_deg;
    int mapping_steps, nav_steps;
    int frontier_target_changes;
    bool pass;
    char reason[256];
} ScenarioResult;

#define K_V 0.01
#define K_W 0.01
#define DT (1.0 / 20.0)
#define POS_TOLERANCE 0.15      /* 최종 위치 허용오차 */
#define HEADING_TOLERANCE_DEG 20.0 /* 최종 주차각도 허용오차 (docking 자체 15도 + sim 근사오차 여유) */

static ScenarioResult run_scenario(const Scenario *sc) {
    ScenarioResult res;
    memset(&res, 0, sizeof(res));

    double true_x = sc->start_x, true_y = sc->start_y, true_theta = sc->start_theta;
    unsigned char *known_grid = NULL;
    int rows = 0, cols = 0;
    double resolution = 0.05;

    /* ---------- 1단계: 지도 확보 (SLAM 매핑 또는 고정맵) ---------- */
    if (sc->use_slam) {
        OccupancyGridSLAM slam;
        slam_init(&slam, sc->room_size, sc->room_size, resolution, 0.0, 0.0,
                  true_x, true_y, true_theta, 0.15, 12.0, 0.03, 3.0);

        FrontierExplorer fe;
        frontier_explorer_init(&fe, 3);
        fe.stuck_timeout_steps = 20 * 10;

        int max_steps = 20 * 60 * 5;
        int step;
        for (step = 0; step < max_steps; step++) {
            LidarScan scan;
            simulate_lidar(sc->obstacles, sc->n_obstacles, sc->room_size, true_x, true_y, true_theta, &scan);
            slam_update_map(&slam, true_x, true_y, true_theta, &scan);

            double dist_to_target = fe.has_target ? hypot(true_x - fe.target_x, true_y - fe.target_y) : 1e300;
            bool reached_current = fe.has_target && dist_to_target < 0.4;
            bool changed = frontier_explorer_step(&fe, &slam, true_x, true_y, reached_current);
            if (changed) res.frontier_target_changes++;

            if (frontier_explorer_done(&fe)) break;

            if (fe.has_target) {
                GoalPIDController goal_ctrl; goal_pid_init_default(&goal_ctrl, 0.4);
                ObstacleAvoidance avoider; obstacle_avoidance_init_default(&avoider);
                SafeNavigator local_ctrl; safe_navigator_init(&local_ctrl, &goal_ctrl, &avoider);
                WheelCmd cmd = safe_navigator_compute(&local_ctrl, true_x, true_y, true_theta,
                                                       fe.target_x, fe.target_y, &scan);
                double linear = (cmd.left + cmd.right) / 2.0;
                double angular = (cmd.right - cmd.left) / 2.0;
                double v = linear * K_V, w = angular * K_W;
                true_x += v * cos(true_theta) * DT;
                true_y += v * sin(true_theta) * DT;
                true_theta = normalize_angle(true_theta + w * DT);
            }
        }
        res.mapping_steps = step;
        if (step >= max_steps) {
            snprintf(res.reason, sizeof(res.reason), "매핑 단계 타임아웃 (5분 동안 탐사 안 끝남)");
        }

        known_grid = slam_to_binary_grid(&slam, 0.6);
        rows = slam.rows; cols = slam.cols;
        slam_free(&slam);
    } else {
        ObstacleList olist;
        obstacle_list_init(&olist);
        for (int i = 0; i < sc->n_known_obstacles; i++) {
            obstacle_list_add_if_new(&olist, (Obstacle){ sc->known_obstacles[i].x, sc->known_obstacles[i].y,
                                                           sc->known_obstacles[i].r });
        }
        known_grid = make_grid(sc->room_size, sc->room_size, resolution, 0.0, 0.0, &olist, &rows, &cols);
        obstacle_list_free(&olist);
        res.mapping_steps = 0;
    }

    /* ---------- 2단계: A* 경로계획 + PID추종 + 재탐색 + 도킹(주차각도 필수) ---------- */
    DynamicNavigator nav;
    dynnav_init(&nav, known_grid, rows, cols, resolution, 0.10, 0.0, 0.0);
    free(known_grid);

    double goal_theta = sc->goal_theta_deg * ANGLE_PI / 180.0;
    bool planned = dynnav_set_goal(&nav, true_x, true_y, sc->goal_x, sc->goal_y,
                                    /*has_goal_theta=*/true, goal_theta, NULL, NULL, 0);
    res.mapping_or_plan_ok = planned;
    if (!planned) {
        snprintf(res.reason, sizeof(res.reason), "A* 경로계획 실패 (알고 있는 지도로 목표 도달 불가)");
        res.final_x = true_x; res.final_y = true_y; res.final_theta = true_theta;
        dynnav_free(&nav);
        res.pass = false;
        return res;
    }

    int max_nav_steps = 20 * 180; /* 3분 */
    int nstep;
    bool done = false;
    for (nstep = 0; nstep < max_nav_steps; nstep++) {
        LidarScan scan;
        simulate_lidar(sc->obstacles, sc->n_obstacles, sc->room_size, true_x, true_y, true_theta, &scan);

        ClusterResult cluster_result;
        cluster_lidar_scan(&scan, true_x, true_y, true_theta, 0.15, 2, 3.0, &cluster_result);
        DetectedObstacle detected[CLUSTER_MAX_CLUSTERS];
        int n_detected = cluster_result_to_detected(&cluster_result, detected, CLUSTER_MAX_CLUSTERS);

        WheelCmd cmd = dynnav_update(&nav, true_x, true_y, true_theta, &scan,
                                      n_detected > 0 ? detected : NULL, n_detected);
        if (cmd.done) { done = true; break; }

        double linear = (cmd.left + cmd.right) / 2.0;
        double angular = (cmd.right - cmd.left) / 2.0;
        double v = linear * K_V, w = angular * K_W;
        true_x += v * cos(true_theta) * DT;
        true_y += v * sin(true_theta) * DT;
        true_theta = normalize_angle(true_theta + w * DT);
    }
    res.nav_steps = nstep;
    res.nav_done = done;
    res.final_x = true_x; res.final_y = true_y; res.final_theta = true_theta;
    res.pos_error = hypot(true_x - sc->goal_x, true_y - sc->goal_y);
    double heading_err_rad = normalize_angle(true_theta - goal_theta);
    res.heading_error_deg = fabs(heading_err_rad) * 180.0 / ANGLE_PI;

    dynnav_free(&nav);

    if (!done) {
        snprintf(res.reason, sizeof(res.reason), "내비게이션 타임아웃 (3분 동안 도킹 완료 안 됨), pos_err=%.3f",
                 res.pos_error);
        res.pass = false;
        return res;
    }
    if (res.pos_error > POS_TOLERANCE) {
        snprintf(res.reason, sizeof(res.reason), "도착판정 났지만 위치오차 초과: %.3fm > %.3fm",
                 res.pos_error, POS_TOLERANCE);
        res.pass = false;
        return res;
    }
    if (res.heading_error_deg > HEADING_TOLERANCE_DEG) {
        snprintf(res.reason, sizeof(res.reason), "위치는 맞았지만 주차각도 오차 초과: %.1f도 > %.1f도",
                 res.heading_error_deg, HEADING_TOLERANCE_DEG);
        res.pass = false;
        return res;
    }

    snprintf(res.reason, sizeof(res.reason), "정상 완료: pos_err=%.3fm, heading_err=%.1f도",
             res.pos_error, res.heading_error_deg);
    res.pass = true;
    return res;
}

int main(void) {
    Scenario scenarios[16];
    int n = 0;

    /* ===== SLAM 매핑 시나리오 ===== */

    /* S1: 회귀테스트 - 지난번과 같은 배치 (장애물 3개), 주차각도 90도 */
    scenarios[n++] = (Scenario){
        .name = "S1_SLAM_기본3장애물_주차90도",
        .room_size = 3.0,
        .obstacles = {{1.00,1.00,0.20},{1.75,2.00,0.20},{2.00,0.90,0.20}}, .n_obstacles = 3,
        .start_x=0.3,.start_y=0.3,.start_theta=0.0,
        .goal_x=2.5,.goal_y=2.5,.goal_theta_deg=90.0,
        .use_slam = true,
    };

    /* S2: 장애물 1개(성긴 환경) - 원래 알려진 한계, 나쁜 결과가 "정상"이라 참고용으로만 확인 */
    scenarios[n++] = (Scenario){
        .name = "S2_SLAM_장애물1개_성긴환경_주차0도",
        .room_size = 3.0,
        .obstacles = {{1.5,1.5,0.2}}, .n_obstacles = 1,
        .start_x=0.3,.start_y=0.3,.start_theta=0.0,
        .goal_x=2.5,.goal_y=2.5,.goal_theta_deg=0.0,
        .use_slam = true,
    };

    /* S3: 장애물 8개, 크기 다양(작은 것/큰 것 섞임), 주차각도 180도 */
    scenarios[n++] = (Scenario){
        .name = "S3_SLAM_장애물8개_크기다양_주차180도",
        .room_size = 3.0,
        .obstacles = {
            {0.6,0.6,0.10},{1.2,0.5,0.15},{0.5,1.5,0.10},{1.5,1.5,0.30},
            {2.2,0.6,0.10},{2.4,1.6,0.15},{1.0,2.3,0.10},{2.0,2.3,0.20}
        }, .n_obstacles = 8,
        .start_x=0.3,.start_y=0.3,.start_theta=0.0,
        .goal_x=2.6,.goal_y=2.6,.goal_theta_deg=180.0,
        .use_slam = true,
    };

    /* S4: 좁은 통로(장애물 2개가 통로처럼 좁게 막음), 주차각도 -90도 */
    scenarios[n++] = (Scenario){
        .name = "S4_SLAM_좁은통로_주차-90도",
        .room_size = 3.0,
        .obstacles = {{1.5,0.9,0.35},{1.5,2.1,0.35}}, .n_obstacles = 2,
        .start_x=0.3,.start_y=1.5,.start_theta=0.0,
        .goal_x=2.7,.goal_y=1.5,.goal_theta_deg=-90.0,
        .use_slam = true,
    };

    /* S5: 직사각형 큰 방(4m x 2m), 장애물 4개, 주차각도 45도 */
    scenarios[n++] = (Scenario){
        .name = "S5_SLAM_직사각형방_장애물4개_주차45도",
        .room_size = 4.0,  /* room_size는 정사각형 가정이라, 이 시나리오는 4x4로 취급됨(참고용) */
        .obstacles = {{1.0,1.0,0.15},{3.0,1.0,0.15},{1.0,3.0,0.15},{3.0,3.0,0.15}}, .n_obstacles = 4,
        .start_x=0.3,.start_y=0.3,.start_theta=0.0,
        .goal_x=3.6,.goal_y=3.6,.goal_theta_deg=45.0,
        .use_slam = true,
    };

    /* ===== 고정맵 + 예상치 못한 장애물 시나리오 (A*가 미리 알던 지도와 실제가 다름) ===== */

    /* F1: 고정맵 회귀테스트 - 아는 지도 = 실제 지도, 장애물 없이 순수 A*+도킹만 검증 */
    scenarios[n++] = (Scenario){
        .name = "F1_고정맵_기본_주차90도",
        .room_size = 3.0,
        .obstacles = {{1.5,1.5,0.2}}, .n_obstacles = 1,
        .known_obstacles = {{1.5,1.5,0.2}}, .n_known_obstacles = 1,
        .start_x=0.3,.start_y=0.3,.start_theta=0.0,
        .goal_x=2.5,.goal_y=2.5,.goal_theta_deg=90.0,
        .use_slam = false,
    };

    /* F2: 예상치 못한 장애물 1개 - A*는 빈 방으로 알고 최단경로(대각선)를 짜지만,
     *     실제로는 그 경로 한가운데에 장애물이 있어서 반드시 재탐색해야 도달 가능 */
    scenarios[n++] = (Scenario){
        .name = "F2_고정맵_예상치못한장애물1개_주차0도",
        .room_size = 3.0,
        .obstacles = {{1.5,1.5,0.3}}, .n_obstacles = 1,   /* 실제로는 있음 */
        .known_obstacles = {{99,99,0.01}}, .n_known_obstacles = 1, /* A*는 모름(맵 밖 더미) */
        .start_x=0.3,.start_y=0.3,.start_theta=0.0,
        .goal_x=2.7,.goal_y=2.7,.goal_theta_deg=0.0,
        .use_slam = false,
    };

    /* F3: 예상치 못한 장애물 3개(여러 개) - 재탐색이 반복적으로 잘 되는지 */
    scenarios[n++] = (Scenario){
        .name = "F3_고정맵_예상치못한장애물3개_주차90도",
        .room_size = 3.0,
        .obstacles = {{1.0,1.0,0.2},{1.5,1.5,0.2},{2.0,2.0,0.2}}, .n_obstacles = 3,
        .known_obstacles = {{99,99,0.01}}, .n_known_obstacles = 1,
        .start_x=0.3,.start_y=0.3,.start_theta=0.0,
        .goal_x=2.7,.goal_y=2.7,.goal_theta_deg=90.0,
        .use_slam = false,
    };

    printf("총 %d개 시나리오 실행\n\n", n);
    int pass_count = 0;
    for (int i = 0; i < n; i++) {
        ScenarioResult r = run_scenario(&scenarios[i]);
        printf("[%s]\n", scenarios[i].name);
        printf("  경로계획 성공: %s\n", r.mapping_or_plan_ok ? "예" : "아니오");
        if (scenarios[i].use_slam) {
            printf("  매핑: %d스텝, 목표전환 %d회\n", r.mapping_steps, r.frontier_target_changes);
        }
        printf("  내비게이션 완료(도킹포함): %s (스텝 %d)\n", r.nav_done ? "예" : "아니오", r.nav_steps);
        printf("  최종위치=(%.3f,%.3f) 목표=(%.3f,%.3f) 위치오차=%.3fm\n",
               r.final_x, r.final_y, scenarios[i].goal_x, scenarios[i].goal_y, r.pos_error);
        printf("  최종각도=%.1f도 목표각도=%.1f도 각도오차=%.1f도\n",
               r.final_theta * 180.0 / ANGLE_PI, scenarios[i].goal_theta_deg, r.heading_error_deg);
        printf("  결과: %s -- %s\n\n", r.pass ? "PASS" : "FAIL", r.reason);
        if (r.pass) pass_count++;
    }

    printf("=== 총괄: %d/%d 통과 ===\n", pass_count, n);
    return (pass_count == n) ? 0 : 1;
}
