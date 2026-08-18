#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "obstacle_list.h"
#include "astar.h"
#include "planning_pipeline.h"
#include "dynamic_navigator.h"
#include "lidar_clustering.h"
#include "detected_obstacle.h"
#include "fpga_link.h"
#include "slam.h"

#define CONTROLLER_MAX_SPEED 80
#define PWM_SAFETY_CAP 207
#define PWM_DEADZONE_HW 100
#define ENCODER_TICKS_PER_REV 20
#define WHEEL_DIAMETER 0.066
#define WHEEL_SEPARATION 0.160
#define MAX_WHEEL_RAD_S 5.0

typedef struct { double x, y, r; } Circle;

static double raycast(const Circle *obst, int n, double room, double rx, double ry,
                       double angle, double max_range) {
    double best = max_range;
    double dirx = cos(angle), diry = sin(angle);
    if (dirx > 1e-9) { double t = (room - rx) / dirx; if (t > 0 && t < best) best = t; }
    if (dirx < -1e-9) { double t = (0.0 - rx) / dirx; if (t > 0 && t < best) best = t; }
    if (diry > 1e-9) { double t = (room - ry) / diry; if (t > 0 && t < best) best = t; }
    if (diry < -1e-9) { double t = (0.0 - ry) / diry; if (t > 0 && t < best) best = t; }
    for (int i = 0; i < n; i++) {
        double cx = obst[i].x - rx, cy = obst[i].y - ry;
        double proj = cx * dirx + cy * diry;
        if (proj < 0) continue;
        double perp2 = (cx * cx + cy * cy) - proj * proj;
        double r2 = obst[i].r * obst[i].r;
        if (perp2 > r2) continue;
        double t = proj - sqrt(r2 - perp2);
        if (t > 0 && t < best) best = t;
    }
    return best;
}

static void simulate_lidar(const Circle *obst, int n, double room,
                            double rx, double ry, double rtheta, LidarScan *out) {
    out->count = 0;
    for (double a = -180.0; a < 180.0; a += 2.0) {
        double wa = rtheta + a * ANGLE_PI / 180.0;
        double d = raycast(obst, n, room, rx, ry, wa, 3.0);
        if (d >= 2.999) continue;
        if (out->count < LIDAR_MAX_READINGS) {
            out->readings[out->count].angle_deg = a;
            out->readings[out->count].dist_mm = d * 1000.0;
            out->count++;
        }
    }
}

typedef struct {
    double true_x, true_y, true_theta;
    double accum_left_ticks, accum_right_ticks;
    int raw_left_tick, raw_right_tick;
    WrappingOdometry odom;
} DriftSimState;

static void drift_sim_init(DriftSimState *s, double start_x, double start_y, double start_theta) {
    s->true_x = start_x; s->true_y = start_y; s->true_theta = start_theta;
    s->accum_left_ticks = 0; s->accum_right_ticks = 0;
    s->raw_left_tick = 0; s->raw_right_tick = 0;
    wodom_init(&s->odom, WHEEL_DIAMETER, WHEEL_SEPARATION, ENCODER_TICKS_PER_REV);
}

/* out_raw_x/y/theta: 순수 오도메트리(무보정) 누적치 - 여기서 스텝간 델타를 뽑아서
 * slam_localize_step에 "오도메트리 예측값"으로 넘기는 용도로 씀 */
static void drift_sim_step(DriftSimState *s, WheelCmd cmd, double dt,
                            double *out_raw_x, double *out_raw_y, double *out_raw_theta) {
    char sign_l, sign_r; int mag_l, mag_r;
    speed_to_pwm(cmd.left, CONTROLLER_MAX_SPEED, PWM_SAFETY_CAP, &sign_l, &mag_l);
    speed_to_pwm(cmd.right, CONTROLLER_MAX_SPEED, PWM_SAFETY_CAP, &sign_r, &mag_r);

    double pwm_l = (mag_l < PWM_DEADZONE_HW) ? 0.0 : (sign_l == '+' ? mag_l : -mag_l);
    double pwm_r = (mag_r < PWM_DEADZONE_HW) ? 0.0 : (sign_r == '+' ? mag_r : -mag_r);

    double wheel_l_rad_s = (pwm_l / PWM_SAFETY_CAP) * MAX_WHEEL_RAD_S;
    double wheel_r_rad_s = (pwm_r / PWM_SAFETY_CAP) * MAX_WHEEL_RAD_S;

    double v_l = wheel_l_rad_s * (WHEEL_DIAMETER / 2.0);
    double v_r = wheel_r_rad_s * (WHEEL_DIAMETER / 2.0);
    double d_left_m = v_l * dt, d_right_m = v_r * dt;

    double d_center = (d_left_m + d_right_m) / 2.0;
    double dtheta = (d_right_m - d_left_m) / WHEEL_SEPARATION;
    s->true_x += d_center * cos(s->true_theta + dtheta / 2.0);
    s->true_y += d_center * sin(s->true_theta + dtheta / 2.0);
    s->true_theta = normalize_angle(s->true_theta + dtheta);

    double meters_per_tick = (ANGLE_PI * WHEEL_DIAMETER) / ENCODER_TICKS_PER_REV;
    s->accum_left_ticks += d_left_m / meters_per_tick;
    s->accum_right_ticks += d_right_m / meters_per_tick;
    int whole_l = (int)trunc(s->accum_left_ticks);
    int whole_r = (int)trunc(s->accum_right_ticks);
    s->raw_left_tick += whole_l;
    s->raw_right_tick += whole_r;
    s->accum_left_ticks -= whole_l;
    s->accum_right_ticks -= whole_r;

    wodom_update(&s->odom, s->raw_left_tick, s->raw_right_tick, out_raw_x, out_raw_y, out_raw_theta);
}

typedef struct {
    const char *name;
    double room_size;
    Circle obstacles[8];
    int n_obstacles;
    Circle known_obstacles[8];
    int n_known_obstacles;
    double start_x, start_y, start_theta;
    double goal_x, goal_y, goal_theta_deg;
} Scenario;

static void run_scenario(const Scenario *sc, bool use_lidar_correction) {
    ObstacleList olist;
    obstacle_list_init(&olist);
    for (int i = 0; i < sc->n_known_obstacles; i++) {
        obstacle_list_add_if_new(&olist, (Obstacle){ sc->known_obstacles[i].x, sc->known_obstacles[i].y,
                                                       sc->known_obstacles[i].r });
    }
    int rows, cols;
    unsigned char *known_grid = make_grid(sc->room_size, sc->room_size, 0.05, 0.0, 0.0, &olist, &rows, &cols);
    obstacle_list_free(&olist);

    /* PATCH: make_grid는 등록된 장애물만 표시하고 방 벽(테두리)은 지도에 안 넣음 - A*는
     * "격자 밖=막힘"으로 암묵적 처리해서 상관없지만, 라이다는 실제로 벽에서 반사되므로
     * 위치보정용 지도에 벽이 없으면 스캔매칭이 정답 위치에 오히려 감점을 주는 심각한
     * 역효과가 남(실측 확인됨). 벽을 명시적으로 마킹해서 라이다가 보는 실제 세계와
     * 지도를 일치시킴. */
    for (int c = 0; c < cols; c++) {
        known_grid[0 * cols + c] = 1;
        known_grid[(rows - 1) * cols + c] = 1;
    }
    for (int r = 0; r < rows; r++) {
        known_grid[r * cols + 0] = 1;
        known_grid[r * cols + (cols - 1)] = 1;
    }

    DynamicNavigator nav;
    dynnav_init(&nav, known_grid, rows, cols, 0.05, 0.10, 0.0, 0.0);

    /* 라이다 보정용 SLAM: known_grid(A*가 쓰는 지도와 동일한 "이미 아는 지도")를
     * 그대로 log_odds로 채워서 초기화 - 지도는 안 바뀌고 위치추정만 이걸로 계속 보정 */
    OccupancyGridSLAM loc_slam;
    if (use_lidar_correction) {
        slam_init_from_grid(&loc_slam, known_grid, rows, cols, 0.05, 0.0, 0.0,
                             sc->start_x, sc->start_y, sc->start_theta,
                             0.15, 12.0, 0.03, 3.0);
    }
    free(known_grid);

    DriftSimState sim;
    drift_sim_init(&sim, sc->start_x, sc->start_y, sc->start_theta);
    double raw_x = sc->start_x, raw_y = sc->start_y, raw_theta = sc->start_theta;
    double prev_raw_x = raw_x, prev_raw_y = raw_y, prev_raw_theta = raw_theta;
    double est_x = sc->start_x, est_y = sc->start_y, est_theta = sc->start_theta;

    double goal_theta = sc->goal_theta_deg * ANGLE_PI / 180.0;
    bool planned = dynnav_set_goal(&nav, est_x, est_y, sc->goal_x, sc->goal_y, true, goal_theta, NULL, NULL, 0);

    printf("[%s] (%s)\n", sc->name, use_lidar_correction ? "라이다보정 적용" : "순수 오도메트리");
    if (!planned) {
        printf("  A* 경로계획 실패\n\n");
        dynnav_free(&nav);
        if (use_lidar_correction) slam_free(&loc_slam);
        return;
    }

    const double dt = 0.05;
    int max_steps = 20 * 180;
    int step;
    bool done = false;
    for (step = 0; step < max_steps; step++) {
        LidarScan scan;
        simulate_lidar(sc->obstacles, sc->n_obstacles, sc->room_size, sim.true_x, sim.true_y, sim.true_theta, &scan);

        ClusterResult cr;
        cluster_lidar_scan(&scan, est_x, est_y, est_theta, 0.15, 2, 3.0, &cr);
        DetectedObstacle detected[CLUSTER_MAX_CLUSTERS];
        int n_det = cluster_result_to_detected(&cr, detected, CLUSTER_MAX_CLUSTERS);

        WheelCmd cmd = dynnav_update(&nav, est_x, est_y, est_theta, &scan, n_det > 0 ? detected : NULL, n_det);
        if (cmd.done) { done = true; break; }

        drift_sim_step(&sim, cmd, dt, &raw_x, &raw_y, &raw_theta);

        if (use_lidar_correction) {
            double odom_dx = raw_x - prev_raw_x;
            double odom_dy = raw_y - prev_raw_y;
            double odom_dtheta = normalize_angle(raw_theta - prev_raw_theta);
            slam_localize_step(&loc_slam, &scan, odom_dx, odom_dy, odom_dtheta, &est_x, &est_y, &est_theta);
            prev_raw_x = raw_x; prev_raw_y = raw_y; prev_raw_theta = raw_theta;
        } else {
            est_x = raw_x; est_y = raw_y; est_theta = raw_theta;
        }
    }

    double true_pos_err = hypot(sim.true_x - sc->goal_x, sim.true_y - sc->goal_y);
    double est_pos_err = hypot(est_x - sc->goal_x, est_y - sc->goal_y);
    double true_heading_err = fabs(normalize_angle(sim.true_theta - goal_theta)) * 180.0 / ANGLE_PI;
    double est_heading_err = fabs(normalize_angle(est_theta - goal_theta)) * 180.0 / ANGLE_PI;
    double belief_gap = hypot(sim.true_x - est_x, sim.true_y - est_y);

    printf("  완료여부: %s (스텝 %d)\n", done ? "예" : "타임아웃", step);
    printf("  [믿는 위치 기준] 위치오차=%.3fm, 각도오차=%.1f도\n", est_pos_err, est_heading_err);
    printf("  [실제 참위치 기준] 위치오차=%.3fm, 각도오차=%.1f도\n", true_pos_err, true_heading_err);
    printf("  참위치-추정위치 간 괴리(드리프트): %.3fm\n", belief_gap);
    printf("  결과: %s\n\n", (true_pos_err < 0.15 && true_heading_err < 20.0) ? "PASS(실제로도 성공)" : "FAIL");

    dynnav_free(&nav);
    if (use_lidar_correction) slam_free(&loc_slam);
}

int main(void) {
    Scenario scenarios[3];
    scenarios[0] = (Scenario){
        .name = "F1_고정맵_기본_주차90도",
        .room_size = 3.0,
        .obstacles = {{1.5,1.5,0.2}}, .n_obstacles = 1,
        .known_obstacles = {{1.5,1.5,0.2}}, .n_known_obstacles = 1,
        .start_x=0.3,.start_y=0.3,.start_theta=0.0,
        .goal_x=2.5,.goal_y=2.5,.goal_theta_deg=90.0,
    };
    scenarios[1] = (Scenario){
        .name = "F2_고정맵_예상치못한장애물1개_주차0도",
        .room_size = 3.0,
        .obstacles = {{1.5,1.5,0.3}}, .n_obstacles = 1,
        .known_obstacles = {{99,99,0.01}}, .n_known_obstacles = 1,
        .start_x=0.3,.start_y=0.3,.start_theta=0.0,
        .goal_x=2.7,.goal_y=2.7,.goal_theta_deg=0.0,
    };
    scenarios[2] = (Scenario){
        .name = "F3_고정맵_예상치못한장애물3개_주차90도",
        .room_size = 3.0,
        .obstacles = {{1.0,1.0,0.2},{1.5,1.5,0.2},{2.0,2.0,0.2}}, .n_obstacles = 3,
        .known_obstacles = {{99,99,0.01}}, .n_known_obstacles = 1,
        .start_x=0.3,.start_y=0.3,.start_theta=0.0,
        .goal_x=2.7,.goal_y=2.7,.goal_theta_deg=90.0,
    };

    printf("========== 비교 1: 순수 오도메트리 (지난번과 동일, 대조군) ==========\n\n");
    for (int i = 0; i < 3; i++) run_scenario(&scenarios[i], false);

    printf("========== 비교 2: 라이다 스캔매칭 위치보정 적용 ==========\n\n");
    for (int i = 0; i < 3; i++) run_scenario(&scenarios[i], true);

    return 0;
}
