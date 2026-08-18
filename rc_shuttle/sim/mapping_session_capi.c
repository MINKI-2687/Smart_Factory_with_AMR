/* mapping_session_capi.c
 * mapping_session.h(순수 C, header-only)를 파이썬 ctypes에서 부를 수 있게 감싸는 래퍼.
 * 컴파일: gcc -std=c11 -O2 -shared -fPIC -o libsession.so mapping_session_capi.c -lm
 */
#include <stdlib.h>
#include <math.h>
#include "mapping_session.h"

void *session_capi_create(double size_x, double size_y, double resolution,
                           double start_x, double start_y, double start_theta,
                           double goal_x, double goal_y,
                           int has_goal_theta, double goal_theta_rad,
                           double robot_radius, double occ_threshold,
                           int frontier_max_miss, int max_mapping_steps) {
    MappingSession *ms = (MappingSession *)malloc(sizeof(MappingSession));
    mapping_session_init(ms, size_x, size_y, resolution, start_x, start_y, start_theta,
                          goal_x, goal_y, has_goal_theta != 0, goal_theta_rad,
                          robot_radius, occ_threshold, frontier_max_miss, max_mapping_steps);
    return ms;
}

/* 고정맵 모드: 매핑(프론티어탐색) 단계 없이 바로 NAVIGATE로 시작. known_grid는
 * rows*cols 크기의 0/1 배열(반드시 벽 포함 - obstacle_list.h의 make_grid 사용 권장).
 * 반환된 핸들의 phase가 PLAN_FAILED(3)면 A* 계획 자체가 실패한 것이니 호출부가 확인해야 함. */
void *session_capi_create_fixed_map(const unsigned char *known_grid, int rows, int cols,
                                     double resolution,
                                     double start_x, double start_y, double start_theta,
                                     double goal_x, double goal_y,
                                     int has_goal_theta, double goal_theta_rad,
                                     double robot_radius) {
    MappingSession *ms = (MappingSession *)malloc(sizeof(MappingSession));
    mapping_session_init_fixed_map(ms, known_grid, rows, cols, resolution,
                                    start_x, start_y, start_theta, goal_x, goal_y,
                                    has_goal_theta != 0, goal_theta_rad, robot_radius);
    return ms;
}

/* 매핑전용 모드: 목표를 아직 몰라도 매핑부터 시작함. 매핑이 끝나면 자동 A*계획을
 * 안 하고 phase가 MAP_READY(4)로 멈춤 - GUI가 지도를 보여주고 사용자가 클릭으로
 * 목표를 고르면 session_capi_start_navigation()을 호출해서 진행시켜야 함. */
void *session_capi_create_mapping_only(double size_x, double size_y, double resolution,
                                        double start_x, double start_y, double start_theta,
                                        double robot_radius, double occ_threshold,
                                        int frontier_max_miss, int max_mapping_steps) {
    MappingSession *ms = (MappingSession *)malloc(sizeof(MappingSession));
    mapping_session_init_mapping_only(ms, size_x, size_y, resolution, start_x, start_y, start_theta,
                                       robot_radius, occ_threshold, frontier_max_miss, max_mapping_steps);
    return ms;
}

/* phase가 MAP_READY일 때만 유효. 반환: 1이면 성공(NAVIGATE로 전환), 0이면 실패
 * (인자가 잘못됐거나(phase가 MAP_READY가 아님) A*계획 실패(PLAN_FAILED로 전환)). */
int session_capi_start_navigation(void *handle, double goal_x, double goal_y,
                                   int has_goal_theta, double goal_theta_rad) {
    MappingSession *ms = (MappingSession *)handle;
    bool ok = mapping_session_start_navigation(ms, goal_x, goal_y, has_goal_theta != 0, goal_theta_rad);
    return ok ? 1 : 0;
}

void session_capi_destroy(void *handle) {
    if (!handle) return;
    mapping_session_free((MappingSession *)handle);
    free(handle);
}

/* scan: angle_deg[] / dist_mm[] 배열 n개. 반환(출력 인자): out_left/right, out_done(0/1) */
void session_capi_step(void *handle, const double *angles, const double *dists, int n,
                        double odom_dx, double odom_dy, double odom_dtheta,
                        int *out_left, int *out_right, int *out_done) {
    MappingSession *ms = (MappingSession *)handle;
    LidarScan scan;
    scan.count = (n < LIDAR_MAX_READINGS) ? n : LIDAR_MAX_READINGS;
    for (int i = 0; i < scan.count; i++) {
        scan.readings[i].angle_deg = angles[i];
        scan.readings[i].dist_mm = dists[i];
    }
    bool done = false;
    mapping_session_step(ms, &scan, odom_dx, odom_dy, odom_dtheta, out_left, out_right, &done);
    *out_done = done ? 1 : 0;
}

int session_capi_get_phase(void *handle) {
    return (int)((MappingSession *)handle)->phase;
}

void session_capi_get_pose(void *handle, double *x, double *y, double *theta) {
    MappingSession *ms = (MappingSession *)handle;
    *x = ms->est_x; *y = ms->est_y; *theta = ms->est_theta;
}

/* 반환: 목표가 있으면 1(+x/y 채움), 없으면 0 */
int session_capi_get_frontier_target(void *handle, double *x, double *y) {
    MappingSession *ms = (MappingSession *)handle;
    if (!ms->frontier.has_target) return 0;
    *x = ms->frontier.target_x; *y = ms->frontier.target_y;
    return 1;
}

int session_capi_get_rows(void *handle) { return ((MappingSession *)handle)->slam.rows; }
int session_capi_get_cols(void *handle) { return ((MappingSession *)handle)->slam.cols; }
double session_capi_get_resolution(void *handle) { return ((MappingSession *)handle)->slam.resolution; }

/* rows*cols 크기의 out_buf에 각 칸의 점유확률(0~1)을 채움 (시각화용) */
void session_capi_get_prob_grid(void *handle, double *out_buf) {
    MappingSession *ms = (MappingSession *)handle;
    int n = ms->slam.rows * ms->slam.cols;
    for (int i = 0; i < n; i++) {
        out_buf[i] = 1.0 / (1.0 + exp(-ms->slam.log_odds[i]));
    }
}

int session_capi_get_waypoint_count(void *handle) {
    MappingSession *ms = (MappingSession *)handle;
    if (!ms->nav_created) return 0;
    return ms->nav.waypoints.count;
}

void session_capi_get_waypoint(void *handle, int i, double *x, double *y) {
    MappingSession *ms = (MappingSession *)handle;
    *x = ms->nav.waypoints.points[i].x;
    *y = ms->nav.waypoints.points[i].y;
}

int session_capi_get_current_wp_idx(void *handle) {
    MappingSession *ms = (MappingSession *)handle;
    if (!ms->nav_created) return -1;
    return ms->nav.current_wp_idx;
}

/* Debug-only: expose FrontierExplorer internals for diagnosing premature-termination
 * issues (map finishing "too early" under realistic sensor noise). */
void session_capi_get_frontier_debug(void *handle, int *has_target, double *target_x, double *target_y,
                                      int *miss_count, int *avoid_history_count,
                                      int *steps_on_current_target) {
    MappingSession *ms = (MappingSession *)handle;
    *has_target = ms->frontier.has_target ? 1 : 0;
    *target_x = ms->frontier.target_x;
    *target_y = ms->frontier.target_y;
    *miss_count = ms->frontier.miss_count;
    *avoid_history_count = ms->frontier.avoid_history_count;
    *steps_on_current_target = ms->frontier.steps_on_current_target;
}

/* Whether graph-SLAM (loop closure) correction was actually applied to the final
 * map/pose (it can fail silently and fall back to the raw map if there weren't
 * enough re-observed landmarks - this lets callers show that in a GUI/log). */
int session_capi_get_used_graph_slam_correction(void *handle) {
    return ((MappingSession *)handle)->used_graph_slam_correction ? 1 : 0;
}

/* 셔틀 왕복용 - DONE 상태에서 새 목표로 재설정. 반환: 1이면 성공(NAVIGATE로 전환),
 * 0이면 A* 계획 실패(PLAN_FAILED로 전환). */
int session_capi_retarget(void *handle, double new_goal_x, double new_goal_y,
                           int has_goal_theta, double new_goal_theta_rad) {
    MappingSession *ms = (MappingSession *)handle;
    bool ok = mapping_session_retarget(ms, new_goal_x, new_goal_y,
                                        has_goal_theta != 0, new_goal_theta_rad);
    return ok ? 1 : 0;
}
