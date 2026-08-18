/* nav_capi.c
 * dynamic_navigator.h (A* + PID + docking) and safe_navigator.h (SafeNavigator,
 * used standalone during the SLAM mapping phase) wrapped for Python ctypes.
 *
 * Build: gcc -std=c11 -O2 -shared -fPIC -o libnav.so nav_capi.c -lm
 */
#include <stdlib.h>
#include <string.h>
#include "dynamic_navigator.h"
#include "lidar_clustering.h"

unsigned char *nav_capi_make_grid_from_circles(double size_x, double size_y, double resolution,
                                                const double *cx, const double *cy, const double *cr,
                                                int n_circles, int *out_rows, int *out_cols) {
    int cols = (int)(size_x / resolution);
    int rows = (int)(size_y / resolution);
    unsigned char *grid = (unsigned char *)calloc((size_t)rows * cols, 1);
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            double wx = c * resolution + resolution / 2.0;
            double wy = r * resolution + resolution / 2.0;
            for (int i = 0; i < n_circles; i++) {
                double dx = wx - cx[i], dy = wy - cy[i];
                if (dx * dx + dy * dy <= cr[i] * cr[i]) {
                    grid[r * cols + c] = 1;
                    break;
                }
            }
        }
    }
    *out_rows = rows;
    *out_cols = cols;
    return grid;
}

void nav_capi_free_buffer(void *ptr) { free(ptr); }

void *nav_capi_create(const unsigned char *grid, int rows, int cols, double resolution,
                       double robot_radius, double origin_x, double origin_y) {
    DynamicNavigator *nav = (DynamicNavigator *)malloc(sizeof(DynamicNavigator));
    dynnav_init(nav, grid, rows, cols, resolution, robot_radius, origin_x, origin_y);
    return nav;
}

void nav_capi_destroy(void *handle) {
    DynamicNavigator *nav = (DynamicNavigator *)handle;
    dynnav_free(nav);
    free(nav);
}

int nav_capi_set_goal(void *handle, double start_x, double start_y, double goal_x, double goal_y,
                       int has_goal_theta, double goal_theta) {
    DynamicNavigator *nav = (DynamicNavigator *)handle;
    bool ok = dynnav_set_goal(nav, start_x, start_y, goal_x, goal_y,
                               has_goal_theta != 0, goal_theta, NULL, NULL, 0);
    return ok ? 1 : 0;
}

int nav_capi_get_waypoint_count(void *handle) {
    return ((DynamicNavigator *)handle)->waypoints.count;
}

void nav_capi_get_waypoint(void *handle, int idx, double *x, double *y) {
    DynamicNavigator *nav = (DynamicNavigator *)handle;
    *x = nav->waypoints.points[idx].x;
    *y = nav->waypoints.points[idx].y;
}

int nav_capi_get_current_wp_idx(void *handle) {
    return ((DynamicNavigator *)handle)->current_wp_idx;
}

int nav_capi_get_rows(void *handle) { return ((DynamicNavigator *)handle)->rows; }
int nav_capi_get_cols(void *handle) { return ((DynamicNavigator *)handle)->cols; }
double nav_capi_get_resolution(void *handle) { return ((DynamicNavigator *)handle)->resolution; }

void nav_capi_get_known_grid(void *handle, unsigned char *out_buffer) {
    DynamicNavigator *nav = (DynamicNavigator *)handle;
    memcpy(out_buffer, nav->known_grid, (size_t)nav->rows * nav->cols);
}

int nav_capi_get_replanned_this_call(void *handle) {
    return ((DynamicNavigator *)handle)->replanned_this_call ? 1 : 0;
}

void nav_capi_update(void *handle, double x, double y, double theta,
                      const double *angles_deg, const double *dists_mm, int n,
                      int *out_left, int *out_right, int *out_done) {
    DynamicNavigator *nav = (DynamicNavigator *)handle;
    LidarScan scan;
    scan.count = (n < LIDAR_MAX_READINGS) ? n : LIDAR_MAX_READINGS;
    for (int i = 0; i < scan.count; i++) {
        scan.readings[i].angle_deg = angles_deg[i];
        scan.readings[i].dist_mm = dists_mm[i];
    }
    ClusterResult cr;
    cluster_lidar_scan(&scan, x, y, theta, 0.15, 2, 3.0, &cr);
    DetectedObstacle detected[CLUSTER_MAX_CLUSTERS];
    int n_detected = cluster_result_to_detected(&cr, detected, CLUSTER_MAX_CLUSTERS);

    WheelCmd cmd = dynnav_update(nav, x, y, theta, &scan, n_detected > 0 ? detected : NULL, n_detected);
    *out_left = cmd.left;
    *out_right = cmd.right;
    *out_done = cmd.done ? 1 : 0;
}

typedef struct {
    GoalPIDController goal_ctrl;
    ObstacleAvoidance avoider;
    SafeNavigator nav;
} SafeNavBundle;

void *safenav_capi_create(double goal_tolerance) {
    SafeNavBundle *b = (SafeNavBundle *)malloc(sizeof(SafeNavBundle));
    goal_pid_init_default(&b->goal_ctrl, goal_tolerance);
    obstacle_avoidance_init_default(&b->avoider);
    safe_navigator_init(&b->nav, &b->goal_ctrl, &b->avoider);
    return b;
}

void safenav_capi_destroy(void *handle) { free(handle); }

void safenav_capi_compute(void *handle, double x, double y, double theta, double goal_x, double goal_y,
                           const double *angles_deg, const double *dists_mm, int n,
                           int *out_left, int *out_right, int *out_done) {
    SafeNavBundle *b = (SafeNavBundle *)handle;
    LidarScan scan;
    scan.count = (n < LIDAR_MAX_READINGS) ? n : LIDAR_MAX_READINGS;
    for (int i = 0; i < scan.count; i++) {
        scan.readings[i].angle_deg = angles_deg[i];
        scan.readings[i].dist_mm = dists_mm[i];
    }
    WheelCmd cmd = safe_navigator_compute(&b->nav, x, y, theta, goal_x, goal_y, &scan);
    *out_left = cmd.left;
    *out_right = cmd.right;
    *out_done = cmd.done ? 1 : 0;
}

/* PATCH (2026-08-06): pure pursuit 설정을 파이썬 시뮬레이션(test_slot_map.py)에서도
 * 켜고 끌 수 있게 노출 - 실기와 같은 조건으로 비교 검증하기 위함. */
void nav_capi_set_pure_pursuit(void *handle, int enable, double lookahead_m) {
    DynamicNavigator *nav = (DynamicNavigator *)handle;
    nav->use_pure_pursuit = (enable != 0);
    if (lookahead_m > 0) nav->lookahead_m = lookahead_m;
}

/* 목적지 도달에 실패(경로 막힘)했는지. done=1 이어도 이게 1이면 '도착'이 아니라
 * '포기'이므로 호출부는 반드시 확인해야 함 - dynamic_navigator.h의 goal_unreachable 참고. */
int nav_capi_get_goal_unreachable(void *handle) {
    return ((DynamicNavigator *)handle)->goal_unreachable ? 1 : 0;
}
