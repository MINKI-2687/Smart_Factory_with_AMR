/* slam_capi.c
 * slam.h(순수 C, header-only)를 파이썬 ctypes에서 바로 부를 수 있게 감싸는 얇은 래퍼.
 * "진짜 계산은 C가 하고, 화면은 파이썬(matplotlib)이 그린다"는 구조.
 *
 * 컴파일: gcc -std=c11 -O2 -shared -fPIC -o libslam.so slam_capi.c -lm
 */
#include <stdlib.h>
#include <string.h>
#include "slam.h"

void *slam_capi_create(double size_x, double size_y, double resolution,
                        double origin_x, double origin_y,
                        double start_x, double start_y, double start_theta,
                        double search_window_m, double search_window_theta_deg,
                        double search_step_m, double search_step_theta_deg) {
    OccupancyGridSLAM *s = (OccupancyGridSLAM *)malloc(sizeof(OccupancyGridSLAM));
    slam_init(s, size_x, size_y, resolution, origin_x, origin_y,
              start_x, start_y, start_theta,
              search_window_m, search_window_theta_deg, search_step_m, search_step_theta_deg);
    return s;
}

/* 고정맵(이미 알고 있는 지도)을 log_odds로 미리 채워서 초기화 - slam_capi_localize_only와
 * 짝을 이룸(위치추정 전용, 지도는 갱신 안 함). raw_grid: rows*cols 크기의 0/1 배열
 * (obstacle_list.h의 make_grid로 만들면 벽까지 자동 포함됨 - 라이다 위치보정엔
 * 벽이 지도에 있어야 함, 없으면 스캔매칭이 역효과를 냄. 반드시 확인). */
void *slam_capi_create_from_grid(const unsigned char *raw_grid, int rows, int cols, double resolution,
                                  double origin_x, double origin_y,
                                  double start_x, double start_y, double start_theta,
                                  double search_window_m, double search_window_theta_deg,
                                  double search_step_m, double search_step_theta_deg) {
    OccupancyGridSLAM *s = (OccupancyGridSLAM *)malloc(sizeof(OccupancyGridSLAM));
    slam_init_from_grid(s, raw_grid, rows, cols, resolution, origin_x, origin_y,
                         start_x, start_y, start_theta,
                         search_window_m, search_window_theta_deg, search_step_m, search_step_theta_deg);
    return s;
}

void slam_capi_destroy(void *handle) {
    OccupancyGridSLAM *s = (OccupancyGridSLAM *)handle;
    slam_free(s);
    free(s);
}

void slam_capi_update(void *handle, const double *angles_deg, const double *dists_mm, int n,
                       double odom_dx, double odom_dy, double odom_dtheta,
                       double *out_x, double *out_y, double *out_theta) {
    OccupancyGridSLAM *s = (OccupancyGridSLAM *)handle;
    LidarScan scan;
    scan.count = (n < LIDAR_MAX_READINGS) ? n : LIDAR_MAX_READINGS;
    for (int i = 0; i < scan.count; i++) {
        scan.readings[i].angle_deg = angles_deg[i];
        scan.readings[i].dist_mm = dists_mm[i];
    }
    slam_update(s, &scan, odom_dx, odom_dy, odom_dtheta, out_x, out_y, out_theta);
}

void slam_capi_update_map_only(void *handle, const double *angles_deg, const double *dists_mm, int n) {
    OccupancyGridSLAM *s = (OccupancyGridSLAM *)handle;
    LidarScan scan;
    scan.count = (n < LIDAR_MAX_READINGS) ? n : LIDAR_MAX_READINGS;
    for (int i = 0; i < scan.count; i++) {
        scan.readings[i].angle_deg = angles_deg[i];
        scan.readings[i].dist_mm = dists_mm[i];
    }
    slam_update_map(s, s->x, s->y, s->theta, &scan);
}

/* B condition (map-only, no correction): caller tracks pose externally (pure odometry)
 * and just wants the map rasterized at that externally-supplied pose, without touching
 * the SLAM object's own internal x/y/theta at all. */
void slam_capi_update_map_at_pose(void *handle, const double *angles_deg, const double *dists_mm, int n,
                                   double x, double y, double theta) {
    OccupancyGridSLAM *s = (OccupancyGridSLAM *)handle;
    LidarScan scan;
    scan.count = (n < LIDAR_MAX_READINGS) ? n : LIDAR_MAX_READINGS;
    for (int i = 0; i < scan.count; i++) {
        scan.readings[i].angle_deg = angles_deg[i];
        scan.readings[i].dist_mm = dists_mm[i];
    }
    slam_update_map(s, x, y, theta, &scan);
}

void slam_capi_get_pose(void *handle, double *x, double *y, double *theta) {
    OccupancyGridSLAM *s = (OccupancyGridSLAM *)handle;
    *x = s->x; *y = s->y; *theta = s->theta;
}

/* 위치추정 전용(localization-only): 이미 만들어진 지도는 안 건드리고, 그 지도에 맞춰
 * 위치(x,y,theta)만 스캔매칭으로 보정함. navigate 단계처럼 "지도는 고정, 위치만 계속
 * 다듬고 싶을 때" 씀. slam_capi_update()와 달리 지도(log_odds)는 안 바뀜.
 *
 * PATCH: 이 로직(오도메트리 예측 + scan_match_correct_pose)은 slam.h의
 * slam_localize_step()과 완전히 같아서, 여기서 다시 풀어쓰지 않고 그걸 그대로 재사용함
 * (같은 로직을 두 곳에 따로 두면 한쪽만 고쳐지는 사고가 남 - 실제로 이 프로젝트에서
 * 여러 번 겪었던 패턴이라 의식적으로 피함). */
void slam_capi_localize_only(void *handle,
                              const double *angles_deg, const double *dists_mm, int n,
                              double odom_dx, double odom_dy, double odom_dtheta,
                              double *out_x, double *out_y, double *out_theta) {
    OccupancyGridSLAM *s = (OccupancyGridSLAM *)handle;
    LidarScan scan;
    scan.count = (n < LIDAR_MAX_READINGS) ? n : LIDAR_MAX_READINGS;
    for (int i = 0; i < scan.count; i++) {
        scan.readings[i].angle_deg = angles_deg[i];
        scan.readings[i].dist_mm = dists_mm[i];
    }
    slam_localize_step(s, &scan, odom_dx, odom_dy, odom_dtheta, out_x, out_y, out_theta);
}

int slam_capi_get_rows(void *handle) { return ((OccupancyGridSLAM *)handle)->rows; }
int slam_capi_get_cols(void *handle) { return ((OccupancyGridSLAM *)handle)->cols; }
double slam_capi_get_resolution(void *handle) { return ((OccupancyGridSLAM *)handle)->resolution; }

void slam_capi_get_log_odds(void *handle, double *out_buffer) {
    OccupancyGridSLAM *s = (OccupancyGridSLAM *)handle;
    memcpy(out_buffer, s->log_odds, sizeof(double) * (size_t)s->rows * s->cols);
}

/* PATCH (2026-08-06): 실기(robot_runner.h)는 고정맵/불러온맵일 때 부드러운
 * likelihood-field 점수판(slam.h의 slam_build_likelihood_field)을 붙여서 위치추정
 * 정밀도를 올림(mm 단위 오차로 개선 확인됨). 시뮬레이션(test_slot_map.py 등)도
 * 실기와 같은 조건으로 검증하려면 이걸 켤 수 있어야 함 - 이 함수가 그 스위치. */
void slam_capi_enable_likelihood_field(void *handle) {
    OccupancyGridSLAM *s = (OccupancyGridSLAM *)handle;
    if (s->lf_field) { free(s->lf_field); s->lf_field = NULL; }
    s->lf_field = slam_build_likelihood_field(s);
}
