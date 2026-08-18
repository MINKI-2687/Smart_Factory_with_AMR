/* graph_slam_capi.c
 * graph_slam.h(순수 C, header-only)를 파이썬 ctypes에서 바로 부를 수 있게 감싸는 얇은 래퍼.
 * slam_capi.c와 동일한 스타일.
 *
 * 컴파일: gcc -std=c11 -O2 -shared -fPIC -o libgraphslam.so graph_slam_capi.c -lm
 */
#include <stdlib.h>
#include <string.h>
#include "graph_slam.h"

void *gslam_capi_create(double start_x, double start_y, double motion_noise,
                         double measurement_noise, double landmark_match_threshold) {
    GraphSlam *g = (GraphSlam *)malloc(sizeof(GraphSlam));
    gslam_init(g, start_x, start_y, motion_noise, measurement_noise, landmark_match_threshold);
    return g;
}

void gslam_capi_destroy(void *handle) {
    GraphSlam *g = (GraphSlam *)handle;
    gslam_free(g);
    free(g);
}

void gslam_capi_step(void *handle, double dx, double dy,
                      const double *cluster_angles_rad, const double *cluster_dists_m, int n_obs,
                      double robot_theta,
                      const double *scan_angles_deg, const double *scan_dists_mm, int scan_n) {
    GraphSlam *g = (GraphSlam *)handle;

    LidarScan scan;
    LidarScan *scan_ptr = NULL;
    if (scan_n > 0) {
        scan.count = (scan_n < LIDAR_MAX_READINGS) ? scan_n : LIDAR_MAX_READINGS;
        for (int i = 0; i < scan.count; i++) {
            scan.readings[i].angle_deg = scan_angles_deg[i];
            scan.readings[i].dist_mm = scan_dists_mm[i];
        }
        scan_ptr = &scan;
    }

    gslam_step(g, dx, dy, cluster_angles_rad, cluster_dists_m, n_obs, robot_theta, scan_ptr);
}

int gslam_capi_get_n_poses(void *handle) { return ((GraphSlam *)handle)->n_poses; }
int gslam_capi_get_n_landmarks(void *handle) { return ((GraphSlam *)handle)->n_landmarks; }

int gslam_capi_solve(void *handle, double *out_pose_x, double *out_pose_y,
                      double *out_landmark_x, double *out_landmark_y) {
    GraphSlam *g = (GraphSlam *)handle;
    bool ok = gslam_solve(g, out_pose_x, out_pose_y, out_landmark_x, out_landmark_y);
    return ok ? 1 : 0;
}

int gslam_capi_rebuild_grid(void *handle, void *slam_grid_handle) {
    GraphSlam *g = (GraphSlam *)handle;
    OccupancyGridSLAM *s = (OccupancyGridSLAM *)slam_grid_handle;
    bool ok = gslam_rebuild_grid(g, s);
    return ok ? 1 : 0;
}

void gslam_capi_get_raw_pose(void *handle, int idx, double *out_x, double *out_y) {
    GraphSlam *g = (GraphSlam *)handle;
    *out_x = g->raw_pose_x[idx];
    *out_y = g->raw_pose_y[idx];
}
