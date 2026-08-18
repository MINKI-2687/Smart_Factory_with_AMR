// graph_slam.h
// GraphSLAM (landmark re-observation based full-trajectory re-optimization / loop closure).
// omega/xi managed as fixed max-size arrays (same pattern used throughout this project).

#ifndef GRAPH_SLAM_H
#define GRAPH_SLAM_H

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "linear_solve.h"
#include "lidar_types.h"
#include "slam.h"

#define GSLAM_MAX_POSES 3000
#define GSLAM_MAX_LANDMARKS 50
#define GSLAM_MAX_DIM (GSLAM_MAX_POSES + GSLAM_MAX_LANDMARKS)

typedef struct {
    double *omega;
    double *xi_x;
    double *xi_y;

    int n_poses;
    int n_landmarks;
    double landmark_x[GSLAM_MAX_LANDMARKS];
    double landmark_y[GSLAM_MAX_LANDMARKS];

    double raw_x, raw_y;
    double raw_pose_x[GSLAM_MAX_POSES];
    double raw_pose_y[GSLAM_MAX_POSES];
    double theta_history[GSLAM_MAX_POSES];
    LidarScan *scan_history;
    bool scan_present[GSLAM_MAX_POSES];

    double motion_noise;
    double measurement_noise;
    double landmark_match_threshold;
} GraphSlam;

static inline int gslam__dim(const GraphSlam *g) { return g->n_poses + g->n_landmarks; }

static inline void gslam_init(GraphSlam *g, double start_x, double start_y,
                               double motion_noise, double measurement_noise,
                               double landmark_match_threshold) {
    g->omega = (double *)calloc((size_t)GSLAM_MAX_DIM * GSLAM_MAX_DIM, sizeof(double));
    g->xi_x = (double *)calloc(GSLAM_MAX_DIM, sizeof(double));
    g->xi_y = (double *)calloc(GSLAM_MAX_DIM, sizeof(double));
    g->scan_history = (LidarScan *)malloc(sizeof(LidarScan) * GSLAM_MAX_POSES);

    g->n_poses = 1;
    g->n_landmarks = 0;
    g->motion_noise = motion_noise;
    g->measurement_noise = measurement_noise;
    g->landmark_match_threshold = landmark_match_threshold;

    g->omega[0 * GSLAM_MAX_DIM + 0] = 1.0;
    g->xi_x[0] = start_x;
    g->xi_y[0] = start_y;

    g->raw_x = start_x; g->raw_y = start_y;
    g->raw_pose_x[0] = start_x; g->raw_pose_y[0] = start_y;
    g->theta_history[0] = 0.0;
    g->scan_present[0] = false;
}

static inline void gslam_free(GraphSlam *g) {
    free(g->omega); free(g->xi_x); free(g->xi_y); free(g->scan_history);
    g->omega = NULL; g->xi_x = NULL; g->xi_y = NULL; g->scan_history = NULL;
}

static inline int gslam__match_or_create_landmark(GraphSlam *g, double wx, double wy) {
    for (int i = 0; i < g->n_landmarks; i++) {
        double d = hypot(wx - g->landmark_x[i], wy - g->landmark_y[i]);
        if (d < g->landmark_match_threshold) return i;
    }
    if (g->n_landmarks >= GSLAM_MAX_LANDMARKS) return g->n_landmarks - 1;
    int idx = g->n_landmarks;
    g->landmark_x[idx] = wx;
    g->landmark_y[idx] = wy;
    g->n_landmarks++;
    int col = GSLAM_MAX_POSES + idx;
    g->omega[col * GSLAM_MAX_DIM + col] += 1e-6;
    return idx;
}

static inline void gslam_step(GraphSlam *g, double dx, double dy,
                               const double *cluster_angles_rad, const double *cluster_dists_m, int n_obs,
                               double robot_theta, const LidarScan *raw_scan) {
    if (g->n_poses >= GSLAM_MAX_POSES) return;

    int cur_pose_idx = g->n_poses - 1;
    int new_pose_idx = g->n_poses;
    g->n_poses++;

    g->omega[new_pose_idx * GSLAM_MAX_DIM + new_pose_idx] += 1e-6;

    g->raw_x += dx; g->raw_y += dy;
    g->raw_pose_x[new_pose_idx] = g->raw_x;
    g->raw_pose_y[new_pose_idx] = g->raw_y;
    g->theta_history[new_pose_idx] = robot_theta;
    if (raw_scan) {
        g->scan_history[new_pose_idx] = *raw_scan;
        g->scan_present[new_pose_idx] = true;
    } else {
        g->scan_present[new_pose_idx] = false;
    }

    for (int i = 0; i < n_obs; i++) {
        double world_angle = robot_theta + cluster_angles_rad[i];
        double wx = g->raw_x + cluster_dists_m[i] * cos(world_angle);
        double wy = g->raw_y + cluster_dists_m[i] * sin(world_angle);
        int lm_idx = gslam__match_or_create_landmark(g, wx, wy);
        int lm_col = GSLAM_MAX_POSES + lm_idx;

        double rel_x = cluster_dists_m[i] * cos(world_angle);
        double rel_y = cluster_dists_m[i] * sin(world_angle);
        double w = 1.0 / g->measurement_noise;

        g->omega[new_pose_idx * GSLAM_MAX_DIM + new_pose_idx] += w;
        g->omega[new_pose_idx * GSLAM_MAX_DIM + lm_col] -= w;
        g->omega[lm_col * GSLAM_MAX_DIM + new_pose_idx] -= w;
        g->omega[lm_col * GSLAM_MAX_DIM + lm_col] += w;
        g->xi_x[new_pose_idx] -= rel_x * w;
        g->xi_y[new_pose_idx] -= rel_y * w;
        g->xi_x[lm_col] += rel_x * w;
        g->xi_y[lm_col] += rel_y * w;
    }

    double wm = 1.0 / g->motion_noise;
    g->omega[cur_pose_idx * GSLAM_MAX_DIM + cur_pose_idx] += wm;
    g->omega[new_pose_idx * GSLAM_MAX_DIM + new_pose_idx] += wm;
    g->omega[new_pose_idx * GSLAM_MAX_DIM + cur_pose_idx] -= wm;
    g->omega[cur_pose_idx * GSLAM_MAX_DIM + new_pose_idx] -= wm;
    g->xi_x[cur_pose_idx] -= dx * wm;
    g->xi_y[cur_pose_idx] -= dy * wm;
    g->xi_x[new_pose_idx] += dx * wm;
    g->xi_y[new_pose_idx] += dy * wm;
}

static inline bool gslam_solve(GraphSlam *g, double *corrected_pose_x, double *corrected_pose_y,
                                double *corrected_landmark_x, double *corrected_landmark_y) {
    int dim = gslam__dim(g);

    double *A = (double *)malloc(sizeof(double) * (size_t)dim * dim);
    double *bx = (double *)malloc(sizeof(double) * (size_t)dim);
    double *by = (double *)malloc(sizeof(double) * (size_t)dim);
    double *xx = (double *)malloc(sizeof(double) * (size_t)dim);
    double *xy = (double *)malloc(sizeof(double) * (size_t)dim);
    if (!A || !bx || !by || !xx || !xy) {
        free(A); free(bx); free(by); free(xx); free(xy);
        return false;
    }
    for (int r = 0; r < dim; r++) {
        int full_r = (r < g->n_poses) ? r : (GSLAM_MAX_POSES + (r - g->n_poses));
        for (int c = 0; c < dim; c++) {
            int full_c = (c < g->n_poses) ? c : (GSLAM_MAX_POSES + (c - g->n_poses));
            A[r * dim + c] = g->omega[full_r * GSLAM_MAX_DIM + full_c];
        }
        bx[r] = g->xi_x[full_r];
        by[r] = g->xi_y[full_r];
    }

    bool ok1 = linear_solve(A, bx, xx, dim);
    bool ok2 = linear_solve(A, by, xy, dim);

    if (ok1 && ok2) {
        for (int i = 0; i < g->n_poses; i++) { corrected_pose_x[i] = xx[i]; corrected_pose_y[i] = xy[i]; }
        for (int j = 0; j < g->n_landmarks; j++) {
            corrected_landmark_x[j] = xx[g->n_poses + j];
            corrected_landmark_y[j] = xy[g->n_poses + j];
        }
    }

    free(A); free(bx); free(by); free(xx); free(xy);
    return ok1 && ok2;
}

static inline bool gslam_rebuild_grid(GraphSlam *g, OccupancyGridSLAM *slam_grid) {
    double *cpx = (double *)malloc(sizeof(double) * GSLAM_MAX_POSES);
    double *cpy = (double *)malloc(sizeof(double) * GSLAM_MAX_POSES);
    double *clx = (double *)malloc(sizeof(double) * GSLAM_MAX_LANDMARKS);
    double *cly = (double *)malloc(sizeof(double) * GSLAM_MAX_LANDMARKS);
    if (!cpx || !cpy || !clx || !cly) {
        free(cpx); free(cpy); free(clx); free(cly);
        return false;
    }

    bool ok = gslam_solve(g, cpx, cpy, clx, cly);
    if (!ok) { free(cpx); free(cpy); free(clx); free(cly); return false; }

    for (int i = 0; i < slam_grid->rows * slam_grid->cols; i++) slam_grid->log_odds[i] = 0.0;

    for (int i = 0; i < g->n_poses; i++) {
        if (!g->scan_present[i]) continue;
        slam_update_map(slam_grid, cpx[i], cpy[i], g->theta_history[i], &g->scan_history[i]);
    }

    free(cpx); free(cpy); free(clx); free(cly);
    return true;
}

#endif /* GRAPH_SLAM_H */
