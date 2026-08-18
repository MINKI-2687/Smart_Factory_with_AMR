#include "lidar_clustering.h"


void cluster_lidar_scan(const LidarScan *scan, double robot_x, double robot_y,
                                       double robot_theta, double cluster_distance_threshold,
                                       int min_cluster_points, double max_range_m,
                                       ClusterResult *result) {
    result->n_clusters = 0;

    double wx[LIDAR_MAX_READINGS], wy[LIDAR_MAX_READINGS];
    int n_pts = 0;

    for (int i = 0; i < scan->count; i++) {
        double dist_mm = scan->readings[i].dist_mm;
        if (dist_mm <= 0) continue;
        double dist_m = dist_mm / 1000.0;
        if (dist_m > max_range_m) continue;
        double world_angle = robot_theta + scan->readings[i].angle_deg * ANGLE_PI / 180.0;
        if (n_pts < LIDAR_MAX_READINGS) {
            wx[n_pts] = robot_x + dist_m * cos(world_angle);
            wy[n_pts] = robot_y + dist_m * sin(world_angle);
            n_pts++;
        }
    }

    if (n_pts == 0) return;

    int cluster_start = 0;
    for (int i = 1; i <= n_pts; i++) {
        bool boundary = (i == n_pts) ||
                         (hypot(wx[i] - wx[i - 1], wy[i] - wy[i - 1]) > cluster_distance_threshold);
        if (boundary) {
            int cluster_len = i - cluster_start;
            if (cluster_len >= min_cluster_points && result->n_clusters < CLUSTER_MAX_CLUSTERS) {
                LidarCluster *c = &result->clusters[result->n_clusters];
                c->count = 0;
                for (int k = cluster_start; k < i && c->count < CLUSTER_MAX_POINTS_PER_CLUSTER; k++) {
                    c->xs[c->count] = wx[k];
                    c->ys[c->count] = wy[k];
                    c->count++;
                }
                result->n_clusters++;
            }
            cluster_start = i;
        }
    }

    if (result->n_clusters >= 2) {
        LidarCluster *first = &result->clusters[0];
        LidarCluster *last = &result->clusters[result->n_clusters - 1];
        double fx = first->xs[0], fy = first->ys[0];
        double lx = last->xs[last->count - 1], ly = last->ys[last->count - 1];
        if (hypot(fx - lx, fy - ly) <= cluster_distance_threshold) {
            int last_count = last->count;
            if (last_count + first->count <= CLUSTER_MAX_POINTS_PER_CLUSTER) {
                double tmp_x[CLUSTER_MAX_POINTS_PER_CLUSTER], tmp_y[CLUSTER_MAX_POINTS_PER_CLUSTER];
                int tc = 0;
                for (int k = 0; k < last_count; k++) { tmp_x[tc] = last->xs[k]; tmp_y[tc] = last->ys[k]; tc++; }
                for (int k = 0; k < first->count; k++) { tmp_x[tc] = first->xs[k]; tmp_y[tc] = first->ys[k]; tc++; }
                for (int k = 0; k < tc; k++) { first->xs[k] = tmp_x[k]; first->ys[k] = tmp_y[k]; }
                first->count = tc;
            }
            result->n_clusters--;
        }
    }
}


int cluster_result_to_detected(const ClusterResult *cr, DetectedObstacle *out,
                                              int max_out) {
    int n = cr->n_clusters < max_out ? cr->n_clusters : max_out;
    for (int i = 0; i < n; i++) {
        out[i].xs = cr->clusters[i].xs;
        out[i].ys = cr->clusters[i].ys;
        out[i].count = cr->clusters[i].count;
    }
    return n;
}
