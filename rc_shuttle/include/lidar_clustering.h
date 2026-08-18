#ifndef LIDAR_CLUSTERING_H
#define LIDAR_CLUSTERING_H

#include <math.h>
#include <stdbool.h>
#include "angle_utils.h"
#include "lidar_types.h"
#include "detected_obstacle.h"

#define CLUSTER_MAX_CLUSTERS 20
#define CLUSTER_MAX_POINTS_PER_CLUSTER 128

typedef struct {
    double xs[CLUSTER_MAX_POINTS_PER_CLUSTER];
    double ys[CLUSTER_MAX_POINTS_PER_CLUSTER];
    int count;
} LidarCluster;

typedef struct {
    LidarCluster clusters[CLUSTER_MAX_CLUSTERS];
    int n_clusters;
} ClusterResult;
void cluster_lidar_scan(const LidarScan *scan, double robot_x, double robot_y,
                         double robot_theta, double cluster_distance_threshold,
                         int min_cluster_points, double max_range_m,
                         ClusterResult *result);
int cluster_result_to_detected(const ClusterResult *cr, DetectedObstacle *out,
                                int max_out);

#endif /* LIDAR_CLUSTERING_H */
