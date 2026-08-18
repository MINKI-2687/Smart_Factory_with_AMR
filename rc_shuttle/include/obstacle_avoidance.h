#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

#include <math.h>
#include "angle_utils.h"
#include "lidar_types.h"

typedef struct {
    double danger_distance;
    double repulsive_gain;
    double tangent_gain;
} ObstacleAvoidance;
void obstacle_avoidance_init(ObstacleAvoidance *o,
                              double danger_distance, double repulsive_gain,
                              double tangent_gain);
void obstacle_avoidance_init_default(ObstacleAvoidance *o);
void compute_repulsion(const ObstacleAvoidance *o, const LidarScan *scan,
                        double robot_theta, double *repel_x, double *repel_y);

#endif /* OBSTACLE_AVOIDANCE_H */
