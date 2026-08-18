#ifndef PLANNING_PIPELINE_H
#define PLANNING_PIPELINE_H

#include "astar.h"
#include "smoothing.h"
bool planning_pipeline_plan(const OccupancyGridMap *map,
                             double start_x, double start_y,
                             double goal_x, double goal_y,
                             double simplify_min_dist,
                             bool enable_smoothing,
                             double smooth_weight_data, double smooth_weight_smooth,
                             double smooth_tolerance, int smooth_max_iterations,
                             Path *out);

#endif /* PLANNING_PIPELINE_H */
