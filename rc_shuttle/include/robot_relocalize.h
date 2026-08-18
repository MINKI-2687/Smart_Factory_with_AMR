#ifndef ROBOT_RELOCALIZE_H
#define ROBOT_RELOCALIZE_H

#include "robot_config.h"
double robot_runner__map_fit(const OccupancyGridSLAM *s, const LidarScan *scan,
                              double x, double y, double th);
bool robot_runner__relocalize_global(LidarThread *lidar, const RobotConfig *cfg,
                                      OccupancyGridSLAM *slam, const char *why);
bool robot_runner__relocalize_wide(LidarThread *lidar, const RobotConfig *cfg,
                                    OccupancyGridSLAM *slam, const char *why);

#endif /* ROBOT_RELOCALIZE_H */
