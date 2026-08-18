#ifndef ROBOT_NAVIGATE_H
#define ROBOT_NAVIGATE_H

#include "robot_config.h"
bool navigate_one_leg_inner(FPGALink *fpga, LidarThread *lidar, DynamicNavigator *nav,
                       OccupancyGridSLAM *nav_slam, const RobotConfig *cfg,
                       bool use_slam_rolling_map,
                       double *prev_raw_x, double *prev_raw_y, double *prev_raw_theta);
bool navigate_one_leg(FPGALink *fpga, LidarThread *lidar, DynamicNavigator *nav,
                       OccupancyGridSLAM *nav_slam, const RobotConfig *cfg,
                       bool use_slam_rolling_map,
                       double *prev_raw_x, double *prev_raw_y, double *prev_raw_theta);

#endif /* ROBOT_NAVIGATE_H */
