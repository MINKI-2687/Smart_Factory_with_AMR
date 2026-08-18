#ifndef ROBOT_SLOT_DRIVE_H
#define ROBOT_SLOT_DRIVE_H

#include "robot_config.h"
bool slot_drive(FPGALink *fpga, LidarThread *lidar, OccupancyGridSLAM *nav_slam,
                 const RobotConfig *cfg,
                 double lane_x, double heading_rad,
                 double stop_y, bool forward, const char *label,
                 double entry_lat_err, bool *out_touched);
bool shuttle_validate_point(const unsigned char *grid, int rows, int cols,
                             double res, const RobotConfig *cfg,
                             ShuttlePoint p, const char *label);
bool robot_runner__slot_confirm_park(FPGALink *fpga, LidarThread *lidar,
                                      const RobotConfig *cfg,
                                      OccupancyGridSLAM *slam,
                                      double lane_x, double heading_rad,
                                      double park_y);
void slot_seat(FPGALink *fpga, const RobotConfig *cfg, double heading_rad,
                bool already_touched);

#endif /* ROBOT_SLOT_DRIVE_H */
