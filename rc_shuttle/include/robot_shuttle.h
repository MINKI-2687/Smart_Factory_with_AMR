#ifndef ROBOT_SHUTTLE_H
#define ROBOT_SHUTTLE_H

#include "robot_config.h"
int run_shuttle(int fpga_fd, int lidar_fd, RobotConfig *cfg,
                 MapSource map_source, const char *map_load_path,
                 const char *map_save_path,
                 ShuttlePoint point_a, ShuttlePoint point_b,
                 const TriggerConfig *trigger_cfg);

#endif /* ROBOT_SHUTTLE_H */
