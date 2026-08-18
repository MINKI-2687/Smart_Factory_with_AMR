#ifndef ROBOT_MAP_H
#define ROBOT_MAP_H

#include "robot_config.h"
bool run_slam_mapping_phase(FPGALink *fpga, LidarThread *lidar, const RobotConfig *cfg,
                             unsigned char **out_grid, int *out_rows, int *out_cols,
                             double *out_x, double *out_y, double *out_theta);
void robot_runner__print_map_line(const unsigned char *grid, int rows, int cols,
                                    double resolution);
bool acquire_map(FPGALink *fpga, LidarThread *lidar, RobotConfig *cfg,
                  MapSource map_source, const char *map_load_path,
                  const char *map_save_path,
                  unsigned char **out_grid, int *out_rows, int *out_cols,
                  double *out_final_x, double *out_final_y, double *out_final_theta);

#endif /* ROBOT_MAP_H */
