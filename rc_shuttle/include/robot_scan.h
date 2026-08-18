#ifndef ROBOT_SCAN_H
#define ROBOT_SCAN_H

#include "robot_config.h"

void apply_lidar_frame_fix(LidarScan *scan, bool mirror, double offset_deg,
                            double offset_forward_m);
bool wait_for_lidar_ready(LidarThread *lidar, int min_points,
                            double timeout_sec, LidarScan *out_scan);
bool wait_for_fresh_scan(LidarThread *lidar, LidarScan *out, double timeout_sec);
void robot_runner__sleep_sec(double sec);

bool robot_runner__take_scan(LidarThread *lidar, const RobotConfig *cfg,
                             LidarScan *out, double timeout_sec);

#endif /* ROBOT_SCAN_H */
