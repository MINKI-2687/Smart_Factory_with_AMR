#ifndef ROBOT_SLOT_ALIGN_H
#define ROBOT_SLOT_ALIGN_H

#include "robot_config.h"
#include "robot_rotrate.h"
bool robot_runner__sweep_rotate(FPGALink *fpga, const RobotConfig *cfg,
                                 const LidarScan *scan, RotRateEst *rot_est,
                                 double err_rad, double *out_pred_dth,
                                 double *out_sec);
bool slot_lane_trim(FPGALink *fpga, LidarThread *lidar,
                     OccupancyGridSLAM *nav_slam, const RobotConfig *cfg,
                     double lane_x, double tol_m,
                     double *prev_rx, double *prev_ry, double *prev_rth);
bool robot_runner__ensure_rotation_room(FPGALink *fpga, LidarThread *lidar,
OccupancyGridSLAM *nav_slam, const RobotConfig *cfg,
double turn_rad, double out_sign_y, double max_total_m,
double *prev_rx, double *prev_ry, double *prev_rth);
bool slot_align_inner(FPGALink *fpga, LidarThread *lidar, OccupancyGridSLAM *nav_slam,
                 const RobotConfig *cfg, double heading_rad, double tol_deg);
bool slot_align(FPGALink *fpga, LidarThread *lidar, OccupancyGridSLAM *nav_slam,
                 const RobotConfig *cfg, double heading_rad, double tol_deg);

/* ============================================================
 * 슬롯 진입 전 '좌우' 정렬 (2026-08-08 추가)
 *
 * 왜 필요한가: slot_align()은 각도만 맞춤. 그래서 대기점 도착 위치가 차선에서
 * 40mm 벗어나 있어도 그대로 진입을 시작했음. 슬롯 폭 170mm, 차폭 130mm라
 * 좌우 여유가 ±20mm뿐이므로 40mm면 깔때기에 코를 박는 게 당연함.
 *
 * 실기 로그(A슬롯): 진입 시작 시 좌우오차 +40mm
 *   -> 진입하면서 +65mm까지 벌어지고, 각도는 -0.5도에서 +21.5도까지 틀어짐
 *   -> 차가 y=0.38에 박힌 채 조향을 최대(diff=10)로 걸고 계속 밀기만 함
 *
 * 직진하면서는 못 고침: 진입 시작(y=0.545)부터 슬롯 입구(y=0.45)까지 95mm 안에
 * 40mm를 지우려면 진행각이 23도나 필요한데, 제어기는 각도를 -90도로 붙잡고 있음.
 * 원리적으로 불가능하므로 '들어가기 전에' 옆으로 옮겨야 함.
 *
 * 방법: 차동구동은 옆으로 못 가니 사람이 주차할 때처럼
 *   (1) 차선 쪽을 향하도록 90도 돌고  (2) 오차만큼 전진하고  (3) 다시 슬롯을 봄.
 * 대기점은 통로 한복판이라 이 기동을 할 공간이 있음.
 * ============================================================ */
bool slot_align(FPGALink *fpga, LidarThread *lidar, OccupancyGridSLAM *nav_slam,
                               const RobotConfig *cfg, double heading_rad, double tol_deg);
bool slot_lateral_align(FPGALink *fpga, LidarThread *lidar,
                         OccupancyGridSLAM *nav_slam, const RobotConfig *cfg,
                         double lane_x, double slot_heading_rad);
bool slot_fix_heading_inner(FPGALink *fpga, LidarThread *lidar,
                       OccupancyGridSLAM *nav_slam, const RobotConfig *cfg,
                       double target_heading_rad, double tol_deg, bool jammed,
                       double back_limit_y, int release_dir,
                       double *prev_rx, double *prev_ry, double *prev_rth,
                       double *out_herr_deg);
bool slot_fix_heading(FPGALink *fpga, LidarThread *lidar,
                       OccupancyGridSLAM *nav_slam, const RobotConfig *cfg,
                       double target_heading_rad, double tol_deg, bool jammed,
                       double back_limit_y, int release_dir,
                       double *prev_rx, double *prev_ry, double *prev_rth,
                       double *out_herr_deg);
bool *robot_runner__slot_no_wall(const RobotConfig *cfg);

#endif /* ROBOT_SLOT_ALIGN_H */
