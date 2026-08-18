#ifndef ROBOT_ODOM_H
#define ROBOT_ODOM_H

#include "robot_config.h"
void robot_runner__get_odom_delta(FPGALink *fpga, bool has_encoder,
                                    double *prev_x, double *prev_y, double *prev_theta,
                                    double *out_dx, double *out_dy, double *out_dtheta);
void robot_runner__init_prev_pose(FPGALink *fpga, bool has_encoder,
                                    double *prev_x, double *prev_y, double *prev_theta);
double *robot_runner__travel_acc(void);
double robot_runner__reloc_budget(void);
void robot_runner__travel_acc_reset(void);
void robot_runner__travel_acc_set(double m);
void robot_runner__predict_delta(FPGALink *fpga, const RobotConfig *cfg,
                                  double heading,
                                  double *prev_x, double *prev_y,
                                  double *prev_theta,
                                  double *out_dx, double *out_dy,
                                  double *out_dtheta);

/* 스캔매칭 탐색창 임시 교체 (2026-08-09h).
 * 제자리 회전 전용 창으로 바꾸면 한 스텝에 두 배를 돌 수 있다 - RobotConfig 의
 * rot_win_* 주석 참고. 반드시 짝을 맞춰 되돌릴 것. */
typedef struct {
    double win_m, step_m, win_th, step_th;
    bool saved;
} SlamWinSave;
void slam_win_push_rotation(OccupancyGridSLAM *s, const RobotConfig *cfg,
                             SlamWinSave *sv);
void slam_win_pop(OccupancyGridSLAM *s, SlamWinSave *sv);
double robot_runner__clamp_rotation_sec_rate(const RobotConfig *cfg,
                                              const OccupancyGridSLAM *nav_slam,
                                              double rate_rad_s, double sec);
void robot_runner__odom_report(const RobotConfig *cfg, FPGALink *fpga,
                                double odom_dx, double odom_dy, double odom_dth,
                                double slam_dx, double slam_dy, double slam_dth);

#endif /* ROBOT_ODOM_H */
