#ifndef ROBOT_ESCAPE_H
#define ROBOT_ESCAPE_H

#include "robot_config.h"

/* ============================================================
 * 구석 끼임 방지/탈출 모듈 (2026-08-10b).
 *
 * 이 위치에서 include 하는 이유: corner_escape.h 의 함수들이 바로 위에 정의된
 * sector_min / body_reach / rotation_is_safe / max_safe_rotation_frac /
 * escape_speed / move_sec_for_safe / predict_delta / emit_state 를 그대로 쓰기
 * 때문이다. 파일 맨 위로 올리면 컴파일되지 않는다.
 * 반대로 아래 escape_push / escape_to_open 은 corner_escape.h 가 제공하는
 * axis_free_dist 를 쓰므로, include 는 반드시 이 두 함수보다 앞이어야 한다.
 * ============================================================ */
bool robot_runner__escape_push(FPGALink *fpga, LidarThread *lidar,
                                const RobotConfig *cfg, int dir, double room,
                                double before_dist, double *out_after_dist);
bool robot_runner__escape_to_open(FPGALink *fpga, LidarThread *lidar,
                                   const RobotConfig *cfg, double max_move_m,
                                   double need_clearance_m, bool allow_crab);

#endif /* ROBOT_ESCAPE_H */
