#ifndef CORNER_ESCAPE_H
#define CORNER_ESCAPE_H

#include "robot_config.h"
/* ============================================================
 * 구석 끼임 방지 / 탈출 (2026-08-10b 신규)
 *
 * *** 이 헤더는 robot_runner.h 의 중간에서 include 된다 ***
 * (robot_runner__escape_push() 정의 바로 뒤). 아래 함수들이 그 위에 정의된
 * sector_min / body_reach / rotation_is_safe / move_sec_for_safe 등을 쓰기 때문이다.
 * 단독으로 include 하면 컴파일되지 않는다.
 *
 * ------------------------------------------------------------
 * 왜 만드나 - 실기 로그와 지도에서 확인된 사실
 * ------------------------------------------------------------
 * set_map.txt (150x90, 10mm 격자) 기준 실제 치수:
 *     통로   : y 0.45 ~ 0.85  (높이 400mm),  x 0.05 ~ 1.45
 *     B슬롯  : x 1.17 ~ 1.38  (폭 210mm),  중심 x = 1.275
 *     B대기점: (1.275, 0.6475)  ->  오른쪽 벽(x=1.45)까지 175mm
 *     차체   : 255 x 130mm  ->  반길이 127.5, 반폭 65, 반대각선 143.1mm
 *
 * 즉 B 대기점에서 제자리로 90도를 돌려면 오른쪽 벽 방향으로 143.1 + 여유 15
 * = 158mm 가 필요한데 실제로는 175mm 뿐이다. 여유가 겨우 17mm 다.
 * (A 대기점은 왼쪽 벽까지 175mm 로 같지만, A->B 로 올 때는 로봇이 +x 로
 *  달려와서 멈추므로 넘침이 항상 벽 쪽으로 생긴다. 그래서 B에서만 터진다.)
 *
 * 그런데 예전 robot_runner__ensure_rotation_room() 은 물러날 방향을
 *     double fwd_y = sin(th) * out_sign_y;
 *     int dir = (fwd_y >= 0.0) ? +1 : -1;
 * 로 골랐다. 대기점에 막 도착한 로봇은 통로를 따라 왔으므로 heading 이 0도
 * 근처다. sin(0)=0 이라 조건이 참이 되어 '전진'이 뽑히는데, heading 0도에서
 * 전진은 곧 +x, 즉 오른쪽 벽 쪽이다. 슬롯 축(y)으로는 sin(0)*d = 0mm 밖에
 * 못 벌면서 벽까지 거리만 그대로 까먹는다.
 * 실기 로그가 정확히 그 모습이다:
 *     [slot] 돌 자리 부족(-66deg 방향 144mm < 필요 158mm) -> 슬롯 밖으로 전진 14mm
 *     [slot] 돌 자리확보: 정면 여유가 142mm 뿐이라 더 못 물러납니다
 * 142mm = 반길이 127.5 + 여유 15. 즉 코앞 15mm 까지 벽에 밀어붙인 뒤 멈췄다.
 * 그 자세에서 90도를 돌리라고 하니 당연히 안 돌고, 끼임 -> 탈출 -> 실패가 된다.
 *
 * 두 번째 문제. 그렇게 물리면 robot_runner__escape_to_open() 이 불리는데,
 * 그 함수는 '앞/뒤 부채꼴(+-25도) 최소거리'로 갈 수 있는 거리를 판단하고
 *     room = clearance - 127.5mm - MARGIN(40mm)
 *     if (room <= 10mm) 포기
 * 를 쓴다. 즉 clearance 가 177.5mm 보다 작으면 시도조차 안 한다. 로그의
 *     [main] 탈출 실패: 앞(155mm) 뒤(149mm) 모두 막혔습니다. 손으로 조금 빼주세요.
 * 가 그것인데, 앞으로 155-127.5 = 27.5mm 는 실제로 비어 있었다. 게다가
 * 부채꼴 +-25도 는 옆벽까지 같이 물고 들어와서(구석에서는 항상 그렇다)
 * 정면 여유를 실제보다 작게 읽는다. 즉 '갈 수 있는데 못 간다고 판단'했다.
 *
 * ------------------------------------------------------------
 * 이 파일이 제공하는 것
 * ------------------------------------------------------------
 *  1) robot_runner__axis_free_dist()
 *     부채꼴이 아니라 '직사각형 차체를 축방향으로 밀어넣는 스윕' 으로 정확히
 *     계산한 전/후진 가능거리. 옆벽이 섞여 들어오지 않는다.
 *  2) robot_runner__rot_gap_shifted()
 *     "축방향으로 s 만큼 옮긴 다음 dtheta 를 돌면 여유가 얼마나 되나" 를
 *     스캔 복사 없이 예측한다.
 *  3) robot_runner__best_axis_shift()
 *     그 예측을 써서 '필요한 여유를 만드는 가장 작은 이동'을 찾는다.
 *     최대가 아니라 최소를 고르는 이유: 대기점에서 많이 벗어나면 뒤이어
 *     slot_lateral_align(90도 회전 두 번)이 불려서 오히려 손해다.
 *  4) robot_runner__arc_pass()
 *     전/후진하면서 동시에 회전하는 호(arc) 이동. 제자리 회전이 불가능한
 *     구석에서도 각도를 바꿀 수 있는 유일한 수단이다.
 *  5) robot_runner__corner_escape()
 *     사용자 요구 그대로 "구석 반대쪽으로 확 크게 돌아서 빠져나오기".
 *     좌우 두 회전방향 중 여유가 큰 쪽을 라이다로 직접 골라서, 한 번에
 *     최대 60도까지 연속 회전한다.
 * ============================================================ */

/* 한 라운드에 눈 감고 도는 상한(도). 스캔매칭 각도창(±25도)보다 크지만,
 * 탈출은 어차피 끝난 뒤 호출부가 relocalize 를 하므로 허용한다.
 * 대신 라운드마다 반드시 멈춰서 다시 관측한다. */
#define CORNER_TURN_MAX_DEG    60.0
#define CORNER_TURN_MIN_DEG     8.0    /* 이보다 작게 돌면 의미가 없다 */
#define CORNER_ROUNDS_MAX       6
#define CORNER_SHIFT_MAX_M      0.060  /* 한 라운드 직선 이동 상한 */
#define CORNER_WALL_MARGIN_M    0.012  /* 벽 앞에 남길 여유 - 예전 40mm 는 과했다 */
#define CORNER_SIDE_MARGIN_M    0.010  /* 축방향 스윕 폭에 더할 좌우 여유 */
#define CORNER_ROT_MARGIN_M     0.008  /* 탈출 회전에 요구할 여유(주행용 15mm보다 작게) */
/* 어느 쪽으로든 이만큼 직진할 수 있게 되면 '풀렸다'고 본다. 그 다음은 호출부의
 * 재위치추정 + 경로 재계획이 정상적으로 이어받는다. */
#define CORNER_FREE_RUN_M       0.050
/* 사방 여유가 전부 0 이하라 '안전한 회전량'이 계산조차 안 될 때 쓰는 강제 회전.
 * 제자리 회전은 차체 중심을 옮기지 않으므로 직진과 달리 벽으로 더 밀지 않는다. */
#define CORNER_FORCED_DEG      45.0
#define CORNER_FORCED_MAX         3
double robot_runner__axis_free_dist(const LidarScan *scan, int dir,
                                     double half_len, double half_w,
                                     double side_margin);
double robot_runner__rot_gap_shifted(const LidarScan *scan, double shift_m,
                                      double dtheta_rad,
                                      double half_len, double half_w,
                                      double *out_dir_deg);
double robot_runner__best_axis_shift(const LidarScan *scan, double dtheta_rad,
                                      double half_len, double half_w,
                                      double back_limit, double fwd_limit,
                                      double need_gap, double *out_gap);
void robot_runner__arc_pass(FPGALink *fpga, const RobotConfig *cfg,
                             bool forward, bool ccw, double sec);
double robot_runner__corner_spin(FPGALink *fpga, const RobotConfig *cfg,
                                  double dtheta_rad);
bool robot_runner__corner_escape(FPGALink *fpga, LidarThread *lidar,
                                  const RobotConfig *cfg,
                                  double want_turn_rad, double need_gap_m,
                                  const char *tag, double *out_moved);
bool robot_runner__corner_backoff(FPGALink *fpga, LidarThread *lidar,
                                   OccupancyGridSLAM *nav_slam,
                                   const RobotConfig *cfg,
                                   double turn_rad, double need_gap_m,
                                   double max_total_m,
                                   double *prev_rx, double *prev_ry,
                                   double *prev_rth);

#endif /* CORNER_ESCAPE_H */
