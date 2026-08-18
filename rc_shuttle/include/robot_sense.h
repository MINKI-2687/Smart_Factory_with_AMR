#ifndef ROBOT_SENSE_H
#define ROBOT_SENSE_H

#include "robot_config.h"
double robot_runner__sector_min(const LidarScan *scan,
                                 double center_deg, double half_deg);
double robot_runner__min_clearance_dir(const LidarScan *scan,
                                        double *out_dist);
double robot_runner__scan_change(const LidarScan *a, const LidarScan *b);
double robot_runner__body_reach(double phi, double half_len, double half_w);
bool robot_runner__rotation_is_safe(const LidarScan *scan, double dtheta_rad,
                                     double half_len, double half_w,
                                     double margin,
                                     double *out_min_gap, double *out_dir_deg,
                                     double *out_need_m);
double robot_runner__max_safe_rotation_frac(const LidarScan *scan,
                                             double dtheta_rad,
                                             double half_len, double half_w,
                                             double margin);
void robot_runner__front_side_clearance(const LidarScan *scan,
                                         double *out_left, double *out_right);
int robot_runner__blocked_side(const LidarScan *scan,
                                double *out_l, double *out_r);
bool robot_runner__slot_lateral_from_scan(const LidarScan *scan,
                                           double *out_off_m);
bool robot_runner__lane_offset_body(double x, double lane_x,
                                     double heading_rad, double *out_left_m);

/* ============================================================
 * 끼임 탈출 (2026-08-10 전면 개정) *** 1번 문제의 근본 원인 ***
 *
 * 사용자 신고: "A슬롯 탈출 후 그 자리에서 앞뒤로 계속 반복적으로 움직인다.
 *   무의미하고 답답하다. 전진/후진 둘 중 하나만 천천히 한 다음 각도 틀고
 *   이동하면 되지 않나? 너무 확확 움직이면 벽에 부딪혀서 오히려 더 나빠진다."
 *
 * 사용자 진단이 정확하다. 예전 코드가 왕복한 이유는 정확히 세 가지였다.
 *
 * (1) 방향을 매 시도마다 다시 골랐다.
 *         int dir = (front >= rear) ? +1 : -1;
 *     이 줄이 루프 안에 있었다. 후진하면 후면 여유가 줄고 정면 여유가 늘어나므로,
 *     다음 시도에서는 반드시 전진이 선택된다. 그리고 전진하면 다시 후진이 선택된다.
 *     즉 이 코드는 구조적으로 앞뒤 왕복을 하게 되어 있었다. 여기에
 *         crab_left = -crab_left;  "탈출 효과 없음 - 미는 방향을 반대로 바꿉니다"
 *     까지 겹쳐서 왕복이 확정됐다.
 *     -> 고침: 방향을 처음 한 번만 정하고 끝까지 고정한다(dir_lock). 그 방향이
 *        물리적으로 막혔을 때만 딱 한 번 뒤집는다.
 *
 * (2) 한 번에 너무 크게 밀었다. 상한이 120mm 였고, 배율 보정이 수렴하는 중이라
 *     실제로는 178mm 까지 나갔다(로그 확인). 벽 앞 40mm 여유를 두고 밀었는데
 *     60mm 를 더 가면 그대로 들이받고, 자세가 틀어져서 또 탈출해야 한다.
 *     -> 고침: 상한 45mm, 보수적 시간 환산(move_sec_for_safe), 느린 속도.
 *
 * (3) SLAM 에게 움직였다고 알려주지 않았다. 탈출은 120~178mm 를 가는데 그 사이
 *     자세추정은 한 번도 갱신되지 않았고, 끝난 뒤 호출부는 '탈출 전 자세'로
 *     경로를 다시 그렸다. 즉 방금 빠져나온 그 자리로 되돌아가는 경로가 나왔다.
 *     게다가 다음 스텝의 탐색창은 ±100mm 인데 실제로는 178mm 를 움직였으므로
 *     정답이 창 밖이었다 - 원리적으로 복구 불가능.
 *     -> 고침: 이동량을 cmd_odom 에 기록하고, 호출부가
 *        robot_runner__relocalize_after_move() 로 다시 위치를 잡은 뒤 재계획한다.
 *
 * max_move_m: 이번 탈출에서 허용할 최대 직선 이동거리(m). 0 이하면 기본값(45mm).
 * ============================================================ */
#define ESCAPE_STEP_M      0.045   /* 한 번에 미는 최대 거리 - 예전 0.12 에서 축소 */
#define ESCAPE_MAX_TRY     5       /* 45mm x 5 = 225mm 까지 한 방향으로 밀 수 있음 */

#endif /* ROBOT_SENSE_H */
