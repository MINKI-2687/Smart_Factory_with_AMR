#ifndef SAFE_NAVIGATOR_H
#define SAFE_NAVIGATOR_H

#include <math.h>
#include <stdbool.h>
#include "angle_utils.h"
#include "lidar_types.h"
#include "wheel_cmd.h"
#include "goal_pid.h"
#include "obstacle_avoidance.h"

/* ============================================================
 * SafeNavigator (목표 방향 + 장애물 회피를 합쳐 최종 좌/우 바퀴속도 산출)
 * goal_pid.h(목표추종)와 obstacle_avoidance.h(회피)를 조합만 함 - 둘 중
 * 어느 쪽 로직도 재구현하지 않고 그대로 부품처럼 사용.
 *
 * PATCH (2026-08-05): wander_avoid.c에서 실측으로 확인된 것과 같은 원인의 진동
 * 버그가 여기(=main_shuttle.c가 실제로 쓰는 주행경로)에도 그대로 있었음 - 정면에
 * 좌우대칭인 장애물(평평한 벽 등)을 만나면 compute_repulsion()의 반발벡터가 매
 * 스텝(라이다 노이즈 수준으로) 아주 조금씩만 달라지는데, 그 미세한 차이가
 * atan2(combined_y, combined_x)의 "방향"을 스텝마다 뒤집기에는 충분해서 - 특히
 * 목표유도력(attract)과 회피력(repel)이 서로 반대방향으로 거의 상쇄될 때 결합벡터의
 * 크기 자체가 작아지고, 크기가 작은 벡터일수록 atan2 결과가 미세한 잡음에도
 * 민감해짐 - 좌우 바퀴명령이 스텝마다 뒤집히는 "까딱거림"이 발생함.
 * wander_avoid.c는 "이산적인 좌/우 선택"에 데드밴드+메모리(직전 방향 유지)를 걸어서
 * 고쳤는데, 여기는 회피가 연속적인 벡터라 방식을 그대로 못 옮기고, 같은 목적을
 * 반발벡터 자체의 저역통과 필터링(직전 스텝 값을 기억해서 매 스텝 급격히 안 바뀌게)
 * 으로 구현함 - 결국 "노이즈만으로는 방향이 안 뒤집히고, 확실한 변화만 반영된다"는
 * 같은 효과를 냄. 새 목표(다음 웨이포인트/구간)로 넘어갈 때는 이전 장애물 상황과
 * 무관하므로 필터를 초기화함(아래 was_at_goal 리셋 지점 참고). */
#define SAFE_NAVIGATOR_REPEL_SMOOTH_ALPHA 0.15  /* 0~1, 작을수록 더 안정적(느리게 반응),
                                                    클수록 더 즉각적(노이즈에 더 민감) */

/* ============================================================
 * 제자리 회전 전환 히스테리시스 (2026-08-09e)
 *
 * 예전엔 |angle_error| > 30도 하나로 "선속도 0 = 제자리 회전"을 결정했다. 경계에
 * 걸치면 스텝마다 회전/전진이 뒤집히는데, 이 코드베이스에서 회전 스텝은
 * navigate_one_leg 의 '회전 여유 검사 -> 끼임 탈출(앞뒤로 12cm)'을 불러오기 때문에
 * 경계 떨림 한 번이 곧바로 큰 앞뒤 왕복이 된다. 들어가는 문턱과 나오는 문턱을
 * 분리해서 한 번 전진 모드로 들어오면 웬만해선 유지되게 한다.
 *
 * 값 근거: path_follow.h 의 적응형 전방주시가 목표 방위각을 경로 접선에서 최대
 * 21.8도까지만 꺾으므로, 차체가 접선에서 13도 이내면 총 오차는 35도 미만이다.
 * ============================================================ */
#define SAFE_NAV_ROTATE_ENTER_DEG 35.0
#define SAFE_NAV_ROTATE_EXIT_DEG  22.0

typedef struct {
    GoalPIDController *goal_controller;
    ObstacleAvoidance *avoider;
    bool was_at_goal;
    double filt_repel_x, filt_repel_y;
    bool has_filt_repel;
    bool rotate_only;   /* 히스테리시스 상태: 지금 제자리 회전 모드인가 */
    /* 직전 계산의 방위각 오차(rad). 상위(navigate_one_leg)가 "이번엔 크게 한 번에
     * 돌려도 되는가"를 판단하는 데 쓴다 (2026-08-09j). */
    double last_angle_error;
} SafeNavigator;
void safe_navigator_init(SafeNavigator *s, GoalPIDController *goal_controller,
                          ObstacleAvoidance *avoider);
WheelCmd safe_navigator_compute(SafeNavigator *s, double x, double y, double theta,
                                 double target_x, double target_y, const LidarScan *scan);

#endif /* SAFE_NAVIGATOR_H */
