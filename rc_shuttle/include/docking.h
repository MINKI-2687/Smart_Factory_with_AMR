#ifndef DOCKING_H
#define DOCKING_H

#include "angle_utils.h"
#include <math.h>
#include <stdbool.h>
#include "wheel_cmd.h"

typedef struct {
    double k_rho, k_align, k_final_align;
    double position_tolerance, angle_tolerance;
    double max_linear_speed, max_angular_speed;
    double min_linear_speed, min_angular_speed;
    int max_rotation_steps;
    int rotation_step_count;
    bool position_reached;
    /* PATCH (2026-08-03): 회전정렬 단계의 핑퐁(제자리 진동) 조기감지용 */
    double best_heading_error;
    int steps_since_improvement;

    /* PATCH (2026-08-08e): 접근 단계를 '제자리 회전 <-> 직진' 교대로 바꾸기 위한 상태.
     * 아래 docking_compute() 의 turn-then-straight 주석 참고. */
    bool approach_rotating;          /* 지금 회전 중인가(true) 직진 중인가(false) */
    double approach_align_enter;     /* 회전 -> 직진 전환 각도(rad) */
    double approach_align_exit;      /* 직진 -> 회전 전환 각도(rad). 히스테리시스용 */
    double best_rho;                 /* 접근 단계 정체 감지용 */
    int approach_no_improve;
    int approach_stall_limit;
    int approach_steps;              /* 접근 단계 총 스텝 수 */
    int approach_max_steps;          /* 이만큼 쓰면 위치는 포기하고 각도정렬로 */
    double close_range_m;            /* 이보다 가까우면 '목표 방향각' 추종을 그만둠 */
    bool close_mode;                 /* 근거리 모드는 한번 들어가면 유지(왔다갔다 방지) */
    bool approach_have_latch;        /* 회전 목표각을 걸어뒀는가 */
    double approach_target_heading;  /* 회전 시작 시점에 '한 번만' 계산한 목표 자세각 */

    /* 근거리 전후보정의 핑퐁 감지 (2026-08-09e).
     * close_mode 는 '목표 자세각 축을 따라 앞/뒤로 밀어 전후오차를 지우는' 단계인데,
     * position_tolerance 가 20mm 인 반면 이 하드웨어의 최소 이동(정지마찰 하한
     * 0.07초)이 실측 20mm 안팎이라, 목표를 사이에 두고 전진/후진을 무한 반복할 수
     * 있다(사용자 신고: "대기점 근처에서 앞뒤로 계속 왔다갔다"). 부호가 두 번
     * 뒤집히면 이미 하드웨어 분해능 바닥이므로 거기서 받아들인다. */
    int    close_sign_flips;
    int    close_prev_sign;
} DockingController;
void docking_init(DockingController *d);
void docking_reset(DockingController *d);
double docking_apply_floor(double value, double min_mag, double max_mag);
WheelCmd docking_compute(DockingController *d,
                          double x, double y, double theta,
                          double goal_x, double goal_y, double goal_theta);

#endif /* DOCKING_H */
