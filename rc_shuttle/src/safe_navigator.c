#include "safe_navigator.h"


void safe_navigator_init(SafeNavigator *s, GoalPIDController *goal_controller,
                                        ObstacleAvoidance *avoider) {
    s->goal_controller = goal_controller;
    s->avoider = avoider;
    s->was_at_goal = true;
    s->filt_repel_x = 0.0; s->filt_repel_y = 0.0;
    s->has_filt_repel = false;
    s->rotate_only = true;   /* 첫 스텝은 안전하게 각도부터 맞추고 시작 */
    s->last_angle_error = 0.0;
}


WheelCmd safe_navigator_compute(SafeNavigator *s, double x, double y, double theta,
                                               double target_x, double target_y, const LidarScan *scan) {
    WheelCmd cmd;
    double dx = target_x - x, dy = target_y - y;
    double distance = hypot(dx, dy);
    GoalPIDController *gc = s->goal_controller;

    if (distance < gc->goal_tolerance) {
        s->was_at_goal = true;
        goal_pid_reset(gc);
        cmd.left = 0; cmd.right = 0; cmd.done = true;
        return cmd;
    }

    if (s->was_at_goal) {
        goal_pid_reset(gc);
        s->was_at_goal = false;
        s->has_filt_repel = false;  /* 새 구간 시작 - 이전 장애물 상황의 잔상을 버림 */
    }

    double attract_x = dx / distance, attract_y = dy / distance;
    double raw_repel_x, raw_repel_y;
    compute_repulsion(s->avoider, scan, theta, &raw_repel_x, &raw_repel_y);

    if (!s->has_filt_repel) {
        s->filt_repel_x = raw_repel_x;
        s->filt_repel_y = raw_repel_y;
        s->has_filt_repel = true;
    } else {
        s->filt_repel_x += SAFE_NAVIGATOR_REPEL_SMOOTH_ALPHA * (raw_repel_x - s->filt_repel_x);
        s->filt_repel_y += SAFE_NAVIGATOR_REPEL_SMOOTH_ALPHA * (raw_repel_y - s->filt_repel_y);
    }

    double combined_x = attract_x + s->filt_repel_x, combined_y = attract_y + s->filt_repel_y;

    double target_angle = atan2(combined_y, combined_x);
    double angle_error = normalize_angle(target_angle - theta);

    double linear_speed, angular_speed;
    goal_pid_compute(gc, distance, angle_error, &linear_speed, &angular_speed);

    /* PATCH (2026-08-08e): 좌우 부호가 갈리는 명령은 '의도한 제자리 회전'일 때만 낸다.
     *
     * 근거(docking.h 의 같은 날짜 주석과 동일한 실측): PWM 데드존이 170~255라
     * cmd(+34,-9) 같은 명령은 실제로 PWM(+206,-180) 이 되어 회전반경 5mm,
     * 즉 제어기가 "반경 4.7cm 곡선"이라고 믿는 명령이 하드웨어에서는 자전이 된다.
     * 이 하드웨어에 완만한 곡선은 존재하지 않으므로, 전진하려는 스텝에서는
     * 조향량을 선속도보다 작게 묶어 두 바퀴 부호가 반드시 같도록 강제한다.
     * 각도가 많이 틀어져 있으면 아예 선속도를 0으로 두고 제자리 회전만 한다
     * (예전의 0.3배 감속은 부호가 갈린 명령을 만들어 자전을 유발했음). */
    /* PATCH (2026-08-09e): 문턱 하나로 딱 자르던 것을 히스테리시스로 바꿈.
     * 이유는 위 SAFE_NAV_ROTATE_* 주석 참고 - 경계 떨림 한 번이 '끼임 탈출'로
     * 번역돼 앞뒤 12cm 왕복을 유발했다. */
    s->last_angle_error = angle_error;
    double ae_deg = fabs(angle_error) * 180.0 / ANGLE_PI;
    if (s->rotate_only) {
        if (ae_deg <= SAFE_NAV_ROTATE_EXIT_DEG) s->rotate_only = false;
    } else {
        if (ae_deg >= SAFE_NAV_ROTATE_ENTER_DEG) s->rotate_only = true;
    }
    if (s->rotate_only) {
        linear_speed = 0;      /* 많이 틀어졌을 때만 제자리 회전 */
    }

    if (linear_speed > gc->max_linear_speed) linear_speed = gc->max_linear_speed;
    if (linear_speed < -gc->max_linear_speed) linear_speed = -gc->max_linear_speed;
    if (angular_speed > gc->max_angular_speed) angular_speed = gc->max_angular_speed;
    if (angular_speed < -gc->max_angular_speed) angular_speed = -gc->max_angular_speed;

    if (linear_speed != 0.0) {
        /* 전진 중 - 부호가 갈리지 않도록 조향 제한.
         *
         * PATCH (2026-08-09e): 0.5 -> 0.8.
         * 0.5 로 묶으면 좌우 명령비가 최대 3:1 이고, 이 값이 speed_to_pwm() 의
         * 데드존 재매핑(170~255)을 통과하면 PWM비가 겨우 1.13 -> 선회반경 약 1.3m 다.
         * 폭 1.4m 통로에서 그 반경으로는 경로 복귀가 사실상 불가능해서, 결국
         * "각도가 안 줄어듦 -> 제자리 회전 -> 회전 여유 부족 -> 앞뒤 왕복"으로
         * 이어졌다. 0.8 이면 명령비 9:1, PWM비 1.20, 반경 약 0.9m 로 통로 안에서
         * 충분히 붙는다. 부호는 여전히 갈리지 않으므로 '의도치 않은 자전'은 생기지
         * 않는다(docking.h 의 데드존 실측 주석 참고). */
        double lim = 0.8 * fabs(linear_speed);
        if (angular_speed > lim) angular_speed = lim;
        if (angular_speed < -lim) angular_speed = -lim;
    }

    cmd.left = (int)(linear_speed - angular_speed);
    cmd.right = (int)(linear_speed + angular_speed);
    cmd.done = false;
    return cmd;
}
