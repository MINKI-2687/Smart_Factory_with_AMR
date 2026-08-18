#include "docking.h"


void docking_init(DockingController *d) {
    d->k_rho = 60; d->k_align = 70; d->k_final_align = 70;
    d->position_tolerance = 0.02;
    d->angle_tolerance = 15.0 * ANGLE_PI / 180.0;
    d->max_linear_speed = 50; d->max_angular_speed = 30;
    /* PATCH (2026-08-05): min_linear=6 / min_angular=4는 fpga_serial.h의 펄스구동
     * 기준으로 ON 듀티 10%(0.6초에 한 번 60ms 움직임) 수준이라 반응이 지나치게 굼뜨고,
     * 그 사이사이 각도보정만 계속 들어가서 제자리 진동처럼 보임. 이 하드웨어는 애초에
     * 연속 저속이 불가능하고 "짧게 확 움직이고 쉬기"만 가능하므로, 바닥값을 올려서
     * 한 번 움직일 때 확실히 움직이게 함(듀티 30~40%). */
    d->min_linear_speed = 18; d->min_angular_speed = 14;
    d->max_rotation_steps = 400;
    d->rotation_step_count = 0;
    d->position_reached = false;
    d->best_heading_error = 1e300;
    d->steps_since_improvement = 0;

    d->approach_rotating = true;    /* 첫 스텝은 안전하게 각도부터 맞추고 시작 */
    d->approach_align_enter = 8.0 * ANGLE_PI / 180.0;
    d->approach_align_exit = 20.0 * ANGLE_PI / 180.0;
    d->best_rho = 1e300;
    d->approach_no_improve = 0;
    d->approach_stall_limit = 12;
    d->approach_steps = 0;
    d->approach_max_steps = 60;
    d->close_range_m = 0.05;
    d->close_mode = false;
    d->approach_have_latch = false;
    d->approach_target_heading = 0.0;
    d->close_sign_flips = 0;
    d->close_prev_sign = 0;
}


void docking_reset(DockingController *d) {
    d->position_reached = false;
    d->rotation_step_count = 0;
    d->best_heading_error = 1e300;
    d->steps_since_improvement = 0;
    d->approach_rotating = true;
    d->best_rho = 1e300;
    d->approach_no_improve = 0;
    d->close_mode = false;
    d->approach_have_latch = false;
    d->approach_steps = 0;
    d->close_sign_flips = 0;
    d->close_prev_sign = 0;
}


double docking_apply_floor(double value, double min_mag, double max_mag) {
    if (value == 0) return 0;
    double mag = fabs(value);
    if (mag < min_mag) mag = min_mag;
    if (mag > max_mag) mag = max_mag;
    return value > 0 ? mag : -mag;
}


WheelCmd docking_compute(DockingController *d,
                                        double x, double y, double theta,
                                        double goal_x, double goal_y, double goal_theta) {
    WheelCmd cmd;
    double dx = goal_x - x;
    double dy = goal_y - y;
    double rho = hypot(dx, dy);
    double heading_error = normalize_angle(theta - goal_theta);

    if (rho < d->position_tolerance) {
        d->position_reached = true;
    }

    /* ============================================================
     * 근거리(<= close_range_m) 보정: '목표 방향각' 추종을 그만둔다.
     *
     * BUGFIX (2026-08-08e): 목표까지 남은 거리가 짧아지면 방향각 제어가 원리적으로
     * 발산한다. 이 로봇은 자세기준점(스캔매칭이 내놓는 좌표)이 실제 회전중심과
     * 일치하지 않아서, 제자리 회전을 해도 자세가 그 차이만큼 원을 그린다.
     *
     * 실기 로그(2026-08-08) 실측: cmd=(30,-30) 으로 88도 제자리 회전하는 동안
     * pose 가 53mm 이동 -> 자세기준점이 회전중심에서 38mm 떨어져 있음.
     * 그때 목표까지 거리는 51mm 였다. 즉 로봇이 도는 동안 목표의 '방향'도 거의
     * 같은 속도로 같이 돌아버려서, alpha 가 -0.88 ~ -0.96 rad 에 고정된 채
     * 12스텝이 지나도 전혀 줄지 않았다(로그의 계산으로 확인). 그 뒤 100스텝 넘게
     * 제자리에서 계속 자전한 것이 이 무한 궤도다.
     *
     * 방향각을 못 쓰면 무엇을 쓰나: '목표 자세각(goal_theta)' 은 위치와 무관하므로
     * 회전중심 오차에 영향을 받지 않는다. 그래서
     *   (1) 먼저 goal_theta 로 자세를 맞추고
     *   (2) 그 축을 따라 앞/뒤 직진으로 '전후 성분'만 지운다
     * 로 바꾼다. 남는 '좌우 성분'은 차동구동으로는 직접 못 지우므로, 상위에서
     * slot_lateral_align() 이 (90도 회전 -> 직진 -> 복귀) 기동으로 처리한다.
     * 두 동작 모두 직진 아니면 제자리 회전이라 이 하드웨어에 맞는다.
     * ============================================================ */
    if (rho <= d->close_range_m) d->close_mode = true;   /* 한번 들어가면 유지 */
    if (!d->position_reached && d->close_mode) {
        double herr = normalize_angle(theta - goal_theta);
        if (fabs(herr) > d->approach_align_enter) {
            double w = d->k_final_align * (-herr);
            w = docking_apply_floor(w, d->min_angular_speed, d->max_angular_speed);
            cmd.left = (int)lround(-w);
            cmd.right = (int)lround(w);
            cmd.done = false;
            return cmd;
        }
        /* 목표오차를 자세각 축으로 투영 - 전후 성분만 직진으로 제거 */
        double along = dx * cos(goal_theta) + dy * sin(goal_theta);

        /* PATCH (2026-08-09e): 핑퐁 감지.
         * along 의 부호가 두 번 뒤집혔다면 최소 이동단위가 허용오차보다 커서
         * 목표를 계속 넘나들고 있는 것이다. 여기서 한 번 더 밀어봐야 반대편으로
         * 같은 거리만큼 갈 뿐이므로 받아들이고 최종 각도정렬로 넘어간다.
         * (남은 전후오차는 슬롯 진입이 어차피 다시 잡아준다) */
        int sgn = (along > 0.0) ? +1 : -1;
        if (fabs(along) > d->position_tolerance) {
            if (d->close_prev_sign != 0 && sgn != d->close_prev_sign) d->close_sign_flips++;
            d->close_prev_sign = sgn;
        }
        if (d->close_sign_flips >= 2) {
            d->position_reached = true;
        } else if (fabs(along) > d->position_tolerance) {
            double v = d->k_rho * fabs(along);
            v = docking_apply_floor(v, d->min_linear_speed, d->max_linear_speed);
            int sp = (along > 0) ? (int)lround(v) : -(int)lround(v);
            cmd.left = sp;
            cmd.right = sp;
            cmd.done = false;
            return cmd;
        }
        /* 전후 성분까지 지웠으면 여기서 할 수 있는 건 끝 - 최종 각도정렬로 넘어감 */
        d->position_reached = true;
    }

    if (d->position_reached) {
        if (fabs(heading_error) < d->angle_tolerance) {
            cmd.left = 0; cmd.right = 0; cmd.done = true;
            return cmd;
        }

        /* PATCH (2026-08-03): 엔코더 최소 회전단위(실측 7.43도/틱)의 절반보다 이미
         * 오차가 작으면, 여기서 보정 명령을 한 번 더 내려봤자 최소 1틱만큼(약 7.43도)
         * 무조건 튀어서 반대편으로 더 벌어질 뿐임. 지금이 이 하드웨어로 도달 가능한
         * 사실상 최선이므로 정적 angle_tolerance(15도)보다 더 타이트하게 여기서
         * 미리 멈춤. (PWM 데드존 보정이 같이 적용돼야 의미 있음 - 데드존 보정 없이
         * 이 로직만 있으면 애초에 그 전에 회전 자체가 안 됨) */
        const double MIN_ROTATION_QUANTUM_DEG = 7.43;
        double half_quantum = (MIN_ROTATION_QUANTUM_DEG / 2.0) * ANGLE_PI / 180.0;
        if (fabs(heading_error) < half_quantum) {
            cmd.left = 0; cmd.right = 0; cmd.done = true;
            return cmd;
        }

        /* 오차가 더 안 줄고 제자리에서 맴돌면(핑퐁) 최대스텝(400)까지 기다리지 않고
         * 빨리 포기 - 어차피 더 기다려도 이 근방을 벗어나지 못함 */
        double abs_err = fabs(heading_error);
        if (abs_err < d->best_heading_error - 1e-6) {
            d->best_heading_error = abs_err;
            d->steps_since_improvement = 0;
        } else {
            d->steps_since_improvement++;
            if (d->steps_since_improvement >= 3) {
                cmd.left = 0; cmd.right = 0; cmd.done = true;
                return cmd;
            }
        }

        d->rotation_step_count++;
        if (d->rotation_step_count > d->max_rotation_steps) {
            cmd.left = 0; cmd.right = 0; cmd.done = true;
            return cmd;
        }

        double w = d->k_final_align * (-heading_error);
        if (w > d->max_angular_speed) w = d->max_angular_speed;
        if (w < -d->max_angular_speed) w = -d->max_angular_speed;
        cmd.left = (int)lround(-w);
        cmd.right = (int)lround(w);
        cmd.done = false;
        return cmd;
    }

    /* ============================================================
     * 접근 단계: '제자리 회전' 과 '직진' 을 번갈아 함 (turn-then-straight)
     *
     * 왜 곡선주행을 버렸나 (BUGFIX 2026-08-08e):
     * 이 하드웨어는 PWM 데드존이 170~255로 매우 좁아서(speed_to_pwm 참고), 좌우 바퀴
     * 명령의 '비율'이 실제 속도비로 전혀 나오지 않는다. 실측 환산:
     *
     *   cmd(+34, -9)  -> PWM(+206, -180)  실제 v=0.008m/s, w=-89deg/s, 회전반경 5mm
     *                    (제어기가 의도한 반경은 47mm - 사실상 제자리 회전이 되어버림)
     *   cmd(+34, +9)  -> PWM(+206, +180)  실제 v=0.125m/s, w= -6deg/s, 회전반경 1.19m
     *
     * 즉 **좌우 부호가 갈리면 크기와 무관하게 거의 제자리 회전**이고, 부호가 같으면
     * 거의 직진이다. 그 중간(완만한 곡선)은 이 하드웨어에 존재하지 않는다.
     *
     * 예전 코드는 v=k_rho*rho 를 min_linear_speed(18)로 바닥을 깔고 w 는 최대 30까지
     * 허용해서, 목표에 가까워지면 항상 cmd(+48,-12) 같은 '부호가 갈린' 명령을 냈다.
     * 제어기는 "반경 4.6cm 로 돌아 들어간다"고 믿지만 하드웨어는 그냥 자전한다.
     * 그래서 rho 가 전혀 줄지 않고 alpha 도 수렴하지 않는 무한 궤도에 빠졌다.
     * 실기 로그(2026-08-08): 목표 5cm 앞에서 cmd=(48,-12)->(34,-9) 를 100스텝 넘게
     * 반복하며 pose 각도만 -0.5rad -> -3.7rad 로 계속 돌았음.
     *
     * 해결: 부호가 갈리는 명령은 '의도한 제자리 회전'일 때만 낸다.
     *   (1) 목표 방향과의 각도차가 크면 -> v=0 으로 제자리 회전만
     *   (2) 각도가 맞으면          -> 조향 없이 양 바퀴 같은 값으로 직진만
     * 직진하는 동안 각도가 다시 틀어지면 (2)->(1) 로 돌아온다. 전환 경계에 히스테리시스
     * (들어갈 땐 8도, 나올 땐 20도)를 둬서 경계에서 떨리지 않게 한다.
     * stop-and-go 주행이라 제자리 회전 중에는 위치가 안 변하므로 이 방식이 안전하다.
     * ============================================================ */
    double angle_to_goal = atan2(dy, dx);
    double alpha = normalize_angle(angle_to_goal - theta);

    /* 접근 정체 감지: 더 이상 가까워지지 않으면 위치는 여기까지로 보고 각도정렬로 넘어감.
     * (무한 궤도/벽에 막힘 등으로 영원히 도는 것을 막는 안전망)
     *
     * 중요: '직진 구간'에서만 센다. 제자리 회전 중에는 원래 거리가 줄지 않는 게 정상인데,
     * 회전 스텝까지 세면 180도 회전(약 21스텝) 도중에 정체로 오판해서 목표에서 한참
     * 떨어진 채 접근을 포기해버린다(시뮬레이션에서 잔여 좌우오차 250mm로 확인됨). */
    /* 접근에 쓸 수 있는 총 스텝 상한. 정체 감지에 안 걸리는 느린 발산(예: 회전중심
     * 오차가 커서 조금씩만 가까워지는 경우)도 여기서 반드시 끝난다. */
    if (++d->approach_steps > d->approach_max_steps) {
        d->position_reached = true;
    }

    if (!d->approach_rotating) {
        if (rho < d->best_rho - 0.005) {
            d->best_rho = rho;
            d->approach_no_improve = 0;
        } else if (++d->approach_no_improve >= d->approach_stall_limit) {
            d->position_reached = true;
            d->approach_no_improve = 0;
        }
    }

    /* 회전 목표각 '래칭' (BUGFIX 2026-08-08e).
     *
     * 회전 중에 매 스텝 alpha 를 다시 계산해서 쫓아가면, 자세기준점이 회전중심에서
     * 벗어나 있는 만큼 '목표의 방향' 도 로봇과 같이 돌기 때문에 영원히 못 따라잡는다.
     * 실기 로그에서 alpha 가 -0.88 ~ -0.96 rad 에 12스텝 내내 고정돼 있던 게 그것이다.
     * 그래서 회전 구간에 들어가는 '그 순간에만' 목표 자세각을 한 번 계산해서 걸어두고,
     * 그 각도에 도달하면 무조건 회전을 끝낸다. 피드백 고리를 끊었으므로 회전 구간은
     * 반드시 유한 스텝에 끝나고, 남은 오차는 다음 '직진 -> 재조준' 사이클이 줄인다. */
    if (!d->approach_rotating && fabs(alpha) > d->approach_align_exit) {
        d->approach_rotating = true;
        d->approach_have_latch = false;
    }
    if (d->approach_rotating && !d->approach_have_latch) {
        d->approach_target_heading = normalize_angle(theta + alpha);
        d->approach_have_latch = true;
    }

    if (d->approach_rotating) {
        double herr = normalize_angle(theta - d->approach_target_heading);
        if (fabs(herr) <= d->approach_align_enter) {
            d->approach_rotating = false;
            d->approach_have_latch = false;
        } else {
            double w = d->k_align * (-herr);
            w = docking_apply_floor(w, d->min_angular_speed, d->max_angular_speed);
            cmd.left = (int)lround(-w);
            cmd.right = (int)lround(w);
            cmd.done = false;
            return cmd;
        }
    }

    /* 직진 구간.
     *
     * PATCH (2026-08-09f): 조향을 '완전히 0'에서 '부호가 안 갈리는 범위 안에서
     * 약간 허용'으로 바꿈.
     *
     * 왜 (= 사용자 신고 "조금 앞으로 가다가 각도 조정, 또 조금 가다가 각도 조정"):
     * 조향이 0이면 직진하는 동안 alpha(목표 방위각 오차)를 전혀 못 줄인다. 그래서
     * 조금만 틀어져 있어도 approach_align_exit(20도)를 금방 넘고, 다시 제자리 회전
     * 구간으로 돌아간다. 이 왕복이 접근 내내 반복돼서 "가다 서다 돌다"가 됐다.
     *
     * 2026-08-08e 의 "조향을 걸면 자전이 된다"는 관측은 정확하지만, 그건 좌우 부호가
     * 갈릴 때 얘기다. 부호가 같으면 speed_to_pwm() 을 지나도 여전히 완만한 호다:
     *   cmd(20,50) -> PWM(191,223) -> 회전반경 약 0.5m
     * 0.5m 반경이면 30cm 접근 동안 34도를 지울 수 있어 alpha 를 충분히 붙잡는다.
     * 그래서 조향량을 선속도의 0.6배로 묶어 부호가 절대 안 갈리게만 한다.
     *
     * 단, 목표에 가까우면(rho <= CLOSE_STEER_M) alpha 자체가 못 믿을 값이 되므로
     * (자세기준점이 회전중심에서 벗어난 만큼 목표 방향도 같이 돈다 - 위 close_mode
     * 주석 참고) 조향을 끄고 예전처럼 곧게 간다. */
    const double CLOSE_STEER_M = 0.10;
    double v = d->k_rho * rho;
    v = docking_apply_floor(v, d->min_linear_speed, d->max_linear_speed);

    double w2 = 0.0;
    if (rho > CLOSE_STEER_M) {
        w2 = d->k_align * alpha;
        double lim = 0.6 * v;
        if (w2 >  lim) w2 =  lim;
        if (w2 < -lim) w2 = -lim;
    }
    cmd.left  = (int)lround(v - w2);
    cmd.right = (int)lround(v + w2);
    cmd.done = false;
    return cmd;
}
