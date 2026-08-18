#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include "angle_utils.h"
#include "wheel_cmd.h"
#include "docking.h"

/* ---- 실측 기반 하드웨어 모델 ---- */
#define CONTROLLER_MAX_SPEED 80
#define PWM_SAFETY_CAP 207
#define PWM_DEADZONE_HW 100      /* 실측: 이 미만이면 정지마찰 못 이겨서 안 돎 */
#define ENCODER_TICKS_PER_REV 20
#define WHEEL_DIAMETER 0.066
#define WHEEL_SEPARATION 0.160

/* 패치 전 방식: 선형 매핑 (데드존 보정 없음) */
static int pwm_mag_linear(int speed) {
    int mag = (int)lround((double)abs(speed) / CONTROLLER_MAX_SPEED * PWM_SAFETY_CAP);
    if (mag > PWM_SAFETY_CAP) mag = PWM_SAFETY_CAP;
    return mag;
}
/* 패치 후 방식: 데드존 보정 매핑 */
static int pwm_mag_deadzone_compensated(int speed) {
    int abs_speed = abs(speed);
    if (abs_speed == 0) return 0;
    int mag = PWM_DEADZONE_HW + (int)lround((double)abs_speed / CONTROLLER_MAX_SPEED * (PWM_SAFETY_CAP - PWM_DEADZONE_HW));
    if (mag < PWM_DEADZONE_HW) mag = PWM_DEADZONE_HW;
    if (mag > PWM_SAFETY_CAP) mag = PWM_SAFETY_CAP;
    return mag;
}

/* 실제 하드웨어: PWM이 데드존 미만이면 물리적으로 0 rad/s. 그 이상이면 PWM에 비례한 각속도.
 * (대략적 선형모델: PWM_SAFETY_CAP에서 이론적 최대 바퀴각속도) */
static double meters_per_tick(void) { return (ANGLE_PI * WHEEL_DIAMETER) / ENCODER_TICKS_PER_REV; }

typedef struct {
    bool use_deadzone_fix;
    bool use_quantum_stop_fix; /* docking.h 자체 패치는 코드에 이미 들어있어서 항상 적용됨.
                                  여기서는 "실측 하드�디어 물리 모델을 켜고 끄는" 용도만 구분 */
    double true_theta;         /* 물리적 실제 각도 */
    double measured_theta;     /* 엔코더로 측정되는 각도 (틱 단위로만 갱신됨) */
    double accumulated_ticks;  /* 아직 1틱이 안 찬 잔여 회전량 누적 */
} SimState;

/* 한 스텝(50ms) 실행: cmd(left,right)를 실제 물리 회전으로 변환 + 엔코더 양자화 반영 */
static void sim_step(SimState *st, WheelCmd cmd, double dt) {
    int mag_l = st->use_deadzone_fix ? pwm_mag_deadzone_compensated(cmd.left)
                                      : pwm_mag_linear(cmd.left);
    int mag_r = st->use_deadzone_fix ? pwm_mag_deadzone_compensated(cmd.right)
                                      : pwm_mag_linear(cmd.right);
    int sign_l = (cmd.left >= 0) ? 1 : -1;
    int sign_r = (cmd.right >= 0) ? 1 : -1;

    /* 데드존 미만이면 물리적으로 완전히 정지 (실측 사실) */
    double pwm_l = (mag_l < PWM_DEADZONE_HW) ? 0.0 : sign_l * mag_l;
    double pwm_r = (mag_r < PWM_DEADZONE_HW) ? 0.0 : sign_r * mag_r;

    /* PWM -> 바퀴 각속도(대략적 선형모델, PWM_SAFETY_CAP에서 최대 5 rad/s 가정) */
    const double MAX_WHEEL_RAD_S = 5.0;
    double wheel_l_rad_s = (pwm_l / PWM_SAFETY_CAP) * MAX_WHEEL_RAD_S;
    double wheel_r_rad_s = (pwm_r / PWM_SAFETY_CAP) * MAX_WHEEL_RAD_S;

    double v_l = wheel_l_rad_s * (WHEEL_DIAMETER / 2.0);
    double v_r = wheel_r_rad_s * (WHEEL_DIAMETER / 2.0);
    double dtheta_true = (v_r - v_l) / WHEEL_SEPARATION * dt;
    st->true_theta = normalize_angle(st->true_theta + dtheta_true);

    /* 엔코더 양자화: 실제 회전량을 tick 단위로만 누적/반영 (소수점 이하는 다음 스텝으로 이월) */
    double mpt = meters_per_tick();
    double d_left_m = wheel_l_rad_s * (WHEEL_DIAMETER / 2.0) * dt;
    double d_right_m = wheel_r_rad_s * (WHEEL_DIAMETER / 2.0) * dt;
    double d_center = (d_left_m + d_right_m) / 2.0;
    (void)d_center;
    double dtheta_wheels_m = (d_right_m - d_left_m); /* 두 바퀴 이동량 차이(m) */
    st->accumulated_ticks += dtheta_wheels_m / mpt;  /* 틱 단위로 환산해서 누적 */

    double whole_ticks = trunc(st->accumulated_ticks);
    if (fabs(whole_ticks) >= 1.0) {
        double realized_dtheta = whole_ticks * mpt / WHEEL_SEPARATION;
        st->measured_theta = normalize_angle(st->measured_theta + realized_dtheta);
        st->accumulated_ticks -= whole_ticks;
    }
    /* whole_ticks가 0이면(=아직 1틱도 안 참) measured_theta는 그대로 - 엔코더 양자화 재현 */
}

static void run_trial(bool use_deadzone_fix, double initial_error_deg, int *out_final_err_millideg,
                       int *out_steps, bool *out_stalled) {
    DockingController d;
    docking_init(&d);
    d.position_reached = true; /* 위치는 이미 도착, 회전정렬만 테스트 */

    SimState st = {0};
    st.use_deadzone_fix = use_deadzone_fix;
    st.true_theta = initial_error_deg * ANGLE_PI / 180.0; /* goal_theta=0 기준 */
    st.measured_theta = st.true_theta; /* 시작 시점엔 실제=측정 동일하다고 가정 */

    const double dt = 0.05;
    int step;
    bool stalled = true;
    for (step = 0; step < 800; step++) {
        WheelCmd cmd = docking_compute(&d, 0, 0, st.measured_theta, 0, 0, 0.0);
        if (cmd.done) { stalled = false; break; }
        sim_step(&st, cmd, dt);
    }

    *out_final_err_millideg = (int)lround(fabs(st.true_theta) * 180.0 / ANGLE_PI * 1000.0);
    *out_steps = step;
    *out_stalled = (step >= 800);
    (void)stalled;
}

int main(void) {
    double test_errors_deg[] = { 5, 10, 15, 20, 25, 30, 45, 60, 90 };
    int n = sizeof(test_errors_deg) / sizeof(test_errors_deg[0]);

    printf("=== 패치 전 (데드존 보정 없음, 선형 PWM 매핑) ===\n");
    printf("%-12s %-14s %-8s %-8s\n", "초기오차(도)", "최종오차(도)", "스텝", "타임아웃?");
    for (int i = 0; i < n; i++) {
        int final_err_md, steps; bool timed_out;
        run_trial(false, test_errors_deg[i], &final_err_md, &steps, &timed_out);
        printf("%-12.1f %-14.3f %-8d %-8s\n", test_errors_deg[i], final_err_md/1000.0, steps,
               timed_out ? "예(타임아웃)" : "아니오");
    }

    printf("\n=== 패치 후 (데드존 보정 적용) ===\n");
    printf("%-12s %-14s %-8s %-8s\n", "초기오차(도)", "최종오차(도)", "스텝", "타임아웃?");
    for (int i = 0; i < n; i++) {
        int final_err_md, steps; bool timed_out;
        run_trial(true, test_errors_deg[i], &final_err_md, &steps, &timed_out);
        printf("%-12.1f %-14.3f %-8d %-8s\n", test_errors_deg[i], final_err_md/1000.0, steps,
               timed_out ? "예(타임아웃)" : "아니오");
    }

    return 0;
}
