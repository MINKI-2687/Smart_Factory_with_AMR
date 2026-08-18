#include "fpga_link.h"


void speed_to_pwm(int speed, int controller_max_speed, int pwm_safety_cap,
                                 char *out_sign, int *out_mag) {
    *out_sign = (speed >= 0) ? '+' : '-';
    int abs_speed = abs(speed);
    if (abs_speed == 0) { *out_mag = 0; return; }

    int mag = PWM_DEADZONE + (int)lround(
        (double)abs_speed / controller_max_speed * (pwm_safety_cap - PWM_DEADZONE));
    if (mag < PWM_DEADZONE) mag = PWM_DEADZONE;
    if (mag > pwm_safety_cap) mag = pwm_safety_cap;
    *out_mag = mag;
}


int format_m_command(int left_speed, int right_speed,
                                    int controller_max_speed, int pwm_safety_cap, char *out) {
    char l_sign, r_sign;
    int l_pwm, r_pwm;
    speed_to_pwm(left_speed, controller_max_speed, pwm_safety_cap, &l_sign, &l_pwm);
    speed_to_pwm(right_speed, controller_max_speed, pwm_safety_cap, &r_sign, &r_pwm);
    int n = snprintf(out, 13, "M %c%03d %c%03d\n", l_sign, l_pwm, r_sign, r_pwm);
    return n;
}


int format_stop_command(int controller_max_speed, int pwm_safety_cap, char *out) {
    return format_m_command(0, 0, controller_max_speed, pwm_safety_cap, out);
}


/* PATCH (2026-08-04): 엔코더 틱 값이 10진수가 아니라 "부호+16진수 5자리" 형식으로
 * 옵니다 (실측 로그로 확인됨: "P -0773B +0791C" - B는 10진수가 아니라 16진수 자릿수).
 * 원래 %d(10진수)로 파싱하고 있어서 16진수 문자(A~F)를 만나는 순간 파싱이 거기서
 * 끊겨 항상 실패했던 것 - 그래서 fpga_fail 비율이 계속 70%대로 나왔던 것으로 확인됨.
 * %x는 C표준상 부호(+/-)도 그대로 처리해줌(strtoul과 동일 규칙) - "-0773B"를
 * 정확히 음수로 파싱함. int*를 unsigned int*로 캐스팅해서 넘기는 이유: %x는
 * 표준상 unsigned int* 를 요구하는데, int와 unsigned int는 같은 크기/표현이라
 * 캐스팅해서 넘기면 안전하게 그대로 int로 재해석해서 쓸 수 있음(2의 보수 표현
 * 그대로 유지됨). */
bool parse_p_packet(const char *line, int *left_tick, int *right_tick) {
    char tag;
    int matched = sscanf(line, " %c %x %x", &tag, (unsigned int *)left_tick, (unsigned int *)right_tick);
    if (matched != 3 || tag != 'P') return false;
    return true;
}


void wodom_init(WrappingOdometry *o, double wheel_diameter,
                               double wheel_separation, int ticks_per_rev) {
    o->wheel_radius = wheel_diameter / 2.0;
    o->wheel_separation = wheel_separation;
    o->ticks_per_rev = ticks_per_rev;
    o->meters_per_tick = (ANGLE_PI * wheel_diameter) / ticks_per_rev;
    o->x = 0.0; o->y = 0.0; o->theta = 0.0;
    o->has_prev = false;
}


int wodom__wrapped_delta(int new_tick, int prev_tick) {
    int raw = new_tick - prev_tick;
    if (raw > ODOM_WRAP_THRESHOLD) raw -= ODOM_WRAP;
    else if (raw < -ODOM_WRAP_THRESHOLD) raw += ODOM_WRAP;
    return raw;
}


void wodom_update(WrappingOdometry *o, int left_tick, int right_tick,
                                 double *out_x, double *out_y, double *out_theta) {
    if (!o->has_prev) {
        o->prev_left_tick = left_tick;
        o->prev_right_tick = right_tick;
        o->has_prev = true;
        *out_x = o->x; *out_y = o->y; *out_theta = o->theta;
        return;
    }

    int dl = wodom__wrapped_delta(left_tick, o->prev_left_tick);
    int dr = wodom__wrapped_delta(right_tick, o->prev_right_tick);
    o->prev_left_tick = left_tick;
    o->prev_right_tick = right_tick;

    double d_left = dl * o->meters_per_tick;
    double d_right = dr * o->meters_per_tick;
    double d_center = (d_left + d_right) / 2.0;
    double dtheta = (d_right - d_left) / o->wheel_separation;

    o->x += d_center * cos(o->theta + dtheta / 2.0);
    o->y += d_center * sin(o->theta + dtheta / 2.0);
    o->theta = normalize_angle(o->theta + dtheta);

    *out_x = o->x; *out_y = o->y; *out_theta = o->theta;
}
