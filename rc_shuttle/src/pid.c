#include "pid.h"


void pid_init(PID *p, double kp, double ki, double kd, double integral_limit) {
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->integral_limit = integral_limit;
    p->integral = 0.0;
    p->prev_error = 0.0;
    p->has_prev_error = false;
    p->has_prev_time = false;
}


void pid_reset(PID *p) {
    p->integral = 0.0;
    p->has_prev_error = false;
    p->has_prev_time = false;
}


double pid_get_dt(PID *p) {
    struct timespec now;
    timespec_get(&now, TIME_UTC);  /* C11 표준 함수. POSIX 매크로 불필요 -> include 순서 안 타서 안전함 */
    double dt = 1e-6;
    if (p->has_prev_time) {
        dt = (now.tv_sec - p->prev_time.tv_sec) + (now.tv_nsec - p->prev_time.tv_nsec) / 1e9;
        if (dt < 1e-6) dt = 1e-6;
    }
    p->prev_time = now;
    p->has_prev_time = true;
    return dt;
}


double pid_compute(PID *p, double error) {
    double dt = pid_get_dt(p);

    double p_term = p->kp * error;

    p->integral += error * dt;
    if (p->integral_limit >= 0.0) {
        if (p->integral > p->integral_limit) p->integral = p->integral_limit;
        if (p->integral < -p->integral_limit) p->integral = -p->integral_limit;
    }
    double i_term = p->ki * p->integral;

    double d_term = 0.0;
    if (p->has_prev_error) {
        d_term = p->kd * (error - p->prev_error) / dt;
    }
    p->prev_error = error;
    p->has_prev_error = true;

    return p_term + i_term + d_term;
}
