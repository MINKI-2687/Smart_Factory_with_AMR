#ifndef PID_H
#define PID_H

#include <time.h>
#include <stdbool.h>

/* ============================================================
 * PID: 오차 하나를 받아 출력을 계산하는 순수 PID 계산기.
 * (Python pid_controller.py의 PID 클래스와 동일 로직)
 *
 * C에는 "클래스"가 없으니, 상태를 담는 struct와
 * 그 struct을 첫 인자(포인터)로 받는 함수들로 구성한다.
 * ============================================================ */
typedef struct {
    double kp, ki, kd;
    double integral_limit;   /* < 0 이면 "제한 없음" */
    double integral;
    double prev_error;
    bool has_prev_error;
    struct timespec prev_time;
    bool has_prev_time;
} PID;
void pid_init(PID *p, double kp, double ki, double kd, double integral_limit);
void pid_reset(PID *p);
double pid_get_dt(PID *p);
double pid_compute(PID *p, double error);

#endif /* PID_H */
