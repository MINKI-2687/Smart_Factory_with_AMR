#ifndef FPGA_LINK_H
#define FPGA_LINK_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "angle_utils.h"

/* PATCH (2026-08-03): 실측으로 확인됨 - PWM 100 미만에서는 정지마찰을 못 이겨서
 * 바퀴가 아예 안 돎. 그런데 기존 speed_to_pwm은 0~controller_max_speed를 0~pwm_safety_cap
 * 으로 선형매핑해서, docking.h의 min_angular_speed=4/min_linear_speed=6 같은 "바닥값"이나
 * angle_tolerance 근처의 작은 오차 보정값들이 전부 PWM 10~20대로 나가버림 - 명령은
 * 나가는데 물리적으로는 하나도 안 움직이는 상태였음. 그 결과 도킹 2단계(회전정렬)는
 * heading_error가 약 31.6도보다 작아지는 순간부터 사실상 아무 동작도 못 하고
 * max_rotation_steps 타임아웃까지 그 자리에 멈춰있었을 것으로 추정됨(계산상 확인).
 *
 * 데드존 보정: speed가 0이 아니면(=움직이라는 의도가 있으면) 최소 PWM_DEADZONE은
 * 항상 보장하고, 그 위쪽 구간(PWM_DEADZONE~pwm_safety_cap)에 나머지 speed 범위를
 * 매핑함. PWM_DEADZONE=100은 1차 실측값이라 근사치임 - 로봇을 들어서 바퀴가 자유롭게
 * 돌 수 있는 상태로 60,70,...,110 등 PWM을 직접 순서대로 걸어보고 "이 값부터
 * 확실히 돈다"하는 지점을 찾아서 갱신 권장. */
/* 실측(2026-08-04): PWM 200, 207(최대) 둘 다 확실히 바퀴가 돎을 확인함. 원래 100으로
 * 잡았던 값은 최초 대략적 추정치였고, 그 근처(154 등) 값에서 실제로 안정적으로 도는지는
 * 확인이 덜 된 상태였음 - 안전하게 확실히 검증된 200을 최소보장치로 올림. 이후 더
 * 정밀한 하한(예: 180, 190 등)이 필요하면 그 사이 값들도 순서대로 실측해서 좁혀가면 됨. */
#define PWM_DEADZONE 170
void speed_to_pwm(int speed, int controller_max_speed, int pwm_safety_cap,
                   char *out_sign, int *out_mag);

#define THEORETICAL_MAX_SPEED_MPS ((250.0 / 60.0) * (ANGLE_PI * 0.066))
int format_m_command(int left_speed, int right_speed,
                      int controller_max_speed, int pwm_safety_cap, char *out);
int format_stop_command(int controller_max_speed, int pwm_safety_cap, char *out);
bool parse_p_packet(const char *line, int *left_tick, int *right_tick);

#define ODOM_WRAP 65536
#define ODOM_WRAP_THRESHOLD (ODOM_WRAP / 2)

typedef struct {
    double wheel_radius, wheel_separation, meters_per_tick;
    int ticks_per_rev;
    double x, y, theta;
    int prev_left_tick, prev_right_tick;
    bool has_prev;
} WrappingOdometry;
void wodom_init(WrappingOdometry *o, double wheel_diameter,
                 double wheel_separation, int ticks_per_rev);
int wodom__wrapped_delta(int new_tick, int prev_tick);
void wodom_update(WrappingOdometry *o, int left_tick, int right_tick,
                   double *out_x, double *out_y, double *out_theta);

#endif /* FPGA_LINK_H */
