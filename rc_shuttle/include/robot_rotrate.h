#ifndef ROBOT_ROTRATE_H
#define ROBOT_ROTRATE_H

#include "robot_config.h"

/* ============================================================
 * 회전 각속도 실측 추정기 (2026-08-08d)
 *
 * 위 모델은 "PWM 듀티에 비례해서 바퀴가 돈다"는 1차 근사일 뿐임. 실제로는 부하,
 * 제자리 회전 시의 스크럽 마찰, 배터리 전압, 바닥 재질에 따라 쉽게 ±50% 달라짐.
 * 모델이 틀리면 한 스텝 회전량이 탐색창을 넘어 자세추정이 통째로 무너지므로,
 * 모델을 믿는 대신 "스캔매칭이 실제로 관측한 회전량 / 그 스텝에 준 시간"으로
 * 계속 재추정한다.
 *
 * 포화 처리가 핵심: 관측값은 탐색창(±12도)에서 잘리므로, 관측이 창 가장자리에
 * 붙었다면 "실제로는 더 돌았는데 못 본 것"으로 봐야 한다. 그때는 관측값을 쓰지 않고
 * 추정 각속도를 곧바로 1.5배 올려서 다음 스텝을 짧게 만든다(= 창 안으로 되돌아옴).
 * 반대로 벽에 걸려 거의 안 돈 스텝은 학습에서 제외한다(각속도를 0으로 배우면
 * 스텝이 무한히 길어짐).
 * ============================================================ */
typedef struct {
    double rate;       /* 추정 회전 각속도 (rad/s) */
    double rate_min;   /* 추정치 하한 - 벽에 걸린 스텝을 배워 무한정 길어지는 것 방지 */
    double rate_max;   /* 추정치 상한 - 포화 보정이 발산하는 것 방지 */
    /* 모델 각속도(rad/s). 학습값이 물리적으로 말이 되는지 검산하는 기준 (2026-08-10m).
     * 스윕 학습 쪽에는 이미 'model 의 1.6배까지만 인정' 검산이 있었는데
     * 일반 스텝 학습에는 없어서, 자세추정 점프를 회전으로 배우는 길이 열려 있었다. */
    double rate_model;
    bool   measured;   /* 한 번이라도 실측으로 갱신됐는지 */
    int    n_update;
} RotRateEst;
void rot_rate_init(RotRateEst *e, const RobotConfig *cfg, int speed);
void rot_rate__clip(RotRateEst *e);
double rot_rate_get(const RotRateEst *e);

/* 로봇은 하나뿐이고 회전 속도도 항상 sg_step_speed 이므로, 추정치를 구간마다
 * 새로 배우지 않고 공유한다(navigate_one_leg <-> slot_align 사이에서도 유지). */
extern RotRateEst g_rot_est;
extern bool g_rot_est_ready;
RotRateEst *rot_rate_shared(const RobotConfig *cfg, int speed);
double rot_rate_min_step_deg(const RotRateEst *e, const RobotConfig *cfg);
void rot_rate_update(RotRateEst *e, double observed_rad_signed,
                      double predicted_rad_signed,
                      double commanded_sec, double window_rad,
                      double commanded_dir);
void rot_rate_on_bad_match(RotRateEst *e, double commanded_sec,
                            double window_rad);
void robot_runner__cmd_odom_take(const RobotConfig *cfg,
                                  const RotRateEst *rot_est,
                                  double heading,
                                  double *out_dx, double *out_dy,
                                  double *out_dth);

#endif /* ROBOT_ROTRATE_H */
