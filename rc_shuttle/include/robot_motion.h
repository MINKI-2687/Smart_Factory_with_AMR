#ifndef ROBOT_MOTION_H
#define ROBOT_MOTION_H

#include "robot_config.h"

/* ============================================================
 * 주차 슬롯 진입 / 후진
 *
 * A*는 로봇을 원(반경 robot_radius)으로 근사하므로, 17cm 슬롯은 반경 10cm로 팽창하면
 * 통째로 막혀서 경로가 안 나옴(실측 확인). 하지만 로봇이 슬롯 방향으로 정렬만 되어
 * 있으면 실제로는 폭 15.1cm라 여유 ±0.95cm로 들어갈 수 있음 - 원 근사가 지나치게
 * 보수적인 경우임.
 *
 * 그래서 A*는 슬롯 입구 바깥의 '대기점'까지만 담당하고, 거기서부터는 이 함수가
 * 직진/후진만으로 처리함. 슬롯의 V자 입구와 평행 벽이 기계적으로 정렬을 잡아주므로,
 * 소프트웨어는 "대충 가운데로, 대충 똑바로" 정도만 유지하면 됨 - 정밀도는 센서가
 * 아니라 벽이 만들어냄.
 *
 * 진입 방향은 슬롯이 위로 열려 있으므로 -y(아래쪽). heading_rad로 지정.
 * ============================================================ */
/* 슬롯 진입 직전, 제자리에서 각도만 정밀하게 맞춤.
 *
 * PATCH (2026-08-06): 슬롯이 좁아서(폭 17~21cm, 차체 15.1cm) 진입 각도가 조금만
 * 틀어져도 깊이 40cm 들어가는 동안 좌우로 크게 밀림 - 계산상 1도당 7mm. 17cm 슬롯의
 * 허용 좌우오차가 ±9.5mm이므로 진입 각도가 ±1.4도 이내여야 하는데, dynnav의 도킹
 * 허용각(angle_tolerance=15도)으로 출발하면 103mm나 밀려서 반드시 벽에 부딪힘
 * (시뮬레이션에서 차체 충돌판정을 제대로 넣자마자 진입 실패로 드러남).
 *
 * 제자리 회전은 좌우 위치를 건드리지 않으므로 안전하게 각도만 다듬을 수 있음.
 * 좌우 어긋남은 제자리 회전으로 못 고치지만, 그건 V자 입구가 기계적으로 잡아줌. */
/* ============================================================
 * 회전 스텝 시간 제한 (2026-08-07 추가)
 *
 * 증상: 슬롯 정렬을 위해 회전할 때, 90도만 돌면 되는데 반대로 270도를 돌아버림.
 *
 * 원인: 스캔매칭의 각도 탐색창이 ±12도인데(slam_init_from_grid의 인자), 한 스텝의
 * 회전량이 그보다 큼.
 *     회전 각속도 = 2 * speed * MAX_WHEEL_MPS / controller_max_speed / wheel_separation
 *                 = 2 * 34 * 0.165 / 80 / 0.160 = 0.877 rad/s = 약 50 deg/s
 *     sg_move_sec_max = 0.45초  ->  한 스텝에 최대 22.6도
 * 엔코더가 없으면 예측자세 = 직전자세이므로, 실제로 22.6도 돌았는데 탐색창이 ±12도면
 * "정답이 창 밖"이라 매칭은 창 가장자리(12도)를 고를 수밖에 없음. 즉 추정각도가
 * 실제보다 계속 덜 돈 것으로 나옴(스텝당 약 10도씩 뒤처짐).
 * 제어기는 그 뒤처진 추정값을 보고 "아직 멀었다"며 같은 방향으로 계속 회전을 명령하고,
 * 결과적으로 몸통은 목표를 한참 지나쳐서 한 바퀴 가까이 돌게 됨. 추정이 완전히
 * 어긋나면 오차 부호까지 뒤집혀서 "먼 쪽으로 도는" 것처럼 보임.
 * (제어 로직 자체는 정상임 - normalize_angle로 항상 최단방향을 고르고 있음.
 *  고장난 건 '입력'인 각도 추정값이지 '판단'이 아님.)
 *
 * 해결: 한 스텝의 회전량이 탐색창을 넘지 않도록 이동시간에 상한을 둠. 여러 스텝으로
 * 나눠 돌 뿐이라 총 회전시간은 거의 그대로고, 매 스텝 추정이 따라오므로 지나치지 않음.
 * 엔코더가 붙으면 예측자세가 회전량을 이미 반영하므로 탐색창은 '오도메트리 오차'만
 * 덮으면 됨 - 그래서 has_encoder일 때는 상한을 훨씬 느슨하게 잡음.
 * ============================================================ */
#define ROBOT_MAX_WHEEL_MPS 0.165
double robot_runner__rotation_rate(const RobotConfig *cfg, int speed);
double robot_runner__linear_speed(const RobotConfig *cfg, int speed);
double *robot_runner__lin_scale(void);
int *robot_runner__lin_scale_n(void);
void robot_runner__lin_scale_update(double commanded_m, double actual_m);
void robot_runner__lin_scale_observe(double predicted_m, double observed_m);
double robot_runner__move_sec_for(const RobotConfig *cfg, int speed,
                                   double dist_m, double cap_sec);
double robot_runner__move_sec_for_safe(const RobotConfig *cfg, int speed,
                                        double dist_m, double cap_sec);
void robot_runner__emit_state(const char *phase, int step,
                               double x, double y, double theta,
                               const LidarScan *scan);

int robot_runner__escape_speed(const RobotConfig *cfg);

#endif /* ROBOT_MOTION_H */
