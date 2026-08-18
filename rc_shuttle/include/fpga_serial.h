#ifndef FPGA_SERIAL_H
#define FPGA_SERIAL_H
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif

#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>
#include "fpga_link.h"

typedef struct {
    int fd;

    int controller_max_speed;
    int pwm_safety_cap;
    double send_period_sec;

    WrappingOdometry odometry;
    bool encoder_invert_left, encoder_invert_right;

    pthread_mutex_t lock;
    int current_left, current_right;
    double pose_x, pose_y, pose_theta;
    int last_raw_left, last_raw_right;
    bool has_last_raw;

    /* PATCH (2026-08-05 v2): 실측으로 확인된 하드웨어 사실 - PWM 170은 "이미 돌고 있는
     * 바퀴를 계속 돌리는" 유지값이고, "멈춘 바퀴를 출발시키는" 값이 아님(정지마찰이
     * 운동마찰보다 훨씬 큼 - 손으로 툭 밀어줘야 그제서야 돎). 그래서 단순 듀티
     * 사이클링(170짜리 짧은 펄스)은 정지 상태에서 아무 일도 안 일어남 - 실기에서
     * 도킹 접근단계(min_linear_speed=6)가 전혀 전진하지 못하고 각도보정만 남아
     * 좌우로 까딱거리는 것이 확인됨.
     *
     * 그래서 펄스를 다음 형태로 바꿈:
     *   [킥 kick_cycles사이클: PWM 최대치로 정지마찰 돌파]
     *   [유지 (burst_cycles-kick_cycles)사이클: min_effective_speed로 굴러가게 유지]
     *   [OFF: 나머지 시간]
     * 한 번 켜지면 최소 burst_cycles만큼 연속으로 켜두기 때문에(=단발 10ms 블립이
     * 아니라 수십ms짜리 실제 이동), 바퀴가 확실히 떨어지고 실제로 조금씩 전진함.
     * 누산기 방식이라 OFF 시간 비율로 평균 속도를 맞춤 - 즉 "느리게"가 연속 저속이
     * 아니라 "짧게 확 움직이고 쉬기"의 반복으로 구현됨. 이 하드웨어로 낼 수 있는
     * 저속의 유일한 형태임. */
    int min_effective_speed;   /* 굴러가는 걸 유지할 수 있는 최소 속도(컨트롤러 단위) */
    int kick_speed;            /* 정지마찰 돌파용 속도 (보통 controller_max_speed) */
    int burst_cycles;          /* 한 번 켜지면 최소 이만큼 연속 ON */
    int kick_cycles;           /* 그 중 앞의 이만큼은 kick_speed로 */
    int duty_acc_left, duty_acc_right;
    int burst_rem_left, burst_rem_right;
    int burst_pos_left, burst_pos_right;

    volatile bool running;
    pthread_t send_thread, recv_thread;

    long sent_count, recv_count, parse_fail_count;
} FPGALink;

void *fpga_link__send_loop(void *arg);
void *fpga_link__recv_loop(void *arg);
void fpga_link_init(FPGALink *link, int fd,
                     int controller_max_speed, int pwm_safety_cap,
                     double wheel_diameter, double wheel_separation, int ticks_per_rev,
                     int send_rate_hz);
void fpga_link_set_pulse_params(FPGALink *link, int min_speed, int kick_speed,
                                 int burst_cycles, int kick_cycles);
int fpga_link__pulse_speed(int desired, int min_speed, int kick_speed,
                            int burst_cycles, int kick_cycles,
                            int *acc, int *burst_rem, int *burst_pos);
void fpga_link_start(FPGALink *link);
void fpga_link_stop(FPGALink *link);
void fpga_link_close(FPGALink *link);

/* ============================================================
 * 명령 적분기 (2026-08-10 신규)
 *
 * 엔코더가 없으면 "얼마나 움직였나"를 알 방법이 없다 - 고 생각했지만, 사실
 * '무엇을 얼마나 오래 명령했는지'는 우리가 정확히 안다. 그걸 여기서 적분한다.
 *
 * 왜 하필 여기(set_speed)인가:
 * 모터 명령은 robot_runner.h 의 수십 군데에서 나간다(주행/회전/탈출/슬롯정렬/
 * 깔때기복구...). 각 호출부마다 "방금 이만큼 명령했다"를 따로 기록하게 하면
 * 반드시 빠뜨리는 곳이 생기고, 빠뜨린 곳에서 자세추정이 조용히 망가진다.
 * 모든 명령이 반드시 통과하는 이 함수 하나에서 적분하면 누락이 원리적으로 없다.
 *
 * 단위: '부호 있는 PWM x 초'. 실제 거리 환산은 ROBOT_MAX_WHEEL_MPS 와
 * 속도 자가보정 배율이 있는 robot_runner.h 에서 한다(여기는 하드웨어 계층이라
 * 차체 치수를 모르는 게 맞다).
 * ============================================================ */
typedef struct {
    double acc_l, acc_r;   /* 부호 있는 PWM x 초 (누적) */
    double last_ts;        /* 직전 명령 시각 */
    int    last_l, last_r; /* 직전 명령 값 */
    int    cap;            /* pwm_safety_cap */
    int    ctrl_max;       /* controller_max_speed */
    bool   armed;
} FpgaCmdTrack;
FpgaCmdTrack *fpga_cmd_track(void);
double fpga_cmd__now(void);
void fpga_cmd_track__integrate(FpgaCmdTrack *t, double now);
void fpga_cmd_track_take(double *out_l_pwmsec, double *out_r_pwmsec);
void fpga_cmd_track_reset(void);
void fpga_link_set_speed(FPGALink *link, int left, int right);
void fpga_link_get_pose(FPGALink *link, double *x, double *y, double *theta);
void fpga_link_set_encoder_invert(FPGALink *link, bool inv_left, bool inv_right);
void fpga_link_get_stats(FPGALink *link, long *sent, long *recv, long *fail);
void *fpga_link__send_loop(void *arg);
void *fpga_link__recv_loop(void *arg);

#endif /* FPGA_SERIAL_H */
