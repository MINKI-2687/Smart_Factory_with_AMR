#include "fpga_serial.h"


void fpga_link_init(FPGALink *link, int fd,
                                   int controller_max_speed, int pwm_safety_cap,
                                   double wheel_diameter, double wheel_separation, int ticks_per_rev,
                                   int send_rate_hz) {
    link->fd = fd;
    link->controller_max_speed = controller_max_speed;
    link->pwm_safety_cap = pwm_safety_cap;
    link->send_period_sec = 1.0 / send_rate_hz;

    wodom_init(&link->odometry, wheel_diameter, wheel_separation, ticks_per_rev);
    /* 엔코더 부호 보정 기본값 - 필요하면 fpga_link_set_encoder_invert()로 바꿈 */
    link->encoder_invert_left = false;
    link->encoder_invert_right = false;

    pthread_mutex_init(&link->lock, NULL);
    link->current_left = 0;
    link->current_right = 0;
    link->pose_x = link->pose_y = link->pose_theta = 0.0;
    link->has_last_raw = false;

    link->running = false;
    link->sent_count = link->recv_count = link->parse_fail_count = 0;

    /* 기본값: PWM_DEADZONE이 최대치의 몇 %인지로 환산 */
    link->min_effective_speed =
        (int)lround((double)controller_max_speed * PWM_DEADZONE / pwm_safety_cap);
    if (link->min_effective_speed < 1) link->min_effective_speed = 1;
    if (link->min_effective_speed > controller_max_speed) link->min_effective_speed = controller_max_speed;
    link->kick_speed = controller_max_speed;   /* 정지마찰 돌파는 최대 출력으로 */
    link->burst_cycles = 6;   /* 100Hz 기준 60ms - 실제로 눈에 보이게 움직이는 길이 */
    link->kick_cycles = 2;    /* 그 중 앞 20ms는 최대출력으로 바퀴를 떼어냄 */
    link->duty_acc_left = link->duty_acc_right = 0;
    link->burst_rem_left = link->burst_rem_right = 0;
    link->burst_pos_left = link->burst_pos_right = 0;
}


/* 실측 튜닝용. min_speed는 "굴러가는 걸 유지하는 최소 속도", kick_speed는 "정지에서
 * 떼어내는 속도", burst_cycles는 한 번 켤 때 연속 ON 사이클 수, kick_cycles는 그 중
 * 킥 구간 길이. min_speed<=0으로 주면 펄스구동을 끔(기존 연속 동작). */
void fpga_link_set_pulse_params(FPGALink *link, int min_speed, int kick_speed,
                                               int burst_cycles, int kick_cycles) {
    pthread_mutex_lock(&link->lock);
    link->min_effective_speed = min_speed;
    link->kick_speed = kick_speed;
    link->burst_cycles = (burst_cycles > 0) ? burst_cycles : 1;
    link->kick_cycles = (kick_cycles >= 0) ? kick_cycles : 0;
    link->duty_acc_left = link->duty_acc_right = 0;
    link->burst_rem_left = link->burst_rem_right = 0;
    link->burst_pos_left = link->burst_pos_right = 0;
    pthread_mutex_unlock(&link->lock);
}


/* 킥+유지 버스트 펄스 구동. 반환: 이번 사이클에 실제로 내보낼 속도 명령. */
int fpga_link__pulse_speed(int desired, int min_speed, int kick_speed,
                                          int burst_cycles, int kick_cycles,
                                          int *acc, int *burst_rem, int *burst_pos) {
    if (desired == 0) { *acc = 0; *burst_rem = 0; *burst_pos = 0; return 0; }

    int sign = (desired > 0) ? 1 : -1;
    int a = abs(desired);

    /* min_speed<=0: 듀티 사이클링(끊어서 평균 낮추기)은 끄고, "정지->출발" 순간의
     * 킥만 유지함. Stop-and-go 주행에서 쓰는 모드 - 어차피 이동구간에는 충분히 큰
     * 속도를 연속으로 주고 시간으로 이동량을 조절하므로, 끊을 이유가 없음(끊으면
     * 바퀴만 덜컹거림). 대신 매번 정지 상태에서 출발하므로 킥은 반드시 필요함. */
    if (min_speed <= 0) {
        (*burst_pos)++;
        int level = a;
        if (*burst_pos <= kick_cycles && kick_speed > level) level = kick_speed;
        return sign * level;
    }

    bool on;
    int level;

    if (a >= min_speed) {
        on = true; level = a;         /* 연속 구동 가능 구간 */
    } else if (*burst_rem > 0) {
        on = true; level = min_speed; (*burst_rem)--;   /* 버스트 진행 중 */
    } else {
        *acc += a;
        if (*acc >= min_speed * burst_cycles) {
            *acc -= min_speed * burst_cycles;
            *burst_rem = burst_cycles - 1;
            on = true; level = min_speed;
        } else {
            on = false; level = 0;
        }
    }

    if (!on) { *burst_pos = 0; return 0; }

    /* 정지 상태에서 막 켜진 직후 몇 사이클은 최대출력으로 정지마찰을 돌파 */
    (*burst_pos)++;
    if (*burst_pos <= kick_cycles && kick_speed > level) level = kick_speed;

    return sign * level;
}


void fpga_link_start(FPGALink *link) {
    if (link->running) return;
    link->running = true;
    pthread_create(&link->send_thread, NULL, fpga_link__send_loop, link);
    pthread_create(&link->recv_thread, NULL, fpga_link__recv_loop, link);
}


void fpga_link_stop(FPGALink *link) {
    if (!link->running) return;
    char buf[13];
    int n = format_stop_command(link->controller_max_speed, link->pwm_safety_cap, buf);
    if (write(link->fd, buf, n) < 0) { /* 종료 중이라 무시 */ }
    link->running = false;
    pthread_join(link->send_thread, NULL);
    pthread_join(link->recv_thread, NULL);
}


void fpga_link_close(FPGALink *link) {
    fpga_link_stop(link);
    pthread_mutex_destroy(&link->lock);
}


FpgaCmdTrack *fpga_cmd_track(void) {
    static FpgaCmdTrack t = { 0.0, 0.0, 0.0, 0, 0, 255, 80, false };
    return &t;
}


double fpga_cmd__now(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}


/* 직전 명령이 now 까지 유지됐다고 보고 누적한다. */
void fpga_cmd_track__integrate(FpgaCmdTrack *t, double now) {
    if (!t->armed) return;
    double dt = now - t->last_ts;
    if (dt < 0.0) dt = 0.0;
    if (dt > 3.0) dt = 3.0;          /* 시계 이상/일시정지 방어 */
    if (dt > 0.0) {
        char sg;
        int pwm;
        speed_to_pwm(t->last_l, t->ctrl_max, t->cap, &sg, &pwm);
        t->acc_l += ((t->last_l < 0) ? -pwm : pwm) * dt;
        speed_to_pwm(t->last_r, t->ctrl_max, t->cap, &sg, &pwm);
        t->acc_r += ((t->last_r < 0) ? -pwm : pwm) * dt;
    }
    t->last_ts = now;
}


/* 지금 이 순간까지를 확정하고 누적값을 꺼낸다(1회용 - 꺼내면 0으로 비워진다). */
void fpga_cmd_track_take(double *out_l_pwmsec, double *out_r_pwmsec) {
    FpgaCmdTrack *t = fpga_cmd_track();
    fpga_cmd_track__integrate(t, fpga_cmd__now());
    *out_l_pwmsec = t->acc_l;
    *out_r_pwmsec = t->acc_r;
    t->acc_l = 0.0; t->acc_r = 0.0;
}


/* 누적값을 그냥 버린다(탈출 등에서 별도로 처리할 때). */
void fpga_cmd_track_reset(void) {
    FpgaCmdTrack *t = fpga_cmd_track();
    fpga_cmd_track__integrate(t, fpga_cmd__now());
    t->acc_l = 0.0; t->acc_r = 0.0;
}


void fpga_link_set_speed(FPGALink *link, int left, int right) {
    /* 명령이 바뀌기 직전까지를 먼저 적분한다 */
    {
        FpgaCmdTrack *t = fpga_cmd_track();
        double now = fpga_cmd__now();
        t->cap      = link->pwm_safety_cap;
        t->ctrl_max = link->controller_max_speed;
        fpga_cmd_track__integrate(t, now);
        t->last_l = left; t->last_r = right;
        t->last_ts = now; t->armed = true;
    }
    pthread_mutex_lock(&link->lock);
    link->current_left = left;
    link->current_right = right;
    pthread_mutex_unlock(&link->lock);
}


void fpga_link_get_pose(FPGALink *link, double *x, double *y, double *theta) {
    pthread_mutex_lock(&link->lock);
    *x = link->pose_x; *y = link->pose_y; *theta = link->pose_theta;
    pthread_mutex_unlock(&link->lock);
}


/* 엔코더 틱 부호 보정 설정. 직진(양쪽 PWM +) 시 좌우 틱이 '같은 부호로 증가'해야
 * 정상 - 반대로 나오면 그 쪽을 true로. */
void fpga_link_set_encoder_invert(FPGALink *link, bool inv_left, bool inv_right) {
    pthread_mutex_lock(&link->lock);
    link->encoder_invert_left = inv_left;
    link->encoder_invert_right = inv_right;
    pthread_mutex_unlock(&link->lock);
}


void fpga_link_get_stats(FPGALink *link, long *sent, long *recv, long *fail) {
    pthread_mutex_lock(&link->lock);
    *sent = link->sent_count; *recv = link->recv_count; *fail = link->parse_fail_count;
    pthread_mutex_unlock(&link->lock);
}


void *fpga_link__send_loop(void *arg) {
    FPGALink *link = (FPGALink *)arg;
    while (link->running) {
        int left, right;
        pthread_mutex_lock(&link->lock);
        left  = fpga_link__pulse_speed(link->current_left, link->min_effective_speed,
                                        link->kick_speed, link->burst_cycles, link->kick_cycles,
                                        &link->duty_acc_left, &link->burst_rem_left, &link->burst_pos_left);
        right = fpga_link__pulse_speed(link->current_right, link->min_effective_speed,
                                        link->kick_speed, link->burst_cycles, link->kick_cycles,
                                        &link->duty_acc_right, &link->burst_rem_right, &link->burst_pos_right);
        pthread_mutex_unlock(&link->lock);

        char buf[13];
        int n = format_m_command(left, right, link->controller_max_speed, link->pwm_safety_cap, buf);
        ssize_t w = write(link->fd, buf, n);
        if (w > 0) {
            pthread_mutex_lock(&link->lock);
            link->sent_count++;
            pthread_mutex_unlock(&link->lock);
        }

        struct timespec ts;
        ts.tv_sec = (time_t)link->send_period_sec;
        ts.tv_nsec = (long)((link->send_period_sec - ts.tv_sec) * 1e9);
        nanosleep(&ts, NULL);
    }
    return NULL;
}


void *fpga_link__recv_loop(void *arg) {
    FPGALink *link = (FPGALink *)arg;
    char line[128];
    int line_len = 0;

    while (link->running) {
        char c;
        ssize_t r = read(link->fd, &c, 1);
        if (r <= 0) {
            struct timespec ts = {0, 5 * 1000000L};
            nanosleep(&ts, NULL);
            continue;
        }
        if (c == '\n') {
            line[line_len] = '\0';
            int left_tick, right_tick;
            bool ok = parse_p_packet(line, &left_tick, &right_tick);

            pthread_mutex_lock(&link->lock);
            link->recv_count++;
            if (!ok) {
                link->parse_fail_count++;
                /* DEBUG (2026-08-04): 파싱 실패 원인 진단용 - 실제로 뭐가 들어왔는지
                 * hex+ASCII로 같이 찍음. 원인 찾으면(EMI노이즈 vs 다른프로토콜메시지
                 * 섞임 등) 이 블록은 지워도 됨. */
                fprintf(stderr, "[fpga_serial][PARSE_FAIL] len=%d raw=\"", line_len);
                for (int i = 0; i < line_len; i++) {
                    unsigned char uc = (unsigned char)line[i];
                    if (uc >= 32 && uc < 127) fputc(uc, stderr);
                    else fprintf(stderr, "\\x%02X", uc);
                }
                fprintf(stderr, "\"\n");
            } else {
                link->last_raw_left = left_tick;
                link->last_raw_right = right_tick;
                link->has_last_raw = true;
                double x, y, theta;
                /* PATCH (2026-08-07): 엔코더 부호 보정.
                 * 좌우 모터가 마주보게 달리면 FPGA가 모터축 기준 raw 방향을 그대로
                 * 내보내서, 직진 중인데도 좌우 틱 부호가 반대로 나옴. 그 상태로
                 * wodom_update에 넣으면 d_center≈0(안 움직임) + dtheta 거대(계속 회전)로
                 * 계산돼서 오도메트리가 통째로 무의미해짐. 여기서 한 번에 맞춰줌. */
                if (link->encoder_invert_left)  left_tick  = -left_tick;
                if (link->encoder_invert_right) right_tick = -right_tick;
                wodom_update(&link->odometry, left_tick, right_tick, &x, &y, &theta);
                link->pose_x = x; link->pose_y = y; link->pose_theta = theta;
            }
            pthread_mutex_unlock(&link->lock);

            line_len = 0;
        } else {
            if (line_len < (int)sizeof(line) - 1) {
                line[line_len++] = c;
            } else {
                line_len = 0;
            }
        }
    }
    return NULL;
}
