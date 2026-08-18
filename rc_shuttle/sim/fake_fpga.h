#ifndef FAKE_FPGA_H
#define FAKE_FPGA_H

#include <pthread.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>

typedef struct {
    int fd;
    double pwm_to_tick_rate;

    pthread_mutex_t lock;
    int left_pwm_signed, right_pwm_signed;
    struct timespec last_m_time;

    long cum_left_tick, cum_right_tick;

    volatile bool running;
    pthread_t thread;

    char last_received_m[64];  /* 디버깅/출력용 */
} FakeFPGA;

static void *fake_fpga__loop(void *arg);

static inline void fake_fpga_init(FakeFPGA *f, int fd, double pwm_to_tick_rate) {
    f->fd = fd;
    f->pwm_to_tick_rate = pwm_to_tick_rate;
    pthread_mutex_init(&f->lock, NULL);
    f->left_pwm_signed = 0;
    f->right_pwm_signed = 0;
    clock_gettime(CLOCK_MONOTONIC, &f->last_m_time);
    f->cum_left_tick = 0;
    f->cum_right_tick = 0;
    f->running = false;
    f->last_received_m[0] = '\0';
}

static inline void fake_fpga_start(FakeFPGA *f) {
    f->running = true;
    pthread_create(&f->thread, NULL, fake_fpga__loop, f);
}

static inline void fake_fpga_stop(FakeFPGA *f) {
    f->running = false;
    pthread_join(f->thread, NULL);
    pthread_mutex_destroy(&f->lock);
}

static inline void fake_fpga_get_last_m(FakeFPGA *f, char *out, size_t out_size) {
    pthread_mutex_lock(&f->lock);
    strncpy(out, f->last_received_m, out_size - 1);
    out[out_size - 1] = '\0';
    pthread_mutex_unlock(&f->lock);
}

/* fd로부터 M 패킷을 읽는 것과, 주기적으로 P 패킷을 쓰는 것을 한 스레드에서 처리.
 * (실제로는 "읽기"와 "물리시뮬레이션+쓰기"를 논블로킹으로 번갈아 함) */
static void *fake_fpga__loop(void *arg) {
    FakeFPGA *f = (FakeFPGA *)arg;
    char line[64];
    int line_len = 0;
    struct timespec last_physics;
    clock_gettime(CLOCK_MONOTONIC, &last_physics);

    while (f->running) {
        /* 1) 논블로킹으로 들어온 바이트 읽어서 M 패킷 파싱 시도 */
        char c;
        ssize_t r = read(f->fd, &c, 1);
        if (r > 0) {
            if (c == '\n') {
                line[line_len] = '\0';
                char l_sign, r_sign;
                int l_pwm, r_pwm;
                /* "M +255 -050" 형태 파싱 */
                if (sscanf(line, "M %c%d %c%d", &l_sign, &l_pwm, &r_sign, &r_pwm) == 4 &&
                    (l_sign == '+' || l_sign == '-') && (r_sign == '+' || r_sign == '-')) {
                    int left = (l_sign == '+') ? l_pwm : -l_pwm;
                    int right = (r_sign == '+') ? r_pwm : -r_pwm;
                    pthread_mutex_lock(&f->lock);
                    f->left_pwm_signed = left;
                    f->right_pwm_signed = right;
                    clock_gettime(CLOCK_MONOTONIC, &f->last_m_time);
                    snprintf(f->last_received_m, sizeof(f->last_received_m), "%s", line);
                    pthread_mutex_unlock(&f->lock);
                }
                line_len = 0;
            } else if (line_len < (int)sizeof(line) - 1) {
                line[line_len++] = c;
            } else {
                line_len = 0;
            }
        }

        /* 2) 50ms마다 물리모델 갱신 + P 패킷 전송 (스펙 문서의 "50ms 주기 자동전송"과 동일) */
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        double elapsed = (now.tv_sec - last_physics.tv_sec) + (now.tv_nsec - last_physics.tv_nsec) / 1e9;
        if (elapsed >= 0.05) {
            pthread_mutex_lock(&f->lock);
            int left_pwm = f->left_pwm_signed, right_pwm = f->right_pwm_signed;

            /* 워치독: 500ms 동안 M이 안 오면 강제 정지 */
            double since_m = (now.tv_sec - f->last_m_time.tv_sec) +
                              (now.tv_nsec - f->last_m_time.tv_nsec) / 1e9;
            if (since_m > 0.5) { left_pwm = 0; right_pwm = 0; }

            /* 실제 FPGA 하드웨어는 16비트 카운터라 자연스럽게 wrap됨(WrappingOdometry가 그걸
             * 전제로 설계돼있음). 이 가짜 FPGA도 긴 시뮬레이션에서 실제와 어긋나지 않도록
             * 똑같이 16비트 범위에서 wrap시킴 (short로 캐스팅하면 자동으로 그렇게 됨) */
            f->cum_left_tick = (short)(f->cum_left_tick + lround(left_pwm * f->pwm_to_tick_rate));
            f->cum_right_tick = (short)(f->cum_right_tick + lround(right_pwm * f->pwm_to_tick_rate));

            char p_line[64];
            int n = snprintf(p_line, sizeof(p_line), "P %+06ld %+06ld\n", f->cum_left_tick, f->cum_right_tick);
            pthread_mutex_unlock(&f->lock);

            if (write(f->fd, p_line, n) < 0) { /* 무시: 다음 주기에 재시도 */ }
            last_physics = now;
        } else {
            struct timespec sleep_ts = {0, 2 * 1000000L}; /* 2ms */
            nanosleep(&sleep_ts, NULL);
        }
    }
    return NULL;
}

#endif /* FAKE_FPGA_H */
