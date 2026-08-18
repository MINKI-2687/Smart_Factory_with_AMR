#include "trigger.h"


void trigger_config_default(TriggerConfig *cfg) {
    cfg->type = TRIGGER_STDIN;
    cfg->gpio_chip = "/dev/gpiochip0";
    cfg->gpio_line_count = 0;
    for (int i = 0; i < TRIGGER_MAX_GPIO_LINES; i++) cfg->gpio_lines[i] = -1;
    cfg->gpio_active_low = false;
    cfg->gpio_pull_up = false;
    cfg->gpio_debounce_sec = 0.05;
    cfg->allow_enter_override = false;
}


bool trigger_gpio_open(TriggerGpio *g, const TriggerConfig *cfg) {
    g->fd = -1; g->use_v2 = false; g->count = cfg->gpio_line_count;
    if (g->count <= 0 || g->count > TRIGGER_MAX_GPIO_LINES) {
        fprintf(stderr, "[trigger] GPIO 라인이 지정되지 않았습니다 "
                        "(--trigger-gpio-lines 17,27,22,23 형태로 주세요)\n");
        return false;
    }
    int chip = open(cfg->gpio_chip, O_RDONLY | O_CLOEXEC);
    if (chip < 0) {
        fprintf(stderr, "[trigger] GPIO 칩 열기 실패 %s: %s\n", cfg->gpio_chip, strerror(errno));
        fprintf(stderr, "          `gpiodetect`로 칩 이름을 확인하세요 "
                        "(Pi5는 보통 /dev/gpiochip4).\n");
        return false;
    }

#ifdef GPIO_V2_GET_LINE_IOCTL
    struct gpio_v2_line_request req;
    memset(&req, 0, sizeof(req));
    for (int i = 0; i < g->count; i++) req.offsets[i] = (unsigned int)cfg->gpio_lines[i];
    req.num_lines = (unsigned int)g->count;
    req.config.flags = GPIO_V2_LINE_FLAG_INPUT;
    if (cfg->gpio_active_low) req.config.flags |= GPIO_V2_LINE_FLAG_ACTIVE_LOW;
    if (cfg->gpio_pull_up)    req.config.flags |= GPIO_V2_LINE_FLAG_BIAS_PULL_UP;
    snprintf(req.consumer, sizeof(req.consumer), "shuttle-trigger");
    if (ioctl(chip, GPIO_V2_GET_LINE_IOCTL, &req) >= 0) {
        g->fd = req.fd; g->use_v2 = true; close(chip); return true;
    }
#endif

    struct gpiohandle_request h;
    memset(&h, 0, sizeof(h));
    for (int i = 0; i < g->count; i++) h.lineoffsets[i] = (unsigned int)cfg->gpio_lines[i];
    h.lines = (unsigned int)g->count;
    h.flags = GPIOHANDLE_REQUEST_INPUT;
    if (cfg->gpio_active_low) h.flags |= GPIOHANDLE_REQUEST_ACTIVE_LOW;
    snprintf(h.consumer_label, sizeof(h.consumer_label), "shuttle-trigger");
    if (ioctl(chip, GPIO_GET_LINEHANDLE_IOCTL, &h) < 0) {
        fprintf(stderr, "[trigger] GPIO 라인 요청 실패: %s\n", strerror(errno));
        close(chip);
        return false;
    }
    g->fd = h.fd; close(chip);
    return true;
}


void trigger_gpio_close(TriggerGpio *g) {
    if (g->fd >= 0) close(g->fd);
    g->fd = -1;
}


/* 라인별 값을 bits에 담아 읽음(비트 i = gpio_lines[i]). 실패하면 false. */
bool trigger_gpio_read_bits(TriggerGpio *g, unsigned long long *bits) {
#ifdef GPIO_V2_GET_LINE_IOCTL
    if (g->use_v2) {
        struct gpio_v2_line_values v;
        memset(&v, 0, sizeof(v));
        v.mask = (g->count >= 64) ? ~0ULL : ((1ULL << g->count) - 1ULL);
        if (ioctl(g->fd, GPIO_V2_LINE_GET_VALUES_IOCTL, &v) < 0) return false;
        *bits = v.bits;
        return true;
    }
#endif
    struct gpiohandle_data d;
    memset(&d, 0, sizeof(d));
    if (ioctl(g->fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &d) < 0) return false;
    unsigned long long b = 0;
    for (int i = 0; i < g->count; i++) if (d.values[i]) b |= (1ULL << i);
    *bits = b;
    return true;
}


/* OR 판정: 하나라도 1이면 '박스 있음' */
bool trigger_gpio_box_present(TriggerGpio *g, bool *out, unsigned long long *raw) {
    unsigned long long bits = 0;
    if (!trigger_gpio_read_bits(g, &bits)) return false;
    if (raw) *raw = bits;
    *out = (bits != 0ULL);
    return true;
}


/* 켜져 있는 센서 개수 */
int trigger__count_on(unsigned long long bits, int n) {
    int c = 0;
    for (int i = 0; i < n; i++) if ((bits >> i) & 1ULL) c++;
    return c;
}


/* ============================================================
 * 박스 유무 판정 (2026-08-10 개정) *** B지점에서 못 떠나던 원인 ***
 *
 * 사용자 신고: "A지점에서 짐을 올리면 잘 출발하는데, B지점에서 짐을 빼면
 *   출발을 못 한다. 지금은 Enter로 임시 출발시키고 있다."
 *
 * 원인은 판정식이 OR 하나뿐이었던 것이다.
 *     present = (bits != 0)   // 하나라도 1이면 '박스 있음'
 * 이 식은 두 지점에서 성격이 정반대다.
 *   A지점(짐이 올라오길 기다림): 센서 넷 중 하나만 잡아도 출발 -> 미검출에 강함. 좋다.
 *   B지점(짐이 내려가길 기다림): 넷이 '전부' 0이 되어야 출발 -> 하나라도 1로 붙어
 *     있으면 영원히 못 떠난다. 오검출에 극단적으로 약하다.
 * 이 파일 위쪽 TriggerConfig 주석에 이미 이렇게 적혀 있다:
 *     "대신 한 센서가 1로 고착되면 '항상 박스 있음'이 되어 B지점에서 영영 출발 못 하므로"
 * 예견해 놓고 판정식은 안 고친 상태였다. 사용자 증상이 정확히 이것이다.
 * 조도센서 4개면 그림자/형광등/배선 접촉 중 하나만 어긋나도 이 상태가 된다.
 *
 * 고침: B지점은 '전부 0'이 아니라 '도착했을 때 켜져 있던 개수의 절반 이하'로
 * 떨어지면 내려간 것으로 본다(적응형 기준). 도착 시 4개가 켜져 있었으면
 * 2개 이하가 되는 순간 출발하므로, 센서 하나가 1로 고착돼도 정상 출발한다.
 * 반대로 박스가 그대로 얹혀 있으면 개수가 안 줄어드니 오출발도 없다.
 * A지점은 OR 그대로 - 그쪽은 원래 잘 되고 있었고 바꿀 이유가 없다.
 * ============================================================ */
bool trigger__present_from_count(int n_on, int baseline_on, bool want_present) {
    if (want_present) return (n_on >= 1);          /* A지점: OR (기존과 동일) */
    int release_at = baseline_on / 2;              /* B지점: 절반 이하로 떨어지면 '없음' */
    return (n_on > release_at);
}


/* 대기를 시작하기 전에 "이미 눌려 있던" 입력을 전부 버림.
 * 터미널이면 tcflush가 커널 라인버퍼까지 확실히 비워줌. 터미널이 아니면(파이프 등)
 * 논블로킹으로 읽을 수 있는 만큼 읽어서 버림. */
void trigger__drain_stale_input(void) {
    int fd = STDIN_FILENO;
    if (isatty(fd)) {
        tcflush(fd, TCIFLUSH);
        return;
    }
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0) return;
    if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) return;
    char buf[256];
    while (read(fd, buf, sizeof(buf)) > 0) { /* 버림 */ }
    fcntl(fd, F_SETFL, flags);
}


/* 지금 당장 stdin에 개행이 와 있는지 논블로킹으로 확인(GPIO 대기 중 오버라이드용) */
bool trigger__enter_pressed_now(void) {
    int fd = STDIN_FILENO;
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);
    struct timeval tv = {0, 0};
    if (select(fd + 1, &rfds, NULL, NULL, &tv) <= 0) return false;
    char buf[64];
    ssize_t got = read(fd, buf, sizeof(buf));
    if (got <= 0) return false;
    for (ssize_t i = 0; i < got; i++)
        if (buf[i] == '\n' || buf[i] == '\r') return true;
    return false;
}


/* 새 개행이 들어올 때까지 블로킹. 성공 true, EOF/오류/중단 false.
 * abort_flag는 NULL 가능(주면 Ctrl+C 때 빠져나옴). */
bool trigger__wait_fresh_newline(volatile sig_atomic_t *abort_flag) {
    int fd = STDIN_FILENO;
    for (;;) {
        if (abort_flag && *abort_flag) return false;

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        struct timeval tv = {0, 200 * 1000};   /* 0.2초마다 중단신호 확인 */
        int n = select(fd + 1, &rfds, NULL, NULL, &tv);
        if (n < 0) {
            if (errno == EINTR) continue;      /* 시그널로 깬 것 - 재시도 */
            fprintf(stderr, "[trigger] stdin select 오류: %s\n", strerror(errno));
            return false;
        }
        if (n == 0) continue;                  /* 타임아웃 - 계속 대기 */

        char buf[128];
        ssize_t got = read(fd, buf, sizeof(buf));
        if (got == 0) {
            fprintf(stderr,
                "[trigger] 오류: stdin이 EOF입니다(터미널이 아님/입력이 닫힘).\n"
                "          Enter 트리거를 쓸 수 없으므로 즉시 출발하지 않고 중단합니다.\n"
                "          GUI/파이프로 실행 중이라면 감압센서 트리거를 쓰거나,\n"
                "          터미널에서 직접 실행하세요.\n");
            return false;
        }
        if (got < 0) {
            if (errno == EINTR || errno == EAGAIN) continue;
            fprintf(stderr, "[trigger] stdin read 오류: %s\n", strerror(errno));
            return false;
        }
        for (ssize_t i = 0; i < got; i++) {
            if (buf[i] == '\n' || buf[i] == '\r') return true;
        }
        /* 개행이 아직 안 왔으면 계속 기다림 */
    }
}


/* ============================================================
 * '지금 이미 출발조건인가'를 한 번만 읽어본다 (2026-08-10h 신규)
 *
 * 왜 필요한가 (사용자 요청):
 *   "로봇이 A 또는 B 주차슬롯 내부에서 시작한다면, 각 슬롯에서의 트리거를
 *    기다리거나, 만약 트리거가 이미 충족되었다면 바로 후진해서 나가도록 해줘."
 *
 * trigger_wait() 는 '엣지'를 본다. 즉 도착 시점에 이미 원하는 상태여도 그건
 * 출발신호가 아니고, 반대 상태를 한 번 거쳐야 인정한다. 왕복 운전 중에는 그게
 * 맞다(짐을 올렸다/내렸다는 '사건'을 잡아야 하니까). 그런데 프로그램을 막 켠
 * 순간은 사건이 있었는지 알 방법이 없고, 사람은 이미 짐을 올려둔 채 실행한다.
 * 그때 엣지를 요구하면 짐을 한 번 내렸다 올려야만 출발하게 되어 이상하다.
 *
 * 그래서 '시작할 때 이미 슬롯 안'인 경우에만 상태(레벨)로 판정한다.
 *   A지점(짐 얹히기 대기): 센서가 하나라도 켜져 있으면 이미 충족
 *   B지점(짐 내려가기 대기): 센서가 전부 꺼져 있으면 이미 충족
 * (B는 도착 시 기준선이 없으므로 '전부 꺼짐'이라는 엄격한 정의를 쓴다.)
 *
 * 반환: 1 = 이미 충족(바로 출발), 0 = 아직, -1 = 읽을 수 없음(호출부가 대기로)
 * ============================================================ */
int trigger_poll_now(const TriggerConfig *cfg, TriggerWaitKind wait_kind) {
    if (cfg->type != TRIGGER_GPIO && cfg->type != TRIGGER_PRESSURE_SENSOR)
        return -1;   /* Enter 트리거는 '현재 상태'라는 개념이 없다 */

    TriggerGpio g;
    if (!trigger_gpio_open(&g, cfg)) return -1;
    unsigned long long raw = 0;
    bool dummy = false;
    if (!trigger_gpio_box_present(&g, &dummy, &raw)) {
        trigger_gpio_close(&g);
        return -1;
    }
    int n_lines = cfg->gpio_line_count;
    int n_on = trigger__count_on(raw, n_lines);
    trigger_gpio_close(&g);

    bool satisfied = (wait_kind == TRIGGER_WAIT_LOAD_PRESENT) ? (n_on >= 1)
                                                              : (n_on == 0);
    printf("[trigger] 시작 시점 상태 확인: 센서 %d개 중 %d개 켜짐 | ", n_lines, n_on);
    for (int i = 0; i < n_lines; i++)
        printf("line%d=%d ", cfg->gpio_lines[i], (int)((raw >> i) & 1ULL));
    printf("-> %s\n", satisfied ? "출발조건 이미 충족" : "아직 아님(대기)");
    fflush(stdout);
    return satisfied ? 1 : 0;
}


/* 트리거가 발생할 때까지 여기서 블로킹(대기)함. point_label은 로그용("A지점" 등).
 * 반환: true=출발해도 됨, false=트리거를 받을 수 없음/중단됨 -> 호출부는 셔틀을 멈출 것. */
bool trigger_wait(const TriggerConfig *cfg, const char *point_label,
                                TriggerWaitKind wait_kind, volatile sig_atomic_t *abort_flag) {
    const char *action = (wait_kind == TRIGGER_WAIT_LOAD_PRESENT)
                          ? "짐이 얹히기를" : "짐이 내려가기를";

    if (cfg->type == TRIGGER_GPIO || cfg->type == TRIGGER_PRESSURE_SENSOR) {
        TriggerGpio g;
        if (!trigger_gpio_open(&g, cfg)) {
            fprintf(stderr, "[trigger] GPIO를 열 수 없어 출발하지 않습니다.\n");
            return false;
        }

        /* want = "출발시키는 박스 상태"
         *   A지점: TRIGGER_WAIT_LOAD_PRESENT -> 박스가 올라오면(=OR 결과 1) 출발
         *   B지점: TRIGGER_WAIT_LOAD_ABSENT  -> 박스가 내려가면(=OR 결과 0) 출발 */
        bool want = (wait_kind == TRIGGER_WAIT_LOAD_PRESENT);

        if (cfg->allow_enter_override) trigger__drain_stale_input();

        /* 라인별 현재값을 한 번 찍어줌 - 센서 하나가 고착되면 OR 특성상
         * "박스가 영원히 있음"이 되어 B지점에서 안 떠나는데, 이 로그가 있으면
         * 어느 센서가 범인인지 바로 보임. */
        unsigned long long raw = 0;
        bool dummy = false;
        if (!trigger_gpio_box_present(&g, &dummy, &raw)) {
            fprintf(stderr, "[trigger] GPIO 초기 읽기 실패: %s\n", strerror(errno));
            trigger_gpio_close(&g);
            return false;
        }
        int n_lines   = cfg->gpio_line_count;
        int n_on      = trigger__count_on(raw, n_lines);
        /* B지점 기준선: 도착 시점에 켜져 있는 개수 = '박스가 얹혀 있는 상태' */
        int baseline_on = n_on;
        bool present  = trigger__present_from_count(n_on, baseline_on, want);

        printf("[trigger] %s 대기 중: %s 기다립니다.\n", point_label, action);
        printf("[trigger]   센서 %d개 중 %d개 켜짐, 현재 박스 %s | ",
               n_lines, n_on, present ? "있음" : "없음");
        for (int i = 0; i < n_lines; i++)
            printf("line%d=%d ", cfg->gpio_lines[i], (int)((raw >> i) & 1ULL));
        printf("%s\n", cfg->allow_enter_override ? "[Enter로 수동 통과 가능]" : "");
        if (want) {
            printf("[trigger]   판정: 하나라도 켜지면 출발 (OR)\n");
        } else {
            printf("[trigger]   판정: 켜진 개수가 %d개 이하로 떨어지면 출발 "
                   "(도착 시 %d개 기준, 센서 하나가 1로 고착돼도 떠날 수 있게)\n",
                   baseline_on / 2, baseline_on);
            if (baseline_on == 0) {
                printf("[trigger]   *** 경고: 짐이 얹혀 있어야 하는데 켜진 센서가 하나도 "
                       "없습니다. ***\n"
                       "       센서 배선 또는 --trigger-active-low 설정을 확인하세요. "
                       "이 상태로는 '내려놓음'을 감지할 수 없습니다.\n");
            }
        }
        fflush(stdout);

        /* 엣지 검출: 도착 시점에 이미 want 상태여도 그건 출발신호가 아님.
         * 반대 상태를 한 번 본 뒤 전이해야 인정 - stdin 버퍼 비우기와 같은 취지이고,
         * "짐을 실제로 올렸다/내렸다"는 사건을 잡는 것이지 "지금 상태"를 보는 게 아님. */
        bool seen_opposite = (present != want);
        if (!seen_opposite) {
            printf("[trigger]   주의: 도착 시점에 이미 '%s' 상태입니다. "
                   "한 번 %s 뒤 다시 %s 대기합니다.\n",
                   want ? "박스 있음" : "박스 없음",
                   want ? "치웠다가" : "올렸다가",
                   want ? "올려주기를" : "내려주기를");
            fflush(stdout);
        }

        bool stable_val = present;
        double stable_for = 0.0;
        double waited = 0.0, last_report = 0.0;
        const double POLL = 0.02;

        for (;;) {
            if (abort_flag && *abort_flag) {
                trigger_gpio_close(&g);
                printf("[trigger] %s 대기 중단됨 - 출발하지 않습니다.\n", point_label);
                fflush(stdout);
                return false;
            }
            if (cfg->allow_enter_override && trigger__enter_pressed_now()) {
                trigger_gpio_close(&g);
                printf("[trigger] %s Enter 수동 오버라이드 - 출발합니다.\n", point_label);
                fflush(stdout);
                return true;
            }

            unsigned long long rawnow = 0;
            bool vdummy = false;
            if (!trigger_gpio_box_present(&g, &vdummy, &rawnow)) {
                fprintf(stderr, "[trigger] GPIO 읽기 실패: %s\n", strerror(errno));
                trigger_gpio_close(&g);
                return false;
            }
            int on_now = trigger__count_on(rawnow, n_lines);
            bool v = trigger__present_from_count(on_now, baseline_on, want);

            /* 2초마다 라인별 현재값을 찍는다 (2026-08-10).
             * 예전에는 대기 시작할 때 한 번만 찍어서, 막상 안 떠날 때 어느 센서가
             * 붙어 있는지 볼 방법이 없었다. 대기 중에 계속 보여야 진단이 된다. */
            waited += POLL;
            if (waited - last_report >= 2.0) {
                last_report = waited;
                printf("[trigger]   %s 대기 %.0fs: %d개 켜짐(박스 %s) | ",
                       point_label, waited, on_now, v ? "있음" : "없음");
                for (int i = 0; i < n_lines; i++)
                    printf("line%d=%d ", cfg->gpio_lines[i], (int)((rawnow >> i) & 1ULL));
                if (!want && on_now > baseline_on / 2 && waited > 10.0)
                    printf("<- 이 중 안 꺼지는 라인이 고착 의심");
                printf("\n");
                fflush(stdout);
            }

            /* 디바운스: OR 결과가 gpio_debounce_sec 이상 유지돼야 유효한 값으로 인정.
             * 조도센서는 그림자/형광등 깜빡임(100/120Hz)에 잘 튀고, 센서가 4개면
             * 그 확률도 4배가 되므로 여기서 반드시 걸러야 함. */
            if (v == stable_val) {
                stable_for += POLL;
            } else {
                stable_val = v;
                stable_for = 0.0;
            }

            if (stable_for >= cfg->gpio_debounce_sec) {
                if (stable_val != want) seen_opposite = true;
                else if (seen_opposite) break;   /* 반대 -> 원하는 상태로 전이 = 트리거 */
            }

            struct timespec ts = {0, (long)(POLL * 1e9)};
            nanosleep(&ts, NULL);
        }

        trigger_gpio_close(&g);
        printf("[trigger] %s 트리거 발생(박스 %s), 출발합니다.\n",
               point_label, want ? "올라옴" : "내려감");
        fflush(stdout);
        return true;
    }

    /* 핵심: "대기 시작 전에 쌓여있던 입력"을 먼저 버린다. 주행 중에 눌린 Enter는
     * 유효한 출발신호가 아니므로 여기서 폐기되어야 함. */
    trigger__drain_stale_input();

    printf("[trigger] %s 대기 중: %s 기다립니다. (테스트용: 지금부터 Enter 키를 누르세요)\n",
           point_label, action);
    fflush(stdout);

    if (!trigger__wait_fresh_newline(abort_flag)) {
        printf("[trigger] %s 대기 중단됨 - 출발하지 않습니다.\n", point_label);
        fflush(stdout);
        return false;
    }

    printf("[trigger] %s 트리거 발생, 출발합니다.\n", point_label);
    fflush(stdout);
    return true;
}
