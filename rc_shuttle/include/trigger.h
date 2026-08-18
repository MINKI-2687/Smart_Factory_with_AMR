#ifndef TRIGGER_H
#define TRIGGER_H

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/select.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/gpio.h>
#include <time.h>

/* ============================================================
 * trigger: "출발해도 되는 순간"을 판단하는 부분을 한 곳에 모아둔 추상화.
 *
 * 지금은 감압센서가 아직 안 붙어있어서 키보드(Enter)로 테스트하지만, 나중에
 * 센서가 붙으면 TRIGGER_PRESSURE_SENSOR 케이스 하나만 채워넣으면 됨 - 이 파일
 * 바깥(main.c, robot_runner.h)은 하나도 안 고쳐도 되게 설계함.
 *
 * ------------------------------------------------------------
 * BUGFIX (2026-08-07): "엔터를 안 눌렀는데 출발해 버림"의 원인.
 *
 * 예전 코드는 대기 자체를 이렇게 했음:
 *     while ((c = getchar()) != '\n' && c != EOF) { }   // 주석: "입력 버퍼 비움"
 *
 * 이건 버퍼를 비우는 코드가 아니라 "버퍼에 이미 들어있는 첫 개행까지 읽는" 코드임.
 * 터미널은 프로그램이 주행하느라 바쁜 동안에도 사용자의 키 입력을 계속 쌓아둠.
 * 그래서
 *   (1) 주행 중에 Enter를 한 번이라도 눌러놨으면(초조해서 여러 번 누른 경우 포함)
 *       그 개행이 버퍼에 남아 있다가, 나중에 도착해서 이 함수에 들어오는 순간
 *       첫 getchar()가 즉시 '\n'을 반환 -> 대기 없이 바로 출발.
 *   (2) stdin이 터미널이 아니거나(파이프/리다이렉트/nohup/ssh -T) 이미 EOF면
 *       getchar()가 매번 즉시 EOF를 반환 -> 모든 트리거가 무조건 즉시 통과.
 *       (GUI에서 돌릴 때 특히 위험 - 무한 왕복이 사람 없이 계속 돌아감)
 *
 * 고친 방식:
 *   - 대기 시작 "직전에" 쌓여있던 묵은 입력을 진짜로 버림(tcflush / 논블로킹 배수).
 *     로봇이 멈춘 뒤에 새로 누른 Enter만 유효 - 이게 감압센서의 "엣지 검출"과
 *     같은 의미라 나중에 센서로 바꿔도 동작이 안 바뀜.
 *   - 그 뒤에 "새 개행"이 올 때까지 select()로 진짜 블로킹.
 *   - EOF/오류는 즉시 통과시키지 않고 false를 반환해서 호출부가 셔틀을 멈추게 함.
 *   - Ctrl+C(abort_flag)도 대기 중에 반응.
 * ------------------------------------------------------------
 */

typedef enum {
    TRIGGER_STDIN,              /* 테스트용: 터미널에서 Enter 키 입력 */
    TRIGGER_GPIO,               /* 외부 센서(조도/감압 등)의 1/0 출력을 GPIO로 읽음 */
    TRIGGER_PRESSURE_SENSOR,    /* 하위호환 별칭 - TRIGGER_GPIO와 동일하게 동작 */
} TriggerType;

typedef enum {
    TRIGGER_WAIT_LOAD_PRESENT,  /* A지점용: 짐이 "얹히는" 순간을 기다림 */
    TRIGGER_WAIT_LOAD_ABSENT,   /* B지점용: 짐이 "내려가는" 순간을 기다림 */
} TriggerWaitKind;

typedef struct {
    TriggerType type;

    /* ---- TRIGGER_GPIO 용 ----
     * 조도센서/감압센서처럼 1/0 디지털 출력을 내는 센서를 그대로 받음.
     * gpiochip은 라즈베리파이 4 이하면 보통 "/dev/gpiochip0",
     * 라즈베리파이 5는 "/dev/gpiochip4" (RP1). `gpiodetect`로 확인할 것. */
    const char *gpio_chip;
    /* 조도센서를 여러 개 쓰는 경우를 위해 라인을 배열로 받음.
     * 판정은 OR: 하나라도 '감지됨'이면 박스가 올라온 것으로 봄.
     * 박스가 센서 하나를 살짝 벗어나게 놓여도 나머지가 잡아주므로,
     * 센서 1개짜리보다 미검출(false negative)에 훨씬 강함.
     * 대신 한 센서가 1로 고착되면 '항상 박스 있음'이 되어 B지점에서 영영 출발
     * 못 하므로, 대기 시작할 때 라인별 현재값을 로그로 찍어 진단할 수 있게 함. */
#define TRIGGER_MAX_GPIO_LINES 8
    int gpio_lines[TRIGGER_MAX_GPIO_LINES];
    int gpio_line_count;
    bool gpio_active_low;     /* 센서가 '감지됨'일 때 0을 내면 true */
    bool gpio_pull_up;        /* 내부 풀업 사용(오픈드레인 출력 센서용) */
    double gpio_debounce_sec; /* 이 시간만큼 값이 유지돼야 인정. 기본 0.05 */

    /* GPIO 모드에서도 터미널 Enter를 수동 오버라이드로 함께 받을지.
     * 시연 중 센서가 안 먹을 때 손으로 넘길 수 있어 디버그에 유용함.
     * 기본 false - 켜면 사람이 임의로 출발시킬 수 있으므로 실운전에선 끌 것. */
    bool allow_enter_override;
} TriggerConfig;
void trigger_config_default(TriggerConfig *cfg);

/* ------------------------------------------------------------
 * GPIO 입력 읽기 (libgpiod 없이 커널 chardev ioctl 직접 사용)
 *
 * /sys/class/gpio 방식은 커널 5.11에서 deprecated, 6.x에서 제거됐고 라즈베리파이 5
 * 에서는 아예 안 뜸. 그래서 표준 chardev(v2) ioctl을 씀 - 추가 패키지 설치가
 * 필요없고 Pi 4/5 양쪽에서 그대로 동작함.
 * 커널이 오래돼서 v2가 없으면 v1(GPIOHANDLE)으로 자동 폴백함.
 * ------------------------------------------------------------ */
typedef struct {
    int fd;          /* line 요청으로 받은 fd */
    int count;       /* 요청한 라인 수 */
    bool use_v2;
} TriggerGpio;
bool trigger_gpio_open(TriggerGpio *g, const TriggerConfig *cfg);
void trigger_gpio_close(TriggerGpio *g);
bool trigger_gpio_read_bits(TriggerGpio *g, unsigned long long *bits);
bool trigger_gpio_box_present(TriggerGpio *g, bool *out, unsigned long long *raw);
int trigger__count_on(unsigned long long bits, int n);
bool trigger__present_from_count(int n_on, int baseline_on, bool want_present);
void trigger__drain_stale_input(void);
bool trigger__enter_pressed_now(void);
bool trigger__wait_fresh_newline(volatile sig_atomic_t *abort_flag);
int trigger_poll_now(const TriggerConfig *cfg, TriggerWaitKind wait_kind);
bool trigger_wait(const TriggerConfig *cfg, const char *point_label,
                  TriggerWaitKind wait_kind, volatile sig_atomic_t *abort_flag);

#endif /* TRIGGER_H */
