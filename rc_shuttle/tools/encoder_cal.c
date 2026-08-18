/* encoder_cal.c - 엔코더 "바퀴 1회전당 틱수(ticks_per_rev)" 실측 도구.
 *
 * 왜 필요한가
 * -----------
 * robot_config_defaults()의 ticks_per_rev = 20 은 "바퀴축에 20슬롯 디스크"를 가정한
 * 값인데, 실기 로그를 역산해보면 실제로는 그 60배 이상이었음(감속기 달린 모터축
 * 엔코더로 추정). 이 값이 틀리면 wodom_update()의 meters_per_tick이 통째로 틀려서
 * 오도메트리가 물리적으로 불가능한 값을 내고, 스캔매칭 예측자세가 날아가 자세추정이
 * 발산함(증상: 제자리 진동, GUI에서 맵이 로봇과 같이 회전).
 *
 * 실기 로그(2026-08-08) 역산 예:
 *     보고값 = 스텝당 이동 0.000m, 회전 219.8deg
 *     -> d_right - d_left = 3.836rad * 0.160m = 0.614m, 이동 0이므로 각 바퀴 0.307m
 *     -> 현재 meters_per_tick(10.37mm) 기준 29.6틱/스텝
 *     -> 실제 회전량은 3.5deg(=바퀴 호길이 4.89mm)이므로
 *        실제 meters_per_tick = 4.89 / 29.6 = 0.165mm
 *        실제 ticks_per_rev  = 207.3 / 0.165 = 약 1250
 *
 * 이 프로그램은 모터 명령을 절대 보내지 않음 - P 패킷만 읽어서 누적 틱을 보여줌.
 *
 * 빌드
 * ----
 *   make encoder_cal
 *   (또는) gcc -std=c11 -O2 -I. -o encoder_cal encoder_cal.c -lm
 *
 * 실행
 * ----
 *   ./encoder_cal /dev/ttyUSB1 115200
 *
 * 측정 순서
 * ---------
 *   1) 로봇을 들어올려(또는 뒤집어) 바퀴가 자유롭게 돌 수 있게 둔다.
 *   2) 양 바퀴에 매직으로 기준 표시를 한다.
 *   3) 실행 후 Enter 를 눌러 영점을 잡는다.
 *   4) 양 바퀴를 "같은 방향으로" 정확히 10바퀴 손으로 굴린다.
 *      (전진 방향으로. 표시가 다시 제자리에 오는 것을 10번 센다)
 *   5) 화면의 "/10바퀴 -> L=..., R=..." 값이 곧 ticks_per_rev.
 *   6) 3~5를 두세 번 반복해서 값이 재현되는지 확인한다.
 *
 * 결과 해석
 * ---------
 *   - L 과 R 이 비슷하게 나와야 정상. 크게 다르면 좌우 엔코더 체배/스펙이 다른 것.
 *   - 같은 방향으로 굴렸는데 L 과 R 의 '부호가 반대'로 나오면
 *     --encoder-invert-left 또는 --encoder-invert-right 가 필요함.
 *   - 실측값을 main_shuttle 에 --ticks-per-rev <값> 으로 넣을 것.
 *   - raw 값이 32768(0x8000)을 넘나드는데 delta 가 튀면, FPGA 카운터의 wrap 폭이
 *     fpga_link.h 의 ODOM_WRAP(65536)과 다른 것이므로 그 상수도 맞춰야 함.
 */
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <stdbool.h>

#include "serial_port.h"

/* fpga_link.h 의 parse_p_packet 과 동일한 규칙(부호 + 16진수). 이 파일만 독립적으로
 * 빌드할 수 있게 여기에 복사해 둠 - 진단 도구가 다른 헤더의 상태에 의존하지 않게 함. */
static bool parse_p(const char *line, int *l, int *r) {
    char tag;
    int m = sscanf(line, " %c %x %x", &tag, (unsigned int *)l, (unsigned int *)r);
    return (m == 3 && tag == 'P');
}

static void sleep_ms(long ms) {
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

int main(int argc, char **argv) {
    if (argc != 3) {
        fprintf(stderr, "사용법: %s <fpga장치> <보드레이트>\n", argv[0]);
        fprintf(stderr, "  예: %s /dev/ttyUSB1 115200\n", argv[0]);
        return 1;
    }

    int fd = serial_port_open(argv[1], atoi(argv[2]));
    if (fd < 0) {
        fprintf(stderr, "[encoder_cal] 포트 열기 실패: %s\n", argv[1]);
        return 1;
    }

    /* Enter 로 영점을 잡기 위해 stdin 을 논블로킹으로 */
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    printf("=== encoder_cal ===\n");
    printf("모터 명령은 보내지 않습니다. P 패킷만 읽습니다.\n");
    printf("1) 바퀴가 자유롭게 돌게 두고  2) Enter 로 영점  3) 정확히 10바퀴 굴리기\n");
    printf("Ctrl+C 로 종료\n\n");

    char line[128];
    int len = 0;
    int base_l = 0, base_r = 0;
    bool have_base = false;
    long parsed = 0, failed = 0;

    for (;;) {
        char junk[16];
        if (read(STDIN_FILENO, junk, sizeof(junk)) > 0) {
            have_base = false;
            printf("\n[영점 리셋] 이제 10바퀴 굴리세요.\n");
            fflush(stdout);
        }

        char c;
        ssize_t n = read(fd, &c, 1);
        if (n != 1) { sleep_ms(2); continue; }

        if (c != '\n') {
            if (len < (int)sizeof(line) - 1) line[len++] = c;
            else len = 0;   /* 줄이 비정상적으로 길면 버림 */
            continue;
        }
        line[len] = '\0';
        len = 0;

        int l, r;
        if (!parse_p(line, &l, &r)) {
            failed++;
            if (failed <= 10)
                printf("\n[파싱실패 %ld] \"%s\"\n", failed, line);
            continue;
        }

        if (!have_base) { base_l = l; base_r = r; have_base = true; }
        parsed++;

        if (parsed % 10 == 0) {
            int dl = l - base_l, dr = r - base_r;
            printf("\r누적틱 L=%+8d R=%+8d | raw L=%+8d R=%+8d | /10바퀴 -> L=%8.1f R=%8.1f   ",
                   dl, dr, l, r, dl / 10.0, dr / 10.0);
            fflush(stdout);
        }
    }
}
