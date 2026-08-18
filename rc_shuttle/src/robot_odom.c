#include "robot_odom.h"
#include "robot_runner.h"


/* 오도메트리 변화량(odom_dx/dy/dtheta) 계산을 한 곳에 모음 - has_encoder에 따라
 * fpga_link_get_pose()(엔코더 틱 기반)를 실제로 신뢰할지, 아니면 명시적으로 "이번
 * 스텝도 그대로 제자리"(0,0,0)로 못박고 라이다 스캔매칭에만 위치추정을 맡길지를
 * 이 함수 하나가 결정함. has_encoder=false일 땐 fpga_link_get_pose()를 아예 호출하지
 * 않음 - 엔코더가 없는데도 그 값을 읽어서 우연히 0이길 기대하는 방식은, 배선 노이즈로
 * 유령 틱이 섞이면 조용히 위치추정을 오염시킬 수 있어 위험함(명시적으로 끊는 게 안전). */
void robot_runner__get_odom_delta(FPGALink *fpga, bool has_encoder,
                                                  double *prev_x, double *prev_y, double *prev_theta,
                                                  double *out_dx, double *out_dy, double *out_dtheta) {
    if (!has_encoder) {
        *out_dx = 0.0; *out_dy = 0.0; *out_dtheta = 0.0;
        return;
    }

    /* PATCH (2026-08-08c): 물리적 한계를 '고정 상수'가 아니라 '실제 경과시간'으로 계산.
     *
     * 왜 바꿨나: 예전 한계는 0.149m / 1.86rad(=160도)짜리 고정값이었고, 그 근거는
     * "두 관측 사이 최대 0.9초"라는 가정이었음. 실제 스텝은 0.4~0.5초라 한계가 2배쯤
     * 헐거웠고, 그 틈으로 명백한 쓰레기 값이 통과했음.
     *
     * 실기 로그(2026-08-08): 엔코더가 스텝당 회전 ±219.8deg / 이동 0.000m 를 보고함.
     *   - 219.8deg 짜리는 160deg 한계에 걸려 버려짐(경고 출력)
     *   - 그런데 같은 고장에서 나온 ±117.5deg 짜리는 한계를 '통과'해서 예측자세를
     *     그대로 117도 밀어버림. 스캔매칭 탐색창은 ±12도뿐이라 복구가 원리적으로 불가능.
     *     결과: 자세가 매 스텝 ±117도 왕복 -> 제어기가 명령 방향을 매 스텝 뒤집음
     *     (cmd (-30,30) / (30,-30) 교대) -> 제자리 진동 + GUI에서 맵이 회전.
     *
     * 근본 원인은 ticks_per_rev 설정값이 실제와 다른 것이었음(20으로 잡혀 있었으나
     * 역산하면 1200~1300 수준). 이 필터는 그런 종류의 사고가 '조용히' 자세추정을
     * 오염시키지 못하게 막는 안전망임 - 필터가 근본 해결책은 아니므로,
     * ./encoder_cal 로 실측해서 --ticks-per-rev 로 반드시 맞출 것.
     *
     * 한계식은 "그 시간 동안 바퀴가 최대속도로 돌았어도 이만큼밖에 못 간다"이므로,
     * 물리적으로 가능한 값을 잘못 버릴 일은 없음(여유 1.3배). */
    static struct timespec last_ts;
    static bool has_last_ts = false;
    static int reject_streak = 0;
    struct timespec now_ts;
    clock_gettime(CLOCK_MONOTONIC, &now_ts);
    double dt = 1.0;
    if (has_last_ts)
        dt = (double)(now_ts.tv_sec - last_ts.tv_sec)
           + (double)(now_ts.tv_nsec - last_ts.tv_nsec) * 1e-9;
    last_ts = now_ts;
    has_last_ts = true;
    if (dt < 0.01) dt = 0.01;   /* 시계 이상/첫 호출 방어 */
    if (dt > 2.00) dt = 2.00;

    double raw_x, raw_y, raw_theta;
    fpga_link_get_pose(fpga, &raw_x, &raw_y, &raw_theta);
    double dx = raw_x - *prev_x;
    double dy = raw_y - *prev_y;
    double dth = raw_theta - *prev_theta;
    *prev_x = raw_x; *prev_y = raw_y; *prev_theta = raw_theta;

    const double MAX_WHEEL_MPS = 0.165;   /* 바퀴 최대 선속도 */
    const double MARGIN = 1.3;            /* 측정오차/지터 여유 */
    double wsep = fpga->odometry.wheel_separation;
    if (wsep < 0.01) wsep = 0.16;
    double max_lin = MAX_WHEEL_MPS * dt * MARGIN;
    double max_rot = (2.0 * MAX_WHEEL_MPS / wsep) * dt * MARGIN;

    double lin = hypot(dx, dy);
    if (lin > max_lin || fabs(dth) > max_rot) {
        reject_streak++;
        if (reject_streak <= 3 || reject_streak % 20 == 0) {
            fprintf(stderr,
                "[odom] 버림(%d회 연속): dt=%.2fs 인데 이동 %.3fm / 회전 %+.1fdeg "
                "(물리한계 %.3fm / %.1fdeg)\n",
                reject_streak, dt, lin, dth * 180.0 / ANGLE_PI,
                max_lin, max_rot * 180.0 / ANGLE_PI);
        }
        if (reject_streak == 3) {
            fprintf(stderr,
                "[odom] *** 엔코더 값이 계속 물리한계를 넘습니다. 가장 흔한 원인은\n"
                "       ticks_per_rev 설정값이 실제와 다른 것입니다 (현재 %d).\n"
                "       ./encoder_cal <fpga장치> <보드레이트> 로 실측한 뒤\n"
                "       --ticks-per-rev <실측값> 으로 넣으세요.\n"
                "       좌우 부호가 반대로 나오면 --encoder-invert-left/right 도 확인.\n"
                "       그전까지는 --no-encoder 로 돌리는 것이 안전합니다. ***\n",
                fpga->odometry.ticks_per_rev);
        }
        *out_dx = 0.0; *out_dy = 0.0; *out_dtheta = 0.0;
        return;
    }
    reject_streak = 0;
    *out_dx = dx; *out_dy = dy; *out_dtheta = dth;
}


/* prev_raw_x/y/theta 초기화도 위와 동일한 이유로 has_encoder를 거침 - 엔코더가
 * 없으면 애초에 fpga_link_get_pose()를 호출하지 않고 0으로 시작함. */
void robot_runner__init_prev_pose(FPGALink *fpga, bool has_encoder,
                                                  double *prev_x, double *prev_y, double *prev_theta) {
    if (!has_encoder) {
        *prev_x = 0.0; *prev_y = 0.0; *prev_theta = 0.0;
        return;
    }
    fpga_link_get_pose(fpga, prev_x, prev_y, prev_theta);
}


/* ============================================================
 * 자세 예측 = 엔코더 오도메트리(있으면) 또는 명령 적분(없으면)   2026-08-10
 *
 * 이 프로젝트는 --no-encoder 로 돌기 때문에 get_odom_delta() 가 항상 (0,0,0) 을
 * 돌려줬고, 그래서 스캔매칭의 예측자세가 언제나 "안 움직였다" 였다. 그 결과가
 * 위 CmdOdom 주석에서 분석한 회전 지연 -> 맵 회전이다.
 *
 * slam_localize_step() 을 부르는 곳이 10군데나 되므로 호출부를 하나씩 고치면
 * 반드시 빠뜨리는 곳이 생긴다. 그래서 '예측을 만드는 일'을 이 함수 하나로 모으고
 * 모든 호출부가 이것만 쓰게 한다.
 *
 * heading: 움직이기 직전의 방향각(rad). 스캔매칭 직전이라면 slam->theta 가 그 값이다.
 * ============================================================ */
/* ============================================================
 * 재위치추정 '이동 예산' (2026-08-10e 신규)
 *
 * *** 실기에서 로봇을 슬롯 안으로 순간이동시킨 사고의 방지책 ***
 * 로그:
 *   [main] 대기 구역 진입: 좌우 -4mm, 전후 +14mm      <- 대기점 정상 도착
 *   [nav] 연속 회전: -50deg 를 한 번에 돌립니다        <- 제자리 회전만 함
 *   [slam] 전역 재위치추정 완료: (1.304,0.527,-88deg) -> (1.310,0.180,234deg)
 *          일치도 68% -> 93%, 347mm / 38deg 교정
 * 제자리 회전만 한 로봇이 347mm 를 이동했을 리가 없다. 그런데 예전
 * relocalize_global() 의 채택 조건은 '일치도가 8%p 이상 좋아졌는가' 뿐이었고,
 * 이동량 상한이 아예 없었다. 슬롯 입구에서는 '슬롯 안 깊숙한 자세'도 좌우 벽이
 * 똑같이 보여서 점수가 잘 나오므로, 그 함정에 그대로 빠졌다.
 * 그 뒤의 좌우정렬/진입은 전부 존재하지 않는 자세 위에서 이뤄졌다.
 *
 * 반대로 통로에서는 실제로 490mm 를 이동한 뒤 전역탐색이 그걸 바로잡아야 했다
 * (같은 로그의 62% -> 99% 교정). 그러니 상한을 고정값으로 둘 수는 없다.
 *
 * 그래서 '마지막으로 자세를 확실히 믿었던 순간부터 명령한 이동거리'를 적립해서,
 * 그 값을 재위치추정이 옮겨도 되는 거리의 상한으로 쓴다.
 *   - 통로를 1m 달렸으면 1m 넘게 교정해도 된다.
 *   - 제자리에서 돌기만 했으면 몇 cm 도 못 옮긴다.
 * 명령 적분은 실제보다 최대 2배 작을 수 있으므로(선속도 배율 주석 참고)
 * 예산에 2배 여유와 고정 여유 120mm 를 더해 쓴다.
 * ============================================================ */
double *robot_runner__travel_acc(void) {
    static double d = 0.0;
    return &d;
}


/* 지금 예산으로 허용되는 최대 교정거리(m) */
double robot_runner__reloc_budget(void) {
    return 2.0 * (*robot_runner__travel_acc()) + 0.12;
}


/* 자세를 확실히 믿을 수 있게 됐을 때 예산을 0으로 되돌린다 */
void robot_runner__travel_acc_reset(void) {
    *robot_runner__travel_acc() = 0.0;
}


/* '자세가 틀렸다는 물리적 증거'가 나왔을 때 예산을 강제로 열어준다 (2026-08-10g).
 * 예: 슬롯 깊이까지 밀었는데 안쪽 벽에 안 닿음 = 그 자세는 확정적으로 틀렸다.
 * 이런 경우엔 일치도가 아무리 높아 보여도 예산이 정답을 막으면 안 된다. */
void robot_runner__travel_acc_set(double m) {
    *robot_runner__travel_acc() = m;
}


void robot_runner__predict_delta(FPGALink *fpga, const RobotConfig *cfg,
                                                double heading,
                                                double *prev_x, double *prev_y,
                                                double *prev_theta,
                                                double *out_dx, double *out_dy,
                                                double *out_dtheta) {
    robot_runner__get_odom_delta(fpga, cfg->has_encoder, prev_x, prev_y, prev_theta,
                                  out_dx, out_dy, out_dtheta);
    double cdx = 0.0, cdy = 0.0, cdth = 0.0;
    robot_runner__cmd_odom_take(cfg, rot_rate_shared(cfg, cfg->sg_step_speed),
                                 heading, &cdx, &cdy, &cdth);
    if (cfg->has_encoder) {
        /* ============================================================
         * 엔코더 튐(spike) 방어 (2026-08-10r)  *** 맵이 통째로 틀어진 직접 원인 ***
         *
         * 이 예측값은 단순한 참고치가 아니라 '스캔매칭 탐색의 중심'이다.
         *     slam_localize_step(nav_slam, &scan, odx, ody, odth, ...)
         * 은 예측자세 주변 ±탐색창만 훑으므로, 예측이 창보다 크게 틀리면
         * 정답이 아예 후보에 없다. 그러면 매칭은 '덜 틀린 오답'을 고를 수밖에 없고
         * 그 순간 자세가 통째로 날아간다.
         *
         * 실기 로그(B슬롯 주차 중):
         *     [main] 회전 스텝: 70ms -> 90ms (예상 5.4deg, 각속도 60deg/s-실측)
         *     [odom] 예측 0.002m +59.4deg | 라이다 관측 0.069m +61.7deg
         *     [slam] 도약 제한: 186mm/10.0deg ... 매칭품질 0.51
         * 90ms 동안 60deg/s 면 5.4도인데 엔코더는 +59.4도(=16틱)를 냈다.
         * 물리적으로 685deg/s 라 불가능하다. 그런데 탐색창이 ±25도이므로
         * 검색 범위가 [+34, +84]도가 되어 참값(+5도)이 통째로 빠진다.
         * 이후 자세가 300mm 를 미끄러지며 벽 안쪽으로 들어갔다.
         *
         * 그래서 엔코더 값을 '명령 적분 예측' 과 대조해 물리적으로 말이 되는
         * 범위(2.5배 + 양자화 여유)로 자른다. 엔코더가 멀쩡하면 이 검사는
         * 한 번도 안 걸리고, 튀면 명령 예측 근처로 되돌려 매칭을 지켜준다.
         *
         * ticks_per_rev 20 기준 회전 1틱 = 3.71도라 양자화 여유를 2틱(8도)으로 둔다.
         * ============================================================ */
        const double SPIKE_K        = 2.5;    /* 명령 예측 대비 허용 배수 */
        const double SPIKE_TH_FLOOR = 8.0 * ANGLE_PI / 180.0;  /* 회전 양자화 여유 */
        const double SPIKE_LIN_FLOOR = 0.030;                  /* 이동 양자화 여유 */
        static int spike_n = 0;

        double lim_th = fabs(cdth) * SPIKE_K + SPIKE_TH_FLOOR;
        if (fabs(*out_dtheta) > lim_th) {
            double bad = *out_dtheta;
            *out_dtheta = (bad > 0.0 ? +1.0 : -1.0) * lim_th;
            spike_n++;
            printf("[odom] 엔코더 회전 %+.1fdeg 는 물리적으로 불가능합니다 "
                   "(명령 기준 최대 %.1fdeg) -> %+.1fdeg 로 자릅니다\n",
                   bad * 180.0 / ANGLE_PI, lim_th * 180.0 / ANGLE_PI,
                   *out_dtheta * 180.0 / ANGLE_PI);
            fflush(stdout);
        }
        double od_lin = hypot(*out_dx, *out_dy);
        double lim_lin = hypot(cdx, cdy) * SPIKE_K + SPIKE_LIN_FLOOR;
        if (od_lin > lim_lin && od_lin > 1e-6) {
            double k = lim_lin / od_lin;
            spike_n++;
            printf("[odom] 엔코더 이동 %.0fmm 는 물리적으로 불가능합니다 "
                   "(명령 기준 최대 %.0fmm) -> %.0fmm 로 자릅니다\n",
                   od_lin * 1000.0, lim_lin * 1000.0, lim_lin * 1000.0);
            fflush(stdout);
            *out_dx *= k; *out_dy *= k;
        }
        if (spike_n > 0 && spike_n % 10 == 0) {
            printf("[odom] *** 엔코더 튐이 %d번 걸렸습니다. 배선 노이즈(모터 PWM 선과\n"
                   "       나란히 지나가는지), 풀업/차폐, --ticks-per-rev 값을 확인하세요.\n"
                   "       튐이 잦으면 --no-encoder 로 쓰는 편이 오히려 안전합니다. ***\n",
                   spike_n);
            fflush(stdout);
        }
        /* 재위치추정 '이동 예산'은 엔코더가 있을 때도 쌓아야 한다 (2026-08-10r).
         * 예전에는 엔코더 경로에서 곧바로 return 해서 이 적립이 통째로 빠졌고,
         * 그래서 운동 타당성 게이트가 '한 발도 안 움직였다'는 전제로 동작했다. */
        *robot_runner__travel_acc() += hypot(*out_dx, *out_dy)
                                     + fabs(*out_dtheta) * 0.05;
        return;
    }
    *out_dx += cdx; *out_dy += cdy; *out_dtheta += cdth;
    /* 재위치추정의 '이동 예산'을 여기서 함께 적립한다 (2026-08-10e).
     * 아래 robot_runner__travel_acc() 주석 참고. */
    /* 회전도 예산에 넣는다 (2026-08-10f): 자세기준점이 실제 회전중심에서
     * 40mm 쯤 벗어나 있어서(실측), 제자리 회전만 해도 자세는 그만큼 움직인다.
     * 1rad 당 50mm 로 잡으면 90도 회전에 78mm - 실측 상한과 맞는다. */
    *robot_runner__travel_acc() += hypot(*out_dx, *out_dy)
                                + fabs(*out_dtheta) * 0.05;
}


void slam_win_push_rotation(OccupancyGridSLAM *s, const RobotConfig *cfg,
                                           SlamWinSave *sv) {
    sv->win_m = 0.0; sv->step_m = 0.0; sv->win_th = 0.0; sv->step_th = 0.0;
    sv->saved = false;
    if (s == NULL || cfg->rot_win_deg <= 0.0) return;
    sv->win_m   = s->search_window_m;
    sv->step_m  = s->search_step_m;
    sv->win_th  = s->search_window_theta;
    sv->step_th = s->search_step_theta;
    sv->saved = true;
    s->search_window_m     = cfg->rot_win_m;
    s->search_step_m       = cfg->rot_step_m;
    s->search_window_theta = cfg->rot_win_deg  * ANGLE_PI / 180.0;
    s->search_step_theta   = cfg->rot_step_deg * ANGLE_PI / 180.0;
}


void slam_win_pop(OccupancyGridSLAM *s, SlamWinSave *sv) {
    if (s == NULL || !sv->saved) return;
    s->search_window_m     = sv->win_m;
    s->search_step_m       = sv->step_m;
    s->search_window_theta = sv->win_th;
    s->search_step_theta   = sv->step_th;
    sv->saved = false;
}


/* 회전 스텝 시간을 "한 스텝 회전량 <= 탐색창의 안전계수배"가 되도록 깎음.
 * rate_rad_s 로 각속도를 직접 넘김(실측 추정치를 쓰기 위함). */
double robot_runner__clamp_rotation_sec_rate(const RobotConfig *cfg,
                                                            const OccupancyGridSLAM *nav_slam,
                                                            double rate_rad_s, double sec) {
    if (nav_slam == NULL || nav_slam->search_window_theta <= 0.0) return sec;
    if (rate_rad_s < 1e-3) rate_rad_s = 1e-3;

    /* 안전계수 이력:
     *   1.5 -> 1.0 : 근거 없이 크게 풀었던 값을 test_encoder_ab.c 스윕 결과로 조정
     *   1.0 -> 2.0 : 엔코더 실장 후 sim_encoder_ab.py로 재측정 (엔코더 있을 때)
     *   0.6 -> 0.9 : 엔코더 없을 때, sim_encoder_ab 측정으로 상향
     *   0.9 -> 0.7 (2026-08-08d) : 위 값들은 전부 "각속도 모델이 정확하다"는 전제에서
     *       나온 것인데, 그 모델이 PWM 데드존을 빼먹어 실제의 절반이었음(위 BUGFIX).
     *       모델을 고쳐도 부하/스크럽 편차가 남으므로, 창의 70%로 여유를 둔다.
     *       (12도 창 기준 8.4도/스텝. 예전 실동작 3.5도보다 여전히 2.4배 빠름)
     *
     * 주의: stop-and-go라 스캔은 항상 정지 상태에서 찍힘 - 회전을 빨리 해도
     * 라이다 모션 왜곡이 안 생기는 구조라서 이 여유가 성립함.
     * 연속주행이면 1회전 102ms 동안의 뭉개짐이 별도 상한이 됨. */
    double safe_frac = cfg->rot_clamp_frac > 0.0 ? cfg->rot_clamp_frac
                                                  : (cfg->has_encoder ? 2.0 : 0.7);
    double max_step_rad = nav_slam->search_window_theta * safe_frac;
    double max_sec = max_step_rad / rate_rad_s;
    if (max_sec < 0.02) max_sec = 0.02;          /* 병적으로 짧아지는 것 방지 */
    if (max_sec < cfg->sg_move_sec_min) {
        /* 정지마찰 하한이 탐색창보다 크면 회전속도를 낮추는 수밖에 없음 - 알려줌 */
        static bool warned = false;
        if (!warned) {
            warned = true;
            fprintf(stderr,
                "[nav] 경고: 최소 이동시간(%.2fs)만 줘도 한 스텝에 %.1fdeg를 돌아서\n"
                "      스캔매칭 탐색창(±%.1fdeg)을 넘습니다. --sg-step-speed를 낮추거나\n"
                "      엔코더를 붙이세요.\n",
                cfg->sg_move_sec_min,
                rate_rad_s * cfg->sg_move_sec_min * 180.0 / ANGLE_PI,
                nav_slam->search_window_theta * 180.0 / ANGLE_PI);
        }
        return cfg->sg_move_sec_min;
    }
    return (sec > max_sec) ? max_sec : sec;
}


/* ============================================================
 * 엔코더 검증 리포트 (2026-08-08d)
 *
 * "오도메트리가 예측한 변화량"과 "라이다 스캔매칭이 실제로 관측한 변화량"을 나란히
 * 찍어준다. 이 둘의 비율이 곧 ticks_per_rev 의 배율 오차이므로, 로그만 보고
 * --ticks-per-rev 를 얼마로 고쳐야 하는지 바로 알 수 있다.
 *
 * 읽는 법:
 *   - 회전비가 1.0 근처   -> ticks_per_rev 정상
 *   - 회전비가 예: 2.5    -> ticks_per_rev 를 현재값의 2.5배로
 *   - 부호가 계속 반대     -> --encoder-invert-left / --encoder-invert-right 필요
 *   - 직진 중인데 회전비만 크게 나옴 -> 좌우 중 한쪽 부호 반전 (가장 흔한 배선 실수)
 *
 * 주의: 라이다 관측값은 탐색창(±12도)에서 잘리므로, 한 스텝 회전이 창을 넘는
 * 상황에서는 비율이 과대평가된다. 회전이 창 안에 있는 구간의 값을 볼 것. */
void robot_runner__odom_report(const RobotConfig *cfg, FPGALink *fpga,
                                              double odom_dx, double odom_dy, double odom_dth,
                                              double slam_dx, double slam_dy, double slam_dth) {
    if (!cfg->has_encoder) return;
    static double acc_odom_rot = 0.0, acc_slam_rot = 0.0;
    static double acc_odom_lin = 0.0, acc_slam_lin = 0.0;
    static int n = 0;

    double odom_lin = hypot(odom_dx, odom_dy);
    double slam_lin = hypot(slam_dx, slam_dy);
    n++;
    acc_odom_rot += fabs(odom_dth);  acc_slam_rot += fabs(slam_dth);
    acc_odom_lin += odom_lin;        acc_slam_lin += slam_lin;

    printf("[odom] 예측 %.3fm %+.1fdeg | 라이다 관측 %.3fm %+.1fdeg\n",
           odom_lin, odom_dth * 180.0 / ANGLE_PI,
           slam_lin, slam_dth * 180.0 / ANGLE_PI);

    if (n % 15 == 0) {
        int cur = fpga->odometry.ticks_per_rev;
        printf("[odom] --- 누적 %d스텝: 회전 예측/관측=%s, 이동 예측/관측=%s\n",
               n,
               (acc_slam_rot > 0.30) ? "" : "(관측 회전이 적어 판단보류)",
               (acc_slam_lin > 0.05) ? "" : "(관측 이동이 적어 판단보류)");
        if (acc_slam_rot > 0.30)
            printf("[odom]     회전비 %.2f -> --ticks-per-rev %d 근처 (현재 %d)\n",
                   acc_odom_rot / acc_slam_rot,
                   (int)lround(cur * (acc_odom_rot / acc_slam_rot)), cur);
        if (acc_slam_lin > 0.05)
            printf("[odom]     이동비 %.2f -> --ticks-per-rev %d 근처 (현재 %d)\n",
                   acc_odom_lin / acc_slam_lin,
                   (int)lround(cur * (acc_odom_lin / acc_slam_lin)), cur);
        /* 0.05 -> 0.25m (2026-08-10r). 제자리 회전만 한 구간에서는 '관측 이동'이
         * 전부 스캔매칭 지터와 라이다 오프셋 오차라, 이동비가 무조건 작게 나와
         * '부호 반전' 오진이 뜬다(실기에서 회전비 0.91 / 이동비 0.24 로 오진).
         * 이 진단은 진짜 직진이 충분히 섞였을 때만 의미가 있다. */
        if (acc_slam_lin > 0.25 && acc_slam_rot > 0.30) {
            double rot_ratio = acc_odom_rot / acc_slam_rot;
            double lin_ratio = acc_odom_lin / acc_slam_lin;
            if (rot_ratio > 3.0 * lin_ratio)
                printf("[odom]     *** 회전비가 이동비보다 훨씬 큽니다 = 좌우 틱 부호가\n"
                       "[odom]         반대일 때 나오는 전형적 패턴입니다.\n"
                       "[odom]         --encoder-invert-left 또는 --encoder-invert-right 를\n"
                       "[odom]         켜보세요. ***\n");
        }
        acc_odom_rot = acc_slam_rot = acc_odom_lin = acc_slam_lin = 0.0;
        n = 0;
    }
    fflush(stdout);
}
