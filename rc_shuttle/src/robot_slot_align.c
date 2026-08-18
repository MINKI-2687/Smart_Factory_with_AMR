#include "robot_slot_align.h"
#include "robot_runner.h"

/* ============================================================
 * slot_align_inner() 분해 (2026-08-11 리팩토링)
 *
 * 예전에는 753줄이 한 함수였다. 로직은 손대지 않고, 스텝을 넘어 사는 값을
 * AlignState 로, 한 번 정하면 안 바뀌는 값을 AlignCtx 로, 이번 스텝 관측을
 * AlignStep 으로 묶은 뒤 단계별 헬퍼로 잘랐다. 각 헬퍼의 주석은 원래 그
 * 자리에 있던 것 그대로다.
 *
 * 제어흐름은 AlignAction 으로 돌려준다. 원본이 루프 안에서 continue / return
 * 을 직접 쓰던 자리가 그대로 대응된다.
 *     ALIGN_GO    : 계속 진행 (원본의 fall-through)
 *     ALIGN_RETRY : 다시 관측부터 (원본의 continue)
 *     ALIGN_DONE  : 정렬 성공 (원본의 return true)
 *     ALIGN_FAIL  : 정렬 실패 (원본의 return false)
 * ============================================================ */

/* ---- 원래 함수 안 지역상수였던 것들 (값은 그대로) ---- */
#define ALIGN_MAX_STEPS        40
/* 정렬 중 탈출은 '접촉만 푸는' 용도 - 준비자세를 잃지 않도록 짧게 제한 (2026-08-09a) */
#define ALIGN_ESCAPE_MAX_M     0.04
/* '돌 자리 확보'는 탈출이 아니라 정상적인 자리잡기다 (2026-08-10m).
 * 예전에는 이것도 align_escape 를 깎아서, 좁은 통로에서 자리를 세 번만
 * 잡아도 정작 진짜로 물렸을 때 쓸 탈출 예산이 0 이 되어 있었다.
 * 실기 로그의 "탈출 시도 1/3, 2/3, 3/3" 이 전부 이렇게 소진된 것이다. */
#define ALIGN_ROOM_TRIES_MAX   8
/* 어떤 탈출에도 리셋되지 않는 전체 진행 감시견 (2026-08-10m) */
#define ALIGN_STAGNANT_MAX     14

/* 한 번 정하면 안 바뀌는 값들 */
typedef struct {
    FPGALink          *fpga;
    LidarThread       *lidar;
    OccupancyGridSLAM *nav_slam;
    const RobotConfig *cfg;
    double             heading_rad;   /* 맞추려는 목표 방향 */
    double             tol_deg;       /* 허용 각도오차 */
} AlignCtx;

/* 스텝 사이에 들고 다니는 상태. 예전에는 전부 지역변수였다. */
typedef struct {
    double prev_rx, prev_ry, prev_rth;   /* 오도메트리 기준 자세 */
    /* 정렬 중 끼임 감지 */
    double last_th;
    int    align_stall, align_escape;
    int    room_tries;                   /* '돌 자리 확보' 횟수 (2026-08-10m) */
    double best_err_seen;                /* 전체 진행 감시견 */
    int    stagnant_steps;
    /* 정지마찰 돌파용 펄스 배수 (2026-08-09k) */
    double kick_gain;
    int    no_motion;
    /* 각속도 실측 (2026-08-08d) */
    RotRateEst *rot_est;
    double rot_prev_theta, rot_prev_sec;
    double rot_prev_dir;                 /* 직전 회전 명령 방향 (+1 반시계 / -1 시계) */
    bool   rot_pending;
    /* 핑퐁 감지 (2026-08-09a) */
    double best_abs_err;
    int    no_improve;
    int    sign_flips;
    double prev_err_deg;
    bool   have_prev_err;
    int    low_q_streak;                 /* 일치도 미달이 몇 스텝 이어졌나 (2026-08-10f) */
    /* 회전중심 진단 (2026-08-08e) */
    double rc_x0, rc_y0, rc_th0;
    bool   rc_have, rc_done;
    /* 연속 스윕 회전 (2026-08-09j) */
    bool   sweep_used, sweep_pending;
    double pending_sweep_dth;
    double rot_sweep_prev_theta, rot_sweep_sec;
} AlignState;

/* 이번 스텝에 관측/계산한 것 */
typedef struct {
    int       step;
    LidarScan scan;
    double    x, y, theta;
    double    odx, ody, odth;
    double    slam_prev_x, slam_prev_y, slam_prev_th;
    double    err, err_deg;              /* 목표까지 남은 각도 */
    bool      rate_learn_ok;             /* 이 스텝의 자세추정을 학습에 써도 되나 */
    double    rate_cmd, sec_cmd;         /* 검사와 명령이 공유하는 회전 속도/시간 */
    double    rot_frac;                  /* 여유상 허용된 회전 비율 */
} AlignStep;

typedef enum {
    ALIGN_GO = 0,
    ALIGN_RETRY,
    ALIGN_DONE,
    ALIGN_FAIL,
} AlignAction;



/* 반환: 실제로 스윕했으면 true. out_pred_dth 에 '스캔매칭에 넘길 예측 회전량'을 담는다. */
bool robot_runner__sweep_rotate(FPGALink *fpga, const RobotConfig *cfg,
                                               const LidarScan *scan, RotRateEst *rot_est,
                                               double err_rad, double *out_pred_dth,
                                               double *out_sec) {
    double err_deg = err_rad * 180.0 / ANGLE_PI;
    if (fabs(err_deg) < SWEEP_MIN_DEG) return false;

    /* 각속도 추정이 아직 안 익었으면 눈을 감지 않는다 (2026-08-10) */
    if (!rot_est->measured || rot_est->n_update < SWEEP_MIN_UPDATES) {
        printf("[nav] 연속 회전 보류: 각속도 실측이 %d회뿐이라(최소 %d회) 눈 감고 돌지 "
               "않습니다 - 끊어서 돕니다\n", rot_est->n_update, SWEEP_MIN_UPDATES);
        fflush(stdout);
        return false;
    }

    double sweep_rad = err_rad * SWEEP_FRACTION;
    /* 한 번에 도는 각도를 제한한다 - 추정이 2배 틀려도 잔차가 탐색창 안에 남게 */
    double cap_rad = SWEEP_MAX_DEG * ANGLE_PI / 180.0;
    if (fabs(sweep_rad) > cap_rad) {
        double capped = (sweep_rad > 0 ? +1.0 : -1.0) * cap_rad;
        printf("[nav] 연속 회전 분할: %+.0fdeg 중 이번엔 %+.0fdeg 만 돌립니다 "
               "(눈 감고 도는 한 번의 상한 %.0fdeg)\n",
               sweep_rad * 180.0 / ANGLE_PI, capped * 180.0 / ANGLE_PI, SWEEP_MAX_DEG);
        fflush(stdout);
        sweep_rad = capped;
    }
    double gap = 0.0, gdir = 0.0, need = 0.0;
    if (!robot_runner__rotation_is_safe(scan, sweep_rad, 0.5 * cfg->body_length,
                                         0.5 * cfg->body_width, SWEEP_MARGIN_M,
                                         &gap, &gdir, &need)) {
        printf("[nav] 연속 회전 보류: %+.0fdeg 를 한 번에 돌기엔 %+.0fdeg 방향 여유가 "
               "%.0fmm 모자랍니다 - 끊어서 돕니다\n",
               sweep_rad * 180.0 / ANGLE_PI, gdir, -gap * 1000.0);
        fflush(stdout);
        return false;
    }

    /* ============================================================
     * '지나치는 것'과 '모자라는 것'의 대가가 다르다 (2026-08-10g)
     *
     * *** 세 번의 실기 로그에서 자세가 무너진 지점이 전부 여기였다 ***
     *   [nav] 연속 회전: -50deg (0.70s, 71deg/s)  -> 실제 114deg
     *   [nav] 연속 회전: +35deg (0.33s, 106deg/s) -> 실제 114deg
     *   [nav] 연속 회전: -35deg (0.73s,  48deg/s) -> 잔차 66deg, 자세 (…,169deg)
     * 각속도 추정이 48~106deg/s 사이를 오가는데 참값은 79~86deg/s 근처다.
     * 즉 추정이 최대 1.8배 어긋난다. 명령 시간을 '추정 각속도'로 나눠 정하면
     * 그만큼 그대로 지나치고, 잔차가 스캔매칭 각도창(±25도) 밖으로 나가면
     * 그 순간 자세를 영영 잃는다(그 뒤 모든 정렬/진입이 허공 위에서 벌어졌다).
     *
     * 그런데 이 둘은 대가가 전혀 다르다.
     *   - 모자라면 : 다음 스텝이 남은 각도를 마저 돈다. 손해는 시간 몇 백 ms.
     *   - 지나치면 : 자세를 잃는다. 회복 불가.
     * 그래서 일부러 '실제가 추정보다 1.8배 빠를 수 있다'고 보고 시간을 나눈다.
     * 최악의 경우에도 명령한 각도를 넘지 않는다.
     *
     * 잔차 검산 (스윕 상한 35도, 탐색창 ±25도):
     *   실제가 추정의 1.8배  -> 실제회전 35도, 예측 19.4도, 잔차 15.6도  < 25 OK
     *   실제가 추정의 0.6배  -> 실제회전 11.7도, 예측 19.4도, 잔차 7.7도 < 25 OK
     * 어느 쪽으로 틀려도 창 안에 남는다.
     * ============================================================ */
    double rate = rot_rate_get(rot_est);
    if (rate < 1e-3) rate = 1e-3;
    const double SWEEP_RATE_GUARD = 1.8;   /* 실제가 추정보다 이만큼 빠를 수 있다고 가정 */
    double sec = fabs(sweep_rad) / (rate * SWEEP_RATE_GUARD);
    if (sec > 2.0) sec = 2.0;             /* 눈 감고 도는 시간의 상한 */
    if (sec < cfg->sg_move_sec_min) sec = cfg->sg_move_sec_min;
    double actual_rad = (sweep_rad > 0 ? +1.0 : -1.0) * rate * sec;

    int w = cfg->sg_step_speed;
    int l = (sweep_rad > 0) ? -w : w;     /* +각도 = 반시계 = 왼바퀴 후진 */
    printf("[nav] 연속 회전: %+.0fdeg 를 한 번에 돌립니다 (%.2fs, 각속도 %.0fdeg/s, "
           "남은 %+.0fdeg 는 멈춘 뒤 마무리)\n",
           actual_rad * 180.0 / ANGLE_PI, sec, rate * 180.0 / ANGLE_PI,
           (err_rad - actual_rad) * 180.0 / ANGLE_PI);
    fflush(stdout);

    fpga_link_set_speed(fpga, l, -l);
    robot_runner__sleep_sec(sec);
    fpga_link_set_speed(fpga, 0, 0);

    if (out_pred_dth) *out_pred_dth = actual_rad;
    if (out_sec) *out_sec = sec;
    return true;
}


/* ============================================================
 * 차선 맞추기 - 통로를 보고 있는 동안 x 를 맞춘다 (2026-08-09i 신규)
 *
 * 사용자 지적: "양쪽 A* 마지막 포인트에서 0도/180도로 열심히 돌았다가 다시 -90도로
 * 돌아오는데, 각도까지 맞춰 도달할 필요 없지 않나? 낭비다."
 *
 * 정확한 지적이고, 원인은 slot_lateral_align() 이었다. 그 함수는 좌우오차를
 *   (1) 차선 쪽을 향해 90도 회전 (2) 오차만큼 전진 (3) 다시 슬롯쪽으로 90도 회전
 * 으로 지운다. 90도 제자리 회전이 두 번이라 약 5.6초가 든다.
 *
 * 그런데 통로를 따라 올 때 로봇은 이미 0도 또는 180도를 보고 있다. 그 방향이
 * 바로 x 축이다. 즉 슬롯 쪽으로 돌기 '전에' 앞뒤로 조금 움직이기만 하면
 * 회전 없이 x 가 맞는다. 회전 두 번이 통째로 사라진다.
 *
 * 이 함수는 그 일만 한다 - 차체가 통로 축을 보고 있을 때(|cos(theta)|>=0.5)만
 * 동작하고, 아니면 그냥 넘어간다(호출부가 기존 경로로 처리).
 * ============================================================ */
bool slot_lane_trim(FPGALink *fpga, LidarThread *lidar,
                                   OccupancyGridSLAM *nav_slam, const RobotConfig *cfg,
                                   double lane_x, double tol_m,
                                   double *prev_rx, double *prev_ry, double *prev_rth) {
    const int MAX_STEPS = 8;
    double x0, y0, th0;
    slam_get_pose(nav_slam, &x0, &y0, &th0);
    /* 0.5 -> 0.30 (2026-08-09m): 차체가 통로 축에서 좀 벗어나 있어도 앞뒤 이동의
     * x 성분이 30% 남아 있으면 그걸로 지우는 편이 낫다. 회전이 0이기 때문이다.
     * (30% 면 오차 20mm 를 지우는 데 67mm 를 가야 하지만, 회전 왕복 110도가
     *  드는 slot_lateral_align 보다 훨씬 싸다) */
    if (fabs(cos(th0)) < 0.30) {
        printf("[slot] 차선 맞추기 건너뜀: 차체가 통로 축에서 너무 벗어나 있어 "
               "(전진방향 x성분 %.0f%%) 앞뒤 이동으로는 x 를 못 지웁니다\n",
               fabs(cos(th0)) * 100.0);
        fflush(stdout);
        return true;
    }
    if (fabs(x0 - lane_x) <= tol_m) return true;

    printf("[slot] 차선 맞추기: 좌우오차 %+.0fmm - 통로를 보고 있는 지금 앞뒤로만 "
           "지웁니다 (회전 없음)\n", (x0 - lane_x) * 1000.0);
    fflush(stdout);

    double last_err = 1e9;
    int stall = 0;
    for (int step = 0; step < MAX_STEPS && !g_stop_requested; step++) {
        fpga_link_set_speed(fpga, 0, 0);
        robot_runner__sleep_sec(cfg->sg_settle_sec);
        LidarScan scan;
        if (!robot_runner__take_scan(lidar, cfg, &scan, 2.0)) continue;
        double x, y, th, odx, ody, odth;
        robot_runner__predict_delta(fpga, cfg, nav_slam->theta, prev_rx, prev_ry, prev_rth,
                                     &odx, &ody, &odth);
        slam_localize_step(nav_slam, &scan, odx, ody, odth, &x, &y, &th);
        robot_runner__emit_state("lane", step, x, y, th, &scan);

        double err = x - lane_x;
        if (fabs(err) <= tol_m) {
            printf("[slot] 차선 맞추기 완료: 오차 %+.0fmm (허용 %.0fmm), %d스텝\n",
                   err * 1000.0, tol_m * 1000.0, step);
            fflush(stdout);
            return true;
        }
        /* 개선이 멈추면 그만 - 남은 오차는 진입 기울임이 흡수한다 */
        if (fabs(err) > fabs(last_err) - 0.003) {
            if (++stall >= 2) {
                printf("[slot] 차선 맞추기 중단: 오차 %+.0fmm 에서 더 안 줄어듭니다 "
                       "(남은 건 기울여 진입하며 흡수)\n", err * 1000.0);
                fflush(stdout);
                return true;
            }
        } else stall = 0;
        last_err = err;

        /* 차체 전진방향의 x 성분 부호로 전진/후진을 고른다 */
        int dir = ((-err) * cos(th) >= 0.0) ? +1 : -1;
        /* 앞뒤로 d 만큼 가면 x 는 d*cos(theta) 만큼 바뀌므로, 필요한 이동거리는
         * |오차| / |cos(theta)| 이다 (2026-08-09m). 예전엔 cos 을 안 나눠서
         * 비스듬할 때 매번 모자라게 가고 스텝만 낭비했다. */
        double cth_abs = fabs(cos(th));
        if (cth_abs < 0.05) cth_abs = 0.05;
        double want = fabs(err) / cth_abs;
        /* 70%만 간다 (2026-08-09m): 이 하드웨어의 최소 이동(정지마찰 하한 0.07초)이
         * 실측 20mm 안팎이라, 필요한 만큼 정확히 명령하면 반드시 조금씩 넘어간다.
         * 넘어가면 다음 스텝이 반대로 가고, 그게 사용자가 본 "전진/후진 반복"이다.
         * 매번 70%씩만 지우면 20 -> 14 -> 10 -> 7mm 로 넘어가지 않고 수렴한다. */
        want *= 0.7;
        if (want > 0.08) want = 0.08;
        double clear = robot_runner__sector_min(&scan, (dir > 0) ? 0.0 : 180.0, 25.0);
        double room = clear - 0.5 * cfg->body_length - 0.02;
        if (room < want) want = room;
        if (want < 0.005) {
            printf("[slot] 차선 맞추기 중단: %s 여유 %.0fmm 라 더 못 갑니다\n",
                   dir > 0 ? "정면" : "후면", clear * 1000.0);
            fflush(stdout);
            return true;
        }
        int trim_speed = (cfg->dock_step_speed > 0) ? cfg->dock_step_speed
                                                    : cfg->slot_step_speed;
        double sec = robot_runner__move_sec_for(cfg, trim_speed, want, 1.0);
        int sp = dir * trim_speed;
        fpga_link_set_speed(fpga, sp, sp);
        robot_runner__sleep_sec(sec);
        fpga_link_set_speed(fpga, 0, 0);
    }
    return true;
}



/* 한 스텝의 관측: 정지 -> 안정화 -> 새 스캔 -> 위치추정 -> 화면 출력. */
static AlignAction align__observe(const AlignCtx *ctx, AlignState *st, AlignStep *sp) {
    FPGALink          *fpga     = ctx->fpga;
    LidarThread       *lidar    = ctx->lidar;
    OccupancyGridSLAM *nav_slam = ctx->nav_slam;
    const RobotConfig *cfg      = ctx->cfg;

    fpga_link_set_speed(fpga, 0, 0);
    robot_runner__sleep_sec(cfg->sg_settle_sec);
    if (g_stop_requested) return ALIGN_FAIL;

    if (!robot_runner__take_scan(lidar, cfg, &sp->scan, 2.0)) return ALIGN_RETRY;

    robot_runner__predict_delta(fpga, cfg, nav_slam->theta, &st->prev_rx, &st->prev_ry, &st->prev_rth,
                                 &sp->odx, &sp->ody, &sp->odth);
    /* 스윕 직후라면 '방금 이만큼 돌았다'를 예측으로 더해준다. 이게 없으면
     * 70도 넘는 변화가 탐색창(±25도) 밖이라 매칭이 통째로 실패한다. */
    if (st->sweep_pending) { sp->odth += st->pending_sweep_dth; st->pending_sweep_dth = 0.0; }
    sp->slam_prev_x = nav_slam->x; sp->slam_prev_y = nav_slam->y;
    sp->slam_prev_th = nav_slam->theta;
    slam_localize_step(nav_slam, &sp->scan, sp->odx, sp->ody, sp->odth, &sp->x, &sp->y, &sp->theta);
    robot_runner__odom_report(cfg, fpga, sp->odx, sp->ody, sp->odth,
                               sp->x - sp->slam_prev_x, sp->y - sp->slam_prev_y,
                               normalize_angle(sp->theta - sp->slam_prev_th));
    /* GUI가 이 구간에서 멈춰 보이던 문제 (2026-08-09a) - emit_state 주석 참고 */
    robot_runner__emit_state("align", sp->step, sp->x, sp->y, sp->theta, &sp->scan);
    return ALIGN_GO;
}


static AlignAction align__quality_gate(const AlignCtx *ctx, AlignState *st, AlignStep *sp) {
    FPGALink          *fpga     = ctx->fpga;
    LidarThread       *lidar    = ctx->lidar;
    OccupancyGridSLAM *nav_slam = ctx->nav_slam;
    const RobotConfig *cfg      = ctx->cfg;
    (void)sp;   /* 시그니처를 다른 단계와 맞추려고 받아만 둔다 */
/* ============================================================
 * 품질 관문 (2026-08-10f) - 사용자 지적 "어긋난 채로 왜 그냥 움직이나"
 *
 * 예전에는 이 루프에 일치도 감시가 한 줄도 없어서, 실기 로그처럼
 *   [slam] 도약 거부: 74mm/4.0deg - 품질 0.56 가 예측자세 0.55 보다 낫지 않아
 *   [slot] 정렬 중: 각도오차 -44.39deg -> cmd=(-34,34)
 * 품질 0.55 로 수십 스텝을 계속 돌았다. 그 자세로 맞춘 각도는 의미가 없다.
 * 연속 3스텝 기준 미달이면 멈추고 자세부터 고친다. 못 고치면 정렬을
 * 실패로 끝내서(호출부가 재접근하도록) 틀린 자세로 슬롯에 들어가지 않게 한다.
 * ============================================================ */
if (cfg->min_match_quality > 0.0) {
    if (nav_slam->last_match_quality < cfg->min_match_quality) st->low_q_streak++;
    else                                                       st->low_q_streak = 0;
    if (st->low_q_streak >= 3) {
        st->low_q_streak = 0;
        printf("[slot] 일치도가 3스텝 연속 %.0f%% 미만입니다 "
               "(마지막 %.0f%%) - 회전을 멈추고 자세부터 고칩니다\n",
               cfg->min_match_quality * 100.0,
               nav_slam->last_match_quality * 100.0);
        fflush(stdout);
        if (!robot_runner__require_good_pose(fpga, lidar, cfg, nav_slam,
                                              "슬롯 정렬 중")) {
            fpga_link_set_speed(fpga, 0, 0);
            return ALIGN_FAIL;
        }
        robot_runner__init_prev_pose(fpga, cfg->has_encoder,
                                      &st->prev_rx, &st->prev_ry, &st->prev_rth);
        st->rc_have = false; st->rc_done = false;
        st->last_th = 1e9;
        st->sweep_used = false;
        return ALIGN_RETRY;
    }
}

/* 스캔매칭이 '큰 교정이 필요하다'고 올린 요청을 슬롯 구간에서도 받는다
 * (2026-08-10d). 여기가 제자리 회전이 가장 많은 구간이라 자세가 가장 잘
 * 무너지는데, 예전에는 이 요청을 받는 곳이 아예 없었다.
 * 전역탐색 대신 넓은 국소탐색을 쓰는 이유는 relocalize_wide 주석 참고
 * (A/B 슬롯이 똑같이 생겨서 전역탐색은 반대쪽으로 튈 수 있다). */
if (nav_slam->relocalize_request > 0) {
    nav_slam->relocalize_request = 0;
    fpga_link_set_speed(fpga, 0, 0);
    printf("[slot] 스캔매칭이 %.0fmm 교정을 요청했습니다 - 회전을 멈추고 "
           "자세를 다시 잡습니다\n", nav_slam->reloc_req_move_m * 1000.0);
    fflush(stdout);
    if (robot_runner__relocalize_wide(lidar, cfg, nav_slam, "슬롯 정렬 중")) {
        robot_runner__init_prev_pose(fpga, cfg->has_encoder,
                                      &st->prev_rx, &st->prev_ry, &st->prev_rth);
        st->rc_have = false; st->rc_done = false;
        st->last_th = 1e9;
        st->sweep_used = false;
        return ALIGN_RETRY;
    }
}
    return ALIGN_GO;
}


/* 직전 회전 결과로 각속도를 배우고, 회전중심 어긋남을 진단한다. */
static void align__learn_rotation(const AlignCtx *ctx, AlignState *st, AlignStep *sp) {
    OccupancyGridSLAM *nav_slam = ctx->nav_slam;
    const RobotConfig *cfg      = ctx->cfg;

/* ============================================================
 * 각속도 학습에 '자세추정을 믿을 수 있을 때만' 쓴다 (2026-08-10c)
 *
 * *** B슬롯 진입 실패의 근본 원인이 여기였다 ***
 * 실기 로그:
 *   [nav] 연속 회전: -41deg 를 한 번에 돌립니다 (0.33s, 각속도 125deg/s)
 *   [slam] 도약 제한: 131mm/4.5deg 중 53%만 반영합니다 (품질 0.63)
 *   [nav] 연속 회전 결과: 110deg 돌았습니다 -> 각속도 실측 331deg/s
 *   [nav] 회전 각속도 실측 추정: 206 deg/s (30회 갱신)
 *   [slot] 정렬 중단(하드웨어 한계): 오차 -6.11deg. 최소 회전 스텝이 16.4deg라
 *   [shuttle] B지점 진입 보류: 각도오차 -6.1deg (한계 6.0)   -> 3번 반복 후 중단
 *
 * 0.33초에 110도면 333deg/s 인데, 이 하드웨어가 speed 34(PWM 206)에서
 * 낼 수 있는 모델값은 95deg/s 다. 3.5배는 물리적으로 불가능하고, 그 순간
 * 스캔매칭 품질이 0.63 이었으며 131mm 도약 제한이 걸려 있었다.
 * 즉 '로봇이 그만큼 돌았다'가 아니라 '자세추정이 그만큼 튀었다'를 배운 것이다.
 *
 * 그 결과 각속도 추정이 상한(236deg/s)까지 올라갔고,
 *     최소 회전 스텝 = 236deg/s x 0.07s = 16.5deg
 * 가 되어 slot_align 이 6도 오차에서 "하드웨어 한계"라며 손을 놨다.
 * 그런데 진입 게이트는 6.0도라, 정렬은 성공을 반환하고 게이트는 거부하는
 * 교착이 생긴다 - 실기에서 3번 재시도가 전부 같은 자리에서 끝난 이유다.
 *
 * navigate_one_leg 쪽에는 이미 품질 가드가 있었는데 여기만 없었다. 맞춘다.
 * ============================================================ */
sp->rate_learn_ok = (nav_slam->bad_match_streak == 0
                      && nav_slam->last_match_quality > 0.60);
if (st->rot_pending) {
    if (sp->rate_learn_ok)
        rot_rate_update(st->rot_est, normalize_angle(sp->theta - st->rot_prev_theta),
                         sp->odth, st->rot_prev_sec, nav_slam->search_window_theta,
                         st->rot_prev_dir);
    else
        /* 매칭이 깨진 스텝은 학습을 못 한다 - 대신 '너무 크게 돌았다'로
         * 보고 각속도를 올려서 다음 스텝을 짧게 만든다 (2026-08-10n) */
        rot_rate_on_bad_match(st->rot_est, st->rot_prev_sec,
                               nav_slam->search_window_theta);
    st->rot_pending = false;
}
if (st->sweep_pending) {
    /* 스윕은 탐색창보다 훨씬 크게 도므로 '포화' 규칙을 쓰면 안 된다.
     * 관측값을 그대로 각속도 실측으로 학습시킨다. */
    double obs = fabs(normalize_angle(sp->theta - st->rot_sweep_prev_theta));
    /* 물리적으로 가능한 상한. 이 차량이 speed 에서 낼 수 있는 모델 각속도의
     * 1.6배까지만 인정한다. 스키드 스티어는 회전할 때 타이어를 옆으로
     * 문질러야 해서, 직진(실측 약 2배)만큼 모델을 초과하지 않는다.
     * 실제로 정렬이 잘 되던 구간의 실효 각속도는 76~95deg/s 였다
     * (오차 7.17도 -> 1.68도를 0.07초 한 스텝에 지웠으므로 약 79deg/s). */
    double model = robot_runner__rotation_rate(cfg, cfg->sg_step_speed);
    double meas  = (st->rot_sweep_sec > 0.05) ? (obs / st->rot_sweep_sec) : 0.0;
    bool plausible = (meas > 0.0 && meas <= 1.6 * model);
    if (st->rot_sweep_sec > 0.05 && obs > 5.0 * ANGLE_PI / 180.0
        && sp->rate_learn_ok && plausible) {
        st->rot_est->rate = st->rot_est->measured ? (0.6 * st->rot_est->rate + 0.4 * meas) : meas;
        rot_rate__clip(st->rot_est);
        st->rot_est->measured = true;
        printf("[nav] 연속 회전 결과: %.0fdeg 돌았습니다 (명령 %.2fs) "
               "-> 각속도 실측 %.0fdeg/s\n",
               obs * 180.0 / ANGLE_PI, st->rot_sweep_sec, meas * 180.0 / ANGLE_PI);
        fflush(stdout);
    } else if (st->rot_sweep_sec > 0.05 && obs > 5.0 * ANGLE_PI / 180.0) {
        printf("[nav] 연속 회전 결과 %.0fdeg(%.0fdeg/s)는 학습하지 않습니다 "
               "- %s (모델 %.0fdeg/s, 매칭품질 %.2f)\n",
               obs * 180.0 / ANGLE_PI, meas * 180.0 / ANGLE_PI,
               !plausible ? "물리적으로 불가능한 값" : "자세추정을 못 믿는 스텝",
               model * 180.0 / ANGLE_PI, nav_slam->last_match_quality);
        fflush(stdout);
    }
    st->sweep_pending = false;
    st->rc_have = false;   /* 스윕은 순수 회전이지만 기준점을 새로 잡는다 */
}

if (!st->rc_have) { st->rc_x0 = sp->x; st->rc_y0 = sp->y; st->rc_th0 = sp->theta; st->rc_have = true; }
else if (!st->rc_done) {
    double dth_s = normalize_angle(sp->theta - st->rc_th0);   /* 부호 포함 */
    double dth = fabs(dth_s);
    if (dth > 45.0 * ANGLE_PI / 180.0) {
        double chord = hypot(sp->x - st->rc_x0, sp->y - st->rc_y0);
        double r = chord / (2.0 * sin(dth / 2.0));
        st->rc_done = true;
        /* ============================================================
         * 어느 쪽으로 얼마나 고쳐야 하는지를 '부호까지' 계산한다 (2026-08-10d)
         *
         * 예전에는 크기 r 만 알아서 "0.011 또는 0.109 로 바꿔서 줄어드는 쪽을
         * 고르세요"라고 사용자에게 이지선다를 떠넘겼다. 그런데 부호는 계산으로
         * 구할 수 있다.
         *
         * 자세기준점 P 에서 실제 회전중심 C 가 차체 앞쪽으로 c 만큼 있다고 하자.
         * 차체가 C 를 중심으로 dth 만큼 돌면
         *     P' - C = R(dth) (P - C),   P - C = (-c, 0)  [차체좌표]
         * 이므로 P 의 이동량을 '처음 차체좌표'로 보면
         *     dx_body = -c (cos dth - 1),   dy_body = -c sin dth
         * 즉  c = -dy_body / sin(dth)  로 부호까지 바로 나온다.
         *
         * 그리고 apply_lidar_frame_fix() 가 적용한 값 d(--lidar-offset)와
         * 실제 라이다~회전중심 거리 D 사이에는 c = d - D 가 성립하므로
         *     권장값 D = d - c
         * 하나로 확정된다. 더 이상 두 값을 번갈아 넣어볼 필요가 없다.
         * ============================================================ */
        double ddx = sp->x - st->rc_x0, ddy = sp->y - st->rc_y0;
        double dy_body = -ddx * sin(st->rc_th0) + ddy * cos(st->rc_th0);
        double sdth = sin(dth_s);
        double c_off = (fabs(sdth) > 0.2) ? (-dy_body / sdth) : 0.0;
        double recommend = cfg->lidar_offset_forward_m - c_off;
        if (r > 0.015) {
            printf("[slot] 회전중심 진단: 제자리 회전 %+.0fdeg 동안 자세가 %.0fmm "
                   "이동했습니다.\n"
                   "       -> 자세기준점이 실제 회전중심에서 %.0fmm 벗어나 있습니다 "
                   "(차체 %s쪽으로 %.0fmm).\n"
                   "       현재 --lidar-offset %.3f 입니다. "
                   "-> 권장값 %.3f (부호까지 계산한 값이라 하나뿐입니다)\n"
                   "       이 값이 크면 회전할 때마다 자세추정이 최대 %.0fmm 씩 "
                   "출렁여서 스캔매칭이 무너집니다.\n",
                   dth_s * 180.0 / ANGLE_PI, chord * 1000.0, r * 1000.0,
                   (c_off >= 0.0) ? "앞" : "뒤", fabs(c_off) * 1000.0,
                   cfg->lidar_offset_forward_m, recommend,
                   2.0 * fabs(c_off) * 1000.0);
            fflush(stdout);
        }
    }
}
}


/* 남은 각도를 재고, 진행이 멎었으면 포기하고, 큰 각도는 한 번에 쭉 돌린다. */
static AlignAction align__watchdog_and_sweep(const AlignCtx *ctx, AlignState *st,
                                             AlignStep *sp) {
    FPGALink          *fpga     = ctx->fpga;
    const RobotConfig *cfg      = ctx->cfg;
    const double heading_rad = ctx->heading_rad;
    const double tol_deg     = ctx->tol_deg;

sp->err = normalize_angle(sp->theta - heading_rad);
sp->err_deg = sp->err * 180.0 / ANGLE_PI;

/* ============================================================
 * 전체 진행 감시견 (2026-08-10m)
 *
 * 아래에 있는 no_improve/align_escape 카운터들은 탈출을 할 때마다 전부
 * 리셋된다. 그래서 실기 로그처럼 '조금 돌았다 되돌아오기'를 반복하면
 * 어느 카운터도 끝까지 차지 않아서, MAX_STEPS(40) + 탈출 3회를 전부
 * 태우고서야 실패로 끝났다. 한 번의 정렬에 1분 가까이 쓰고, 그 사이
 * 로봇은 계속 벽을 밀고 있었다.
 *
 * 이 감시견은 어떤 탈출에도 리셋되지 않는다. '이 자리에서는 안 되는
 * 것'을 빨리 인정하고 실패로 돌려주면, 호출부(run_shuttle)가 통로로
 * 빼내서 다시 접근한다 - 그게 훨씬 빠르고 안전하다. */
if (fabs(sp->err_deg) > 2.0 * tol_deg) {
    if (fabs(sp->err_deg) < st->best_err_seen - 2.0) {
        st->best_err_seen = fabs(sp->err_deg);
        st->stagnant_steps = 0;
    } else if (++st->stagnant_steps >= ALIGN_STAGNANT_MAX) {
        printf("[slot] *** 정렬 포기: %d스텝 동안 오차가 %.1fdeg 아래로 "
               "내려가지 않았습니다.\n"
               "       이 자리에서는 더 돌 수 없습니다 - 통로로 빼내서 "
               "다시 접근하는 편이 빠릅니다. ***\n",
               ALIGN_STAGNANT_MAX, st->best_err_seen);
        fflush(stdout);
        fpga_link_set_speed(fpga, 0, 0);
        return ALIGN_FAIL;
    }
}

/* PATCH (2026-08-09j): 큰 각도는 끊지 말고 한 번에 쭉 돌린다.
 * 스윕 중에는 스캔을 안 쓰고, 끝난 뒤 '방금 이만큼 돌았다'를 오도메트리
 * 예측으로 넘겨서 매칭이 따라오게 한다 (robot_runner__sweep_rotate 주석 참고). */
if (!st->sweep_used && fabs(sp->err_deg) >= SWEEP_MIN_DEG) {
    double pred = 0.0, ssec = 0.0;
    if (robot_runner__sweep_rotate(fpga, cfg, &sp->scan, st->rot_est, -sp->err, &pred, &ssec)) {
        st->sweep_used = true;
        st->pending_sweep_dth = pred;
        st->rot_sweep_prev_theta = sp->theta;
        st->rot_sweep_sec = ssec;
        st->rot_pending = false;    /* 아래 일반 회전 학습과 겹치지 않게 */
        st->sweep_pending = true;
        robot_runner__sleep_sec(cfg->sg_settle_sec);
        return ALIGN_RETRY;
    }
    st->sweep_used = true;          /* 한 번 보류됐으면 다시 시도하지 않음 */
}
    return ALIGN_GO;
}


static AlignAction align__handle_stall(const AlignCtx *ctx, AlignState *st, AlignStep *sp) {
    FPGALink          *fpga     = ctx->fpga;
    LidarThread       *lidar    = ctx->lidar;
    OccupancyGridSLAM *nav_slam = ctx->nav_slam;
    const RobotConfig *cfg      = ctx->cfg;

/* ============================================================
 * 명령은 나가는데 실제로 안 도는 경우 (2026-08-09k)
 *
 * 실기 로그 (시작 직후, 통로 한복판 pose=(0.265,0.635,261deg)):
 *     [slot] 정렬 중: 각도오차 -9.00deg -> cmd=(-34,34)
 *     [slot] 정렬 중: 각도오차 -8.50deg -> cmd=(-34,34)
 *     [slot] 정렬 중: 각도오차 -8.50deg -> cmd=(-34,34)   ... 영원히 -8.50
 *     [slot] 정렬 중 끼임 - 탈출 시도 1/3
 *     [slot] *** 정렬 실패 ***  -> 재시도 -> 또 같은 자리 -> 무한 반복
 *
 * 그 자리는 위아래 벽까지 185mm/210mm 라 9도 회전은 여유가 넘친다.
 * 즉 '물린' 게 아니라 '안 돈' 것이다. 원인은 펄스가 너무 짧아서다:
 *     sec = |오차| / 각속도 = 8.5deg / 90deg/s = 0.094초
 * 정지 상태에서 0.09초 펄스로는 정지마찰과 모터 기동지연을 못 넘긴다.
 * 특히 시동 직후 첫 움직임이라 더 그렇다(로그상 로봇이 한 번도 안 움직인 상태).
 *
 * 오차가 작을수록 펄스가 짧아지고, 짧을수록 더 안 돌아서, 오차가 영원히
 * 안 줄어드는 자기강화 함정이다. 탈출(escape)도 결국 모터를 짧게 미는
 * 동작이라 똑같이 안 먹혔고, 그래서 3번을 다 쓰고 실패했다.
 *
 * 해결: 안 돌면 다음 펄스를 1.6배씩 늘린다(상한 sg_move_sec_max).
 * 그래도 안 되면 속도까지 올린다. 움직이기 시작하면 배수를 1로 되돌린다.
 * ============================================================ */
if (st->last_th < 1e8 && fabs(normalize_angle(sp->theta - st->last_th)) < 0.005) {  /* 0.3도 미만 */
    st->no_motion++;
    if (st->kick_gain < 4.0) st->kick_gain *= 1.6;
} else if (fabs(normalize_angle(sp->theta - st->last_th)) > 0.017) {
    st->no_motion = 0;
    st->kick_gain = 1.0;
}

/* PATCH (2026-08-08): 대기점에서 정렬하려고 제자리 회전하는데 차체 모서리가
 * 통로 벽에 걸리면 계속 헛돌기만 함. 속도를 올려도 더 세게 밀 뿐이므로,
 * 짧게 물러났다가 다시 돌게 함. */
if (st->last_th < 1e8 && fabs(normalize_angle(sp->theta - st->last_th)) < 0.017) {  /* 1도 미만 */
    st->align_stall++;
    if (st->align_stall >= 4 && st->align_escape < 3) {
        st->align_escape++;
        printf("[slot] 정렬 중 끼임 - 탈출 시도 %d/3 (이동 상한 %.0fmm)\n",
               st->align_escape, ALIGN_ESCAPE_MAX_M * 1000.0);
        fflush(stdout);
        /* BUGFIX (2026-08-09i): 회전중심 진단 기준점을 여기서 리셋한다.
         * 진단은 '순수 제자리 회전 동안 자세가 얼마나 옮겨갔나'로 회전중심
         * 오차를 역산하는데, 탈출(특히 게걸음)은 설계상 차체를 옆으로
         * 옮기므로 그 이동량이 통째로 오차로 잡혔다.
         * 실기 로그에서 같은 로봇이 21mm(=오차 25mm)였다가 탈출을 한 번
         * 끼우자 89mm(=오차 106mm)로 뛴 것이 이 때문이다. 그대로 두면
         * --lidar-offset 을 엉뚱한 값으로 맞추게 된다. */
        st->rc_have = false;
        st->rc_done = false;
        /* ============================================================
         * 두 번째 끼임부터는 '다른 동작'을 한다 (2026-08-10m)
         *
         * escape_to_open() 의 성공 기준은 '사방 최소거리 >= robot_radius+10mm'
         * 다. --robot-radius 0.14 면 150mm 인데, 이 값은 반대각선(143mm)보다
         * 크므로 '제자리 회전이 기하학적으로 가능한 상태'를 뜻한다.
         * 그래서 실기 로그처럼
         *     [slot] 정렬 중 끼임 - 탈출 시도 1/3
         *     [slot] 정렬 중: 각도오차 +119.44deg -> cmd=(34,-34)   (곧바로 재개)
         * 즉 '이미 조건 충족'이라며 한 발짝도 안 움직이고 true 를 돌려주고,
         * 호출부는 방금 안 통한 제자리 회전을 똑같이 다시 시도했다.
         * 기하학적으로 돌 수 있는데 안 도는 상태에서 필요한 건 같은 명령의
         * 반복이 아니라 '다른 기동'(크게 돌기 / 호로 빼면서 꺾기)이다.
         * 첫 번째만 직선 탈출을 써 보고, 그 다음부터는 바로 corner_escape 로 간다.
         * ============================================================ */
        bool straight_ok = false;
        if (st->align_escape <= 1) {
            /* PATCH (2026-08-09a): 여기서 12cm 를 밀어버리면 슬롯 앞 준비자세가
             * 통째로 날아감. 접촉만 푸는 정도(4cm)로 제한. */
            straight_ok = robot_runner__escape_to_open(fpga, lidar, cfg,
                                                        ALIGN_ESCAPE_MAX_M, 0.0,
                                                        cfg->allow_crab);
        } else {
            printf("[slot] 직선 탈출은 이미 한 번 해봤고 효과가 없었습니다 "
                   "- 이번엔 크게 돌아서 빠져나오겠습니다\n");
            fflush(stdout);
        }
        if (!straight_ok) {
            /* PATCH (2026-08-10b): 직선으로 안 풀리면 구석에 물린 것이다.
             * 구석 반대쪽으로 크게 돌아서 빠져나온다(사용자 요구사항).
             * 크게 돈 뒤에는 자세추정이 스캔매칭 각도창(±25도)을 벗어날 수
             * 있으므로 반드시 재위치추정을 한다. */
            double moved_c = 0.0;
            if (robot_runner__corner_escape(fpga, lidar, cfg,
                                             -sp->err, CORNER_ROT_MARGIN_M,
                                             "정렬 중 끼임", &moved_c)) {
                robot_runner__relocalize_after_move(lidar, cfg, nav_slam);
                robot_runner__init_prev_pose(fpga, cfg->has_encoder,
                                              &st->prev_rx, &st->prev_ry, &st->prev_rth);
            }
        }
        st->align_stall = 0;
        st->last_th = 1e9;
        return ALIGN_RETRY;
    }
} else {
    st->align_stall = 0;
}
st->last_th = sp->theta;
    return ALIGN_GO;
}


/* 끝낼 때인가: 허용오차 안 / 핑퐁 바닥 / 하드웨어 한계 / 개선 없음. */
static AlignAction align__check_finish(const AlignCtx *ctx, AlignState *st, AlignStep *sp) {
    FPGALink          *fpga     = ctx->fpga;
    LidarThread       *lidar    = ctx->lidar;
    OccupancyGridSLAM *nav_slam = ctx->nav_slam;
    const RobotConfig *cfg      = ctx->cfg;
    const double tol_deg     = ctx->tol_deg;

/* 오차 부호가 뒤집혔는지 세기 (핑퐁 감지, 2026-08-09a) */
if (st->have_prev_err && ((st->prev_err_deg > 0.0) != (sp->err_deg > 0.0))) st->sign_flips++;
st->prev_err_deg = sp->err_deg;
st->have_prev_err = true;

if (fabs(sp->err_deg) <= tol_deg) {
    printf("[slot] 정렬 완료: 각도오차 %.2fdeg (허용 %.2f), pose=(%.3f,%.3f)\n",
           sp->err_deg, tol_deg, sp->x, sp->y);
    fflush(stdout);
    fpga_link_set_speed(fpga, 0, 0);
    return ALIGN_DONE;
}

/* 이 하드웨어가 만들 수 있는 최소 회전량의 절반보다 오차가 작으면, 여기서
 * 한 번 더 돌려봤자 반대편으로 더 벌어질 뿐임(docking.h의 양자화 처리와 같은 취지). */
double min_step_deg = rot_rate_min_step_deg(st->rot_est, cfg);
double accept_deg = (tol_deg > 0.5 * min_step_deg) ? tol_deg : 0.5 * min_step_deg;
/* ============================================================
 * 포기해도 되는 오차의 상한 (2026-08-10c) *** 교착의 직접 원인 ***
 *
 * 호출부(run_shuttle)의 진입 게이트는 각도오차 6.0도다. 그런데 위 식은
 * 각속도 추정이 튀면 accept_deg 를 8.2도까지 올려버린다. 그러면
 *   slot_align  : "8.2도 안이니 하드웨어 한계" -> 성공(true) 반환
 *   진입 게이트 : "6.0도를 넘었다" -> 진입 보류
 * 가 되어 재시도를 몇 번 하든 똑같은 자리에서 끝난다(실기에서 3/3 전부 실패).
 *
 * 정렬이 스스로 포기해도 되는 값은 게이트가 받아주는 값보다 클 수 없다.
 * 그래서 허용오차의 2배(2.5 -> 5.0도)로 못을 박는다. 게이트 6.0도보다
 * 확실히 작으므로 이 교착은 구조적으로 사라진다. */
double accept_cap = 2.0 * tol_deg;
if (accept_deg > accept_cap) accept_deg = accept_cap;

/* 핑퐁 종료 (2026-08-09a): 부호가 두 번 이상 뒤집혔고 오차가 이미 최소 회전
 * 스텝 수준이면, 여기서 한 번 더 돌리는 건 반대편으로 넘기는 것일 뿐이다.
 * 이 검사가 없으면 MAX_STEPS(40스텝, 20초 이상)를 노이즈 쫓는 데 다 쓰고
 * 결국 "한도 초과"로 끝났음 - 사용자가 본 "계속 각도를 틀어버린다"가 이것. */
if (st->sign_flips >= 2 && fabs(sp->err_deg) <= 1.2 * min_step_deg
    && fabs(sp->err_deg) <= accept_cap) {
    printf("[slot] 정렬 종료(분해능 바닥): 오차 %+.2fdeg, 부호 뒤집힘 %d회.\n"
           "       최소 회전 스텝이 %.1fdeg 라 이보다 더는 못 줄입니다. 진입합니다.\n",
           sp->err_deg, st->sign_flips, min_step_deg);
    fflush(stdout);
    fpga_link_set_speed(fpga, 0, 0);
    return ALIGN_DONE;
}

/* '하드웨어 한계'는 계산이 아니라 실측으로만 인정한다 (2026-08-10c).
 *
 * 예전에는 min_step_deg(모델 계산값) 하나로 곧바로 포기했다. 실기 로그에서
 *   [slot] 정렬 중: 각도오차 -11.53deg -> cmd=(-34,34)
 *   [slot] 정렬 중단(하드웨어 한계): 오차 -6.11deg
 * 처럼 6.11도를 '한 번도 줄여보지 않고' 포기했다. 그런데 같은 실행의 앞부분에서
 *   각도오차 +7.17deg -> ... -> 정렬 완료: 각도오차 1.68deg
 * 즉 7.17도를 한 스텝에 1.68도까지 줄인 기록이 있다. 못 하는 게 아니라
 * 안 해본 것이었다.
 * 이제는 '한 번 이상 시도했는데 안 줄었다'(no_improve>=1)는 증거가 있을 때만
 * 이 종료를 쓴다. */
if (fabs(sp->err_deg) <= accept_deg && st->no_improve >= 1) {
    printf("[slot] 정렬 중단(하드웨어 한계): 오차 %.2fdeg. 최소 회전 스텝이 %.1fdeg라\n"
           "       허용오차 %.2fdeg 안으로는 물리적으로 못 들어갑니다.\n"
           "       (%d번 더 돌려봤지만 안 줄었습니다. --sg-move-min 을 낮추면\n"
           "        더 작은 스텝을 만들 수 있습니다 - 현재 %.2fs)\n",
           sp->err_deg, min_step_deg, tol_deg, st->no_improve, cfg->sg_move_sec_min);
    fflush(stdout);
    fpga_link_set_speed(fpga, 0, 0);
    return ALIGN_DONE;
}

/* 오차가 더 이상 줄지 않을 때.
 *
 * BUGFIX (2026-08-08g): 예전엔 무조건 true(정렬 성공)를 반환했음. 그래서
 * 실기에서 "오차가 129.50deg 아래로 안 내려감 -> 최소 회전 스텝 2.1deg가
 * 허용오차보다 커서" 라는 앞뒤가 안 맞는 메시지를 내고 성공으로 넘어갔고,
 * 호출부는 그 상태로 슬롯에 진입해서 벽으로 돌진했음.
 * 오차가 하드웨어 한계 근처면 진짜 양자화 문제지만, 그보다 훨씬 크면
 * '차가 물려서 못 도는 것'이다 - 그때는 탈출을 시도하고, 그래도 안 되면
 * 실패로 알려야 한다. */
if (fabs(sp->err_deg) < st->best_abs_err - 0.1) {
    st->best_abs_err = fabs(sp->err_deg);
    st->no_improve = 0;
} else if (++st->no_improve >= 4) {
    /* 2.0*accept_deg -> accept_cap (2026-08-10c). 진입 게이트(6.0도)보다
     * 큰 오차를 '성공'으로 넘기면 게이트에서 반드시 거부당해 교착이 된다. */
    if (fabs(sp->err_deg) <= accept_cap) {
        printf("[slot] 정렬 중단(개선 없음): 오차 %.2fdeg 에서 더 줄지 않습니다.\n"
               "       최소 회전 스텝 %.1fdeg 수준이라 여기가 한계입니다.\n",
               sp->err_deg, min_step_deg);
        fflush(stdout);
        fpga_link_set_speed(fpga, 0, 0);
        return ALIGN_DONE;
    }
    if (st->no_motion >= 2) {
        /* 아예 안 도는 중이면 탈출(=또 다른 짧은 모터 명령)도 소용없다.
         * 펄스 배수를 계속 키우면서 회전을 다시 시도한다 (2026-08-09k). */
        printf("[slot] 오차 %.1fdeg 가 안 줄지만 '물린' 게 아니라 '안 도는' 상태입니다 "
               "- 탈출 대신 펄스를 키워 다시 시도합니다\n", fabs(sp->err_deg));
        fflush(stdout);
        st->no_improve = 0;
        st->best_abs_err = 1e9;
        return ALIGN_RETRY;
    }
    if (st->align_escape < 3) {
        st->align_escape++;
        printf("[slot] 정렬이 %.1fdeg 에서 멈췄습니다(한계 %.1fdeg의 %.1f배). "
               "물린 것으로 보고 탈출 시도 %d/3\n",
               fabs(sp->err_deg), accept_deg, fabs(sp->err_deg) / accept_deg, st->align_escape);
        fflush(stdout);
        if (!robot_runner__escape_to_open(fpga, lidar, cfg, ALIGN_ESCAPE_MAX_M, 0.0,
                                           cfg->allow_crab)) {
            /* PATCH (2026-08-10b): 직선 탈출이 안 먹히면 구석에 비스듬히
             * 물린 것이다. 이때 사람은 핸들을 끝까지 꺾어서 크게 돈다.
             * corner_escape 가 좌/우 중 덜 막힌 쪽을 라이다로 골라 크게 돌고,
             * 제자리 회전조차 불가능하면 호(arc)로 빼면서 꺾는다.
             * 크게 돌고 나면 자세추정이 각도 탐색창(±25도)을 벗어날 수 있으므로
             * 반드시 재위치추정을 하고, 명령 적분 기준점도 다시 잡는다. */
            double moved_c = 0.0;
            robot_runner__corner_escape(fpga, lidar, cfg, -sp->err,
                                         CORNER_ROT_MARGIN_M,
                                         "정렬 정체", &moved_c);
            robot_runner__relocalize_after_move(lidar, cfg, nav_slam);
            robot_runner__init_prev_pose(fpga, cfg->has_encoder,
                                          &st->prev_rx, &st->prev_ry, &st->prev_rth);
            /* 각속도/스윕 상태도 초기화 - 크게 돈 직후라 이전 학습 표본이 오염됨 */
            st->rot_pending = false;
            st->sweep_pending = false;
            st->sweep_used = false;
            st->rc_have = false;
            st->rc_done = false;
        }
        st->no_improve = 0;
        st->best_abs_err = 1e9;
        st->last_th = 1e9;
        st->align_stall = 0;
        st->sign_flips = 0;
        st->have_prev_err = false;
        return ALIGN_RETRY;
    }
    if (st->no_motion >= 3) {
        printf("[slot] *** 정렬 실패: 오차 %.1fdeg 인데 바퀴가 명령에 전혀 "
               "반응하지 않습니다.\n"
               "       펄스를 %.1f배(최대 %.2fs)까지, 속도를 %d까지 올렸는데도 "
               "안 돕니다.\n"
               "       확인할 것: (1) 모터 전원/배터리 전압 (2) FPGA 링크\n"
               "                  (3) 바퀴가 바닥에 닿아 있는지 (4) --pwm-deadzone %d 이\n"
               "                      실제 기동 임계값보다 낮지 않은지 ***\n",
               sp->err_deg, st->kick_gain, cfg->sg_move_sec_max,
               (int)lround(cfg->sg_step_speed * (1.0 + 0.25 * (st->no_motion - 2))),
               PWM_DEADZONE);
    } else {
        printf("[slot] *** 정렬 실패: 오차 %.1fdeg 가 남았는데 더 돌지 못합니다.\n"
               "       탈출을 3번 시도했지만 풀리지 않았습니다. 이 각도로 슬롯에\n"
               "       진입하면 벽으로 돌진하므로 여기서 중단합니다. ***\n", sp->err_deg);
    }
    fflush(stdout);
    fpga_link_set_speed(fpga, 0, 0);
    return ALIGN_FAIL;
}
    return ALIGN_GO;
}


/* 이번 스텝의 회전 시간을 확정하고, 그 각도만큼 돌 자리가 있는지 본다.
 * 검사와 명령이 반드시 같은 값을 써야 한다 - 그 어긋남이 2026-08-10m 버그였다. */
static AlignAction align__plan_rotation(const AlignCtx *ctx, AlignState *st, AlignStep *sp) {
    FPGALink          *fpga     = ctx->fpga;
    LidarThread       *lidar    = ctx->lidar;
    OccupancyGridSLAM *nav_slam = ctx->nav_slam;
    const RobotConfig *cfg      = ctx->cfg;

/* ============================================================
 * 돌기 전에 '이번 스텝 각도만큼 돌 자리가 있나'를 본다 (2026-08-10b 신규)
 *
 * navigate_one_leg 쪽에는 이 검사가 있는데 slot_align 에는 아예 없었다.
 * 그래서 대기점 근처에서 벽에 물려도 그냥 명령을 계속 내보냈고, 바퀴가
 * 헛돌면서 차체를 벽에 밀어붙였다. 폼보드 세트장에서는 그게 제일 위험하고,
 * 사용자가 본 "구석에 몸을 박고 못 나온다"의 마지막 고리이기도 하다.
 *
 * 대응은 navigate_one_leg 과 같은 3단계다.
 *   1) 여유가 있으면 그대로 돈다.
 *   2) 빠듯하면 '돌 수 있는 비율'만큼만 돈다(시간에 그 비율을 곱한다).
 *   3) 최소 각도조차 못 돌면 밀지 말고, 벽 반대쪽으로 물러나 자리를 만든다.
 *      (corner_backoff 는 벽 쪽으로는 절대 움직이지 않는다)
 * ============================================================ */
/* ============================================================
 * (2026-08-10m) *** 이 루프가 구석에서 못 빠져나오던 직접 원인 ***
 *
 * 아래 안전검사는 "이번 스텝에 돌 각도" 기준이어야 하는데, 예전에는
 * 그 각도를 이렇게 구했다.
 *     sec_pre = min(|오차| / 각속도, sg_move_sec_max)      <- 창 제한 없음
 *     step_rad = 각속도 * sec_pre * 1.15
 * 그런데 '실제로 명령하는' 시간은 저 아래에서 한 단계를 더 거친다.
 *     sec = clamp_rotation_sec_rate(...)   // 스캔매칭 탐색창의 70%
 * 즉 검사는 창 제한을 빼먹은 값으로 하고, 명령은 창 제한을 걸어서 냈다.
 *
 * 실기 로그의 숫자로 확인하면 (각속도 85deg/s, 창 ±25도, sg-move-max 0.45s):
 *     검사가 쓴 각도 = 85 * 0.45 * 1.15 = 44.0deg  -> 필요 여유 142.5mm
 *     실제 명령 각도 = 85 * (25*0.7/85) = 17.5deg  -> 필요 여유  88.2mm
 * 그 순간 라이다가 잰 거리는 125mm 였다. 즉
 *     17.5도만 돌면 여유 +37mm 로 충분한데, 44도 기준으로 재서 -17mm 로 읽고
 *     "지금 자리에서는 4.0deg 도 못 돕니다" 라며 회전을 통째로 포기했다.
 * 그 뒤로는 15mm/5mm 씩 앞뒤로 깨작거리는 것 말고 할 수 있는 게 없어서,
 * 오차가 113도에 붙박인 채 60스텝을 태우고 "정렬 실패"로 끝났다.
 * (navigate_one_leg 에는 2026-08-09f 에 같은 수정이 이미 들어가 있었다.
 *  거기는 clamp_rotation_sec_rate 를 먼저 걸고 rot_step_rad 를 만든다.)
 *
 * 해결: 명령 시간을 '검사보다 먼저' 한 번만 확정하고, 검사와 명령이
 * 똑같은 값을 쓰게 한다. 이렇게 하면 두 값이 다시 어긋날 수 없다.
 * ============================================================ */
sp->rate_cmd = rot_rate_get(st->rot_est);
sp->sec_cmd  = fabs(sp->err) / sp->rate_cmd;
sp->sec_cmd *= st->kick_gain;                 /* 안 돌면 점점 길게 (2026-08-09k) */
if (sp->sec_cmd < cfg->sg_move_sec_min) sp->sec_cmd = cfg->sg_move_sec_min;
if (sp->sec_cmd > cfg->sg_move_sec_max) sp->sec_cmd = cfg->sg_move_sec_max;
{
    /* 한 스텝에 탐색창보다 크게 돌면 위치추정이 회전을 못 따라감 -> 지나침.
     * 단, 아예 안 돌고 있는 중이면 이 제한이 오히려 함정이다 - 짧아서 못 도는데
     * 더 짧게 만들 뿐이라, 정지마찰 돌파 중에는 최소 이동시간을 보장한다. */
    double capped = robot_runner__clamp_rotation_sec_rate(cfg, nav_slam,
                                                           sp->rate_cmd, sp->sec_cmd);
    sp->sec_cmd = (st->no_motion > 0 && capped < sp->sec_cmd) ? sp->sec_cmd : capped;
    if (st->no_motion > 0 && sp->sec_cmd > cfg->sg_move_sec_max)
        sp->sec_cmd = cfg->sg_move_sec_max;
}

sp->rot_frac = 1.0;
{
    const double HL_A = 0.5 * cfg->body_length;
    const double HW_A = 0.5 * cfg->body_width;
    double step_rad = -((sp->err > 0.0) ? +1.0 : -1.0) * sp->rate_cmd * sp->sec_cmd * 1.15;
    double gap_a = 0.0, gdir_a = 0.0, need_a = 0.0;
    if (!robot_runner__rotation_is_safe(&sp->scan, step_rad, HL_A, HW_A, 0.010,
                                         &gap_a, &gdir_a, &need_a)) {
        double frac = robot_runner__max_safe_rotation_frac(&sp->scan, step_rad,
                                                            HL_A, HW_A, 0.010);
        double allow_deg = fabs(step_rad) * 180.0 / ANGLE_PI * frac;
        if (allow_deg >= 4.0) {
            sp->rot_frac = frac;
            printf("[slot] 회전 여유가 빠듯해(%+.0fdeg 방향 %.0fmm) 이번엔 "
                   "%.1fdeg 만 돕니다\n",
                   gdir_a, (need_a + gap_a) * 1000.0, allow_deg);
            fflush(stdout);
        } else if (st->room_tries < ALIGN_ROOM_TRIES_MAX) {
            st->room_tries++;
            printf("[slot] 지금 자리에서는 %.1fdeg 도 못 돕니다 "
                   "(%+.0fdeg 방향: 실제 %.0fmm, 차체가 %.0fmm 뻗음 "
                   "-> 여유 %+.0fmm, 필요 %.0fmm).\n"
                   "       벽에 밀어붙이지 않고 먼저 자리를 만들겠습니다 (%d/%d)\n",
                   4.0, gdir_a, (need_a + gap_a) * 1000.0, need_a * 1000.0,
                   gap_a * 1000.0, 10.0, st->room_tries, ALIGN_ROOM_TRIES_MAX);
            fflush(stdout);
            fpga_link_set_speed(fpga, 0, 0);
            /* ============================================================
             * 두 단계로 자리를 만든다 (2026-08-10m)
             *
             * 예전에는 언제나 '남은 오차 전체(-err)'를 돌 자리를 요구했다.
             * -err 이 113도쯤 되면 회전 중 차체 모서리가 사방을 다 훑으므로
             * 필요 여유가 반대각선 143mm 로 고정된다. 그런데 이 통로는
             * 높이가 400mm 라, 그 조건은 차체중심이 통로 정중앙 ±57mm
             * 안에 있을 때만 성립한다(set_map.txt 실측). 실기에서 로봇은
             * 대기점(y=0.6475)보다 58mm 위인 y=0.706 에 있었으므로,
             * 이 요구는 '앞뒤로 아무리 움직여도 절대 만족 못 하는 조건'이었다.
             * 그래서 corner_backoff 가 매번 실패 -> corner_escape -> 재위치추정
             * 이라는 비싼 경로만 반복했다.
             *
             *   1) 먼저 전체 회전분을 노린다 (되면 남은 회전이 한 번에 끝난다)
             *   2) 안 되면 '이번 스텝 + 30%' 만큼만 확보한다. 이건 143mm 가
             *      아니라 90mm 안팎이면 되므로 대부분 바로 만들어진다.
             *      한 스텝 돌고 다시 자리를 만드는 식으로 조금씩 풀린다.
             * ============================================================ */
            bool room_ok = robot_runner__corner_backoff(fpga, lidar, nav_slam, cfg,
                                                         -sp->err, 0.015, 0.12,
                                                         &st->prev_rx, &st->prev_ry, &st->prev_rth);
            if (!room_ok) {
                room_ok = robot_runner__corner_backoff(fpga, lidar, nav_slam, cfg,
                                                        step_rad * 1.3, 0.012, 0.08,
                                                        &st->prev_rx, &st->prev_ry, &st->prev_rth);
                if (room_ok) {
                    printf("[slot] 전체 회전분(%.0fdeg)은 이 통로에서 못 만들지만 "
                           "이번 스텝(%.0fdeg) 자리는 확보했습니다 "
                           "- 나눠서 돌겠습니다\n",
                           fabs(sp->err_deg), fabs(step_rad) * 180.0 / ANGLE_PI);
                    fflush(stdout);
                }
            }
            if (!room_ok) {
                double moved_c = 0.0;
                robot_runner__corner_escape(fpga, lidar, cfg, -sp->err,
                                             CORNER_ROT_MARGIN_M,
                                             "정렬 전 자리확보", &moved_c);
                robot_runner__relocalize_after_move(lidar, cfg, nav_slam);
                robot_runner__init_prev_pose(fpga, cfg->has_encoder,
                                              &st->prev_rx, &st->prev_ry, &st->prev_rth);
            }
            /* 자리가 새로 생겼으면 연속 회전(스윕)도 다시 시도할 수 있다.
             * 예전에는 한 번 보류되면 sweep_used 가 영구히 latch 돼서,
             * 여유가 회복돼도 17.5도씩 끊어 도는 느린 경로만 남았다. */
            st->sweep_used = false;
            st->rc_have = false; st->rc_done = false;
            st->last_th = 1e9;
            st->align_stall = 0;
            return ALIGN_RETRY;
        }
    }
}
    return ALIGN_GO;
}


static void align__rotate(const AlignCtx *ctx, AlignState *st, AlignStep *sp) {
    FPGALink          *fpga     = ctx->fpga;
    const RobotConfig *cfg      = ctx->cfg;

/* 제자리 회전. err>0(목표보다 반시계로 더 돌아있음)이면 시계로 되돌려야 하므로
 * 왼바퀴 전진/오른바퀴 후진(= l=+, r=-). slot_drive의 부호 검산과 동일한 규약. */
int w = cfg->sg_step_speed;
/* 여러 번 안 돌았으면 속도까지 올린다(상한 80 = PWM 255) */
if (st->no_motion >= 3) {
    w = (int)lround(cfg->sg_step_speed * (1.0 + 0.25 * (st->no_motion - 2)));
    if (w > 80) w = 80;
}
int l = (sp->err > 0) ? w : -w;
int r = -l;
if (st->no_motion > 0) {
    printf("[slot] 정렬 중: 각도오차 %+.2fdeg -> cmd=(%d,%d) "
           "[안 돌아서 펄스 %.1f배%s]\n",
           sp->err_deg, l, r, st->kick_gain,
           (w > cfg->sg_step_speed) ? ", 속도도 올림" : "");
} else {
    printf("[slot] 정렬 중: 각도오차 %+.2fdeg -> cmd=(%d,%d)\n", sp->err_deg, l, r);
}
fflush(stdout);
fpga_link_set_speed(fpga, l, r);
/* 회전 시간은 위 안전검사 직전에 이미 확정했다 (2026-08-10m).
 * 여기서 다시 계산하면 검사가 본 각도와 명령한 각도가 어긋나므로
 * 절대 재계산하지 않는다 - 그 어긋남이 바로 이번에 고친 버그였다. */
double sec = sp->sec_cmd;
/* '돌 수 있는 만큼만' 판정이 났으면 그 비율만큼 시간을 줄인다 (2026-08-10b).
 * 최소 이동시간은 정지마찰 하한이라 그 아래로는 못 줄인다. */
if (sp->rot_frac < 1.0) {
    sec *= sp->rot_frac;
    if (sec < cfg->sg_move_sec_min) sec = cfg->sg_move_sec_min;
}
st->rot_prev_theta = sp->theta;
st->rot_prev_sec = sec;
/* err>0 이면 시계로 되돌린다(theta 감소) -> 명령 방향 -1 */
st->rot_prev_dir = (sp->err > 0.0) ? -1.0 : +1.0;
st->rot_pending = true;
robot_runner__sleep_sec(sec);
}


/* ============================================================
 * 돌 자리 확보 (2026-08-10b 전면 개정) *** '구석에 몸을 박는' 직접 원인 ***
 *
 * 예전 코드(2026-08-09h)는 물러날 방향을 이렇게 골랐다.
 *     double fwd_y = sin(th) * out_sign_y;
 *     int dir = (fwd_y >= 0.0) ? +1 : -1;
 * "슬롯 축(y)에서 밖으로 나가는 쪽"을 고른다는 뜻인데, 여기에 함정이 있다.
 *
 * A->B 로 통로를 달려와 대기점에 막 도착한 로봇의 heading 은 0도 근처다.
 * sin(0) = 0 이므로 조건이 참이 되어 '전진'이 뽑히는데, heading 0도에서
 * 전진은 곧 +x 방향이다. 슬롯 축(y)으로는 sin(0)*d = 0mm 를 벌면서
 * 오른쪽 벽까지의 거리만 그대로 까먹는다.
 *
 * set_map.txt 실측: B 대기점 (1.275, 0.6475) -> 오른쪽 벽(x=1.45)까지 175mm.
 * 90도 제자리 회전에 필요한 값은 반대각선 143.1 + 여유 15 = 158mm 이므로
 * 원래 여유가 17mm 밖에 없다. 여기서 앞으로 밀면 그대로 구석에 박힌다.
 * 실기 로그가 정확히 그 모습이다.
 *     [slot] 돌 자리 부족(-66deg 방향 144mm < 필요 158mm) -> 슬롯 밖으로 전진 14mm
 *     [slot] 돌 자리확보: 정면 여유가 142mm 뿐이라 더 못 물러납니다   <- 142 = 127.5+15
 *   -> 코앞 15mm 까지 벽에 밀어붙인 뒤 멈췄다. 그 자세로 90도를 돌라고 하니
 *      당연히 안 돌고, 끼임 -> 탈출 -> 정렬실패가 반복된다.
 *
 * 고친 방식: 방향을 '가정'하지 않는다. 전/후 양쪽으로 5mm 씩 옮겨봤다고 치고
 * 회전 여유를 실제 스캔으로 예측해서(robot_runner__rot_gap_shifted), 여유가
 * 실제로 늘어나는 쪽만 고른다. 벽 쪽 이동은 여유가 줄어들므로 절대 뽑히지 않는다.
 * 또 '여유가 최대'가 아니라 '필요한 여유를 만드는 최소 이동'을 고른다 -
 * 대기점에서 30mm 넘게 벗어나면 뒤이어 slot_lateral_align(90도 회전 두 번,
 * 약 5.6초)이 걸려서 오히려 손해이기 때문이다.
 *
 * 함수 시그니처는 호출부 호환을 위해 그대로 두고, 알맹이만 corner_backoff 로
 * 넘긴다. out_sign_y 는 더 이상 쓰지 않는다(라이다가 직접 방향을 정한다).
 * ============================================================ */
bool robot_runner__ensure_rotation_room(FPGALink *fpga, LidarThread *lidar,
        OccupancyGridSLAM *nav_slam, const RobotConfig *cfg,
        double turn_rad, double out_sign_y, double max_total_m,
        double *prev_rx, double *prev_ry, double *prev_rth) {
    (void)out_sign_y;   /* 방향은 이제 스캔으로 정한다 - 아래 주석 참고 */
    return robot_runner__corner_backoff(fpga, lidar, nav_slam, cfg,
                                         turn_rad, 0.015, max_total_m,
                                         prev_rx, prev_ry, prev_rth);
}
bool slot_align_inner(FPGALink *fpga, LidarThread *lidar, OccupancyGridSLAM *nav_slam,
                      const RobotConfig *cfg, double heading_rad, double tol_deg) {
    AlignCtx ctx = { fpga, lidar, nav_slam, cfg, heading_rad, tol_deg };
    AlignState st;
    memset(&st, 0, sizeof(st));
    st.last_th       = 1e9;
    st.best_err_seen = 1e9;
    st.best_abs_err  = 1e9;
    st.kick_gain     = 1.0;
/* navigate_one_leg 과 같은 이유로 각속도를 실측 보정함 (2026-08-08d).
 * 여기가 90도 회전이 실제로 일어나는 곳이라 모델 오차의 영향이 가장 큼. */
    st.rot_est = rot_rate_shared(cfg, cfg->sg_step_speed);

    robot_runner__init_prev_pose(fpga, cfg->has_encoder,
                                 &st.prev_rx, &st.prev_ry, &st.prev_rth);

    for (int step = 0; step < ALIGN_MAX_STEPS && !g_stop_requested; step++) {
        AlignStep sp;
        sp.step = step;
        sp.rot_frac = 1.0;

        AlignAction act = align__observe(&ctx, &st, &sp);
        if (act == ALIGN_RETRY) continue;
        if (act == ALIGN_FAIL)  break;

        act = align__quality_gate(&ctx, &st, &sp);
        if (act == ALIGN_RETRY) continue;
        if (act == ALIGN_FAIL)  return false;

        align__learn_rotation(&ctx, &st, &sp);

        act = align__watchdog_and_sweep(&ctx, &st, &sp);
        if (act == ALIGN_RETRY) continue;
        if (act == ALIGN_FAIL)  return false;

        act = align__handle_stall(&ctx, &st, &sp);
        if (act == ALIGN_RETRY) continue;
        if (act == ALIGN_FAIL)  return false;

        act = align__check_finish(&ctx, &st, &sp);
        if (act == ALIGN_DONE) return true;
        if (act == ALIGN_FAIL) return false;

        act = align__plan_rotation(&ctx, &st, &sp);
        if (act == ALIGN_RETRY) continue;
        if (act == ALIGN_FAIL)  return false;

        align__rotate(&ctx, &st, &sp);
    }
    fpga_link_set_speed(fpga, 0, 0);
    printf("[slot] 정렬: 스텝 한도 초과 (그대로 진입 시도)\n");
    fflush(stdout);
    return false;
}



/* 제자리 회전 전용 탐색창을 씌운 래퍼 (2026-08-09h).
 * slot_align 은 순수 제자리 회전 루프라 위치가 거의 안 변한다(실측: 48도에 21mm).
 * 위치창을 +-150 -> +-60mm 로 좁히고 각도창을 +-12 -> +-25도로 넓히면
 * 한 스텝 회전량이 8.4 -> 17.5도가 되어 정렬 시간이 절반이 된다.
 * 탐색 후보 수는 n_xy^2 * n_th 이라 오히려 1089 -> 650 으로 줄어든다. */
bool slot_align(FPGALink *fpga, LidarThread *lidar, OccupancyGridSLAM *nav_slam,
                               const RobotConfig *cfg, double heading_rad, double tol_deg) {
    SlamWinSave sv;
    slam_win_push_rotation(nav_slam, cfg, &sv);
    bool ok = slot_align_inner(fpga, lidar, nav_slam, cfg, heading_rad, tol_deg);
    slam_win_pop(nav_slam, &sv);
    return ok;
}


bool slot_lateral_align(FPGALink *fpga, LidarThread *lidar,
                                       OccupancyGridSLAM *nav_slam, const RobotConfig *cfg,
                                       double lane_x, double slot_heading_rad) {
    const double LAT_TOL_M = 0.020;   /* 좌우 여유 ±20mm. 나머지는 V자 깔때기가 잡아줌 */
    const int    MAX_TRIES = 2;

    /* PATCH (2026-08-08): --lidar-offset 없이는 이 기동을 하면 안 됨.
     *
     * 자세추정값은 '라이다 위치'인데 라이다가 차체중심보다 앞에 달려 있으면,
     * 제자리 회전만 해도 라이다가 그 반지름만큼 원을 그려서 x가 통째로 바뀜.
     * 실기 로그: 슬롯 방향(-90도)에서 pose=(0.260,0.545), -x 방향(180도)에서
     * pose=(0.225,0.585). 차는 제자리에서 돌기만 했는데 x가 35mm 움직임.
     * 즉 "좌우오차 +35mm"는 실제 어긋남이 아니라 지렛대 착시였고, 옆으로 옮겨도
     * 값이 안 줄어서 1/3, 2/3, 3/3 전부 같은 +35mm가 나왔음(무한 반복).
     *
     * 오프셋을 넣으면 스캔이 차체중심 기준으로 변환되어 회전해도 x가 안 변하고
     * 이 기동이 제대로 동작함. 안 넣었으면 착시를 쫓느라 시간만 버리므로 건너뜀. */
    if (cfg->lidar_offset_forward_m <= 0.0) {
        double x0, y0, th0;
        slam_get_pose(nav_slam, &x0, &y0, &th0);
        printf("[slot] 좌우 정렬 건너뜀: --lidar-offset 이 0이라 회전만 해도 좌우값이\n"
               "       흔들려서 보정이 불가능합니다 (현재 오차 %+.0fmm).\n"
               "       라이다~뒷바퀴축 거리를 재서 --lidar-offset 0.06 처럼 넣어주세요.\n",
               (x0 - lane_x) * 1000.0);
        fflush(stdout);
        return true;
    }

    for (int attempt = 0; attempt < MAX_TRIES && !g_stop_requested; attempt++) {
        double x, y, th;
        slam_get_pose(nav_slam, &x, &y, &th);
        double err = x - lane_x;
        if (fabs(err) <= LAT_TOL_M) {
            printf("[slot] 좌우 정렬 완료: 오차 %+.0fmm (허용 %.0fmm)\n",
                   err * 1000.0, LAT_TOL_M * 1000.0);
            fflush(stdout);
            return true;
        }

        /* ============================================================
         * 대각선 이동으로 좌우 보정 (2026-08-09m)
         *
         * 사용자 제안: "완전히 좌우방향이 아닌 대각선 방향으로 살짝 움직이는
         * 걸로는 x가 잘 안 맞춰져? 정반대 방향으로 각도 정렬하는 횟수를 줄이자."
         * -> 맞습니다. 기하를 정리하면 그대로 됩니다.
         *
         * 슬롯 축 a(=-90도)에서 phi 만큼 기울인 자세 h = a + s*phi 로 u 만큼 가면
         *     dx = u*cos(h) = u*s*sin(phi)      (a=-90 이므로 cos(-90+s*phi)=s*sin(phi))
         *     dy = u*sin(h) = -u*cos(phi)
         * 오차 e 를 지우려면 dx = -e 이므로 u = -e/(s*sin(phi)), 즉
         *     이동거리 |u| = |e| / sin(phi)      <- phi 가 작을수록 많이 가야 함
         *     세로 이동 |dy| = |e| * cot(phi)    <- phi 가 작을수록 세로로 많이 밀림
         * 회전은 왕복 2*phi.
         *
         *   phi=90도: |u|=|e|,     |dy|=0,       회전 180도 (예전 방식, 약 5.6초)
         *   phi=55도: |u|=1.22|e|, |dy|=0.70|e|, 회전 110도 (기본값, 약 3.4초)
         *   phi=40도: |u|=1.56|e|, |dy|=1.19|e|, 회전  80도
         * 오차가 20~60mm 수준이라 세로로 밀리는 양이 14~42mm 뿐이고, 그건
         * 대기점 전후 허용오차(40mm)와 슬롯 진입이 알아서 흡수한다.
         *
         * s 의 부호로 세로 이동 방향을 고른다: dy 부호 = sign(e/s) 이므로,
         * 슬롯에서 멀어지는 쪽(+y)으로 밀리게 하려면 s = sign(e) 를 쓴다.
         * 단 통로 위쪽 벽에 너무 붙지 않도록, 이미 대기점보다 위면 반대로 고른다.
         * ============================================================ */
        double tilt_deg = cfg->slot_trim_tilt_deg;
        if (tilt_deg < 25.0) tilt_deg = 25.0;
        if (tilt_deg > 90.0) tilt_deg = 90.0;
        double phi = tilt_deg * ANGLE_PI / 180.0;

        /* 세로로 밀릴 방향 선택: 기본은 슬롯에서 멀어지는 쪽(+y), 다만 이미
         * 대기점보다 위에 있으면 슬롯 쪽(-y)으로 민다. */
        int s_sign = (err > 0.0) ? +1 : -1;                  /* dy > 0 (위쪽) */
        if (y > cfg->slot_staging_y) s_sign = -s_sign;       /* dy < 0 (슬롯 쪽) */

        double move_heading = normalize_angle(slot_heading_rad + s_sign * phi);
        double u = -err / (s_sign * sin(phi));               /* + 면 전진, - 면 후진 */
        double dy_pred = -u * cos(phi);

        printf("[slot] 좌우 정렬 %d/%d: 오차 %+.0fmm -> 슬롯 축에서 %+.0fdeg 기울여 "
               "%s %.0fmm\n"
               "       (세로로 %+.0fmm 밀림, 회전은 왕복 %.0fdeg - 90도 정렬 대비 "
               "%.0fdeg 절약)\n",
               attempt + 1, MAX_TRIES, err * 1000.0, s_sign * tilt_deg,
               (u > 0.0) ? "전진" : "후진", fabs(u) * 1000.0,
               dy_pred * 1000.0, 2.0 * tilt_deg, 2.0 * (90.0 - tilt_deg));
        fflush(stdout);

        if (!slot_align(fpga, lidar, nav_slam, cfg, move_heading, 3.0)) return false;
        int trim_sp = (u > 0.0) ? cfg->slot_touch_speed : -cfg->slot_touch_speed;

        /* 목표 x에 닿을 때까지 짧게 밀기 */
        double last_x = 1e9;
        int stall = 0;
        /* PATCH (2026-08-09a): 예전엔 오도메트리 델타를 0으로 못박고 있었음.
         * 엔코더를 달아도 이 구간에서는 전혀 안 쓰였다는 뜻이라, 스캔매칭이 한 번
         * 튀면 그대로 좌우 위치가 튀었음. 다른 구간과 동일하게 델타를 넘겨줌. */
        double lat_prev_rx, lat_prev_ry, lat_prev_rth;
        robot_runner__init_prev_pose(fpga, cfg->has_encoder,
                                      &lat_prev_rx, &lat_prev_ry, &lat_prev_rth);
        for (int step = 0; step < 40 && !g_stop_requested; step++) {
            fpga_link_set_speed(fpga, 0, 0);
            robot_runner__sleep_sec(cfg->slot_settle_sec);
            LidarScan scan;
            if (!robot_runner__take_scan(lidar, cfg, &scan, 2.0)) continue;
            double cx, cy, cth;
            double lodx, lody, lodth;
            robot_runner__predict_delta(fpga, cfg, nav_slam->theta,
                                         &lat_prev_rx, &lat_prev_ry, &lat_prev_rth,
                                         &lodx, &lody, &lodth);
            slam_localize_step(nav_slam, &scan, lodx, lody, lodth, &cx, &cy, &cth);
            robot_runner__emit_state("lateral", step, cx, cy, cth, &scan);

            double remain = cx - lane_x;
            bool arrived = (err > 0.0) ? (remain <= LAT_TOL_M) : (remain >= -LAT_TOL_M);
            if (arrived) break;
            if (fabs(cx - last_x) < 0.003) { if (++stall >= 3) break; } else stall = 0;
            last_x = cx;

            fpga_link_set_speed(fpga, trim_sp, trim_sp);
            robot_runner__sleep_sec(cfg->slot_move_sec);
        }
        fpga_link_set_speed(fpga, 0, 0);

        /* 다시 슬롯 방향으로 */
        if (!slot_align(fpga, lidar, nav_slam, cfg, slot_heading_rad, cfg->slot_align_tol_deg))
            return false;
    }

    double x, y, th;
    slam_get_pose(nav_slam, &x, &y, &th);
    double left_mm = fabs(x - lane_x) * 1000.0;
    /* BUGFIX (2026-08-08g): 예전엔 오차가 얼마든 "그대로 진입합니다" 하고 true 를
     * 반환했음. 실기 로그에서 오차 +350mm 인 채로 슬롯 진입을 시작했는데,
     * 슬롯 폭이 210mm 이므로 그건 슬롯 근처도 아닌 자리에서 벽으로 돌진한 것.
     * V자 깔때기가 잡아줄 수 있는 범위(대략 슬롯 반폭)를 넘으면 실패로 알린다. */
    const double LAT_GIVEUP_M = 0.060;
    if (left_mm > LAT_GIVEUP_M * 1000.0) {
        printf("[slot] *** 좌우 정렬 실패: %d회 시도 후에도 오차 %+.0fmm 가 남았습니다.\n"
               "       깔때기가 잡아줄 수 있는 한계(%.0fmm)를 넘으므로 진입하지 않습니다.\n"
               "       (이 상태로 밀면 슬롯이 아니라 벽으로 들어갑니다) ***\n",
               MAX_TRIES, (x - lane_x) * 1000.0, LAT_GIVEUP_M * 1000.0);
        fflush(stdout);
        return false;
    }
    printf("[slot] 좌우 정렬 %d회 시도 후 오차 %+.0fmm - 깔때기가 잡아줄 범위라 진입합니다\n",
           MAX_TRIES, (x - lane_x) * 1000.0);
    fflush(stdout);
    return true;
}


/* ============================================================
 * 슬롯 안에서 각도를 '크게' 바로잡기 (2026-08-09e 신규)
 *
 * 왜 필요한가 (= 사용자 신고 "머리를 박은 채 답답하게 움직인다"):
 *
 * 실기 로그(2026-08-09):
 *     [slot] step=23 pose=(0.225,0.300,300.0deg) 각도오차=+30.0deg cmd=(38,18)
 *     [slot] 깔때기 걸림 (y=0.300, 목표 0.177, 좌우 -0mm, 각도 +30.0deg)
 *     [slot]    ... -> -6.0deg (라이다: 왼쪽 코가 벽에 닿음)
 *     [slot]    제자리에서 -6.0deg 틀고 다시 진입합니다 (0.07s)
 *     [slot] step=25 pose=(0.225,0.305,297.0deg)   <- 겨우 3도 돌았음
 *     [slot] step=26 pose=(0.225,0.295,298.5deg)   <- 밀자마자 벽이 1.5도 되돌림
 *
 * 즉 각도오차가 30도인데 복구 회전 상한이 6도였다(FUNNEL_TURN_MAX_DEG). 한 번에
 * 실제로 도는 건 3도 남짓이고, 그 다음 강한 밀기가 경사벽에 부딪혀 1.5도를
 * 되돌려 놓으므로 순증은 사이클당 약 1.5도뿐이다. 시도 상한이 5회이니 최대
 * 7.5도밖에 못 줄인다 - 30도짜리 오차는 원리적으로 절대 못 고친다.
 * 그래서 "박은 자세로 계속 밀다가 실패"가 반복됐다.
 *
 * 진입 중 조향으로 못 고치는 이유(계산):
 *   diff 상한 10, 기준속도 28 -> cmd(38,18) -> speed_to_pwm() 데드존 재매핑으로
 *   PWM(210,189) -> 선회 각속도 약 5deg/s. 한 스텝 0.16초면 0.8도.
 *   슬롯 깊이 130mm 를 8스텝에 지나가므로 총 보정량이 6도 남짓이다.
 * 30도 오차 앞에서는 무의미하다. 사용자가 본 그대로 "제자리에서 크게 트는 것"만이
 * 답이고, 그게 이 함수다.
 *
 * 기하 검증 (실기 로그의 그 자세로 계산):
 *   차체 130x255mm 가 yaw 28도로 기울면 가로 폭이
 *     130*cos28 + 255*sin28 = 234mm  >  슬롯 폭 210mm
 *   -> 그 자리에서는 물리적으로 낄 수밖에 없다. 그래서 회전 전에 반드시 조금
 *      물러나 접촉을 풀어야 한다. 축방향으로 50mm 물러나면 중심이 y=0.344 로
 *      올라가고 그 높이의 깔때기 폭은 250mm 라 234mm 가 들어간다 - 회전 가능.
 *   물러날 거리를 각도오차에 비례시키는 근거가 이것이다.
 *
 * 동작:
 *   1) (jammed면) 각도오차에 비례해 15~70mm 물러나 접촉을 푼다
 *   2) 목표 각도까지 '닫힌 루프'로 제자리 회전한다 - 명령 -> 재관측 -> 남은 오차
 *      계산 -> 다시 명령. 한 번 쏘고 마는 개루프가 아니라서 3도밖에 안 돌면
 *      다음 스텝이 그만큼 더 돈다.
 *   3) 회전이 두 스텝 연속 안 먹으면(=아직 물려 있음) 더 물러난 뒤 다시 돈다.
 * ============================================================ */
bool slot_fix_heading_inner(FPGALink *fpga, LidarThread *lidar,
                                     OccupancyGridSLAM *nav_slam, const RobotConfig *cfg,
                                     double target_heading_rad, double tol_deg, bool jammed,
                                     double back_limit_y, int release_dir,
                                     double *prev_rx, double *prev_ry, double *prev_rth,
                                     double *out_herr_deg) {
    const int    MAX_STEPS     = 12;
    const double TURN_MAX_DEG  = 20.0;   /* 한 스텝에 명령할 최대 회전량 */
    const double BACK_MIN_M    = 0.015;
    const double BACK_MAX_M    = 0.070;
    const int    MAX_BACKOFFS  = 3;

    RotRateEst *rot_est = rot_rate_shared(cfg, cfg->sg_step_speed);
    double best_abs = 1e9;
    int    no_improve = 0, backoffs = 0;
    bool   need_backoff = jammed;   /* 물린 상태면 먼저 접촉부터 푼다 */
    double herr_deg = 0.0;

    for (int step = 0; step < MAX_STEPS && !g_stop_requested; step++) {
        fpga_link_set_speed(fpga, 0, 0);
        robot_runner__sleep_sec(cfg->slot_settle_sec);
        if (g_stop_requested) break;

        LidarScan scan;
        if (!robot_runner__take_scan(lidar, cfg, &scan, 2.0)) continue;

        double x, y, theta, odx, ody, odth;
        robot_runner__predict_delta(fpga, cfg, nav_slam->theta, prev_rx, prev_ry, prev_rth,
                                     &odx, &ody, &odth);
        slam_localize_step(nav_slam, &scan, odx, ody, odth, &x, &y, &theta);
        /* (2026-08-11a) 여기서도 라이다 실측을 우선한다. 이 루틴은 '각도오차를
         * 0 으로 만들 때까지' 도는데, 그 각도가 틀리면 멀쩡한 차를 계속 돌리게
         * 된다 - 실기 로그의 '각도 복구 0..7' 8연속 회전이 정확히 그 경우다.
         * 슬롯 평행구간 밖이면 아무 일도 일어나지 않는다. */
        {
            SlotLidarFix ffix;
            if (robot_runner__slot_reanchor_ctx(nav_slam, &scan, cfg, "각도 복구 중", &ffix))
                slam_get_pose(nav_slam, &x, &y, &theta);
        }
        robot_runner__emit_state("slotfix", step, x, y, theta, &scan);

        double herr = normalize_angle(theta - target_heading_rad);
        herr_deg = herr * 180.0 / ANGLE_PI;
        if (out_herr_deg) *out_herr_deg = herr_deg;

        if (fabs(herr_deg) <= tol_deg) {
            printf("[slot] 각도 복구 완료: 오차 %+.1fdeg (허용 %.1f), pose=(%.3f,%.3f)\n",
                   herr_deg, tol_deg, x, y);
            fflush(stdout);
            fpga_link_set_speed(fpga, 0, 0);
            return true;
        }

        /* 회전이 실제로 먹히고 있는지 판정 */
        if (fabs(herr_deg) < best_abs - 0.5) {
            best_abs = fabs(herr_deg);
            no_improve = 0;
        } else if (++no_improve >= 2) {
            need_backoff = true;
            no_improve = 0;
        }

        /* 1) 접촉 풀기 - 각도오차가 클수록 깊이 물려 있으므로 더 물러난다.
         * 130x255mm 차체가 yaw e 로 기울 때 늘어나는 가로폭이 대략 255*sin(e) 이고,
         * 축방향으로 b 만큼 물러나면 코가 벽에서 b*cos(e) 만큼 떨어지므로,
         * 필요한 b 는 각도에 거의 비례한다. 1.8mm/deg 로 잡고 상한을 70mm 로 둔다. */
        /* 접촉을 푸는 방향의 한계.
         *  release_dir < 0 : 슬롯 밖(+y)으로 물러남 -> y 가 한계 이상이면 그만
         *  release_dir > 0 : 슬롯 안(-y)으로 밀어 넣음 -> y 가 한계 이하이면 그만
         * (2026-08-09j: 후진 이탈에서도 이 복구를 쓰게 되면서 방향을 일반화) */
        bool limit_hit = (release_dir < 0) ? (y >= back_limit_y) : (y <= back_limit_y);
        if (need_backoff && limit_hit) {
            printf("[slot] 각도 복구: y=%.3f 로 이미 한계(%.3f)라 "
                   "더 %s 않고 회전만 계속합니다\n", y, back_limit_y,
                   release_dir < 0 ? "물러나지" : "밀어넣지");
            fflush(stdout);
            need_backoff = false;
        }
        if (need_backoff && backoffs < MAX_BACKOFFS) {
            backoffs++;
            double back_m = BACK_MIN_M + 0.0018 * fabs(herr_deg) * backoffs;
            if (back_m < BACK_MIN_M) back_m = BACK_MIN_M;
            if (back_m > BACK_MAX_M) back_m = BACK_MAX_M;
            double sec = robot_runner__move_sec_for(cfg, cfg->slot_step_speed, back_m, 0.60);
            printf("[slot] 각도 복구: 접촉을 풀기 위해 %s %.0fmm (%.2fs, 각도오차 %+.1fdeg, %d/%d)\n",
                   release_dir < 0 ? "물러납니다" : "밀어넣습니다",
                   back_m * 1000.0, sec, herr_deg, backoffs, MAX_BACKOFFS);
            fflush(stdout);
            int rel = release_dir * cfg->slot_step_speed;
            fpga_link_set_speed(fpga, rel, rel);
            robot_runner__sleep_sec(sec);
            fpga_link_set_speed(fpga, 0, 0);
            robot_runner__sleep_sec(cfg->slot_settle_sec);
            need_backoff = false;
            best_abs = 1e9;      /* 자세가 바뀌었으니 개선 판정을 새로 시작 */
            continue;
        }

        /* 2) 목표 각도 쪽으로 제자리 회전. 오차 전체를 목표로 하되 한 스텝 상한과
         * 스캔매칭 탐색창 제한을 함께 건다. 다음 루프에서 다시 재서 남은 만큼 돈다. */
        double turn_deg = -herr_deg;
        if (turn_deg >  TURN_MAX_DEG) turn_deg =  TURN_MAX_DEG;
        if (turn_deg < -TURN_MAX_DEG) turn_deg = -TURN_MAX_DEG;

        int w = cfg->sg_step_speed;
        int l = (turn_deg > 0) ? -w : w;   /* +각도 = 반시계 = 왼바퀴 후진 */
        double rate = rot_rate_get(rot_est);
        if (rate < 1e-3) rate = 1e-3;
        double sec = fabs(turn_deg) * ANGLE_PI / 180.0 / rate;
        if (sec < cfg->sg_move_sec_min) sec = cfg->sg_move_sec_min;
        if (sec > cfg->sg_move_sec_max) sec = cfg->sg_move_sec_max;
        sec = robot_runner__clamp_rotation_sec_rate(cfg, nav_slam, rate, sec);

        double fl = 0.0, fr = 0.0;
        robot_runner__front_side_clearance(&scan, &fl, &fr);
        printf("[slot] 각도 복구 %d: 오차 %+.1fdeg -> 제자리 %+.1fdeg 회전 (%.2fs), "
               "앞왼쪽 %.3fm / 앞오른쪽 %.3fm\n",
               step, herr_deg, turn_deg, sec, fl, fr);
        fflush(stdout);

        double th_before = theta;
        fpga_link_set_speed(fpga, l, -l);
        robot_runner__sleep_sec(sec);
        fpga_link_set_speed(fpga, 0, 0);
        robot_runner__sleep_sec(cfg->slot_settle_sec);

        /* 각속도 실측 보정 - slot_align / navigate_one_leg 과 같은 추정기를 공유 */
        {
            LidarScan after;
            if (robot_runner__take_scan(lidar, cfg, &after, 2.0)) {
                double ax, ay, ath, adx, ady, adth;
                robot_runner__predict_delta(fpga, cfg, nav_slam->theta,
                                             prev_rx, prev_ry, prev_rth, &adx, &ady, &adth);
                slam_localize_step(nav_slam, &after, adx, ady, adth, &ax, &ay, &ath);
                rot_rate_update(rot_est, normalize_angle(ath - th_before), adth, sec,
                                 nav_slam->search_window_theta,
                                 (turn_deg > 0.0) ? +1.0 : -1.0);
            }
        }
    }

    fpga_link_set_speed(fpga, 0, 0);
    printf("[slot] 각도 복구 실패: 오차 %+.1fdeg 가 남았습니다 (물러나기 %d회)\n",
           herr_deg, backoffs);
    fflush(stdout);
    if (out_herr_deg) *out_herr_deg = herr_deg;
    return false;
}

/* 회전 전용 탐색창 래퍼 (2026-08-09h) - slot_align 과 같은 이유 */
bool slot_fix_heading(FPGALink *fpga, LidarThread *lidar,
                                     OccupancyGridSLAM *nav_slam, const RobotConfig *cfg,
                                     double target_heading_rad, double tol_deg, bool jammed,
                                     double back_limit_y, int release_dir,
                                     double *prev_rx, double *prev_ry, double *prev_rth,
                                     double *out_herr_deg) {
    SlamWinSave sv;
    slam_win_push_rotation(nav_slam, cfg, &sv);
    bool ok = slot_fix_heading_inner(fpga, lidar, nav_slam, cfg, target_heading_rad, tol_deg,
                                      jammed, back_limit_y, release_dir,
                                      prev_rx, prev_ry, prev_rth, out_herr_deg);
    slam_win_pop(nav_slam, &sv);
    return ok;
}


/* 직전 슬롯 진입에서 '안쪽 벽에 끝내 닿지 않았다'는 표시 (2026-08-10).
 * 슬롯 깊이가 정해져 있으므로 제대로 들어갔으면 반드시 닿는다. 안 닿았다는 것은
 * 자세추정이 슬롯 밖을 슬롯 안이라고 착각했다는 뜻이다. */
bool *robot_runner__slot_no_wall(const RobotConfig *cfg) {
    (void)cfg;
    static bool v = false;
    return &v;
}
