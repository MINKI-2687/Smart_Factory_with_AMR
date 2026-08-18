#include "robot_slot_drive.h"
#include "robot_runner.h"

/* ============================================================
 * slot_drive() 분해 (2026-08-11 리팩토링)
 *
 * 예전에는 1,139줄이 한 함수에 들어 있었다. 로직 자체는 손대지 않고,
 * 스텝 사이에 들고 다니는 값을 SlotDriveState 로, 한 번 정하면 안 바뀌는 값을
 * SlotDriveCtx 로 묶은 뒤 단계별 헬퍼로 잘랐다. 각 헬퍼의 주석은 원래 그 자리에
 * 있던 것 그대로다.
 *
 * 제어흐름은 SlotStepAction 으로 돌려준다. 원본이 루프 안에서 continue / return
 * 을 직접 쓰던 자리가 그대로 대응된다.
 *     SLOT_STEP_GO      : 계속 진행 (원본의 fall-through)
 *     SLOT_STEP_RETRY   : 다시 관측부터 (원본의 continue)
 *     SLOT_STEP_DONE    : 성공 종료 (원본의 return true)
 *     SLOT_STEP_FAIL    : 실패 종료 (원본의 return false)
 * ============================================================ */

/* ---- slot_drive 전용 상수 ----
 * 예전에는 함수 안 지역상수였는데, 분해한 헬퍼들이 같은 값을 봐야 해서
 * 파일 스코프로 올렸다. 값은 그대로다. */
#define SLOT_MAX_STEPS          120
/* 슬롯 안에서는 큰 조향이 오히려 벽에 부딪히게 만드므로 아주 약하게만 보정 */
#define SLOT_K_LATERAL          60.0    /* 좌우 어긋남(m) -> 좌우 바퀴 차 */
#define SLOT_K_HEADING          25.0    /* 각도 어긋남(rad) -> 좌우 바퀴 차 */
#define SLOT_MAX_DIFF           10.0
/* 후진(슬롯 이탈)은 조향 권한을 더 좁힌다 (2026-08-10h) */
#define SLOT_MAX_DIFF_REVERSE   6.0
#define SLOT_STOP_TOL_M         0.015   /* 목표 y에 이만큼 근접하면 도달로 봄 */
#define SLOT_STALL_EPS_M        0.004   /* 이보다 덜 움직이면 '정체'로 셈 */
#define SLOT_STALL_STEPS        3       /* 연속 이만큼 정체하면 벽에 닿은 것 */

/* ============================================================
 * 깔때기 걸림 / 각도 틀어짐 복구 (2026-08-08g -> 2026-08-09e 개정)
 *
 * 원래 요청: "각도가 조금 안 맞아서 V자 깔때기에 걸리면, 슬롯 밖까지 후진해서
 *             처음부터 다시 하지 말고, 각도만 맞춰서 안쪽으로 계속 밀어라."
 * 이 방침 자체는 유지한다. 다만 2026-08-08g 구현은 회전량 상한이 6도여서
 * 30도짜리 오차 앞에서는 원리적으로 수렴할 수 없었다(slot_fix_heading 머리말의
 * 실기 로그 참고). 이제 두 가지가 달라졌다.
 *
 *   (1) '걸릴 때까지' 기다리지 않는다. 각도오차가 SLOT_HERR_GUARD_DEG 를 넘는
 *       순간 밀기를 멈춘다 - 계속 밀면 경사벽이 각도를 더 밀어올릴 뿐이다.
 *   (2) 복구는 slot_fix_heading() 이 한다. 각도에 비례해 물러나 접촉을 풀고
 *       (yaw 28도면 가로폭이 234mm 라 210mm 슬롯에 물리적으로 낀 상태다),
 *       닫힌 루프로 목표 각도까지 크게 돈다. 슬롯 입구 밖까지는 물러나지
 *       않으므로 좌우 정렬은 유지된다.
 *   (3) 그래도 SLOT_HERR_ABORT_DEG 이상 남으면 여기서 실패를 돌려주고
 *       호출부가 대기점까지 완전히 빼서 다시 접근한다 - 못 고칠 자세로
 *       120스텝을 계속 박는 것보다 훨씬 빠르고 세트장에도 안전하다.
 * 진전이 있으면(더 깊이 들어갔으면) 시도 횟수를 되돌려준다.
 * ============================================================ */
#define FUNNEL_MAX_TRIES        5
#define FUNNEL_PROGRESS_M       0.010   /* 이만큼 더 들어갔으면 '진전' */

/* ---- 각도 문턱값들 (2026-08-09e) ----
 * SLOT_HERR_GUARD_DEG : 이만큼 틀어지면 '걸릴 때까지' 기다리지 않고 즉시 개입.
 *     근거: 슬롯 폭 210mm, 차체 130x255mm 이므로 yaw e 에서 필요한 가로폭이
 *     130*cos(e) + 255*sin(e) 이고, 계산하면 이 값이 210mm 를 넘는 임계각이
 *     20.2도다. 그 위로는 평행구간에 물리적으로 못 들어간다.
 *     실기 로그에서 벽이 각도를 밀어올리는 속도는 밀기 한 번에 약 1.5도였으므로,
 *     8도에서 끊으면 임계각까지 8번의 여유가 남는다 - 복구가 실패해도 낀
 *     상태가 되기 전에 한 번 더 손 쓸 수 있는 간격이다.
 * SLOT_HERR_ABORT_DEG : 복구를 하고도 이만큼 남으면 슬롯 안에서는 못 고친다고
 *     보고 실패를 반환한다(호출부가 대기점까지 빼서 재접근).
 *     기울임(slot_escape_bias_deg)까지 얹은 목표 각도 기준이므로,
 *     기울임 + 여유 만큼은 잡아둔다.
 *
 * 2026-08-09f: 예전의 SLOT_HERR_NUDGE_DEG / SLOT_NUDGE_DEG(각도가 맞을 때만
 * 살짝 기울이던 것)는 없앴다. 이제 걸림 복구는 각도오차 크기와 무관하게 항상
 * '빈 쪽으로 기울인 각도'를 목표로 잡는다 - 걸렸다는 사실 자체가 그 벽 쪽으로
 * 치우쳐 있다는 관측이므로, 축으로만 되돌리면 같은 벽을 또 긁는다. */
#define SLOT_HERR_GUARD_DEG     8.0
#define SLOT_HERR_ABORT_DEG     6.0

/* 착석 판정 문턱값 (원래 루프 안 지역상수) */
#define SLOT_SEAT_HERR_D        6.0     /* 정면 접촉으로 인정할 각도오차 */
#define SLOT_SEAT_CONFIRM_M     0.060   /* 이 안이면 재확인 대상 */
#define SLOT_SEAT_FRONT_PAD     0.100   /* 반길이+이만큼보다 멀면 '벽 아님' */

/* 한 번 정하면 안 바뀌는 값들 */
typedef struct {
    FPGALink          *fpga;
    LidarThread       *lidar;
    OccupancyGridSLAM *nav_slam;
    const RobotConfig *cfg;
    double             lane_x;       /* 슬롯 중심선 x */
    double             heading_rad;  /* 슬롯 축 방향 */
    double             stop_y;       /* 목표 깊이 */
    bool               forward;      /* true=진입, false=이탈 */
    const char        *label;
} SlotDriveCtx;

/* 스텝 사이에 들고 다니는 상태. 예전에는 전부 slot_drive() 의 지역변수였다. */
typedef struct {
    /* 진행/정체 추적 */
    double last_y;
    int    stall_count;
    double best_depth_y;         /* 지금까지 도달한 가장 깊은 y (작을수록 깊음) */
    /* 깔때기 걸림 복구 */
    int    funnel_tries;
    bool   funnel_kick;          /* 다음 밀기를 강하게 */
    int    push_grace;           /* 복구 직후 '실제로 밀어보는' 유예 (2026-08-10h) */
    double last_fix_y, last_fix_herr;
    int    fix_no_progress;
    bool   last_ditch_done;      /* '마지막으로 곧게 한 번 더'를 이미 썼는가 */
    int    reloc_asked;          /* 라이다로도 슬롯을 특정 못 한 횟수 (2026-08-11c) */
    /* 재진입 반대편 기울임 (2026-08-09f) */
    double bias_rad;             /* 지금 적용 중인 기울임(+ = 반시계). 0이면 슬롯 축 */
    double bias_from_y;          /* 기울임을 걸기 시작한 깊이 */
    double bias_span_m;          /* 이번 기울임을 유지할 깊이 (2026-08-10n) */
    bool   bias_is_residual;     /* 잔여 기울임 단계로 넘어갔는가 (2026-08-10g) */
    /* 착석 재확인 (2026-08-09g) */
    double seat_confirm_y;
    int    seat_confirm_n;
    /* 오도메트리 기준 자세 */
    double prev_rx, prev_ry, prev_rth;
} SlotDriveState;

/* 이번 스텝에 관측한 것 */
typedef struct {
    int          step;
    LidarScan    scan;
    double       x, y, theta;
    SlotLidarFix sfix;
} SlotStep;

/* 착석 판정 결과 */
typedef struct {
    bool contact;      /* 밀고 있는데 연속으로 안 움직임 = 벽에 닿았다 */
    bool depth_ok;     /* 목표 깊이에 도달 */
    bool seated;       /* 네 조건을 모두 만족 = 주차 완료 */
    bool over_limit;   /* 접촉 없이 목표보다 더 들어감 = 안전한계 */
    bool reached;
    bool stalled;
    bool near_depth;
} SlotSeatEval;

typedef enum {
    SLOT_STEP_GO = 0,   /* 계속 진행 */
    SLOT_STEP_RETRY,    /* 다시 관측부터 (원본의 continue) */
    SLOT_STEP_DONE,     /* 성공 종료 */
    SLOT_STEP_FAIL,     /* 실패 종료 */
} SlotStepAction;


/* ============================================================
 * 진입 시작 좌우오차를 '기울여 들어가기'로 흡수 (2026-08-09i)
 *
 * 왜: 예전에는 남은 좌우오차를 slot_lateral_align() 이 처리했는데, 그건
 *   (1) 차선 쪽을 향해 90도 회전 -> (2) 오차만큼 전진 -> (3) 다시 슬롯쪽으로 90도 회전
 * 이라 90도 제자리 회전이 두 번 든다(약 5.6초). 사용자가 화면에서 본
 * "0도/180도로 열심히 돌았다가 다시 -90도로 돌아오는" 동작이 정확히 이것이다.
 *
 * 그런데 슬롯까지 들어가는 깊이가 대기점 0.6475 에서 깔때기 입구 0.44 까지만 해도
 * 200mm 나 된다. 그 거리를 b 도 기울여 들어가면 옆으로 200*sin(b) 만큼 옮겨간다.
 *   b=8도 -> 28mm,  b=6도 -> 21mm,  b=4도 -> 14mm
 * 즉 30mm 이내의 오차는 '기울여 들어가기'만으로 회전 없이 흡수된다.
 * 회전 두 번(5.6초)이 통째로 사라진다.
 *
 * 필요한 기울기: b = asin(오차 / 진입깊이). 진입깊이는 (시작 y - 깔때기 y)를
 * 직접 쓰지 않고 보수적으로 slot_bias_span_m*2.5(=200mm)로 잡는다. */
static void slot_drive__entry_bias(const SlotDriveCtx *ctx, SlotDriveState *st,
                                   double entry_lat_err) {
    const RobotConfig *cfg = ctx->cfg;
        double span = cfg->slot_bias_span_m * 2.5;
        if (span < 0.05) span = 0.05;
        double ratio = -entry_lat_err / span;      /* +x 로 가야 하면 ratio>0 */
        if (ratio >  0.99) ratio =  0.99;
        if (ratio < -0.99) ratio = -0.99;
        double need_deg = asin(ratio) * 180.0 / ANGLE_PI;
        double cap = cfg->slot_escape_bias_deg;
        if (cap <= 0.0) cap = 8.0;
        if (cap > 15.0) cap = 15.0;
        if (need_deg >  cap) need_deg =  cap;
        if (need_deg < -cap) need_deg = -cap;

        /* 슬롯 축이 -y(아래쪽)이므로, +x 로 옮기려면 시계반대(+)가 아니라
         * 시계방향(-)으로 기울여야 한다: heading -90도에서 각도를 -a 만큼 더 주면
         * 진행방향이 (sin a, -cos a) 라 +x 성분이 생긴다. */
        st->bias_rad = -need_deg * ANGLE_PI / 180.0;
        st->bias_from_y = 1e9;   /* 아래 첫 스텝에서 실제 y 로 갱신 */
        printf("[slot] 진입 시작 좌우오차 %+.0fmm -> 회전 없이 %+.1fdeg 기울여 "
               "들어가며 흡수합니다\n"
               "       (%.0fmm 들어가는 동안 옆으로 약 %.0fmm 이동)\n",
               entry_lat_err * 1000.0, st->bias_rad * 180.0 / ANGLE_PI,
               span * 1000.0, span * fabs(sin(st->bias_rad)) * 1000.0);
        fflush(stdout);
}


/* BUGFIX (2026-08-07): 진행방향 검사.
 * reached 판정이 forward일 때 (y <= stop_y + tol) 뿐이라, stop_y가 현재 y보다
 * "위"에 있으면(=진입 방향과 반대) step 0에서 조건이 곧바로 참이 되어 한 발도
 * 안 움직이고 "진입 완료"를 반환함. 실기에서 --point-b의 y를 잘못 준(0.1775를
 * 1.770으로) 경우 이 경로로 들어가서, 슬롯 근처에도 안 간 자리에서 주차 완료로
 * 보고되고 slot_seat()가 벽 쪽으로 밀어붙였음. 방향이 애초에 말이 안 되면
 * 여기서 실패로 끊는 게 맞음. */
static bool slot_drive__direction_ok(const SlotDriveCtx *ctx) {
    OccupancyGridSLAM *nav_slam = ctx->nav_slam;
    const bool forward = ctx->forward;
    const double stop_y = ctx->stop_y;
    const char *label = ctx->label;
    double cx0, cy0, cth0;
    slam_get_pose(nav_slam, &cx0, &cy0, &cth0);
    double margin = SLOT_STOP_TOL_M;
    if (forward && stop_y > cy0 - margin) {
        printf("[slot] %s 취소: 전진 진입인데 목표 y(%.3f)가 현재 y(%.3f)보다 위에 있음.\n"
               "       --point-%s 의 y값을 확인하세요 (슬롯은 -y 방향으로 들어갑니다).\n",
               label, stop_y, cy0, forward ? "a/b" : "a/b");
        fflush(stdout);
        return false;
    }
    if (!forward && stop_y < cy0 + margin) {
        printf("[slot] %s 취소: 후진 이탈인데 목표 y(%.3f)가 현재 y(%.3f)보다 아래에 있음.\n",
               label, stop_y, cy0);
        fflush(stdout);
        return false;
    }
    return true;
}


/* 한 스텝의 관측: 정지 -> 안정화 -> 새 스캔 -> 위치추정 -> 라이다 재정박.
 * 원본 루프 맨 앞 80줄이 그대로 들어 있다. */
static SlotStepAction slot_drive__observe(const SlotDriveCtx *ctx, SlotDriveState *st,
                                          SlotStep *sp) {
    FPGALink          *fpga     = ctx->fpga;
    LidarThread       *lidar    = ctx->lidar;
    OccupancyGridSLAM *nav_slam = ctx->nav_slam;
    const RobotConfig *cfg      = ctx->cfg;
    const double lane_x = ctx->lane_x, heading_rad = ctx->heading_rad;
    const double stop_y = ctx->stop_y;
    const bool   forward = ctx->forward;
    const char  *label = ctx->label;

    fpga_link_set_speed(fpga, 0, 0);
    robot_runner__sleep_sec(cfg->slot_settle_sec);
    if (g_stop_requested) return SLOT_STEP_FAIL;

    if (!robot_runner__take_scan(lidar, cfg, &sp->scan, 2.0)) return SLOT_STEP_RETRY;

    double odx, ody, odth;
    robot_runner__predict_delta(fpga, cfg, nav_slam->theta,
                                &st->prev_rx, &st->prev_ry, &st->prev_rth,
                                &odx, &ody, &odth);
    slam_localize_step(nav_slam, &sp->scan, odx, ody, odth, &sp->x, &sp->y, &sp->theta);
        /* ============================================================
         * 슬롯 안에서는 라이다 실측이 진실이다 (2026-08-11a)
         * robot_runner__slot_reanchor() 머리말 참고. 스캔매칭이 옆으로 미끄러진
         * 것을 여기서 바로 되돌리므로, 아래 모든 판정(좌우/각도/깊이/착석)이
         * 저절로 올바른 자세 위에서 돌아간다.
         * 슬롯 평행구간 밖(통로/대기점/깔때기)에서는 sfix.ok 가 false 라 아무 일도
         * 일어나지 않는다 - 즉 이 기능이 주행 구간을 건드릴 일은 없다.
         * ============================================================ */

        {
            double snap_dx = 0.0, snap_dy = 0.0;
            bool snapped = robot_runner__slot_reanchor(
                                nav_slam, &sp->scan, cfg, lane_x, heading_rad, stop_y,
                                forward ? "슬롯 진입 중" : "슬롯 이탈 중",
                                &sp->sfix, &snap_dx, &snap_dy);
            if (snapped) {
                slam_get_pose(nav_slam, &sp->x, &sp->y, &sp->theta);
                /* 깊이를 크게 옮겼으면 '얼마나 움직였나' 이력은 무효다 -
                 * 안 그러면 자세를 고친 것을 '차가 움직였다'로 착각해서
                 * 정체(=벽에 닿음) 판정이 엉뚱하게 풀린다. */
                if (fabs(snap_dy) > 0.020) { st->stall_count = 0; st->last_y = 1e9; }
                if (forward && sp->y < st->best_depth_y) st->best_depth_y = sp->y;
            }

            /* ============================================================
             * (2026-08-11c) 재정박이 내놓은 결론에 실제로 반응한다.
             *
             * 예전에는 두 신호 모두 slot_drive 안에서 아무도 안 읽었다.
             *   - slot_wrong_lane     : "여긴 명령한 슬롯이 아니다"
             *   - relocalize_request  : "라이다로도 결론을 못 냈다"
             * 그래서 '전역 재위치추정에 맡깁니다' 라고 찍어 놓고는 아무 일도
             * 일어나지 않았고(사용자 신고 2번 로그), 틀린 자세 그대로 주차까지
             * 마친 뒤 다음 출발에서 무너졌다.
             * ============================================================ */
            if (*robot_runner__slot_wrong_lane()) {
                *robot_runner__slot_wrong_lane() = false;
                printf("[slot] %s 중단: 여기는 명령한 차선(x=%.3f)의 슬롯이 아닙니다.\n"
                       "       자세는 바로잡았으니 호출부가 처음부터 다시 계획합니다\n",
                       label, lane_x);
                fflush(stdout);
                fpga_link_set_speed(fpga, 0, 0);
                return SLOT_STEP_FAIL;
            }
            if (nav_slam->relocalize_request > 0) {
                nav_slam->relocalize_request = 0;
                st->reloc_asked++;
                /* 매 스텝 1초씩 태우면 진입이 하염없이 늘어지므로, 연속으로
                 * 몇 번 요청이 쌓였을 때만 실제로 멈춰서 다시 잡는다. */
                if (st->reloc_asked >= 2) {
                    st->reloc_asked = 0;
                    printf("[slot] 라이다로도 슬롯을 특정하지 못했습니다 - "
                           "여기서 멈추고 지도 전체에서 위치를 다시 잡습니다\n");
                    fflush(stdout);
                    fpga_link_set_speed(fpga, 0, 0);
                    if (!robot_runner__relocalize_global(lidar, cfg, nav_slam,
                                                          forward ? "슬롯 진입 중 자세 불명"
                                                                  : "슬롯 이탈 중 자세 불명")) {
                        robot_runner__relocalize_wide(lidar, cfg, nav_slam,
                                                       "슬롯 구간 자세 불명");
                    }
                    slam_get_pose(nav_slam, &sp->x, &sp->y, &sp->theta);
                    robot_runner__init_prev_pose(fpga, cfg->has_encoder,
                                                  &st->prev_rx, &st->prev_ry, &st->prev_rth);
                    st->stall_count = 0; st->last_y = 1e9;
                    return SLOT_STEP_RETRY;   /* 새 자세로 다시 관측부터 */
                }
            } else {
                st->reloc_asked = 0;
            }
        }
    return SLOT_STEP_GO;
}


/* 얼마나 들어왔나 / 벽에 닿았나 / 주차 완료인가.
 * 원본의 '정지 판정' + '착석 판정' + '안전한계' 세 구간을 그대로 옮겼다. */
static void slot_drive__eval_seat(const SlotDriveCtx *ctx, SlotDriveState *st,
                                  const SlotStep *sp, SlotSeatEval *ev) {
    const RobotConfig *cfg = ctx->cfg;
    const double stop_y = ctx->stop_y, heading_rad = ctx->heading_rad;
    const bool forward = ctx->forward;
    memset(ev, 0, sizeof(*ev));
        /* 정지 판정.
         * PATCH (2026-08-06): 예전엔 목표 y에 정확히 도달하는지만 봤는데, 진입 방향은
         * 코가 안쪽 벽에 닿는 순간 물리적으로 멈추므로 목표값에 정확히 도달하지 못하고
         * 몇 mm 못 미친 채 끝남 - 시뮬레이션에서 좌우오차 16mm로 잘 들어가 놓고도
         * "스텝 한도 초과"로 실패 처리되는 것이 확인됨(y=0.182 도달, 목표 0.1775).
         * 그래서 (1) 목표에 여유를 두고, (2) 밀어도 더 이상 안 들어가면 '다 들어간
         * 것'으로 보는 정체(stall) 판정을 추가함 - 실제 로봇에서도 벽에 닿았는지는
         * 이렇게 판단하는 게 맞음. */
        if (forward && sp->y < st->best_depth_y) {
            if (st->best_depth_y - sp->y > FUNNEL_PROGRESS_M && st->funnel_tries > 0) {
                st->funnel_tries--;    /* 진전이 있었으면 시도 기회를 돌려줌 */
            }
            st->best_depth_y = sp->y;
        }
        if (fabs(sp->y - st->last_y) < SLOT_STALL_EPS_M) {
            st->stall_count++;
        } else {
            st->stall_count = 0;
        }
        st->last_y = sp->y;

        /* ============================================================
         * 착석(주차 완료) 판정 - 2026-08-09g 전면 개정
         *
         * 지적받은 문제: 2026-08-09f 에서 "실제 접촉만을 종료조건으로" 바꿨더니
         * 접촉 하나만으로 완료가 될 수 있었다. 차가 엉뚱한 데서 머리를 박아도
         * 그게 '주차 완료'로 보고될 수 있다는 뜻이라 위험하다. 맞는 지적이다.
         *
         * 이제 네 가지를 모두 만족해야 착석으로 인정한다 (AND 조건).
         *   (1) 깊이   : y <= stop_y + slot_seat_depth_tol_m (기본 30mm)
         *                = "목표 y 에 도달했다"
         *   (2) 접촉   : 밀고 있는데 SLOT_STALL_STEPS 연속으로 안 움직임
         *                = "벽에 닿았다"
         *   (3) 자세   : |각도오차| <= SLOT_SEAT_HERR_D
         *                비스듬히 낀 것과 정면으로 닿은 것을 구분한다
         *   (4) 라이다 : 정면이 '명백히 멀지 않을 것'
         *                착석 시 벽까지 거리는 반길이(127.5mm)에서 라이다 오프셋
         *                (60mm)을 뺀 67mm 라, 라이다 최소측정거리 아래여서 '안 보이는
         *                게 정상'이다. 그래서 "가까움"은 확인할 수 없지만 "멀다"는
         *                확실히 알 수 있다 - 정면이 뚜렷하게 열려 있으면 벽이 아니다.
         *                (음성 판정만 쓰므로 라이다 사양에 안 흔들린다)
         *
         * 지도의 stop_y 가 실제 벽보다 조금 안쪽일 수 있으므로, 30~60mm 모자란
         * 깊이에서 닿았을 때는 곧바로 실패로 몰지 않고 '재확인'한다. 한 번 복구
         * (물러나기+회전+재밀기)를 거친 뒤 같은 깊이에서 또 닿으면 그건 진짜 벽이다.
         *
         * 위 조건을 못 채운 정체는 전부 '낌'으로 보고 아래 복구 루틴으로 보낸다.
         * ============================================================ */

        double herr_seat_deg = normalize_angle(sp->theta - heading_rad) * 180.0 / ANGLE_PI;
        double front_min = robot_runner__sector_min((&sp->scan), 0.0, 25.0);
        bool front_says_far = (front_min < 9.0)
                              && (front_min > 0.5 * cfg->body_length + SLOT_SEAT_FRONT_PAD);
        ev->contact    = (st->stall_count >= SLOT_STALL_STEPS) && (sp->step > 3);
        ev->depth_ok   = (sp->y <= stop_y + cfg->slot_seat_depth_tol_m);
        /* ============================================================
         * 자세 조건 (2026-08-11a 개정) - 사용자 요청:
         *   "각도가 조금 틀어진 정도는 주차 진입 시엔 괜찮지 않나?
         *    주차슬롯 탈출 시에 알아서 나가면 되잖아."
         *
         * 맞는 말이고, 예전에 이걸 못 한 이유는 '각도를 몰라서'였다. 실기 로그의
         *     step=9~20 각도오차 +14~17deg (스캔매칭)
         *     같은 시각 앞왼쪽 0.096m / 앞오른쪽 0.105m (라이다 실측)
         * 두 값은 물리적으로 양립하지 않는다 - 210mm 슬롯에서 15도 틀어져 있으면
         * 좌우 차이가 9mm 로 끝날 수 없다. 즉 15도는 스캔매칭이 만들어낸 유령이었고,
         * 그 유령을 지우려고 다 들어간 차를 20번 넘게 회전시키다 결국 빼냈다.
         *
         * 이제 좌우 벽이 둘 다 보이면 각도는 라이다 실측(오차 1도 이내, 합성스캔
         * 검증)을 쓰고, 그때는 허용치를 slot_seat_yaw_deg(기본 12도)로 연다.
         * 12도면 필요한 가로폭이 130*cos12+255*sin12 = 180mm 라 210mm 슬롯에
         * 30mm 여유가 남는다. 남은 각도는 탈출할 때 통로에서 풀면 된다.
         * 라이다가 확인해 주지 못하면 예전 그대로 6도로 엄격하게 본다.
         * ============================================================ */
        bool   lidar_posture  = (sp->sfix.ok && sp->sfix.yaw_ok);
        double herr_true_deg  = lidar_posture ? (sp->sfix.yaw_err_rad * 180.0 / ANGLE_PI)
                                              : herr_seat_deg;
        /* 재진입 기울임이 아직 걸려 있으면 그만큼은 '의도된 각도'이므로 빼고 본다.
         * (보통은 위에서 목표 근처에 오면 기울임이 먼저 풀린다) */
        double posture_lim = (lidar_posture ? cfg->slot_seat_yaw_deg : SLOT_SEAT_HERR_D)
                             + fabs(st->bias_rad) * 180.0 / ANGLE_PI;
        bool posture_ok = (fabs(herr_true_deg) <= posture_lim);

        ev->seated = false;
        if (forward && ev->contact && posture_ok && !front_says_far) {
            if (ev->depth_ok) {
                ev->seated = true;
                if (lidar_posture && fabs(herr_true_deg) > SLOT_SEAT_HERR_D) {
                    printf("[slot] 착석 인정: 각도오차 %+.1fdeg 는 예전 기준(%.1f)을 넘지만,\n"
                           "       라이다 실측으로 좌벽 %.3fm / 우벽 %.3fm (치우침 %+.0fmm) 이라\n"
                           "       슬롯 안 정위치입니다. 남은 각도는 나갈 때 통로에서 풉니다\n"
                           "       (허용 %.1fdeg = --slot-seat-yaw)\n",
                           herr_true_deg, SLOT_SEAT_HERR_D,
                           sp->sfix.d_left, sp->sfix.d_right, sp->sfix.lat_left_m * 1000.0,
                           cfg->slot_seat_yaw_deg);
                    fflush(stdout);
                }
            } else if (sp->y <= stop_y + SLOT_SEAT_CONFIRM_M) {
                /* 목표보다 30~60mm 모자란데 벽처럼 닿았다 - 지도가 조금 틀렸을 수도,
                 * 깔때기 턱에 얹혔을 수도 있다. 같은 깊이에서 두 번 확인되면 인정. */
                if (st->seat_confirm_n > 0 && fabs(sp->y - st->seat_confirm_y) < 0.010) {
                    printf("[slot] 목표보다 %.0fmm 모자란 깊이에서 두 번 연속 벽에 닿았습니다.\n"
                           "       지도상 목표 y=%.3f 보다 벽이 앞에 있는 것으로 보고 "
                           "착석으로 인정합니다\n"
                           "       (각도오차 %+.1fdeg, 정면 %s)\n",
                           (sp->y - stop_y) * 1000.0, stop_y, herr_true_deg,
                           (front_min < 9.0) ? "가까움" : "측정 안 됨(=매우 가까움)");
                    fflush(stdout);
                    ev->seated = true;
                } else {
                    st->seat_confirm_y = sp->y;
                    st->seat_confirm_n++;
                    printf("[slot] 벽 같은 접촉이지만 목표보다 %.0fmm 모자랍니다 - "
                           "한 번 풀고 다시 확인합니다 (%d회차)\n",
                           (sp->y - stop_y) * 1000.0, st->seat_confirm_n);
                    fflush(stdout);
                }
            }
        }

        /* 안전한계: 접촉이 안 잡히는데 목표보다 slot_press_over_m 이상 더 들어감.
         * 지도/자세추정이 어긋났다는 신호이므로 더 밀지 않고 멈춘다. */
        ev->over_limit = forward && cfg->slot_press_to_wall
                          && (sp->y <= stop_y - cfg->slot_press_over_m);
        if (ev->over_limit) {
            printf("[slot] [경고] 목표 y=%.3f 보다 %.0fmm 더 들어왔는데 벽에 닿지 않았습니다.\n"
                   "       지도나 자세추정이 어긋났을 수 있어 더 밀지 않고 멈춥니다\n",
                   stop_y, (stop_y - sp->y) * 1000.0);
            fflush(stdout);
            /* PATCH (2026-08-10): 이 사실을 호출부에 전달한다.
             * 예전에는 경고만 찍고 reached=true 로 넘어가서 그대로 '주차 완료'가 됐다.
             * 실기 로그가 정확히 그랬다 - 로봇은 통로 구석에 있었는데
             *     [slot] [경고] ... 벽에 닿지 않았습니다
             *     [shuttle] A지점 주차 완료: ... 오차 45.8mm
             * 가 연달아 찍혔다. 슬롯에 제대로 들어갔다면 안쪽 벽에 반드시 닿는다.
             * 안 닿았다는 건 '슬롯 안이 아니다'라는 물리적 반증이므로, 성공으로
             * 보고하면 안 되고 자세추정을 다시 잡아야 한다.
             *
             * 단 (2026-08-11a): 라이다가 좌우 벽을 둘 다 보고 있고 그 폭이 슬롯 폭과
             * 맞으면, '슬롯 안이 아니다'라는 결론 자체가 틀렸다. 그때는 깊이만 덜
             * 들어간 것이므로 전역 재위치추정을 부르지 않는다 - 부르면 애써 맞춰 놓은
             * 자세를 똑같이 생긴 반대쪽 슬롯으로 날려버릴 위험만 생긴다. */
            if (sp->sfix.ok) {
                printf("       (라이다로는 좌벽 %.3fm / 우벽 %.3fm = 폭 %.3fm 이라\n"
                       "        슬롯 안인 것은 확실합니다. 자세를 다시 잡지 않습니다)\n",
                       sp->sfix.d_left, sp->sfix.d_right, sp->sfix.width_m);
                fflush(stdout);
            } else {
                *robot_runner__slot_no_wall(cfg) = true;
            }
        }

        ev->reached = forward ? (ev->seated || ev->over_limit)
                               : (sp->y >= stop_y - SLOT_STOP_TOL_M);
        ev->stalled = forward ? ev->seated
                               : ((st->stall_count >= SLOT_STALL_STEPS) && (sp->step > 3));
        ev->near_depth = forward ? (ev->seated || ev->depth_ok) : true;
}


/* 재진입 기울임을 유지할지 줄일지 지울지. */
static void slot_drive__update_bias(const SlotDriveCtx *ctx, SlotDriveState *st,
                                    const SlotStep *sp) {
    const RobotConfig *cfg = ctx->cfg;
    const double stop_y = ctx->stop_y;
    const bool forward = ctx->forward;
        /* ============================================================
         * 깔때기/슬롯 안 각도 복구 (2026-08-09e 전면 개정)
         *
         * 예전 동작과 그 한계(실기 로그로 확인):
         *   - 걸린 걸 감지한 뒤에야 개입했고, 회전량 상한이 6도였다.
         *   - 6도를 명령해도 실제로는 3도쯤 돌고, 이어지는 강한 밀기가 경사벽에
         *     부딪혀 1.5도를 되돌려 놓아 사이클당 순증이 1.5도뿐이었다.
         *   - 시도 상한 5회 = 최대 7.5도. 실제 오차는 28~30도였으므로
         *     원리적으로 절대 수렴할 수 없었다(사용자 신고 증상 그대로).
         *
         * 개정 내용 두 가지:
         *  (1) '걸린 뒤'가 아니라 '틀어지기 시작할 때' 개입한다.
         *      각도오차가 SLOT_HERR_GUARD_DEG 를 넘으면 그 자리에서 밀기를 멈춘다.
         *      계속 밀면 경사벽이 각도를 더 밀어올릴 뿐이라는 것이 로그로 확인됐다
         *      (-0.5도 -> +21.5도 -> +30도).
         *  (2) 복구는 slot_fix_heading() 이 한다 - 각도에 비례해 물러나 접촉을 풀고,
         *      닫힌 루프로 목표 각도까지 크게 돈다(한 스텝 최대 20도).
         *      각도가 이미 맞는데 걸린 경우(순수 좌우 끼임)에는 라이다로 막힌 쪽을
         *      보고 일부러 반대로 조금 기울여서 흘려 넣는다.
         * ============================================================ */
        /* 지금 적용 중인 목표 각도 = 슬롯 축 + (재진입 기울임).
         * 기울임은 slot_bias_span_m 만큼 더 들어가는 동안만 유지하고, 그 뒤에는
         * 0으로 돌려서 평행구간에서는 반드시 나란해지게 한다 (2026-08-09f). */
        if (st->bias_rad != 0.0 && forward && !st->bias_is_residual && st->bias_from_y > 1e8)
            st->bias_from_y = sp->y;
        if (st->bias_rad != 0.0 && forward && !st->bias_is_residual) {
            double gone = st->bias_from_y - sp->y;                 /* 기울임 건 뒤 들어간 깊이 */
            /* 유지 깊이는 '이번 기울임에 정해진 값'을 쓴다 (2026-08-10n).
             * 반복 끼임에서 각도를 키울 때 유지 깊이도 같이 키우기 때문이다 -
             * 옆으로 옮겨가는 양이 span * sin(각도) 라 둘 다 늘려야 효과가 난다. */
            double hold_span = (st->bias_span_m > 0.0) ? st->bias_span_m : cfg->slot_bias_span_m;
            /* 목표 깊이가 가까우면 남은 거리와 무관하게 즉시 축으로 되돌린다
             * (2026-08-09g): 마지막 구간은 반드시 슬롯 축과 나란해야 벽에 정면으로
             * 닿고, 착석 판정의 자세 조건(6도)도 통과한다. */
            bool near_target = (sp->y <= stop_y + cfg->slot_bias_span_m);
            if (gone >= hold_span || near_target) {
                /* PATCH (2026-08-10g): 축으로 '완전히' 되돌리지 않고, 걸렸던 벽의
                 * 반대쪽으로 잔여 기울임을 남긴다 (사용자 요청).
                 * 다만 마지막 착석 구간(near_target)에서는 반드시 나란해야
                 * 안쪽 벽에 정면으로 닿고 착석 자세 조건도 통과하므로 0으로 되돌린다. */
                double resid = cfg->slot_residual_bias_deg;
                if (resid < 0.0) resid = 0.0;
                if (resid > 5.0) resid = 5.0;      /* 임계각 20도 훨씬 아래로 묶음 */
                double keep = 0.0;
                if (!near_target && resid > 0.0)
                    keep = (st->bias_rad > 0.0 ? +1.0 : -1.0) * resid * ANGLE_PI / 180.0;
                if (keep != 0.0) {
                    printf("[slot] 반대편 기울임 축소: %+.1fdeg -> %+.1fdeg 로 줄이되 "
                           "0 으로 되돌리지 않습니다\n"
                           "       (걸렸던 쪽 반대로 %.1fdeg 를 남겨야 같은 벽을 또 안 긁습니다"
                           " - 남은 깊이 200mm 기준 옆으로 %.0fmm 더 벌어짐)\n",
                           st->bias_rad * 180.0 / ANGLE_PI, keep * 180.0 / ANGLE_PI,
                           resid, 200.0 * sin(resid * ANGLE_PI / 180.0));
                } else {
                    printf("[slot] 반대편 기울임 종료(%s): 슬롯 축으로 되돌립니다\n",
                           near_target ? "목표 깊이 근접" : "충분히 들어옴");
                }
                fflush(stdout);
                st->bias_rad = keep;
                st->bias_is_residual = (keep != 0.0);   /* 잔여분은 끝까지 유지 */
            }
        }
        /* 잔여 기울임도 마지막 착석 구간에서는 반드시 0 으로 - 안쪽 벽에 정면으로
         * 닿아야 착석으로 인정되기 때문 (2026-08-10g) */
        if (st->bias_is_residual && forward && sp->y <= stop_y + cfg->slot_bias_span_m) {
            printf("[slot] 잔여 기울임 %+.1fdeg 해제: 착석 구간이라 슬롯 축과 나란하게 합니다\n",
                   st->bias_rad * 180.0 / ANGLE_PI);
            fflush(stdout);
            st->bias_rad = 0.0;
            st->bias_is_residual = false;
        }
}


/* 후진(슬롯 이탈) 중 걸림/각도 복구. 호출부에서 back_stall||back_angle 일 때만 부른다. */
static SlotStepAction slot_drive__recover_back(const SlotDriveCtx *ctx, SlotDriveState *st,
                                               const SlotStep *sp, bool back_stall,
                                               double herr_now_deg) {
    FPGALink          *fpga     = ctx->fpga;
    LidarThread       *lidar    = ctx->lidar;
    OccupancyGridSLAM *nav_slam = ctx->nav_slam;
    const RobotConfig *cfg      = ctx->cfg;
    const double heading_rad = ctx->heading_rad, stop_y = ctx->stop_y;
    if (st->funnel_tries < FUNNEL_MAX_TRIES) {
        st->funnel_tries++;
        printf("[slot] 후진 중 %s (y=%.3f, 목표 %.3f, 각도 %+.1fdeg)\n"
               "       -> 빼내기를 멈추고 각도를 슬롯 축으로 바로잡습니다 %d/%d\n",
               back_stall ? "걸림" : "각도가 틀어짐",
               sp->y, stop_y, herr_now_deg, st->funnel_tries, FUNNEL_MAX_TRIES);
        fflush(stdout);

        double herr_after = herr_now_deg;
        /* 접촉은 '앞으로' 밀어서 푼다(+1). 한계는 슬롯 바닥 근처까지만. */
        bool fixed = slot_fix_heading(fpga, lidar, nav_slam, cfg,
                                      heading_rad, 2.0, back_stall,
                                      cfg->slot_staging_y - 0.45, +1,
                                      &st->prev_rx, &st->prev_ry, &st->prev_rth, &herr_after);
        if (!fixed && fabs(herr_after) > SLOT_HERR_ABORT_DEG) {
            printf("[slot] 후진 중단: 각도오차 %+.1fdeg 를 못 잡았습니다 "
                   "(y=%.3f, 목표 %.3f)\n", herr_after, sp->y, stop_y);
            fflush(stdout);
            fpga_link_set_speed(fpga, 0, 0);
            return SLOT_STEP_FAIL;
        }
        st->funnel_kick = true;      /* 복구 직후 첫 빼내기는 곧게 + 세게 */
        st->stall_count = 0;
        st->last_y = 1e9;
        return SLOT_STEP_RETRY;
    }
    printf("[slot] 후진 중단: %d번 각도를 고쳐 빼봤지만 더 안 나옵니다 "
           "(y=%.3f, 목표 %.3f, 각도 %+.1fdeg)\n",
           FUNNEL_MAX_TRIES, sp->y, stop_y, herr_now_deg);
    fflush(stdout);
    fpga_link_set_speed(fpga, 0, 0);
    return SLOT_STEP_FAIL;
}


/* 진입 중 깔때기 걸림 / 각도 틀어짐 복구.
 * 원본에서 가장 큰 덩어리(235줄)였다. 호출부에서 jam_stall||jam_angle 일 때만 부른다.
 * '마지막으로 곧게 한 번 더' 경로에서 조향 목표(aim_rad)와 걸림 플래그를 지우므로
 * 그 셋을 포인터로 받는다. */
static SlotStepAction slot_drive__recover_jam(const SlotDriveCtx *ctx, SlotDriveState *st,
                                              const SlotStep *sp, double herr_now_deg,
                                              double *aim_rad,
                                              bool *jam_stall, bool *jam_angle) {
    FPGALink          *fpga     = ctx->fpga;
    LidarThread       *lidar    = ctx->lidar;
    OccupancyGridSLAM *nav_slam = ctx->nav_slam;
    const RobotConfig *cfg      = ctx->cfg;
    const double lane_x = ctx->lane_x, heading_rad = ctx->heading_rad;
    const double stop_y = ctx->stop_y;
    const bool forward = ctx->forward;
        double lat_mm = (sp->x - lane_x) * 1000.0;

        if (st->funnel_tries < FUNNEL_MAX_TRIES) {
            st->funnel_tries++;
            printf("[slot] %s (y=%.3f, 목표 %.3f, 좌우 %+.0fmm, 각도 %+.1fdeg)\n"
                   "       -> 밀기를 멈추고 각도를 바로잡습니다 %d/%d\n",
                   *jam_stall ? "깔때기 걸림" : "각도가 틀어짐",
                   sp->y, stop_y, lat_mm, herr_now_deg, st->funnel_tries, FUNNEL_MAX_TRIES);
            fflush(stdout);

            /* 복구 목표 각도 결정.
             *  - 각도오차가 의미 있으면(SLOT_HERR_NUDGE_DEG 초과) 그냥 슬롯 축으로
             *    되돌린다. 이때는 라이다 좌우 판정을 쓰지 않는다 - 물린 상태의
             *    좌우 판정은 부호가 자주 뒤집히는데, 각도오차의 부호는 뒤집히지
             *    않으므로 훨씬 신뢰할 수 있다.
             *  - 각도는 맞는데 걸렸으면 '어느 쪽 코가 닿았는지'를 라이다로 보고,
             *    막힌 쪽 반대로 살짝 기울인 각도를 목표로 준다. */
            /* ============================================================
             * 복구 목표 각도 = 슬롯 축 + '걸린 벽 반대쪽' 기울임 (2026-08-09f)
             *
             * 예전(2026-08-09e)에는 각도오차가 크면 그냥 슬롯 축(-90도)으로만
             * 되돌렸다. 그런데 벽에 걸렸다는 건 그 벽 쪽으로 치우쳐 있다는 뜻이고,
             * 차동구동은 옆으로 못 가므로 축으로 나란히 만들어 밀면 '치우친 채
             * 나란히' 다시 같은 벽을 긁는다(사용자 관찰: 같은 벽에 2번은 더 걸림).
             *
             * 좌우 위치를 바꾸는 유일한 수단은 비스듬히 전진하는 것이다.
             * 반대쪽으로 b 도 기울여 d 만큼 들어가면 옆으로 d*sin(b) 옮겨간다.
             * b=8도, d=80mm -> 11mm. 편측 여유 40mm 기준으로 충분한 양이고,
             * yaw 8도에서 필요한 가로폭은 164mm 라 210mm 슬롯에 여유롭게 들어간다.
             *
             * 어느 쪽이 '반대쪽'인가는 이 순서로 정한다.
             *   1) 라이다 앞쪽 좌/우 여유 비교 - 물린 상태에서도 신뢰할 수 있는 관측
             *   2) 그게 애매하면 슬롯 좌우 벽까지 거리로 잰 치우침
             *   3) 그것도 없으면 SLAM 차선오차
             * ============================================================ */
            double fl = 0.0, fr = 0.0;
            int blocked = robot_runner__blocked_side((&sp->scan), &fl, &fr);
            double lat_scan = 0.0, lat_body = 0.0;
            bool have_lat = robot_runner__slot_lateral_from_scan((&sp->scan), &lat_scan);
            if (have_lat) lat_body = lat_scan;
            else have_lat = robot_runner__lane_offset_body(sp->x, lane_x, heading_rad, &lat_body);

            int free_dir;          /* +1 = 왼쪽이 비었음, -1 = 오른쪽이 비었음 */
            const char *why;
            if (blocked != 0) {
                free_dir = (blocked > 0) ? -1 : +1;
                why = (blocked > 0) ? "라이다: 왼쪽 코가 벽에 닿음"
                                    : "라이다: 오른쪽 코가 벽에 닿음";
            } else if (have_lat && fabs(lat_body) > 0.004) {
                /* lat_body > 0 = 차체가 왼쪽으로 치우침 -> 오른쪽이 비었음 */
                free_dir = (lat_body > 0.0) ? -1 : +1;
                why = "좌우 치우침 기준";
            } else {
                /* 마지막 수단: 지금 각도가 향한 쪽이 곧 걸린 쪽일 가능성이 큼 */
                free_dir = (herr_now_deg > 0.0) ? -1 : +1;
                why = "각도오차 기준";
            }

            /* ============================================================
             * 같은 벽에 다시 걸리면 기울임을 키운다 (2026-08-10n)
             *
             * 사용자 신고: "한 번 끼면 그 뒤로 2번 정도는 더 끼고 각도 조정을
             *              반복한다. 반대로 회전하는 각도를 더 크게 해달라."
             *
             * 왜 반복되나. 기울임 b 로 깊이 d 를 들어가면 옆으로 d*sin(b) 옮겨간다.
             * 기본값 b=8도, 유지 깊이 slot_bias_span_m=80mm 이므로
             *     80 * sin(8도) = 11mm
             * 밖에 못 옮긴다. 슬롯 폭 210mm / 차폭 130mm 이라 편측 여유가 40mm 인데,
             * 걸릴 정도의 치우침은 보통 20~30mm 다. 즉 한 번의 복구로는 원인이
             * 절반도 안 지워지고, 같은 벽에 다시 걸리는 게 당연하다.
             *
             * 그래서 복구 횟수(funnel_tries)에 따라 기울임과 유지 깊이를 같이 키운다.
             *     1회차 : 8.0도 x  80mm -> 11mm
             *     2회차 : 12.0도 x 110mm -> 23mm
             *     3회차 : 15.0도 x 140mm -> 36mm   (편측 여유 40mm 를 거의 다 씀)
             * 상한 15도는 그대로 지킨다. 차체를 b 도 비틀어 슬롯에 넣으려면
             * 가로폭 255*sin(b) + 130*cos(b) 가 210mm 이하여야 하고, 그 해가
             * b <= 20.2도다. 15도면 가로폭 191mm 로 19mm 여유가 남는다.
             * ============================================================ */
            double bias_deg = cfg->slot_escape_bias_deg;
            double bias_span = cfg->slot_bias_span_m;
            if (st->funnel_tries >= 2) {
                double grow = 1.0 + 0.5 * (double)(st->funnel_tries - 1);   /* 1.5, 2.0, ... */
                bias_deg  *= grow;
                bias_span *= (1.0 + 0.375 * (double)(st->funnel_tries - 1));
                if (bias_span > 0.16) bias_span = 0.16;
            }
            if (bias_deg < 0.0) bias_deg = 0.0;
            if (bias_deg > 15.0) bias_deg = 15.0;   /* 임계각 20.2도 아래로 묶음 */

            /* ============================================================
             * BUGFIX (2026-08-10h): 착석 구간에서는 기울임을 걸지 않는다.
             *
             * 아래쪽 '기울임 해제' 규칙이 y <= stop_y + slot_bias_span_m 에서는
             * 어떤 기울임이든 다음 스텝에 0 으로 지운다. 그런데 여기서 8도를
             * 걸어버리면, 회전을 마치자마자 그 8도가 지워지면서 '8.9도짜리
             * 각도오차'가 생기고 가드(8도)를 다시 넘겨 복구가 또 돈다.
             * pose 가 한 발도 안 움직인 채 5번을 다 태우는 무한루프가 이것이다.
             *
             * 게다가 착석 구간에서 기울임은 원래도 쓸모가 없다. 남은 깊이가
             * 80mm 이하라 기울여서 얻는 좌우 이동은 80*sin(8도)=11mm 뿐인데,
             * 그 각도로는 안쪽 벽에 비스듬히 닿아 착석 판정(자세 6도 이내)을
             * 못 통과한다. 마지막 구간은 축과 나란한 것이 언제나 정답이다.
             * ============================================================ */
            bool seat_zone = forward && (sp->y <= stop_y + cfg->slot_bias_span_m);
            if (seat_zone) bias_deg = 0.0;

            st->bias_rad = free_dir * bias_deg * ANGLE_PI / 180.0;
            st->bias_span_m = seat_zone ? cfg->slot_bias_span_m : bias_span;
            st->bias_from_y = sp->y;
            /* 새로 건 기울임은 '잔여분'이 아니다 - 이 플래그를 안 지우면
             * 위쪽 축소 로직(gone >= span)이 통째로 건너뛰어져서, 8도짜리
             * 기울임이 착석 구간까지 그대로 끌려가 한꺼번에 터진다
             * (실기 로그의 "잔여 기울임 +8.0deg 해제"가 이 경로였다). */
            st->bias_is_residual = false;

            double target_head = heading_rad + st->bias_rad;
            double fix_tol = 2.0;

            if (seat_zone) {
                printf("[slot]    앞왼쪽 %.3fm / 앞오른쪽 %.3fm, 치우침 %+.0fmm(%s)\n"
                       "          -> 목표 깊이까지 %.0fmm 뿐이라 기울이지 않고 "
                       "슬롯 축(%.1fdeg) 그대로 맞춘 뒤 곧게 밀어 넣습니다\n",
                       fl, fr, lat_body * 1000.0, have_lat ? "실측" : "추정",
                       (sp->y - stop_y) * 1000.0, heading_rad * 180.0 / ANGLE_PI);
            } else {
                printf("[slot]    앞왼쪽 %.3fm / 앞오른쪽 %.3fm, 치우침 %+.0fmm(%s)\n"
                       "          -> %s. 슬롯 축(%.1fdeg)이 아니라 빈 쪽으로 %+.1fdeg "
                       "더 튼 %.1fdeg 를 목표로 잡습니다\n"
                       "          (%.0fmm 들어가는 동안 유지 -> 옆으로 약 %.0fmm 옮겨감)\n",
                       fl, fr, lat_body * 1000.0, have_lat ? "실측" : "추정", why,
                       heading_rad * 180.0 / ANGLE_PI, free_dir * bias_deg,
                       target_head * 180.0 / ANGLE_PI,
                       bias_span * 1000.0,
                       bias_span * sin(bias_deg * ANGLE_PI / 180.0) * 1000.0);
            }
            fflush(stdout);

            double herr_after = herr_now_deg;
            bool fixed = slot_fix_heading(fpga, lidar, nav_slam, cfg,
                                          target_head, fix_tol, *jam_stall,
                                          cfg->slot_staging_y - 0.03, -1,
                                          &st->prev_rx, &st->prev_ry, &st->prev_rth, &herr_after);

            /* (2026-08-11a) 각도를 못 잡았더라도, 라이다가 '슬롯 평행구간 안 +
             * 좌우 중앙 + 목표 깊이 근처' 를 확인해 주면 빼내지 않는다.
             * 다 들어간 차를 도로 빼는 것이 사용자가 지적한 최악의 동작이고,
             * 남은 각도는 어차피 나갈 때 통로에서 푸는 편이 안전하다. */
            if (!fixed && fabs(herr_after) > SLOT_HERR_ABORT_DEG
                && forward && sp->sfix.ok && fabs(sp->sfix.lat_left_m) <= 0.030
                && sp->y <= stop_y + SLOT_SEAT_CONFIRM_M) {
                printf("[slot] 각도가 %+.1fdeg 남았지만 라이다로는 정위치입니다 "
                       "(좌벽 %.3f / 우벽 %.3f, 치우침 %+.0fmm, 목표까지 %.0fmm).\n"
                       "       빼내지 않고 곧게 밀어 마무리합니다\n",
                       herr_after, sp->sfix.d_left, sp->sfix.d_right,
                       sp->sfix.lat_left_m * 1000.0, (sp->y - stop_y) * 1000.0);
                fflush(stdout);
                st->bias_rad = 0.0;
                st->bias_is_residual = false;
                st->funnel_tries = 0;
                st->push_grace = 6;
                st->funnel_kick = true;
                st->stall_count = 0;
                st->last_y = 1e9;
                return SLOT_STEP_RETRY;
            }
            if (!fixed && fabs(herr_after) > SLOT_HERR_ABORT_DEG) {
                /* 물러나면서 돌려봤는데도 각도가 안 잡힌다 = 여기서는 못 푼다.
                 * 이 상태로 더 밀면 각도만 더 나빠지므로(로그로 확인), 실패를
                 * 돌려주고 호출부가 '대기점까지 완전 후진 -> 재정렬 -> 재접근'
                 * 을 하게 한다. 예전엔 이 판단이 없어서 120스텝을 계속 박았다. */
                printf("[slot] 진입 중단: 각도오차 %+.1fdeg 를 슬롯 안에서 못 잡았습니다.\n"
                       "       (y=%.3f, 목표 %.3f, 가장 깊이 %.3f) 밖으로 빼서 다시 접근합니다.\n",
                       herr_after, sp->y, stop_y, st->best_depth_y);
                fflush(stdout);
                fpga_link_set_speed(fpga, 0, 0);
                return SLOT_STEP_FAIL;
            }

            st->funnel_kick = true;      /* 복구 직후 첫 밀기는 곧게 + 세게 */
            st->push_grace = 2;          /* 최소 두 번은 실제로 밀어보고 다시 판정 */
            st->stall_count = 0;
            st->last_y = 1e9;
            return SLOT_STEP_RETRY;   /* 다시 관측하고 밀기 */
        }

        /* ============================================================
         * 포기하기 전에 '곧게 한 번 더' (2026-08-10h) - 사용자 요청:
         *   "똑바로 들어가 놓고 완전히 후진했다가 다시 들어오는 게 최악이다.
         *    스캔매칭 맞으면 쑥 들어가서 머리만 박으면 된다."
         *
         * 시도횟수를 다 썼더라도, 이미 슬롯 축과 나란하고(6도 이내) 목표
         * 깊이 근처(60mm 이내)라면 밖으로 빼서 다시 접근할 이유가 없다.
         * 그 상태에서 필요한 건 회전이 아니라 '앞으로 미는 것'뿐이다.
         * 여기서 기울임을 전부 지우고 곧게 밀기로 넘긴다 - 벽에 닿으면
         * 착석 판정이 받아주고, 못 닿으면 아래 stall/over_limit 이 잡는다.
         * ============================================================ */
        double axis_err_deg = normalize_angle(sp->theta - heading_rad) * 180.0 / ANGLE_PI;
        /* (2026-08-11a) 라이다가 좌우 중앙을 확인해 주면 각도 조건은 빼고 본다.
         * 좌우가 중앙이고 깊이가 코앞이면 필요한 건 회전이 아니라 미는 것뿐이다. */
        bool lidar_says_ok = (sp->sfix.ok && fabs(sp->sfix.lat_left_m) <= 0.030);
        bool last_ditch = (forward && !st->last_ditch_done
                           && (fabs(axis_err_deg) <= SLOT_HERR_ABORT_DEG || lidar_says_ok)
                           && sp->y <= stop_y + SLOT_SEAT_CONFIRM_M);
        if (last_ditch) {
            st->last_ditch_done = true;
            printf("[slot] 시도횟수는 다 썼지만 슬롯 축과 나란하고(%+.1fdeg) "
                   "목표까지 %.0fmm 뿐입니다.\n"
                   "       빼내지 않고 곧게 밀어 넣어 마무리합니다 "
                   "(다 들어간 채로 후진하지 않기)\n",
                   axis_err_deg, (sp->y - stop_y) * 1000.0);
            fflush(stdout);
            st->bias_rad = 0.0;
            st->bias_is_residual = false;
            *aim_rad = heading_rad;
            st->funnel_tries = 0;
            st->push_grace = 6;
            st->funnel_kick = true;
            st->stall_count = 0;
            st->last_y = 1e9;
            *jam_stall = false;
            *jam_angle = false;
        } else {
            printf("[slot] 진입 중단: y=%.3f (목표 %.3f) 에서 %d번 각도를 고쳐 밀었지만\n"
                   "       더 들어가지 않습니다. 가장 깊이 들어간 지점 y=%.3f "
                   "(깊이 %.0fmm 부족)\n"
                   "       좌우오차 %+.0fmm, 각도오차 %+.1fdeg\n",
                   sp->y, stop_y, FUNNEL_MAX_TRIES, st->best_depth_y,
                   (st->best_depth_y - stop_y) * 1000.0, lat_mm, herr_now_deg);
            fflush(stdout);
            fpga_link_set_speed(fpga, 0, 0);
            return SLOT_STEP_FAIL;
        }
    return SLOT_STEP_GO;
}


/* 목표에 도달했거나 더 못 가면 여기서 끝낸다.
 * 호출부에서 (reached || stalled) 일 때만 부른다. */
static SlotStepAction slot_drive__finish(const SlotDriveCtx *ctx, const SlotStep *sp,
                                         const SlotSeatEval *ev, bool straight_out,
                                         bool *out_touched) {
    FPGALink *fpga = ctx->fpga;
    const double stop_y = ctx->stop_y;
    const bool forward = ctx->forward;
    const char *label = ctx->label;
    /* PATCH (2026-08-08): 후진(슬롯 이탈)에서도 정체하면 멈춤.
     * 예전엔 done 조건에 (forward && stalled) 만 있어서, 후진 중에 차 뒤끝이
     * 벽에 닿아 더 못 가는데도 120스텝을 계속 밀어붙였음.
     * 실기 로그: pose가 (0.180,0.635)에 붙박이인 채 cmd=(-16,-28)을 반복.
     * 후진은 "슬롯 밖으로 나오기"가 목적이라, 닿아서 멈췄어도 목표 y 근처까지
     * 왔으면 성공으로 인정하고 넘어가는 게 맞음. */
        if (ev->stalled && !ev->reached && forward) {
            printf("[slot] 더 이상 들어가지 않음 - 안쪽 벽에 닿은 것으로 판단\n");
            if (out_touched) *out_touched = true;   /* 이미 닿았으면 추가 밀착 불필요 */
        } else if (ev->stalled && !ev->reached) {
            /* 후진 중 정체: 차 뒤끝이 벽에 닿았을 가능성이 큼 */
            double short_by = fabs(stop_y - sp->y);
            const double BACKOUT_ACCEPT_M = 0.05;
            if (short_by <= BACKOUT_ACCEPT_M) {
                printf("[slot] 후진 중 정체 - 목표까지 %.0fmm 남았지만 슬롯은 벗어났으므로 "
                       "진행합니다 (차 뒤끝이 벽에 닿은 듯)\n", short_by * 1000.0);
            } else if (straight_out) {
                /* 깔때기에서 빼내는 중이면 목표까지 못 갔어도 일단 진행.
                 * 여기서 실패로 끊으면 재시도 자체가 무의미해짐(실기에서 확인). */
                printf("[slot] 후진 중 정체 - 목표까지 %.0fmm 남았지만 빼내기는 계속합니다\n",
                       short_by * 1000.0);
            } else {
                printf("[slot] 후진 중 정체 - 목표까지 %.0fmm이나 남았습니다. "
                       "대기점 y(--slot-staging)가 너무 높거나 차가 끼었습니다.\n",
                       short_by * 1000.0);
                fflush(stdout);
                fpga_link_set_speed(fpga, 0, 0);
                return SLOT_STEP_FAIL;
            }
        }
        printf("[slot] %s 완료: pose=(%.3f, %.3f, %.1fdeg)\n",
               label, sp->x, sp->y, sp->theta * 180.0 / ANGLE_PI);
        fflush(stdout);
        fpga_link_set_speed(fpga, 0, 0);
        return SLOT_STEP_DONE;
}


/* 조향/속도/밀기 시간을 정하고 실제로 한 번 민다.
 * 원본 루프의 마지막 213줄 그대로다. */
static void slot_drive__push(const SlotDriveCtx *ctx, SlotDriveState *st,
                             const SlotStep *sp, double aim_rad, bool straight_out) {
    FPGALink *fpga = ctx->fpga;
    const RobotConfig *cfg = ctx->cfg;
    const double lane_x = ctx->lane_x, heading_rad = ctx->heading_rad;
    const double stop_y = ctx->stop_y;
    const bool forward = ctx->forward;
        /* 슬롯 중심선에서 좌우로 얼마나 벗어났는지. 진입방향이 -y이므로 차체 기준
         * '왼쪽'은 +x 방향 - 전진/후진에 따라 보정 부호가 뒤집힘.
         *
         * PATCH (2026-08-06): test_slot_map.py 시뮬레이션으로 실제 굴려보니, 이
         * 부호가 거꾸로 되어 있어서 오차를 줄이는 게 아니라 오히려 키우고 있었음
         * (양성 피드백) - 슬롯 진입 중 각도오차가 시간이 갈수록 계속 벌어지는 것으로
         * 발견됨(258도->248도->...). 검산: l=base-diff, r=base+diff 이면 diff>0일 때
         * theta가 증가(반시계)함. heading_err=theta-target>0(너무 반시계로 돌아있음)
         * 이면 theta를 줄여야(diff<0) 하는데, 기존엔 diff=+K*heading_err라 err>0일 때
         * diff>0이 나와서 정반대로 작동했음. lateral 항도 동일한 종류의 부호오류였음
         * (검산 결과 둘 다 부호가 반대라, diff 전체를 반전하는 것으로 한 번에 해결). */
        /* PATCH (2026-08-09a): 좌우오차를 '지도상의 x'가 아니라 '차체 기준 왼쪽오차'로
         * 계산하고, 슬롯 벽이 양쪽으로 보이면 라이다로 잰 값을 우선 사용한다.
         *   - 슬롯 평행구간의 편측 여유가 9.5mm 인데 스캔매칭 좌우오차는 10~20mm 라,
         *     SLAM 값으로 조향하면 여유보다 큰 오차를 쫓느라 오히려 벽에 붙었음.
         *   - 좌우 벽까지 거리 d_l, d_r 로부터 e = (d_r - d_l)/2 는 슬롯 폭도 차폭도
         *     몰라도 되고 스캔매칭이 틀려도 맞는 값이라 이 구간에서 훨씬 정확하다.
         * 부호 규약은 그대로: lat_left > 0 = 차체가 왼쪽으로 치우침 -> diff < 0. */
        double lateral_err = 0.0;
        double lat_scan_now = 0.0;
        bool used_scan_lat = robot_runner__slot_lateral_from_scan((&sp->scan), &lat_scan_now);
        if (used_scan_lat) {
            lateral_err = lat_scan_now;
        } else if (!robot_runner__lane_offset_body(sp->x, lane_x, heading_rad, &lateral_err)) {
            lateral_err = 0.0;      /* 세로 슬롯이 아니면 lane 제어를 끔 */
        }
        double heading_err = normalize_angle(sp->theta - aim_rad);
        /* ============================================================
         * BUGFIX (2026-08-10h): *** 후진이 못 나오던 진짜 원인 ***
         *
         * 예전 코드는 후진일 때 diff 를 통째로 뒤집었다:
         *     diff = -(K_LATERAL*lat + K_HEADING*herr);
         *     if (!forward) diff = -diff;      <-- 각도항까지 같이 뒤집힘
         *
         * 차동구동에서 l = base - diff, r = base + diff 이므로
         *     omega  ∝ (v_r - v_l) = 2*diff
         * 이고, 이 관계는 전진/후진과 무관하다(바퀴 속도차가 곧 회전이다).
         * 따라서 '각도를 목표로 되돌리는' 항은 절대 부호를 바꾸면 안 된다.
         * 뒤집으면 herr>0 일 때 theta 를 더 키우는 양성 피드백이 되어
         * 후진하는 동안 각도오차가 계속 벌어진다 - 실기 로그의
         *     각도오차 +17.5deg -> +23.5deg -> 벽에 걸려 정지
         * 가 정확히 이 현상이다. (2026-08-09j 는 이걸 '각도 복구 루틴을
         *  후진에도 붙이는' 방식으로 덮었을 뿐, 원인은 그대로였다.)
         *
         * 반면 '좌우 위치' 항은 뒤집는 게 맞다. 차체 왼쪽오차 e 에 대해
         *     e_dot = v * sin(heading 편차)
         * 이고 후진이면 v<0 이라 같은 편차가 반대로 작용하기 때문이다.
         *
         * => 두 항을 분리해서, 좌우항만 방향에 따라 부호를 바꾼다.
         * ============================================================ */
        double lat_term = SLOT_K_LATERAL * lateral_err * (forward ? 1.0 : -1.0);
        double diff = -(lat_term + SLOT_K_HEADING * heading_err);
        /* 슬롯 안에서 후진할 때는 조향 권한을 더 좁힌다. 폭 210mm 안에서
         * 255mm 짜리 차체를 비틀면 뒤끝이 먼저 벽에 닿기 때문이다.
         * 각도만 유지하고 좌우는 슬롯을 벗어난 뒤 slot_lane_trim 이 맞춘다. */
        double diff_cap = forward ? SLOT_MAX_DIFF : SLOT_MAX_DIFF_REVERSE;
        if (diff > diff_cap) diff = diff_cap;
        if (diff < -diff_cap) diff = -diff_cap;

        /* PATCH (2026-08-07): 슬롯 안에서는 일반 주행속도(sg_step_speed=34, 약 70mm/s)가
         * 너무 빠름. 이 구간은 V자 입구와 안쪽 벽에 "닿으면서" 들어가는 게 설계상
         * 정상이라 접촉 자체는 피할 수 없지만, 충돌 에너지는 속도의 제곱에 비례하므로
         * 속도만 낮춰도 세트장이 받는 힘이 크게 줄어듦(34->22이면 약 42% 수준).
         * 추가로 남은 깊이가 6cm 이내면 최소속도까지 선형으로 더 줄여서, 안쪽 벽에는
         * 항상 가장 느린 속도로 닿게 함. 너무 낮추면 정지마찰을 못 이기므로 하한을 둠. */
        /* PATCH (2026-08-08): 감속 구간을 6cm -> 3cm로 좁힘.
         * 맵을 보강해서 충돌에 견디게 됐고, 오히려 너무 일찍부터 느려지면 V자 경사벽에서
         * 정지마찰을 못 이기고 걸려버림(실기 증상: "미끄러지지 않고 걸림").
         * 미끄러져 들어가려면 경사면에 닿는 순간의 속도가 어느 정도는 필요함. */
        /* PATCH (2026-08-09f): 밀착 모드에서는 감속 구간을 1.5cm 로 더 좁힘.
         * "벽면은 튼튼하니 조금 미는 정도는 괜찮다"는 확인을 받았고, 오히려 너무
         * 일찍부터 slot_touch_speed(18)로 떨어지면 마지막 몇 mm 에서 정지마찰을
         * 못 이기고 벽에 닿기 전에 멈춰버린다(그러면 깊이가 모자란 채 '완료'). */
        const double TAPER_START_M = cfg->slot_press_to_wall ? 0.015 : 0.03;
        int speed = cfg->slot_step_speed;
        /* 후진은 '방금 들어온 길을 그대로 되짚는' 동작이라 슬롯 안에서 가장 안전한
         * 구간이다(앞에 새로 부딪힐 것이 없다). 그래서 진입용 저속(28)이 아니라
         * 일반 주행속도(34)를 쓴다 - 실기에서 잘 되던 '재시도 후진'과 같은 값이고,
         * 데드존 위쪽이라 정지마찰에 걸려 꾸물거리는 것도 줄어든다 (2026-08-10h). */
        if (!forward && cfg->sg_step_speed > speed) speed = cfg->sg_step_speed;
        if (forward) {
            double remain = sp->y - stop_y;
            if (remain < TAPER_START_M) {
                double t = remain / TAPER_START_M;
                if (t < 0.0) t = 0.0;
                int lo = cfg->slot_touch_speed;
                speed = (int)lround(lo + (cfg->slot_step_speed - lo) * t);
            }
        }
        if (speed < cfg->slot_touch_speed) speed = cfg->slot_touch_speed;

        if (straight_out) {
            diff = 0.0;                      /* 조향 없이 곧게 */
            speed = cfg->sg_step_speed;      /* 물린 걸 빼려면 힘이 좀 더 필요 */
        }

        /* 깔때기 복구 직후 첫 밀기는 세게 + 길게. 경사면에서 미끄러져 들어가려면
         * 접촉 순간에 충분한 속도가 필요하고(정지마찰), 조향은 빼야 옆으로 안 물린다. */
        double push_sec = cfg->slot_move_sec;
        if (st->funnel_kick) {
            speed = cfg->sg_step_speed;
            diff = 0.0;
            push_sec = cfg->slot_move_sec * 1.6;
            st->funnel_kick = false;
        }

        /* ============================================================
         * 깔때기 위 '열린 구간'은 한 번에 통째로 민다 (2026-08-10j 개정)
         *
         * 예전(2026-08-09h)에는 이 구간에서 push_sec 에 slot_fast_gain(2.5)을
         * 곱하고 sg_move_sec_max(0.45)로 잘랐다. 그래서 220mm 를 두 스텝에
         * 나눠 갔고, 스텝마다 고정비용 0.36초(정지+새 스캔+스캔매칭)를 또 냈다.
         *
         * 그런데 대기점(y=0.6475)에서 깔때기 입구(y=0.4275)까지는 좌우가 넓게
         * 열려 있어서 중간에 다시 볼 이유가 없는 구간이다. 여기를 한 스텝으로
         * 묶으면 고정비용을 한 번만 내므로 220mm 구간이 1.83초 -> 0.94초가 된다.
         *
         * 두 가지를 같이 바꿔야 의미가 있다.
         *   (1) 속도: 진입용 저속(28)이 아니라 일반 주행속도(34)를 쓴다.
         *       이 구간은 아직 아무것도 닿지 않으므로 느릴 이유가 없다.
         *   (2) 상한: 이번 스텝이 '깔때기 입구를 넘지 않도록' 시간을 자른다.
         *       예전에는 0.45초 고정이라 132mm 를 밀었고, 깔때기 입구를 그만큼
         *       지나쳐 버릴 수 있었다(눈 감고 깔때기에 들어가는 셈).
         *       지금은 정확히 입구에서 멈춰 다시 보므로 오히려 더 안전하다.
         *
         * 후진(이탈)에도 그대로 적용된다 - 나갈 때도 이 구간은 열려 있다.
         * ============================================================ */
        {
            double fast_y = (cfg->slot_fast_above_y > 0.0)
                                ? cfg->slot_fast_above_y
                                : (cfg->slot_staging_y - 0.22);
            if (sp->y > fast_y && cfg->slot_fast_gain > 1.0) {
                /* 열린 구간 전용 속도 - 0 이면 일반 주행속도를 쓴다 */
                int open_sp = (cfg->slot_open_speed > 0) ? cfg->slot_open_speed
                                                         : cfg->sg_step_speed;
                if (!st->funnel_kick && !straight_out && open_sp > speed) speed = open_sp;

                double open_max = (cfg->slot_open_move_sec > 0.0)
                                      ? cfg->slot_open_move_sec
                                      : cfg->sg_move_sec_max;
                push_sec *= cfg->slot_fast_gain;
                if (push_sec > open_max) push_sec = open_max;

                /* 전진이면 깔때기 입구(fast_y)를 넘지 않도록 자름.
                 * 넘어가면 '안 본 채로 좁은 곳에 들어간' 상태가 되기 때문. */
                if (forward) {
                    double v = robot_runner__linear_speed(cfg, speed)
                             * (*robot_runner__lin_scale());
                    if (v > 1e-6) {
                        double to_funnel = (sp->y - fast_y) + 0.010;
                        double cap_sec = to_funnel / v;
                        if (cap_sec < cfg->sg_move_sec_min) cap_sec = cfg->sg_move_sec_min;
                        if (push_sec > cap_sec) push_sec = cap_sec;
                    }
                }
            }
        }

        /* ============================================================
         * 남은 거리보다 더 밀지 않는다 (2026-08-10i)
         *
         * 사용자 신고: "주차슬롯 진입이 답답하게 느리다."
         * 실기 로그에서 역산한 실제 밀기 속도는 294mm/s 로 결코 느리지 않다.
         * 느린 이유는 속도가 아니라 '듀티'다. 한 스텝이
         *     정지 0.30 + 새 스캔 대기 0.20 + 계산 0.05 + 밀기 0.16 = 0.71초
         * 인데 그 중 실제로 미는 건 0.16초, 즉 23% 뿐이다. 76%가 기다리는 시간이다.
         * 그래서 실효속도가 294 -> 66mm/s 로 떨어진다.
         *
         * 해결은 '한 번에 더 크게 무는 것'인데, 예전에는 그게 위험했다.
         * push_sec 이 남은 거리와 무관해서, 목표 40mm 앞에서 88mm 를 밀어버릴 수
         * 있었기 때문이다(감속은 남은 15mm 안에서만 걸린다).
         *
         * 이 블록이 그 위험을 없앤다. 남은 거리를 실제 속도로 나눠 '이번에 밀어도
         * 되는 최대 시간'을 구하고 push_sec 을 거기에 맞춘다.
         *   - 멀리 있으면 : 크게 문다 (빠름)
         *   - 벽 근처면   : 자동으로 짧아진다 (안전)
         * 이게 있어야 slot_move_sec 을 키워도 안전하므로, 기본값도 같이 올린다
         * (0.16 -> 0.22, slot_settle_sec 0.30 -> 0.12).
         * ============================================================ */
        {
            double v = robot_runner__linear_speed(cfg, speed) * (*robot_runner__lin_scale());
            if (v > 1e-6) {
                double remain = forward ? (sp->y - stop_y) : (stop_y - sp->y);
                /* 밀착 모드면 목표를 조금 지나쳐도 되므로 그만큼 더 허용 */
                if (forward && cfg->slot_press_to_wall) remain += cfg->slot_press_over_m;
                remain += 0.010;   /* 모자라면 다음 스텝이 마저 민다 */
                if (remain < 0.0) remain = 0.0;
                double max_sec = remain / v;
                if (max_sec < cfg->sg_move_sec_min) max_sec = cfg->sg_move_sec_min;
                if (push_sec > max_sec) push_sec = max_sec;
            }
        }

        int base = forward ? speed : -speed;
        int l = (int)lround(base - diff);
        int r = (int)lround(base + diff);

        /* 복구 유예: 이번 스텝은 '판정'이 아니라 '실제로 밀어보기'다.
         * 여기서만 깎아야 밀지 않은 스텝이 유예를 소모하지 않는다 (2026-08-10h). */
        if (st->push_grace > 0) st->push_grace--;

        printf("[slot] step=%d pose=(%.3f,%.3f,%.1fdeg) 좌우오차=%+.3fm(%s) "
               "각도오차=%+.1fdeg spd=%d cmd=(%d,%d)%s\n",
               sp->step, sp->x, sp->y, sp->theta * 180.0 / ANGLE_PI, lateral_err,
               used_scan_lat ? "라이다" : "SLAM",
               heading_err * 180.0 / ANGLE_PI, speed, l, r,
               st->push_grace > 0 ? " [복구유예]" : "");
        fflush(stdout);

        fpga_link_set_speed(fpga, l, r);
        /* 한 번에 조금씩 여러 번 미는 것보다, 조금 길게 미는 쪽이 경사벽을 타고
         * 미끄러지기 쉬움(매번 멈추면 정지마찰을 새로 이겨야 함). --slot-move-sec */
        robot_runner__sleep_sec(push_sec);
}


bool slot_drive(FPGALink *fpga, LidarThread *lidar, OccupancyGridSLAM *nav_slam,
                const RobotConfig *cfg,
                double lane_x, double heading_rad,
                double stop_y, bool forward, const char *label,
                double entry_lat_err, bool *out_touched) {
    SlotDriveCtx ctx = { fpga, lidar, nav_slam, cfg,
                         lane_x, heading_rad, stop_y, forward, label };
    SlotDriveState st;
    memset(&st, 0, sizeof(st));
    st.last_y        = 1e9;
    st.best_depth_y  = 1e9;
    st.last_fix_y    = 1e9;
    st.last_fix_herr = 1e9;
    st.bias_span_m   = cfg->slot_bias_span_m;
    st.seat_confirm_y = 1e9;

    if (out_touched) *out_touched = false;
    if (forward) *robot_runner__slot_no_wall(cfg) = false;

    if (forward && fabs(entry_lat_err) > 0.005)
        slot_drive__entry_bias(&ctx, &st, entry_lat_err);

    robot_runner__init_prev_pose(fpga, cfg->has_encoder,
                                 &st.prev_rx, &st.prev_ry, &st.prev_rth);

    /* 하위 루틴(slot_fix_heading 등)도 같은 슬롯을 기준으로 재정박할 수 있게
     * 문맥을 남긴다 (2026-08-11a). probe_y 는 슬롯 축 위 아무 점이면 되므로
     * 진입(주차 y) / 이탈(대기점 y) 어느 쪽이든 그대로 쓸 수 있다. */
    robot_runner__slot_ctx_set(lane_x, stop_y, heading_rad);
    *robot_runner__slot_wrong_lane() = false;

    printf("[slot] %s 시작: 목표 lane_x=%.3f, %s, 정지 y=%.3f\n",
           label, lane_x, forward ? "전진" : "후진", stop_y);
    fflush(stdout);
    /* 깔때기에 박힌 뒤 빠져나올 때는 조향을 걸면 오히려 더 물림.
     * 재시도 후진에서는 좌우 차를 0으로 두고 힘을 올려 곧게 밀어냄. */
    bool straight_out = (!forward && strstr(label, "재시도") != NULL);
    if (!slot_drive__direction_ok(&ctx)) return false;

    for (int step = 0; step < SLOT_MAX_STEPS && !g_stop_requested; step++) {
        SlotStep sp;
        sp.step = step;
        SlotStepAction act = slot_drive__observe(&ctx, &st, &sp);
        if (act == SLOT_STEP_RETRY) continue;
        if (act == SLOT_STEP_FAIL)  break;

        /* PATCH (2026-08-09a): 예전엔 여기서 fflush 를 안 했음. stdout 이 파이프면
         * 블록버퍼라 GUI가 몇 초씩 뭉텅이로 늦게 받거나, 아래 continue 경로로 빠지면
         * 아예 안 나갔음. 공용 emit 함수로 통일(내부에서 fflush). */
        robot_runner__emit_state("slot", step, sp.x, sp.y, sp.theta, &sp.scan);

        SlotSeatEval ev;
        slot_drive__eval_seat(&ctx, &st, &sp, &ev);

        slot_drive__update_bias(&ctx, &st, &sp);

        double aim_rad = heading_rad + st.bias_rad;
        double herr_now_deg = normalize_angle(sp.theta - aim_rad) * 180.0 / ANGLE_PI;
        /* 착석 조건을 못 채운 접촉 = 낌. 깊이가 모자라든, 비스듬하든, 정면이
         * 뚜렷하게 열려 있든 전부 여기로 온다 (2026-08-09g). */
        bool jam_stall = (forward && ev.contact && !ev.seated);

        /* ============================================================
         * 후진(슬롯 이탈)에서도 같은 각도 복구를 쓴다 (2026-08-09j)
         *
         * 사용자 신고 + 실기 로그:
         *   step=18 pose=(0.220,0.280,287.5deg) 각도오차=+17.5deg cmd=(-35,-21)
         *   step=31 pose=(0.215,0.305,293.5deg) 각도오차=+23.5deg cmd=(-38,-18)
         *   [slot] 후진 중 정체 - 목표까지 343mm이나 남았습니다
         *
         * 후진하는 동안 각도오차가 17.5 -> 23.5도로 계속 커지다가 벽에 걸려 멈췄다.
         * 진입 때와 완전히 같은 현상인데, 예전에는 jam_stall / jam_angle 이 둘 다
         * forward 로만 걸려 있어서 후진에는 복구가 아예 안 돌았다.
         * 후진 조향 권한도 진입과 똑같이 약하므로(cmd 차이 최대 20, 스텝당 약 0.8도)
         * 조향으로는 20도짜리 오차를 못 지운다 - 제자리 회전만이 답이다.
         *
         * 다만 접촉을 푸는 방향은 반대다. 진입 중이면 뒤로 물러나 코를 빼야 하고,
         * 후진 중이면 앞으로 밀어 꽁무니를 빼야 한다. release_dir 로 넘긴다.
         * ============================================================ */
        bool back_stall = (!forward && ev.contact
                           && fabs(sp.y - stop_y) > SLOT_SEAT_CONFIRM_M);
        /* ============================================================
         * 이탈(후진) 중 각도 가드 (2026-08-11a 완화) - 사용자 요청:
         *   "각도가 조금 틀어진 정도는 주차 진입 시엔 괜찮지 않나?
         *    주차슬롯 탈출 시에 알아서 나가면 되잖아."
         *
         * 그런데 예전 코드는 그 반대였다. 착석 각도를 12도까지 허용해 놓고
         * 후진 첫 스텝에서 가드가 8도라, 나가자마자 슬롯 바닥에서 제자리 회전을
         * 시작했다. 폭 210mm 안에서 255mm 차체를 돌리는 것은 원래 위험한 기동이고
         * (임계각 20.2도), 평행벽은 곧게 빼기만 하면 기계적으로 자세를 잡아 준다.
         *
         * 그래서 라이다가 '평행구간 안'이라고 확인해 주는 동안에는 가드를
         * slot_seat_yaw_deg + 4도(기본 16도)로 열어 그냥 곧게 빼낸다.
         * 실제로 물려서 안 나오면 위 back_stall 이 잡으므로 안전장치는 그대로다.
         * 평행구간을 벗어나 통로로 나오면(sfix.ok=false) 가드가 8도로 돌아온다
         * - 각도는 거기서 여유 있게 풀면 된다.
         * ============================================================ */
        double back_guard_deg = (sp.sfix.ok && !forward)
                                    ? (cfg->slot_seat_yaw_deg + 4.0)
                                    : SLOT_HERR_GUARD_DEG;
        bool back_angle = (!forward && step > 0 && st.push_grace == 0
                           && fabs(sp.y - stop_y) > SLOT_SEAT_CONFIRM_M
                           && fabs(herr_now_deg) > back_guard_deg);
        /* ============================================================
         * BUGFIX (2026-08-10h) *** "다 들어가 놓고 자꾸 후진하던" 무한루프 ***
         *
         * 실기 로그(사용자 신고 1번):
         *   [slot] 각도 복구 완료: 오차 -0.9deg (허용 2.0), pose=(0.241,0.229)
         *   [slot] 반대편 기울임 종료(목표 깊이 근접): 슬롯 축으로 되돌립니다
         *   [slot] 각도가 틀어짐 (y=0.229, ... 각도 -8.9deg) -> 바로잡습니다 2/5
         *   [slot]    ... 빈 쪽으로 -8.0deg 더 튼 -98.0deg 를 목표로 잡습니다
         *   [slot] 각도 복구 완료: 오차 -0.9deg, pose=(0.241,0.229)   <- 이미 -98.9라 즉시 통과
         *   [slot] 반대편 기울임 종료(목표 깊이 근접) ...              <- 다시 0으로
         *   ... 3/5, 4/5, 5/5 ... [slot] 진입 중단 -> 전체 후진 -> 재접근
         *
         * pose 가 (0.241,0.229) 로 한 발도 안 움직인 채 5번을 다 태웠다. 원인은
         * 서로 모순되는 두 규칙이 매 스텝 번갈아 이겼기 때문이다.
         *   (a) 걸림 복구는 '빈 쪽으로 8도 튼 각도'를 목표로 잡는다
         *   (b) 착석 구간(y <= stop_y + bias_span)에 들어오면 기울임을 즉시 0으로 되돌린다
         * 착석 구간 안에서 (a)가 발동하면, 회전을 마치자마자 (b)가 그 8도를 지워서
         * '8.9도짜리 각도오차'를 인위적으로 만들고, 그게 가드(8도)를 다시 넘긴다.
         *
         * 두 가지로 끊는다.
         *   1) 착석 구간에서는 복구 목표를 슬롯 축 그대로 잡는다(아래 seat_zone).
         *      어차피 (b)가 지울 기울임을 굳이 걸 이유가 없다.
         *   2) 복구 직후에는 최소 한 번은 '실제로 밀어본 뒤'에 다시 판정한다
         *      (push_grace). 관측->회전->관측만 반복하며 시도횟수를 태우는 것을 막는다.
         * ============================================================ */
        bool jam_angle = (forward && !ev.near_depth && step > 0 && st.push_grace == 0
                          && fabs(herr_now_deg) > SLOT_HERR_GUARD_DEG);

        if (back_stall || back_angle) {
            act = slot_drive__recover_back(&ctx, &st, &sp, back_stall, herr_now_deg);
            if (act == SLOT_STEP_RETRY) continue;
            if (act == SLOT_STEP_FAIL)  { fpga_link_set_speed(fpga, 0, 0); return false; }
        }

        if (jam_stall || jam_angle) {
            double lat_mm = (sp.x - lane_x) * 1000.0;

            /* 직전 복구와 '같은 자리 + 같은 오차'면 복구가 아무것도 못 바꾼 것이다.
             * 그런 복구를 다섯 번 반복해봐야 결과는 같으므로(실기 로그 그대로),
             * 시도횟수를 태우지 말고 곧게 밀어 넣는 쪽으로 넘긴다 (2026-08-10h).
             * 슬롯 안쪽이라 앞은 어차피 벽뿐이고, 밀어서 닿으면 착석 판정이 받는다. */
            if (fabs(sp.y - st.last_fix_y) < 0.006
                && fabs(herr_now_deg - st.last_fix_herr) < 1.5) {
                st.fix_no_progress++;
            } else {
                st.fix_no_progress = 0;
            }
            st.last_fix_y = sp.y;
            st.last_fix_herr = herr_now_deg;
            if (st.fix_no_progress >= 2 && forward) {
                printf("[slot] 각도 복구가 두 번 연속 아무것도 바꾸지 못했습니다 "
                       "(y=%.3f 고정, 각도 %+.1fdeg 고정, 좌우 %+.0fmm).\n"
                       "       빼내지 않고, 슬롯 축 그대로 곧게 밀어 넣습니다 "
                       "(목표까지 %.0fmm)\n",
                       sp.y, herr_now_deg, lat_mm, (sp.y - stop_y) * 1000.0);
                fflush(stdout);
                st.bias_rad = 0.0;
                st.bias_is_residual = false;
                aim_rad = heading_rad;       /* 기울임을 지웠으니 조향 목표도 축으로 */
                herr_now_deg = normalize_angle(sp.theta - aim_rad) * 180.0 / ANGLE_PI;
                st.fix_no_progress = 0;
                st.push_grace = 3;              /* 최소 3번은 밀어보고 다시 판정 */
                st.funnel_kick = true;
                st.stall_count = 0;
                st.last_y = 1e9;
                /* 여기서 continue 하지 않고 아래 밀기 코드로 흘려보낸다 */
                jam_stall = false;
                jam_angle = false;
            }
        }

        if (jam_stall || jam_angle) {
            act = slot_drive__recover_jam(&ctx, &st, &sp, herr_now_deg,
                                          &aim_rad, &jam_stall, &jam_angle);
            if (act == SLOT_STEP_RETRY) continue;
            if (act == SLOT_STEP_FAIL)  { fpga_link_set_speed(fpga, 0, 0); return false; }
        }

        if (ev.reached || ev.stalled) {
            act = slot_drive__finish(&ctx, &sp, &ev, straight_out, out_touched);
            if (act == SLOT_STEP_DONE) return true;
            if (act == SLOT_STEP_FAIL) return false;
        }

        slot_drive__push(&ctx, &st, &sp, aim_rad, straight_out);
    }

    fpga_link_set_speed(fpga, 0, 0);
    printf("[slot] %s: 스텝 한도 초과 또는 중단\n", label);
    fflush(stdout);
    return false;
}



/* ============================================================
 * 목적지 좌표 사전검증 (2026-08-07 추가)
 *
 * 실기에서 --point-b 의 y를 0.1775 대신 1.770으로 준 채로 돌렸더니, 지도(1.50x0.90m)
 * 바깥의 좌표인데도 아무 경고 없이 진행돼서 엉뚱한 자리에서 "주차 완료"가 찍혔음.
 * 좌표는 사람이 손으로 넣는 값이라 오타가 반드시 나오므로, 움직이기 전에 한 번
 * 걸러내는 게 맞음. 지도 밖 / 벽 속 / 슬롯 진입방향과 모순되는 값을 여기서 잡음.
 * ============================================================ */
bool shuttle_validate_point(const unsigned char *grid, int rows, int cols,
                                           double res, const RobotConfig *cfg,
                                           ShuttlePoint p, const char *label) {
    double max_x = cols * res, max_y = rows * res;
    bool ok = true;

    if (p.x < 0.0 || p.x > max_x || p.y < 0.0 || p.y > max_y) {
        fprintf(stderr, "[검증] %s (%.3f, %.3f)가 지도 밖입니다. 지도 범위: x 0~%.3f, y 0~%.3f m\n",
                label, p.x, p.y, max_x, max_y);
        ok = false;
    } else {
        int col = (int)(p.x / res), row = (int)(p.y / res);
        if (row >= 0 && row < rows && col >= 0 && col < cols && grid[row * cols + col]) {
            fprintf(stderr, "[검증] %s (%.3f, %.3f)가 지도상 벽/장애물 안입니다.\n",
                    label, p.x, p.y);
            ok = false;
        }
    }

    if (cfg->slot_parking && p.y >= cfg->slot_staging_y) {
        fprintf(stderr,
            "[검증] %s 의 y(%.3f)가 슬롯 대기점 y(%.3f)보다 위입니다. 슬롯은 -y 방향으로\n"
            "       진입하므로 주차자세의 y는 대기점보다 반드시 작아야 합니다.\n"
            "       (make_slot_map.py 기준 주차자세는 (0.225, 0.178) / (1.275, 0.178), -90도)\n",
            label, p.y, cfg->slot_staging_y);
        ok = false;
    }

    if (cfg->slot_parking) {
        double th = fmod(p.theta_deg, 360.0);
        if (th < 0) th += 360.0;
        double off = fabs(th - 270.0);
        if (off > 10.0) {
            fprintf(stderr,
                "[검증] 경고: %s 의 주차각도 %.1fdeg 가 슬롯 진입방향(-90deg = 270deg)에서\n"
                "       %.1fdeg 벗어나 있습니다. 슬롯 안에서는 1도당 약 7mm씩 밀리므로\n"
                "       %.0fmm 어긋난 채로 40cm를 들어가게 됩니다 - 값을 확인하세요.\n",
                label, p.theta_deg, off, off * 7.0);
        }
    }
    return ok;
}


/* ============================================================
 * 주차 최종 확정 (2026-08-11a 신규) *** "잘 주차했는데 혼자 나가버리는" 것 차단 ***
 *
 * 예전에는 slot_seat() 직후 곧바로 스캔매칭 자세로만 주차오차를 쟀다. 그런데 그
 * 자세가 바로 슬롯 안에서 가장 못 믿을 값이다(robot_runner__slot_reanchor 머리말).
 * 실기 로그:
 *     [slot] 슬롯 진입 완료: pose=(1.049, 0.164, ...)   <- x 가 226mm 틀림
 *     [shuttle] B지점 주차 오차 초과: ... 226.7mm -> 재시도 후진
 *
 * 이제 판정 직전에 바퀴를 세우고 깨끗한 스캔을 한 장 찍어서
 *   (1) 자세를 라이다 실측으로 다시 박고
 *   (2) '물리적으로 슬롯 안 정위치인가'를 별도로 판정해 돌려준다.
 * 호출부는 (2)가 참이면 스캔매칭 오차가 얼마든 주차로 인정한다.
 *
 * 반환 true = 라이다가 '슬롯 평행구간 안 + 좌우 중앙 + 목표 깊이' 를 확인함.
 * ============================================================ */
bool robot_runner__slot_confirm_park(FPGALink *fpga, LidarThread *lidar,
                                                    const RobotConfig *cfg,
                                                    OccupancyGridSLAM *slam,
                                                    double lane_x, double heading_rad,
                                                    double park_y) {
    if (!cfg->slot_lidar_fix || slam == NULL || slam->lf_field == NULL) return false;
    fpga_link_set_speed(fpga, 0, 0);
    robot_runner__sleep_sec(cfg->sg_settle_sec);
    LidarScan scan;
    if (!robot_runner__take_scan(lidar, cfg, &scan, 2.0)) return false;
    if (scan.count < 60) return false;

    SlotLidarFix f;
    double dx = 0.0, dy = 0.0;
    robot_runner__slot_reanchor(slam, &scan, cfg, lane_x, heading_rad, park_y,
                                "주차 최종 확정", &f, &dx, &dy);
    if (!f.ok) return false;

    /* 좌우 여유는 편측 (210-130)/2 = 40mm 다. 45mm 를 넘으면 정말로 한쪽 벽에
     * 붙어 있는 것이므로 인정하지 않는다. */
    const double LAT_OK_M = 0.045;
    double y_now = slam->y;
    bool depth_ok = (y_now <= park_y + cfg->slot_seat_depth_tol_m + 0.030);
    bool lat_ok   = (fabs(f.lat_left_m) <= LAT_OK_M);
    bool yaw_ok   = (!f.yaw_ok) ||
                    (fabs(f.yaw_err_rad) * 180.0 / ANGLE_PI <= cfg->slot_seat_yaw_deg + 3.0);

    printf("[slot] 주차 최종 확정(라이다): 좌벽 %.3fm / 우벽 %.3fm (폭 %.3f), "
           "치우침 %+.0fmm, 각도 %s, 깊이 y=%.3f (목표 %.3f)\n",
           f.d_left, f.d_right, f.width_m, f.lat_left_m * 1000.0,
           f.yaw_ok ? "" : "측정불가",
           y_now, park_y);
    if (f.yaw_ok)
        printf("       각도오차 %+.1fdeg (허용 %.1f)\n",
               f.yaw_err_rad * 180.0 / ANGLE_PI, cfg->slot_seat_yaw_deg + 3.0);
    fflush(stdout);
    return lat_ok && depth_ok && yaw_ok;
}


/* 슬롯 안쪽 끝까지 밀어넣어 코를 벽에 붙임 - 마지막 정렬은 벽이 해줌.
 * 위치추정으로는 mm 단위를 못 맞추므로, 확실히 닿을 만큼 짧게 밀어주고 끝냄.
 *
 * PATCH (2026-08-07): 여기가 세트장을 가장 세게 때리던 곳이었음.
 *   - 예전: 이미 벽에 닿아서 멈춘 뒤에도 무조건 sg_step_speed(34, 약 70mm/s)로
 *     0.45초 연속 전진 -> 정지한 벽을 0.45초 내내 밀어붙임(바퀴는 헛돌지만 힘은
 *     계속 걸림). 폼보드가 우그러지는 주 원인.
 *   - 지금: (1) slot_drive가 이미 '벽에 닿음'을 감지했으면 아예 생략하고,
 *           (2) 안 닿았을 때만 slot_touch_speed로 짧게(0.2초) 밀어줌.
 *     닿았는지 여부는 stall 판정이 이미 알고 있으므로 그대로 재활용. */
void slot_seat(FPGALink *fpga, const RobotConfig *cfg, double heading_rad,
                              bool already_touched) {
    (void)heading_rad;
    /* PATCH (2026-08-09f): 요청 "머리가 박을 때까지 완전히 딱 붙여라. 벽면은 튼튼하다."
     *
     * 예전에는 이미 닿았으면(already_touched) 추가 밀착을 통째로 생략했다. 그런데
     * slot_drive 의 접촉 판정은 '3스텝 연속 4mm 미만 이동'이라, 경사면에 살짝
     * 얹혀서 못 움직인 경우도 접촉으로 잡힌다 - 그 상태로 끝내면 몇 mm 뜬 채 주차된다.
     * 이제는 닿았든 아니든 마지막에 한 번 확실히 눌러준다. 속도는 slot_touch_speed
     * (기본 18 = PWM 189) 그대로라 충돌 에너지는 낮게 유지되고, 밀기 시간만
     * slot_press_sec 으로 늘려서 정지마찰을 확실히 넘긴다. */
    double sec = (cfg->slot_press_sec > 0.0) ? cfg->slot_press_sec : 0.20;
    printf("[slot] 안쪽 벽에 밀착 중 (속도 %d, %.2f초%s)...\n",
           cfg->slot_touch_speed, sec,
           already_touched ? ", 이미 닿아 있지만 확실히 앉힘" : "");
    fflush(stdout);
    fpga_link_set_speed(fpga, cfg->slot_touch_speed, cfg->slot_touch_speed);
    robot_runner__sleep_sec(sec);
    fpga_link_set_speed(fpga, 0, 0);
    robot_runner__sleep_sec(0.25);
}
