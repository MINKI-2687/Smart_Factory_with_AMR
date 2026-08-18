#include "robot_navigate.h"
#include "robot_runner.h"

/* ============================================================
 * navigate_one_leg_inner() 분해 (2026-08-11 리팩토링)
 *
 * 예전에는 1,147줄이 한 함수였고, 그 안의 stop-and-go 루프 하나가 942줄이었다.
 * 로직은 손대지 않고, 루프를 넘어 사는 값(NavState)과 한 번 정하면 안 바뀌는
 * 값(NavCtx), 이번 스텝 관측(NavStep) / 이번 스텝 명령(NavCmdPlan)을 구조체로
 * 묶은 뒤 단계별 헬퍼로 잘랐다. 각 헬퍼의 주석은 원래 그 자리에 있던 것 그대로다.
 *
 * 제어흐름은 NavAction 으로 돌려준다. 원본이 루프 안에서 continue / break /
 * return 을 직접 쓰던 자리가 그대로 대응된다.
 *     NAV_GO      : 계속 진행 (원본의 fall-through)
 *     NAV_RETRY   : 다음 스텝으로 (원본의 continue)
 *     NAV_STOP    : 루프 탈출 (원본의 break)
 *     NAV_ARRIVED : 목표 도착 (원본의 return true)
 *     NAV_FAILED  : 실패 종료 (원본의 return false)
 * ============================================================ */

/* ---- 원래 함수 안 지역상수였던 것들 ----
 * 분해한 헬퍼들이 같은 값을 봐야 해서 파일 스코프로 올렸다. 값은 그대로다. */
/* 정체(끼임) 감지용 */
#define NAV_STUCK_EPS_M         0.010   /* 이보다 덜 움직이면 정체로 셈 */
#define NAV_STUCK_EPS_RAD       0.035   /* 약 2도 */
#define NAV_STUCK_STEPS         25      /* 연속 25스텝(대략 15~20초) */
#define NAV_STUCK_ESCAPE_AT     5       /* 5스텝 정체하면 먼저 탈출을 시도 */
#define NAV_STUCK_MAX_ESCAPES   6       /* 이만큼 해보고도 안 되면 포기 */
#define NAV_BLIND_RELOC_AT      4       /* 관측 불가가 이만큼 이어지면 위치를 다시 잰다 */
/* ============================================================
 * '회전 여유 부족 -> 끼임 탈출' 횟수 제한 (2026-08-09e)
 *
 * 사용자 신고 "통로에서 크게 직진/후진만 반복"의 마지막 고리가 여기였다.
 * 아래 회전 여유 검사에는 재시도 횟수 제한이 아예 없어서, 조건이 유지되는 한
 * 매 스텝 robot_runner__escape_to_open() 을 불렀다. 이 통로는 높이가 0.40m 라
 * 사방 최소거리가 아무리 잘해야 0.20m 이고 필요값이 robot_radius+0.01=0.15m 라,
 * 중심이 정중앙에서 5cm만 벗어나면 조건이 계속 참이다. 탈출은 앞/뒤로 최대
 * 12cm 씩 미는데 앞뒤로 움직여봐야 좌우 여유는 1mm도 안 변하므로, 화면에서
 * 보이던 무한 왕복이 된다.
 *
 * 근본원인(경로에서 벗어나면 제자리 회전을 요구하던 것)은 path_follow.h 의
 * 적응형 전방주시로 없앴지만, 만약을 위해 여기에도 상한을 둔다. 상한을 넘으면
 * 탈출을 포기하고 '그냥 천천히 돈다' - 사방최소가 필요값에 2cm 못 미치는
 * 정도라면 모서리가 스칠 수는 있어도, 앞뒤로 영원히 왕복하는 것보다 낫다. */
#define ROT_CLEAR_MAX_ESCAPES   3
/* PATCH (2026-08-10): 탈출 쿨다운.
 * 사용자 신고 "앞뒤로 계속 반복적으로 움직인다"의 마지막 고리가 여기다.
 * 탈출 직후에는 자세추정이 방금 갱신됐고 여유도 조금 늘었지만, 임계값 근처
 * (실기 로그: 필요 141mm vs 실제 138mm - 겨우 3mm 차이)에서는 다음 스텝에
 * 곧바로 다시 '부족' 판정이 난다. 그래서 밀고 - 재고 - 또 밀고가 반복됐다.
 * 한 번 탈출했으면 이만큼의 스텝 동안은 절대 다시 탈출하지 않는다.
 * 그 동안은 '돌 수 있는 만큼만 회전'으로 조금씩 각도를 틀어 스스로 풀린다. */
#define ROT_ESCAPE_COOLDOWN     8
/* 이보다 작은 회전은 정지마찰 때문에 어차피 못 만든다 - 그 아래면 탈출로 간다 */
#define ROT_MIN_USEFUL_DEG      4.0

/* 정체 판정 기준 스캔 (2026-08-10e). 9.6KB 라 스택이 아니라 정적으로 둔다.
 * 예전에는 함수 안 static 이었는데, 분해하면서 파일 스코프로 옮겼다 - 수명은 같다. */
static LidarScan g_nav_stuck_ref_scan;

/* 한 번 정하면 안 바뀌는 값들 */
typedef struct {
    FPGALink          *fpga;
    LidarThread       *lidar;
    DynamicNavigator  *nav;
    OccupancyGridSLAM *nav_slam;
    const RobotConfig *cfg;
    bool               use_slam_rolling_map;
    double            *prev_raw_x, *prev_raw_y, *prev_raw_theta;
    double             control_period;
} NavCtx;

/* 스텝 사이에 들고 다니는 상태. 예전에는 전부 지역변수였다. */
typedef struct {
    int    step;
    /* 정체(끼임) 추적 */
    int    escape_tries;
    double stuck_ref_x, stuck_ref_y, stuck_ref_theta;
    int    stuck_count;
    bool   stuck_ref_init;
    int    blind_steps;              /* 관측 불가 구간이 이어진 스텝 수 */
    /* 회전 여유 부족 -> 탈출 횟수 제한 */
    int    rot_clear_escapes;
    int    rot_escape_cooldown;
    /* 전역 재위치추정 트리거 (2026-08-10) */
    int    lost_streak;
    int    last_global_step;
    /* 회전 명령을 냈는데 실제로 안 돈 스텝 수 / 그때 펄스를 늘리는 배수 */
    int    rot_no_motion;
    double rot_kick_gain;
    bool   sweep_block_warned;       /* 같은 안내를 매 스텝 찍지 않게 */
    /* 연속 스윕 회전 상태 (2026-08-09j) */
    bool   sweep_pending;
    double sweep_prev_theta, sweep_sec;
    /* 회전 각속도 실측 추정기 */
    RotRateEst *rot_est;
    double rot_prev_theta, rot_prev_sec;
    double rot_prev_dir;             /* 직전 회전 스텝의 명령 방향 (+1 반시계 / -1 시계) */
    bool   rot_pending;
    bool   prev_was_rotation;
    /* 기본 탐색창 백업 - 회전 스텝에서만 임시로 회전 전용 창으로 바꿨다가 되돌린다 */
    double base_win_m, base_step_m, base_win_th, base_step_th;
} NavState;

/* 이번 스텝에 관측한 것 */
typedef struct {
    LidarScan scan;
    double x, y, theta;
    double odom_dx, odom_dy, odom_dth;
    double slam_prev_x, slam_prev_y, slam_prev_th;
} NavStep;

/* 이번 스텝에 내보낼 명령 */
typedef struct {
    WheelCmd cmd;                /* dynnav 가 낸 원본 명령 */
    int      m;                  /* 좌우 중 큰 절댓값 - 시간 환산의 기준 */
    int      out_left, out_right;
    double   move_sec;
    bool     rotation_dominant;
    double   rot_step_rad;       /* 이번 스텝에 실제로 돌 각도 */
    double   want0;              /* 안전검사와 명령이 공유하는 회전시간 */
    bool     rot_tight_space;    /* 여유가 모자란 채 도는 중인가 */
    double   rot_allow_frac;     /* 이번 스텝에 실제로 허용된 회전 비율 */
} NavCmdPlan;

typedef enum {
    NAV_GO = 0,   /* 계속 진행 */
    NAV_RETRY,    /* 다음 스텝으로 (원본의 continue) */
    NAV_STOP,     /* 루프 탈출 (원본의 break) */
    NAV_ARRIVED,  /* 목표 도착 */
    NAV_FAILED,   /* 실패 종료 */
} NavAction;


/* 한 스텝의 관측: 탐색창 설정 -> (필요하면) 정지 -> 스캔 -> 프레임 보정 -> 위치추정.
 * 원본 루프 맨 앞 96줄이 그대로 들어 있다. */
static NavAction nav__observe(const NavCtx *ctx, NavState *st, NavStep *ns) {
    FPGALink          *fpga     = ctx->fpga;
    LidarThread       *lidar    = ctx->lidar;
    DynamicNavigator  *nav      = ctx->nav;
    OccupancyGridSLAM *nav_slam = ctx->nav_slam;
    const RobotConfig *cfg      = ctx->cfg;

    /* PATCH (2026-08-09e): 도킹(대기점 정밀 접근) 구간은 연속주행을 끈다.
     * 도킹은 mm 단위로 앞뒤를 맞추는 단계인데, 연속주행이면 멈추지 않은 채
     * 움직이며 찍은 스캔으로 위치를 잡아 오차가 커지고, 목표를 지나쳤다가
     * 되돌아오는 앞뒤 왕복이 생긴다(사용자 신고 증상 중 하나). */
    /* 2026-08-09f 정정: 도킹 '전체'를 stop-and-go 로 돌렸더니 대기점 접근
     * 40cm 를 전부 툭툭 기어가서 매우 느려졌다(사용자 신고).
     * 정밀도가 필요한 건 마지막 5cm(close_mode)뿐이므로 거기만 멈춘다. */
    bool need_stop = st->prev_was_rotation || !cfg->hybrid_cruise
                     || (nav->docking_mode
                         && (nav->docking_ctrl.close_mode
                             || nav->docking_ctrl.position_reached));

    /* PATCH (2026-08-09h): 직전이 제자리 회전이었으면 이번 스텝은 각도 탐색창을
     * 넓힌다. 다음 회전 스텝의 허용 회전량(clamp_rotation_sec_rate)이 창의 70%
     * 이므로, 창을 ±12 -> ±25도로 넓히면 한 스텝에 8.4 -> 17.5도를 돌 수 있다.
     * 회전 중에는 위치가 거의 안 변하므로 위치창은 좁혀도 되고, 후보 수가
     * n_xy^2 * n_th 라 총 계산량은 오히려 줄어든다. */
    if (st->prev_was_rotation) {
        nav_slam->search_window_m     = cfg->rot_win_m;
        nav_slam->search_step_m       = cfg->rot_step_m;
        nav_slam->search_window_theta = cfg->rot_win_deg  * ANGLE_PI / 180.0;
        nav_slam->search_step_theta   = cfg->rot_step_deg * ANGLE_PI / 180.0;
        /* 스윕 직후에는 창을 스윕 크기에 비례해 넓힌다 (2026-08-10).
         * 눈 감고 S도를 돌았으면 각속도 추정오차만큼 예측이 틀어져 있는데,
         * 고정 +-25도 창으로는 그 오차를 못 덮어 정답이 창 밖으로 나간다
         * (실기: 118도 스윕에서 잔차 57도 -> 자세추정 붕괴).
         * 이 스텝만 넓히므로 계산량 증가는 한 번뿐이다. */
        if (st->sweep_pending && st->sweep_sec > 0.0) {
            double sw_deg = fabs(rot_rate_get(st->rot_est)) * st->sweep_sec * 180.0 / ANGLE_PI;
            double want_deg = 0.45 * sw_deg;
            if (want_deg < cfg->rot_win_deg) want_deg = cfg->rot_win_deg;
            if (want_deg > 60.0) want_deg = 60.0;
            if (want_deg > cfg->rot_win_deg + 0.5) {
                nav_slam->search_window_theta = want_deg * ANGLE_PI / 180.0;
                printf("[nav] 스윕 직후 각도 탐색창을 ±%.0fdeg 로 넓힙니다 "
                       "(눈 감고 %.0fdeg 돌았음)\n", want_deg, sw_deg);
                fflush(stdout);
            }
        }
    } else {
        nav_slam->search_window_m     = st->base_win_m;
        nav_slam->search_step_m       = st->base_step_m;
        nav_slam->search_window_theta = st->base_win_th;
        nav_slam->search_step_theta   = st->base_step_th;
    }

    if (need_stop) {
        /* 1) 완전 정지 + 흔들림 가라앉히기 */
        fpga_link_set_speed(fpga, 0, 0);
        robot_runner__sleep_sec(cfg->sg_settle_sec);
        if (g_stop_requested) return NAV_STOP;

        /* 2) 정지 상태에서만 수집된 깨끗한 스캔 확보 */
        if (!wait_for_fresh_scan(lidar, &ns->scan, 2.0)) {
            if (g_stop_requested) return NAV_STOP;
            fprintf(stderr, "[nav] 유효한 스캔을 못 받음 - 재시도\n");
            return NAV_RETRY;
        }
    } else {
        /* 연속 주행 중 - 바퀴를 멈추지 않고 최신 스캔을 그대로 사용 */
        lidar_thread_get_latest(lidar, &ns->scan);
        if (ns->scan.count == 0) {
            robot_runner__sleep_sec(0.02);
            return NAV_RETRY;
        }
    }
    apply_lidar_frame_fix(&ns->scan, cfg->lidar_mirror, cfg->lidar_yaw_offset_deg,
                      cfg->lidar_offset_forward_m);

    /* 3) 위치추정
     * PATCH (2026-08-07): 예전엔 여기에 (0,0,0)이 하드코딩돼 있었고 주석은
     * "정지 상태이므로 오도메트리 변화량은 정의상 0"이었음. 이건 틀린 추론임.
     * '스캔을 찍는 순간' 정지해 있는 건 맞지만, 스캔매칭이 필요로 하는 값은
     * "직전 스캔 이후 얼마나 움직였나"이고 그 사이에 로봇은 move_sec 동안
     * 실제로 움직였음. 그래서 이 하드코딩 때문에 엔코더를 달아도 stop-and-go
     * 경로(=지금 쓰는 경로)에서는 오도메트리가 전혀 안 쓰였음. 이제 제대로
     * 델타를 넘김 - 엔코더가 없으면 get_odom_delta가 알아서 0을 돌려줌. */

    robot_runner__predict_delta(fpga, cfg, nav_slam->theta,
                                 ctx->prev_raw_x, ctx->prev_raw_y, ctx->prev_raw_theta,
                                 &ns->odom_dx, &ns->odom_dy, &ns->odom_dth);
    /* 스윕 회전분은 2026-08-10부터 명령 적분기(predict_delta)가 이미 포함하므로
     * 여기서 또 더하면 두 번 세는 것이 된다. sweep_pending 은 아래 각속도
     * 실측 갱신에만 쓴다. */
    ns->slam_prev_x = nav_slam->x; ns->slam_prev_y = nav_slam->y;
ns->slam_prev_th = nav_slam->theta;
    if (ctx->use_slam_rolling_map) {
        slam_update(nav_slam, &ns->scan, ns->odom_dx, ns->odom_dy, ns->odom_dth, &ns->x, &ns->y, &ns->theta);
    } else {
        slam_localize_step(nav_slam, &ns->scan, ns->odom_dx, ns->odom_dy, ns->odom_dth, &ns->x, &ns->y, &ns->theta);
    }
    robot_runner__odom_report(cfg, fpga, ns->odom_dx, ns->odom_dy, ns->odom_dth,
                               ns->x - ns->slam_prev_x, ns->y - ns->slam_prev_y,
                               normalize_angle(ns->theta - ns->slam_prev_th));
    return NAV_GO;
}


static NavAction nav__watch_lost(const NavCtx *ctx, NavState *st, const NavStep *ns) {
    FPGALink          *fpga     = ctx->fpga;
    LidarThread       *lidar    = ctx->lidar;
    DynamicNavigator  *nav      = ctx->nav;
    OccupancyGridSLAM *nav_slam = ctx->nav_slam;
    const RobotConfig *cfg      = ctx->cfg;
/* ============================================================
 * 길을 잃었는지 감시하고, 잃었으면 지도 전체에서 다시 찾는다 (2026-08-10)
 *
 * 국소 스캔매칭은 예측 주변 +-0.15m / +-12도만 본다. 자세가 그 밖으로
 * 나가면 스캔이 아무리 또렷해도 정답이 후보에 없어서 영영 못 찾는다.
 * 그래서 '일치도가 낮은 상태가 계속되면' 멈추고 전역탐색을 한다.
 * 한 번에 되찾으므로 몇 스텝 헤매는 것보다 오히려 빠르다.
 * ============================================================ */

    /* 0.72 -> 0.80 (2026-08-10, 사용자 제안 "임계값을 더 올리면 나아질까?").
     * 실기 로그 기준: 자세가 맞을 때 0.93, 무너진 구간은 0.54~0.84 였다.
     * 0.72 는 0.81/0.84 같은 '틀렸는데 그럴듯한' 값을 통과시켰다.
     * 0.80 이면 그 구간을 잡으면서, 지도에 없는 물체가 조금 보이는
     * 정상 상황(보통 0.85 이상)은 오탐하지 않는다. */
    double fit = robot_runner__map_fit(nav_slam, &ns->scan, ns->x, ns->y, ns->theta);
    if (fit < 0.80) st->lost_streak++;
    else            st->lost_streak = 0;
    /* 자세를 확실히 믿을 수 있는 스텝에서는 재위치추정 예산을 0으로
     * 되돌린다 (2026-08-10e). 이 리셋이 있어야 '대기점에 잘 도착한 뒤
     * 제자리 회전만 했는데 347mm 순간이동' 같은 후보가 거부된다. */
    if (fit >= 0.92) robot_runner__travel_acc_reset();
    /* ============================================================
     * 스캔매칭이 직접 올린 요청을 받는다 (2026-08-10d)
     *
     * slam_localize_step() 은 '정답을 찾았지만 한 스텝에 옮기기엔
     * 너무 큰 교정' 일 때 relocalize_request 를 올린다. 예전에는 그
     * 신호가 발신 즉시 지워지고 받는 쪽도 없어서, 로그에만
     *     [slam] 위치 재탐색 보류: 146mm 교정은 한 스텝에 불가능합니다
     * 가 찍히고 로봇은 틀린 자세 그대로 계속 달렸다.
     * 이제 그 요청을 여기서 받아 즉시 전역탐색을 돌린다. 일치도가
     * 0.80 이상이어도(= 지도가 애매해서 틀린 자리도 점수가 잘 나와도)
     * 이 경로로 잡히므로, 품질 임계값만으로 못 잡던 구멍이 메워진다. */
    bool reloc_asked = (nav_slam->relocalize_request > 0);
    if (reloc_asked) {
        printf("[main] 스캔매칭이 %.0fmm 교정을 요청했습니다 "
               "(한 스텝에 옮기기엔 너무 큼) - 멈추고 지도 전체에서 "
               "다시 찾습니다\n", nav_slam->reloc_req_move_m * 1000.0);
        fflush(stdout);
        nav_slam->relocalize_request = 0;
    }
    if ((st->lost_streak >= 3 || reloc_asked) && st->step - st->last_global_step > 25) {
        fpga_link_set_speed(fpga, 0, 0);
        st->last_global_step = st->step;
        st->lost_streak = 0;
        if (robot_runner__relocalize_global(lidar, cfg, nav_slam,
                                             reloc_asked
                                               ? "스캔매칭이 큰 교정을 요청함"
                                               : "일치도가 3스텝 연속 80% 미만")) {
            double gx, gy, gth;
            slam_get_pose(nav_slam, &gx, &gy, &gth);
            if (dynnav__replan_from(nav, gx, gy)) {
                printf("[main] 재위치추정 후 경로 재계획 (%d 웨이포인트)\n",
                       nav->waypoints.count);
                printf("WAYPOINTS %d", nav->waypoints.count);
                for (int wi = 0; wi < nav->waypoints.count; wi++)
                    printf(" %.4f %.4f", nav->waypoints.points[wi].x,
                           nav->waypoints.points[wi].y);
                printf("\n");
                fflush(stdout);
            }
            st->stuck_count = 0; st->stuck_ref_init = false;
            st->prev_was_rotation = true;
            st->step++;
            return NAV_RETRY;
        }
    }
    return NAV_GO;
}


/* 직전 스텝이 회전이었다면, 실제로 얼마나 돌았는지로 각속도를 재추정. */
static void nav__update_rot_rate(const NavCtx *ctx, NavState *st, const NavStep *ns) {
    OccupancyGridSLAM *nav_slam = ctx->nav_slam;
    /* 직전 스텝이 회전이었다면, 실제로 얼마나 돌았는지로 각속도를 재추정 */
    if (st->sweep_pending) {
        double obs = fabs(normalize_angle(ns->theta - st->sweep_prev_theta));
        if (st->sweep_sec > 0.05 && obs > 5.0 * ANGLE_PI / 180.0) {
            double meas = obs / st->sweep_sec;
            st->rot_est->rate = st->rot_est->measured ? (0.6 * st->rot_est->rate + 0.4 * meas)
                                              : meas;
            rot_rate__clip(st->rot_est);
            st->rot_est->measured = true;
            printf("[main] 연속 회전 결과: %.0fdeg 돌았습니다 (명령 %.2fs) "
                   "-> 각속도 실측 %.0fdeg/s\n",
                   obs * 180.0 / ANGLE_PI, st->sweep_sec, meas * 180.0 / ANGLE_PI);
            fflush(stdout);
        }
        st->sweep_pending = false;
    }
    if (st->rot_pending) {
        /* 자세추정을 못 믿는 스텝에서는 배우지 않는다 (2026-08-10).
         * 관측값이 스캔매칭 결과인데 그게 틀렸으면, 틀린 각속도를 배우고
         * 그 각속도로 다음 예측을 만들어 더 틀리는 되먹임이 생긴다. */
        /* ============================================================
         * 명령은 나갔는데 실제로 안 돈 스텝 (2026-08-10p)
         *
         * 사용자 신고: "탁탁 소리만 나고 전혀 회전을 못한다."
         * slot_align 에는 2026-08-09k 에 같은 처리가 들어갔는데
         * navigate_one_leg 에는 없어서, 여기서는 짧은 펄스를 영원히
         * 반복하기만 했다. 안 돌면 다음 펄스를 1.6배씩 늘린다.
         * ============================================================ */
        double rot_obs = fabs(normalize_angle(ns->theta - st->rot_prev_theta));
        if (rot_obs < 0.005) {                       /* 0.3도 미만 = 안 돔 */
            st->rot_no_motion++;
            if (st->rot_kick_gain < 3.0) st->rot_kick_gain *= 1.6;
            if (st->rot_no_motion == 2) {
                printf("[main] 회전 명령이 나갔는데 자세가 안 변합니다 "
                       "(%.0fms 펄스) - 정지마찰로 보고 펄스를 %.1f배로 늘립니다\n",
                       st->rot_prev_sec * 1000.0, st->rot_kick_gain);
                fflush(stdout);
            }
        } else if (rot_obs > 0.017) {                /* 1도 넘게 돌았다 */
            st->rot_no_motion = 0;
            st->rot_kick_gain = 1.0;
        }
        if (nav_slam->bad_match_streak == 0
            && nav_slam->last_match_quality > 0.60) {
            rot_rate_update(st->rot_est, normalize_angle(ns->theta - st->rot_prev_theta),
                             ns->odom_dth, st->rot_prev_sec,
                             nav_slam->search_window_theta, st->rot_prev_dir);
        } else {
            /* 매칭이 깨진 회전 스텝 - 학습 대신 자가회복 (2026-08-10n).
             * A->B 통로 중간에서 '맵이 한 번 깨지고 정체' 하던 구간이
             * 여기서 스스로 풀린다. */
            rot_rate_on_bad_match(st->rot_est, st->rot_prev_sec,
                                   nav_slam->search_window_theta);
        }
        st->rot_pending = false;
    }
}


static NavAction nav__watch_stuck(const NavCtx *ctx, NavState *st, const NavStep *ns) {
    FPGALink          *fpga     = ctx->fpga;
    LidarThread       *lidar    = ctx->lidar;
    DynamicNavigator  *nav      = ctx->nav;
    OccupancyGridSLAM *nav_slam = ctx->nav_slam;
    const RobotConfig *cfg      = ctx->cfg;
/* PATCH (2026-08-07): 정체(끼임) 감지.
 * 실기에서 자세추정이 (1.310,0.735,2.73)에 215스텝(30초 이상) 붙박이인 채
 * 계속 바퀴를 돌린 사례가 있었음 - 차가 벽에 물려서 못 움직이는데 제어기는
 * 같은 명령을 무한 반복함. 폼보드 세트장은 이걸 30초 견디지 못하므로
 * 코드가 먼저 끊어야 함. slot_drive에는 이미 stall 판정이 있는데
 * 내비게이션 쪽에만 없었음. */

    if (!st->stuck_ref_init) {
        st->stuck_ref_x = ns->x; st->stuck_ref_y = ns->y; st->stuck_ref_theta = ns->theta;
        g_nav_stuck_ref_scan = ns->scan;      /* 원시 스캔도 같이 기준으로 잡는다 */
        st->stuck_ref_init = true;
    }
    double moved = hypot(ns->x - st->stuck_ref_x, ns->y - st->stuck_ref_y);
    double turned = fabs(normalize_angle(ns->theta - st->stuck_ref_theta));
    if (moved >= NAV_STUCK_EPS_M || turned >= NAV_STUCK_EPS_RAD)
        st->blind_steps = 0;        /* 자세가 움직였으면 관측 불가 구간이 끝난 것 */
    if (moved < NAV_STUCK_EPS_M && turned < NAV_STUCK_EPS_RAD) {
        st->stuck_count++;
        /* PATCH (2026-08-08): 제자리 회전 중 차체가 벽에 걸리면 아무리 돌려도
         * 안 움직임. 통로 높이가 40cm이고 차체 대각 반경이 14.3cm라 여유가
         * ±5cm뿐이라서, 대기점 y가 조금만 어긋나도 회전 중 앞뒤 모서리가
         * 벽에 닿음. 이럴 땐 속도를 올려봐야 더 세게 밀 뿐이고,
         * '살짝 물러났다가 다시 돌기'가 정답임(사람이 주차할 때 하는 것과 같음).
         * 바로 포기하지 않고 짧은 후진/전진으로 몸을 뺀 뒤 재시도함. */
        if (st->stuck_count >= NAV_STUCK_ESCAPE_AT && st->escape_tries < NAV_STUCK_MAX_ESCAPES) {
            /* ============================================================
             * '진짜 물렸는지'를 라이다로 먼저 확인한다 (2026-08-10c)
             *
             * *** 사용자 신고 "벽을 너무 예민하게 인식하고 후진 몇 번 반복" ***
             * 실기 로그:
             *   [main] 끼임 탈출 시작: 막힌 쪽 +83deg(141mm), 정면 0.31m / 후면 1.07m
             *          -> 후진 한 방향으로만 45mm 씩 (필요 여유 150mm)
             * 이게 6번 반복됐다. 그런데 정면이 310mm 나 비어 있다. 즉 차체는
             * 전혀 막혀 있지 않았고, 후진할 이유도 없었다.
             *
             * 왜 그랬나: 탈출의 성공 기준이 '사방(전방향) 최소거리 >=
             * robot_radius + 0.01 = 150mm' 였다. 그런데 이 통로는 높이가
             * 400mm 라 사방 최소가 최대로 잘해야 200mm(정중앙)이고, 150mm 를
             * 만족하려면 중심선에서 +-50mm 안(y 0.598~0.698)에 있어야 한다.
             * 로그의 y=0.695 는 딱 그 경계다. 게다가 통로 축(x)으로 후진해봐야
             * 위아래 벽까지 거리는 1mm 도 안 변한다 - 이 코드의 주석이 이미
             * '무한 왕복'이라고 적어둔 바로 그 조합이다.
             *
             * 사방 최소거리는 '제자리 회전'에나 필요한 값이지, 직진에는
             * 차폭(130mm)만 있으면 된다. 그래서 직진 스텝의 정체에는
             * 직사각형 스윕(axis_free_dist)으로 '갈 길이 막혔나'만 본다.
             * 안 막혔으면 후진하지 않고, 대신 이동 펄스를 키운다
             * (실제 원인은 대개 '한 스텝 이동량이 너무 작다'이기 때문).
             * ============================================================ */
            double blk_d = 0.0;
            double blk_dir = robot_runner__min_clearance_dir(&ns->scan, &blk_d);
            double free_f = robot_runner__axis_free_dist(&ns->scan, +1,
                                0.5 * cfg->body_length, 0.5 * cfg->body_width,
                                CORNER_SIDE_MARGIN_M);
            double free_b = robot_runner__axis_free_dist(&ns->scan, -1,
                                0.5 * cfg->body_length, 0.5 * cfg->body_width,
                                CORNER_SIDE_MARGIN_M);
            const double NAV_STUCK_FREE_M = 0.060;
            if (free_f >= NAV_STUCK_FREE_M || free_b >= NAV_STUCK_FREE_M) {
                /* ============================================================
                 * '바퀴가 안 도는가' vs '자세추정만 멈췄는가' (2026-08-10e)
                 *
                 * 앞 버전은 무조건 '안 도는 것'으로 보고 펄스를 키웠다.
                 * 실기에서는 정반대였다 - 로봇은 잘 달리고 있었고 자세추정만
                 * 36스텝 동안 (0.217,0.552)에 붙박여 있었다. 그래서 펄스를
                 * 3.4배까지 키운 결과 '눈 감고 60cm 를 더 달린' 꼴이 됐다.
                 *
                 * 원시 스캔은 자세추정과 무관하므로 이 둘을 확실히 가른다.
                 * ============================================================ */
                double chg = robot_runner__scan_change(&g_nav_stuck_ref_scan, &ns->scan);
                if (chg > 0.020) {
                    printf("[main] 자세는 %d스텝째 제자리인데 라이다 화면은 "
                           "평균 %.0fmm 나 바뀌었습니다.\n"
                           "       => 바퀴가 안 도는 게 아니라 '자세추정이 "
                           "멈춘' 것입니다. 더 달리지 않고 위치를 다시 잡습니다.\n"
                           "       (그동안 명령한 이동 %.0fmm - 이 값이 재탐색 "
                           "허용범위가 됩니다)\n",
                           st->stuck_count, chg * 1000.0,
                           (*robot_runner__travel_acc()) * 1000.0);
                    fflush(stdout);
                    fpga_link_set_speed(fpga, 0, 0);
                    st->blind_steps = 0;
                    if (robot_runner__relocalize_global(lidar, cfg, nav_slam,
                            "자세는 고정인데 스캔은 계속 바뀜")) {
                        double gx, gy, gth;
                        slam_get_pose(nav_slam, &gx, &gy, &gth);
                        if (dynnav__replan_from(nav, gx, gy)) {
                            printf("[main] 재위치추정 후 경로 재계획 (%d 웨이포인트)\n",
                                   nav->waypoints.count);
                            printf("WAYPOINTS %d", nav->waypoints.count);
                            for (int wi = 0; wi < nav->waypoints.count; wi++)
                                printf(" %.4f %.4f", nav->waypoints.points[wi].x,
                                       nav->waypoints.points[wi].y);
                            printf("\n");
                            fflush(stdout);
                        }
                        robot_runner__init_prev_pose(fpga, cfg->has_encoder,
                                ctx->prev_raw_x, ctx->prev_raw_y, ctx->prev_raw_theta);
                    }
                    st->stuck_count = 0;
                    st->stuck_ref_init = false;
                    st->step++;
                    return NAV_RETRY;
                }
                /* ============================================================
                 * 여기까지 왔다 = 자세도 스캔도 안 변했는데 갈 길은 뚫려 있다.
                 *
                 * 예전에는 이걸 "정말로 안 굴러가는 중"으로 단정하고 펄스를
                 * 1.5~3배로 키웠다. 그런데 이 통로는 벽이 진행방향과 나란해서,
                 * 실제로 굴러가도 스캔이 거의 안 바뀌고 스캔매칭 점수도
                 * 진행방향으로 평평하다 - 즉 '안 가는 것'이 아니라
                 * '진행방향을 관측할 수 없는 것'이다 (2026-08-10h,
                 * 위 nav_kick_gain 제거 주석의 로그 참고).
                 *
                 * 앞이 %.0fmm 나 비어 있는데 차가 물릴 수는 없으므로,
                 * 여기서는 아무 개입도 하지 않고 그대로 주행을 계속한다.
                 * 다만 오래 이어지면 '눈 감고 달리는' 상태가 되므로,
                 * 몇 스텝마다 한 번 위치를 다시 재서 눈을 뜬다.
                 * ============================================================ */
                st->blind_steps++;
                (void)blk_d; (void)blk_dir;
                if (st->blind_steps >= NAV_BLIND_RELOC_AT) {
                    printf("[main] 진행방향이 관측되지 않는 구간입니다 "
                           "(자세 고정 %d스텝, 스캔 변화 %.0fmm, "
                           "앞 %.0fmm / 뒤 %.0fmm 로 막히지 않음).\n"
                           "       주행은 그대로 두고 위치만 한 번 다시 잡습니다.\n",
                           st->stuck_count, (chg >= 0.0) ? chg * 1000.0 : 0.0,
                           free_f * 1000.0, free_b * 1000.0);
                    fflush(stdout);
                    st->blind_steps = 0;
                    fpga_link_set_speed(fpga, 0, 0);
                    if (robot_runner__relocalize_global(lidar, cfg, nav_slam,
                            "진행방향 관측 불가 구간")) {
                        double gx, gy, gth;
                        slam_get_pose(nav_slam, &gx, &gy, &gth);
                        if (dynnav__replan_from(nav, gx, gy)) {
                            printf("[main] 재위치추정 후 경로 재계획 (%d 웨이포인트)\n",
                                   nav->waypoints.count);
                            printf("WAYPOINTS %d", nav->waypoints.count);
                            for (int wi = 0; wi < nav->waypoints.count; wi++)
                                printf(" %.4f %.4f", nav->waypoints.points[wi].x,
                                       nav->waypoints.points[wi].y);
                            printf("\n");
                            fflush(stdout);
                        }
                        robot_runner__init_prev_pose(fpga, cfg->has_encoder,
                                ctx->prev_raw_x, ctx->prev_raw_y, ctx->prev_raw_theta);
                    }
                }
                /* 펄스는 건드리지 않는다 - 이동시간은 늘 정상값 그대로 */
                st->stuck_count = 0;
                st->stuck_ref_init = false;
                st->step++;
                return NAV_RETRY;
            }
            st->escape_tries++;
            printf("[main] 끼임 감지 - 탈출 시도 %d/%d "
                   "(직진 가능 앞 %.0f / 뒤 %.0fmm 로 실제로 막힘)\n",
                   st->escape_tries, NAV_STUCK_MAX_ESCAPES,
                   free_f * 1000.0, free_b * 1000.0);
            fflush(stdout);
            fpga_cmd_track_reset();
            /* 필요 여유를 '직진에 필요한 값'으로 준다 (2026-08-10c).
             * 예전의 0.0 은 robot_radius+0.01=150mm(제자리 회전 기준)로
             * 해석돼, 통로에서는 원리적으로 만족할 수 없는 목표였다. */
            double need_run = 0.5 * cfg->body_width + 0.020;
            bool esc_ok = robot_runner__escape_to_open(fpga, lidar, cfg, 0.0,
                                                        need_run, false);
            if (!esc_ok) {
                /* PATCH (2026-08-10b): 직선으로 못 빠지면 구석에 물린 것이다.
                 * 포기하기 전에 '구석 반대쪽으로 크게 돌기'를 한 번 쓴다.
                 * 회전 기준각은 지금 경로가 요구하는 방향각 오차로 준다 -
                 * 그래야 탈출과 동시에 가야 할 쪽으로 몸이 향한다. */
                double moved_c = 0.0;
                esc_ok = robot_runner__corner_escape(
                             fpga, lidar, cfg,
                             nav->local_ctrl.last_angle_error,
                             CORNER_ROT_MARGIN_M, "주행 중 끼임", &moved_c);
            }
            /* 성공이든 실패든 몸은 움직였으므로 자세를 먼저 다시 잡는다
             * (2026-08-10). 이게 없으면 '탈출 전 자세'로 경로를 그려서
             * 방금 빠져나온 자리로 되돌아간다. */
            robot_runner__relocalize_after_move(lidar, cfg, nav_slam);
            if (!esc_ok) {
                st->escape_tries = NAV_STUCK_MAX_ESCAPES;   /* 앞뒤 다 막힘 - 포기 */
            } else {
                /* 탈출했으면 경로도 다시 그린다 - 안 그리면 방금 걸린
                 * 그 자리로 곧장 되돌아가서 같은 일이 반복됨 */
                double ex, ey, eth;
                slam_get_pose(nav_slam, &ex, &ey, &eth);
                if (dynnav__replan_from(nav, ex, ey)) {
                    printf("[main] 탈출 후 경로 재계획 완료 (%d 웨이포인트)\n",
                           nav->waypoints.count);
                    printf("WAYPOINTS %d", nav->waypoints.count);
                    for (int wi = 0; wi < nav->waypoints.count; wi++)
                        printf(" %.4f %.4f", nav->waypoints.points[wi].x,
                               nav->waypoints.points[wi].y);
                    printf("\n");
                    fflush(stdout);
                }
            }
            st->stuck_count = 0;
            st->stuck_ref_init = false;
            return NAV_RETRY;      /* 다음 스텝에서 다시 관측 */
        }
        if (st->stuck_count >= NAV_STUCK_STEPS) {
            fpga_link_set_speed(fpga, 0, 0);
            printf("[main] *** 정체 감지: %d스텝 동안 자세가 "
                   "(%.3f,%.3f,%.1fdeg)에서 거의 안 변했습니다 ***\n",
                   st->stuck_count, ns->x, ns->y, ns->theta * 180.0 / ANGLE_PI);
            printf("[main] 차가 벽에 물렸거나 바퀴가 헛도는 상태입니다. "
                   "세트장 보호를 위해 정지합니다.\n");
            printf("[main] 확인할 것: (1) 모터 극성 - 양수 명령에 전진하는지 "
                   "(./motor_test), (2) 엔코더 부호 - 전진 시 좌우 틱이 "
                   "같은 부호로 증가하는지, (3) 차체가 물리적으로 끼었는지\n");
            fflush(stdout);
            return NAV_FAILED;
        }
    } else {
        /* 5cm 이상 제대로 움직였으면 '풀렸다'로 보고 탈출 기회를 되살림.
         * 한 구간에서 서로 다른 지점에 여러 번 끼일 수 있기 때문. */
        if (moved > 0.05 && st->escape_tries > 0) st->escape_tries--;
        st->stuck_count = 0;
        st->stuck_ref_x = ns->x; st->stuck_ref_y = ns->y; st->stuck_ref_theta = ns->theta;
    }
    return NAV_GO;
}


/* dynnav 가 낸 명령을 실제로 내보낼 좌우 값과 이동시간으로 환산한다. */
static NavAction nav__scale_command(const NavCtx *ctx, NavState *st,
                                    const NavStep *ns, NavCmdPlan *nc) {
    DynamicNavigator *nav = ctx->nav;
    const RobotConfig *cfg = ctx->cfg;
    const WheelCmd cmd = nc->cmd;
    /* 5) 속도는 고정, 이동'시간'으로 양을 조절해서 짧게 확실히 이동.
     * 좌우 비율은 그대로 유지해야 회전반경(직진/회전 성격)이 안 바뀌므로,
     * 두 바퀴를 같은 배율로 스케일함. */
    nc->m = abs(cmd.left) > abs(cmd.right) ? abs(cmd.left) : abs(cmd.right);
    if (nc->m <= 0) { st->step++; return NAV_RETRY; }

    /* PATCH (2026-08-09m): 구간별 목표 속도.
     * 예전에는 무엇을 하든 봉우리 속도를 sg_step_speed(34)로 맞췄다.
     * 통로 연속주행은 그보다 느려도 되고(경로 이탈이 줄어든다),
     * 마지막 대기점 접근은 보폭이 커서 목표를 넘나드는 게 문제였다. */
    int peak = cfg->sg_step_speed;
    if (nav->docking_mode && cfg->dock_step_speed > 0) {
        peak = cfg->dock_step_speed;
    } else if (((long)cmd.left * (long)cmd.right) >= 0 && cfg->cruise_speed > 0) {
        peak = cfg->cruise_speed;
    }
    if (peak < cfg->min_arc_speed + 4) peak = cfg->min_arc_speed + 4;
    double scale = (double)peak / nc->m;
    nc->out_left  = (int)lround(cmd.left  * scale);
    nc->out_right = (int)lround(cmd.right * scale);

    /* BUGFIX (2026-08-09e -> 2026-08-09j 강화): 곡선 주행의 안쪽 바퀴가
     * '데드존 언저리'로 떨어지는 것을 막는다.
     *
     * 예전에는 0으로 반올림되는 것만 막아 최소 1(=PWM 171)을 보장했는데,
     * PWM 171 은 데드존 170 을 1 넘긴 값이라 실제로는 거의 안 돈다.
     * 실기 로그에서 cmd=(34,3)(=PWM 206/173)으로 4스텝 동안 5mm 만 움직이고
     * '끼임'으로 오판돼 게걸음 탈출까지 갔다.
     * 좌우 부호가 같을 때(=곡선 주행)만 안쪽 바퀴를 min_arc_speed 까지 올린다.
     * 제자리 회전(부호가 갈릴 때)은 손대지 않는다. */
    if (cmd.left  != 0 && nc->out_left  == 0) nc->out_left  = (cmd.left  > 0) ? 1 : -1;
    if (cmd.right != 0 && nc->out_right == 0) nc->out_right = (cmd.right > 0) ? 1 : -1;
    if (cfg->min_arc_speed > 0 && nc->out_left != 0 && nc->out_right != 0
        && ((long)nc->out_left * (long)nc->out_right) > 0) {
        int lo = cfg->min_arc_speed;
        int outer = (abs(nc->out_left) > abs(nc->out_right)) ? abs(nc->out_left) : abs(nc->out_right);
        if (lo > outer) lo = outer;          /* 직진보다 빠를 수는 없음 */
        if (abs(nc->out_left)  < lo) nc->out_left  = (nc->out_left  > 0) ? lo : -lo;
        if (abs(nc->out_right) < lo) nc->out_right = (nc->out_right > 0) ? lo : -lo;
    }

    /* 원래 컨트롤러가 control_period 동안 명령했을 "이동량"(m*control_period)을
     * sg_step_speed로 같은 양 내려면 걸리는 시간. 너무 짧으면 정지마찰을 못
     * 뚫으므로 하한을 둠(그만큼은 의도보다 더 움직이지만, 매 사이클 다시
     * 측정해서 닫힌 루프로 수렴시킴). */
    nc->move_sec = (double)nc->m * ctx->control_period / peak;
    if (nc->move_sec < cfg->sg_move_sec_min) nc->move_sec = cfg->sg_move_sec_min;
    /* (2026-08-10h) nav_kick_gain 제거 - 위 선언부 주석 참고.
     * '안 움직이는 것처럼 보인다'는 이유로 이동시간을 키우면, 관측이 안 되는
     * 구간에서 눈을 감고 더 멀리 달리게 된다. 이동시간은 항상 정상값으로 둔다. */
    if (nc->move_sec > cfg->sg_move_sec_max) nc->move_sec = cfg->sg_move_sec_max;


    printf("[main] step=%d pose=(%.3f,%.3f,%.2f) cmd=(%d,%d)->(%d,%d) move=%.0fms wp=%d/%d\n",
           st->step, ns->x, ns->y, ns->theta, cmd.left, cmd.right, nc->out_left, nc->out_right,
           nc->move_sec * 1000.0, nav->current_wp_idx, nav->waypoints.count);
    fflush(stdout);

    /* 좌우 부호가 반대 = 제자리 회전 -> 다음 스텝은 정지 관측이 필요함 */
    nc->rotation_dominant = ((long)nc->out_left * (long)nc->out_right) < 0;
    return NAV_GO;
}


/* 이번 스텝의 이동시간과 '실제로 돌 각도'를 확정한다.
 * 회전 가능 여부 판정이 이 각도에 달려 있어서 검사보다 먼저 계산한다. */
static void nav__plan_rotation(const NavCtx *ctx, NavState *st,
                               const NavStep *ns, NavCmdPlan *nc) {
    DynamicNavigator  *nav      = ctx->nav;
    OccupancyGridSLAM *nav_slam = ctx->nav_slam;
    const RobotConfig *cfg      = ctx->cfg;
    /* PATCH (2026-08-09f): 직진 스텝의 이동시간을 '남은 거리'로 잡는다.
     *
     * 왜 (= 사용자 신고 "경로 마지막 지점에서 엄청 느리게 툭툭툭"):
     * 위 식은 PID 출력 크기 m 을 이동량으로 환산하는데, 도킹 접근은
     * max_linear_speed=50 에서 포화하므로 m=50 -> 50*0.05/34 = 0.074초다.
     * 실제 이동거리로는 한 스텝에 약 10mm. 그런데 stop-and-go 한 사이클의
     * 고정비용은 안정화 0.30s + 새 스캔 0.2s + 스캔매칭 이라 0.6~0.9초다.
     * 즉 40cm 접근에 40스텝 x 0.7초 = 30초가 걸리고, 그 중 바퀴가 실제로
     * 도는 시간은 3초뿐이다 - 나머지는 전부 오버헤드였다.
     *
     * 남은 거리로 시간을 잡으면 한 스텝에 최대 SG_STRAIGHT_MAX_M 까지 가므로
     * 스텝 수가 6분의 1로 줄어든다. 목표를 지나치지 않도록 남은 거리의
     * 80%까지만 가고, 마지막 5cm(close_mode)는 예전대로 짧게 끊는다.
     * 거리->시간 환산은 실측 자가보정(lin_scale)이 들어간 함수를 쓴다. */
    if (!nc->rotation_dominant && nav->docking_mode
        && !nav->docking_ctrl.close_mode && !nav->docking_ctrl.position_reached) {
        /* 0.06 -> 0.035, 0.8 -> 0.5 (2026-08-09m).
         * 사용자 신고: "마지막 포인트로 갈 때 보폭이 너무 커서 전진/후진을
         * 반복한다". 한 스텝 60mm 는 도킹 허용오차(20mm)의 3배라 반드시
         * 넘어가고, 넘어가면 되돌아오느라 왕복이 생긴다.
         * 35mm 로 줄이고 남은 거리의 50%만 가면 기하급수적으로 수렴한다
         * (40cm -> 200 -> 100 -> 50 -> 25 -> 12mm, 넘어감 없음). */
        const double SG_STRAIGHT_MAX_M = 0.035;
        double rho = hypot(ns->x - nav->goal_x, ns->y - nav->goal_y);
        double want_m = 0.5 * rho;
        if (want_m > SG_STRAIGHT_MAX_M) want_m = SG_STRAIGHT_MAX_M;
        if (want_m > 0.008) {
            double t = robot_runner__move_sec_for(cfg, cfg->sg_step_speed,
                                                  want_m, cfg->sg_move_sec_max);
            if (t > nc->move_sec) nc->move_sec = t;
        }
    }

    /* PATCH (2026-08-08c): 회전 스텝의 '이동시간'을 PID 출력 크기가 아니라
     * 스캔매칭 탐색창이 허용하는 한계에서 역산한다.
     *
     * 문제: 위 move_sec = m * control_period / sg_step_speed 에서, 제자리 회전이면
     * m = |w| <= max_angular_speed = 30 (goal_pid_init_default). 그래서
     *     move_sec = 30 * 0.05 / 34 = 0.044s -> sg_move_sec_min(0.07)로 바닥
     *     회전량   = 0.07 * 50.2deg/s = 3.5도/스텝
     * 즉 클램프가 허용하는 10.8도(--no-encoder 기준)의 1/3만 쓰고 있었음.
     * 그런데 스텝당 고정비용은 안정화 0.12s + 새 스캔 2바퀴 0.2s + 스캔매칭
     * 0.07~0.5s 라서, 90도 회전에 26스텝 x 0.4~0.9s = 10~23초가 걸렸음.
     * 실제 바퀴가 도는 시간은 그 중 1.8초뿐 - 나머지는 전부 오버헤드.
     *
     * 해결: 회전 스텝은 클램프 한계까지 길게 가져가되, PID 출력비(m/max_angular)
     * 를 곱해 목표 근처에서는 자동으로 짧아지게 함(오버슈트 방지). 큰 오차에서
     * 10.8도, 마무리에서 3.5도가 되므로 스텝 수가 약 1/3로 줄어듦.
     * 덤으로 정지마찰 손실도 줄어듦 - 0.07s는 킥(20ms)+유지(50ms)라 바퀴가
     * 떨어지자마자 끊기는 길이여서, 명령한 3.5도 중 실제로는 훨씬 덜 돌았음. */
    /* ============================================================
     * 제자리 회전 전 '사방 여유' 확인 (2026-08-08f)
     *
     * 통로 높이가 39.5cm이고 차체 회전반경(반대각선)이 14.3cm라, 중심이
     * 통로 정중앙에서 5.4cm만 벗어나도 제자리 회전 중 모서리가 벽에 닿는다.
     * 그 상태에서 회전을 명령하면 바퀴만 헛돌면서 차체를 벽에 밀어붙이고,
     * 폼보드 세트장에는 그게 제일 위험하다.
     * 그래서 돌기 전에 라이다로 사방 최소거리를 재서, 돌 자리가 아니면
     * 먼저 열린 쪽으로 빠져나온 뒤 다시 관측한다.
     * (--robot-radius 를 회전반경 기준으로 주면 이 판정이 그대로 맞는다) ==*/
    /* PATCH (2026-08-09f): 회전 스텝의 이동시간을 '검사보다 먼저' 확정한다.
     * 회전 가능 여부는 '이번 스텝에 실제로 돌 각도'에 달려 있는데, 예전에는
     * 검사가 먼저였고 그 각도를 몰라서 360도 기준(회전반경)으로 판정했다. */
    nc->rot_step_rad = 0.0;
    nc->want0 = cfg->sg_move_sec_min;   /* 안전검사와 명령이 공유하는 회전시간 */
    if (nc->rotation_dominant) {
        double rate0 = rot_rate_get(st->rot_est);
        double cap0 = robot_runner__clamp_rotation_sec_rate(cfg, nav_slam, rate0,
                                                            cfg->sg_move_sec_max);
        double ang_max0 = nav->goal_ctrl.max_angular_speed;
        double frac0 = (ang_max0 > 0.0) ? ((double)nc->m / ang_max0) : 1.0;
        if (frac0 > 1.0) frac0 = 1.0;
        if (frac0 < 0.0) frac0 = 0.0;
        /* ============================================================
         * 마무리 회전을 '남은 각도만큼' 한 번에 준다 (2026-08-10n)
         *
         * 사용자 신고: "A* 경로 마지막 포인트에서 엄청 작게 회전한다."
         *
         * 원인은 위 frac0 = m / max_angular_speed 다. goal_pid_init_default
         * 의 각도 게인은 kp=40, 상한 30 이므로 오차 10도(0.175rad)에서
         *     m = 40 * 0.175 = 7.0,  frac0 = 7.0/30 = 0.23
         *     want0 = 0.278s * 0.23 = 0.064s -> 하한 0.07s
         *     회전량 = 63deg/s * 0.07 = 4.4도
         * 10도를 지우는 데 4.4도씩 3스텝이 든다. 그런데 한 스텝의 고정비용은
         * 안정화 0.12s + 새 스캔 2바퀴 0.2s + 스캔매칭 이라 0.35~0.5s 다.
         * 실제로 바퀴가 도는 시간은 0.07s 뿐이고 나머지는 전부 오버헤드다.
         *
         * 속도를 올리는 건 답이 아니다 - 지나치면 되돌아오느라 스텝이 더 는다.
         * 줄여야 하는 건 '스텝 수' 이고, 그러려면 한 스텝에 필요한 각도를
         * 다 돌면 된다. slot_align 이 이미 이 방식(sec = |오차|/각속도)으로
         * 잘 돌고 있으므로 여기도 같게 맞춘다.
         *
         * 안전장치 두 가지는 그대로다.
         *   - cap0 (스캔매칭 탐색창의 70%) 를 절대 넘지 않는다. 즉 이 변경으로
         *     한 스텝 회전량이 창 밖으로 나가는 일은 구조적으로 없다.
         *   - 예전 값(cap0*frac0)을 하한으로 깔아서, 각도오차를 못 읽는
         *     경우에도 최소한 예전과 같게 동작한다.
         * ============================================================ */
        double ang_err0 = fabs(nav->local_ctrl.last_angle_error);
        if (nav->docking_mode && nav->has_goal_theta) {
            /* 도킹 마무리의 기준은 경로 접선이 아니라 목표 자세각이다 */
            ang_err0 = fabs(normalize_angle(ns->theta - nav->goal_theta));
        }
        nc->want0 = cap0 * frac0;
        double want_ang = ang_err0 / rate0;
        if (want_ang > nc->want0) nc->want0 = want_ang;      /* 필요한 만큼까지 늘림 */
        if (nc->want0 > cap0) nc->want0 = cap0;              /* 창 제한이 최종 상한 */
        if (nc->want0 < cfg->sg_move_sec_min) nc->want0 = cfg->sg_move_sec_min;
        if (nc->want0 > cfg->sg_move_sec_max) nc->want0 = cfg->sg_move_sec_max;
        /* 1.5 -> 1.15 (2026-08-10). 예전에는 '다음 스텝 몫까지 미리 확보'한다며
         * 실제로 돌 각도의 1.5배로 안전검사를 했다. 그래서 26.2도만 돌면 되는데
         * 39도 기준(=141mm)으로 판정해 3mm 차이로 탈출을 불렀다(로그 확인).
         * 이제는 실제 스텝 + 15% 여유로만 검사하고, 그래도 모자라면
         * 아래에서 '돌 수 있는 만큼만' 돌기 때문에 안전은 그대로 유지된다. */
        nc->rot_step_rad = rate0 * nc->want0 * 1.15;
        if (nc->out_right < nc->out_left) nc->rot_step_rad = -nc->rot_step_rad;   /* 시계 */
    }
}


static NavAction nav__try_sweep(const NavCtx *ctx, NavState *st,
                                const NavStep *ns, const NavCmdPlan *nc) {
    FPGALink *fpga = ctx->fpga;
    DynamicNavigator *nav = ctx->nav;
    const RobotConfig *cfg = ctx->cfg;
    /* PATCH (2026-08-09j): 큰 제자리 회전은 끊지 말고 한 번에 쭉 돌린다.
     * 레그를 시작할 때 슬롯을 보던 자세(-90도)에서 통로 방향(0/180도)으로
     * 90도를 돌아야 하는데, 예전에는 이걸 6~11스텝으로 쪼개서 "툭.툭." 돌았다.
     * 스윕 중에는 스캔을 안 쓰고, 끝난 뒤 회전량을 오도메트리 예측으로
     * 넘겨서 매칭이 따라오게 한다. */
    if (nc->rotation_dominant && !nav->docking_mode) {
        double ae = nav->local_ctrl.last_angle_error;
        /* 각속도 실측이 아직 없으면 스윕은 어차피 거부된다. 같은 안내를
         * 매 스텝 찍어 로그를 덮지 않도록 한 번만 시도한다 (2026-08-10p). */
        bool sweep_ready = (st->rot_est->measured
                            && st->rot_est->n_update >= SWEEP_MIN_UPDATES);
        if (!sweep_ready && st->sweep_block_warned) {
            /* 조용히 건너뜀 - 끊어서 도는 경로로 진행 */
        } else if (fabs(ae) * 180.0 / ANGLE_PI >= SWEEP_MIN_DEG) {
            if (!sweep_ready) st->sweep_block_warned = true;
            double pred = 0.0, ssec = 0.0;
            if (robot_runner__sweep_rotate(fpga, cfg, &ns->scan, st->rot_est, ae,
                                            &pred, &ssec)) {
                (void)pred;   /* 회전분은 명령 적분기가 이미 반영한다 */
                st->sweep_prev_theta = ns->theta;
                st->sweep_sec = ssec;
                st->sweep_pending = true;
                st->rot_pending = false;
                st->prev_was_rotation = true;
                st->step++;
                return NAV_RETRY;
            }
        }
    }
    return NAV_GO;
}


static NavAction nav__rotation_clearance(const NavCtx *ctx, NavState *st,
                                         const NavStep *ns, NavCmdPlan *nc) {
    FPGALink          *fpga     = ctx->fpga;
    LidarThread       *lidar    = ctx->lidar;
    DynamicNavigator  *nav      = ctx->nav;
    OccupancyGridSLAM *nav_slam = ctx->nav_slam;
    const RobotConfig *cfg      = ctx->cfg;
    /* ============================================================
     * 회전 여유 판정 (2026-08-10 개정) - 1번 문제의 직접 수정
     *
     * 예전: 안전하면 돌고, 아니면 곧바로 앞뒤 탈출(전부-아니면-전무).
     *       3mm 모자란 것만으로도 120mm 를 밀어댔다.
     * 지금: 세 단계로 낮춰간다.
     *   1) 요청한 각도가 안전하면 그대로 돈다.
     *   2) 아니면 '안전하게 돌 수 있는 만큼'을 찾아 그만큼만 돈다.
     *      (사용자 제안: "각도를 틀어서 이동하면 되지 않나?" - 이것이 그 구현)
     *   3) 최소 각도조차 못 돌 때만 한 방향으로 천천히 빠져나온다.
     *      그리고 한 번 탈출하면 쿨다운 동안은 다시 탈출하지 않는다
     *      - 이 쿨다운이 앞뒤 왕복을 구조적으로 불가능하게 만든다.
     * ============================================================ */
    nc->rot_tight_space = false;   /* 여유가 모자란 채 도는 중인가 */
    nc->rot_allow_frac = 1.0;    /* 이번 스텝에 실제로 허용된 회전 비율 */
    if (nc->rotation_dominant) {
        double gap = 0.0, gdir = 0.0, need = 0.0;
        bool ok = robot_runner__rotation_is_safe(&ns->scan, nc->rot_step_rad,
                                                 0.5 * cfg->body_length,
                                                 0.5 * cfg->body_width,
                                                 0.015, &gap, &gdir, &need);
        double minc = need + gap;   /* 그 방향의 실제 거리 */

        if (!ok) {
            /* --- 2) 돌 수 있는 만큼만 --- */
            double frac = robot_runner__max_safe_rotation_frac(
                              &ns->scan, nc->rot_step_rad, 0.5 * cfg->body_length,
                              0.5 * cfg->body_width, 0.015);
            double full_deg = fabs(nc->rot_step_rad) * 180.0 / ANGLE_PI;
            double allow_deg = full_deg * frac;
            /* ============================================================
             * '조금씩 틀기'는 하드웨어가 낼 수 있는 각도일 때만 (2026-08-10p)
             *
             * *** 사용자 신고 "탁탁 소리만 나고 전혀 회전을 못한다" 의 정체 ***
             * 실기 로그 (pose=(1.300,0.665,232deg), 요청 -49.4deg):
             *     회전 여유가 빠듯해(+122deg 방향 149mm) 이번 스텝은 6.2deg 만 돕니다
             *     [main] step=1..17  move=70ms   라이다 관측 +0.0deg  (계속)
             * 6.2도를 돌라는 명령의 실제 펄스는
             *     want = want0 * frac -> 57ms -> 하한 sg_move_sec_min(70ms)
             * 인데, 이 차는 70ms 로는 정지마찰을 못 넘는다(88ms 는 넘었다).
             * 즉 '돌 수 있는 만큼만 돈다'가 실제로는 '한 발도 안 돈다'였다.
             *
             * 게다가 안 도니까
             *   - 각속도 실측이 0회로 남아 연속 회전(스윕)도 영구 차단되고
             *   - 자세가 안 변하니 기하도 그대로여서
             * 같은 판정이 무한 반복된다(로그의 17스텝이 전부 동일).
             *
             * 그래서 기준을 고정 4도가 아니라 '이 하드웨어가 실제로 만들 수
             * 있는 최소 회전량'으로 바꾼다. 그보다 작으면 헛되이 모터를 치지
             * 말고 아래 탈출로 내려가서 자리를 만든다 - 지도로 검산하면
             * 이 자세에서 전진 30mm 면 여유가 152 -> 170mm 가 되어 49.4도를
             * 통째로 돌 수 있다. 한 번 물러나는 편이 압도적으로 빠르다.
             * ============================================================ */
            double brk = (cfg->rot_breakaway_sec > 0.0) ? cfg->rot_breakaway_sec
                                                        : cfg->sg_move_sec_min;
            double hw_min_deg = rot_rate_get(st->rot_est) * brk * 1.15 * 180.0 / ANGLE_PI;
            double useful_deg = (ROT_MIN_USEFUL_DEG > hw_min_deg)
                                ? ROT_MIN_USEFUL_DEG : hw_min_deg;
            if (allow_deg < useful_deg && allow_deg >= ROT_MIN_USEFUL_DEG) {
                printf("[main] 여유상 %.1fdeg 만 돌 수 있는데, 이 차는 %.0fms "
                       "이하 펄스로는 안 돕니다(=%.1fdeg 미만).\n"
                       "       헛돌리지 않고 먼저 자리를 만들겠습니다\n",
                       allow_deg, brk * 1000.0, useful_deg);
                fflush(stdout);
            }
            if (allow_deg >= useful_deg) {
                nc->rot_allow_frac = frac;
                ok = true;
                printf("[main] 회전 여유가 빠듯해(%+.0fdeg 방향 %.0fmm) 이번 스텝은 "
                       "%.1fdeg 만 돕니다 (원래 %.1fdeg) - 왕복 없이 조금씩 틉니다\n",
                       gdir, minc * 1000.0, allow_deg, full_deg);
                fflush(stdout);
            }
        }

        if (ok) {
            st->rot_clear_escapes = 0;   /* 여유가 회복됐으면 기회를 되살림 */
        } else if (st->rot_escape_cooldown > 0) {
            /* --- 방금 탈출했다: 다시 탈출하지 않는다(왕복 금지) --- */
            nc->rot_tight_space = true;
        } else if (st->rot_clear_escapes >= ROT_CLEAR_MAX_ESCAPES) {
            static bool warned_rot_clear = false;
            if (!warned_rot_clear) {
                warned_rot_clear = true;
                printf("[main] 회전 여유가 %+.0fdeg 방향 %.0fmm(필요 %.0fmm)로 계속 "
                       "부족하지만, 탈출을 %d번 해도 안 열려서\n"
                       "       앞뒤 왕복을 멈추고 천천히 회전합니다. "
                       "(--slot-staging 이 통로 정중앙인지, --body-size 가 "
                       "실제 차체 치수인지 확인하세요)\n",
                       gdir, minc * 1000.0, need * 1000.0, ROT_CLEAR_MAX_ESCAPES);
                fflush(stdout);
            }
            nc->rot_tight_space = true;
        }

        if (!ok && st->rot_escape_cooldown == 0
            && st->rot_clear_escapes < ROT_CLEAR_MAX_ESCAPES) {
            st->rot_clear_escapes++;
            st->rot_escape_cooldown = ROT_ESCAPE_COOLDOWN;
            printf("[main] 회전 여유 부족: %+.0fdeg 방향 %.0fmm < 필요 %.0fmm "
                   "(이번 스텝 %.1fdeg 회전 기준) - 한 방향으로 천천히 빠져나옵니다 (%d/%d)\n",
                   gdir, minc * 1000.0, need * 1000.0,
                   fabs(nc->rot_step_rad) * 180.0 / ANGLE_PI,
                   st->rot_clear_escapes, ROT_CLEAR_MAX_ESCAPES);
            fflush(stdout);
            fpga_link_set_speed(fpga, 0, 0);
            fpga_cmd_track_reset();
            bool escaped = robot_runner__escape_to_open(fpga, lidar, cfg, 0.0,
                                                         need + 0.015, cfg->allow_crab);
            /* 성공/실패와 무관하게 몸은 움직였다 - 자세를 먼저 다시 잡는다.
             * (2026-08-10) 예전에는 이 단계가 없어서, 탈출 전 자세로 경로를
             * 다시 그렸고 그 경로가 방금 나온 자리로 되돌아가게 만들었다. */
            robot_runner__relocalize_after_move(lidar, cfg, nav_slam);
            if (escaped) {
                double ex, ey, eth;
                slam_get_pose(nav_slam, &ex, &ey, &eth);
                if (dynnav__replan_from(nav, ex, ey)) {
                    printf("[main] 탈출 후 경로 재계획 완료 (%d 웨이포인트)\n",
                           nav->waypoints.count);
                    printf("WAYPOINTS %d", nav->waypoints.count);
                    for (int wi = 0; wi < nav->waypoints.count; wi++)
                        printf(" %.4f %.4f", nav->waypoints.points[wi].x,
                               nav->waypoints.points[wi].y);
                    printf("\n");
                    fflush(stdout);
                }
                st->stuck_count = 0;
                st->stuck_ref_init = false;
                st->prev_was_rotation = true;
                st->step++;
                return NAV_RETRY;
            }
            /* 탈출이 목표 여유를 못 만들었어도 왕복은 하지 않는다.
             * 쿨다운이 걸려 있으므로 다음 스텝부터는 '돌 수 있는 만큼만'
             * 또는 '천천히' 회전으로 자연스럽게 풀린다. */
            nc->rot_tight_space = true;
            st->prev_was_rotation = true;
            st->step++;
            return NAV_RETRY;
        }
    }
    return NAV_GO;
}


/* 회전 스텝의 펄스 길이와 (안 돌면) 속도를 확정한다. */
static void nav__rotation_command(const NavCtx *ctx, NavState *st,
                                  const NavStep *ns, NavCmdPlan *nc) {
    OccupancyGridSLAM *nav_slam = ctx->nav_slam;
    const RobotConfig *cfg = ctx->cfg;
    if (nc->rotation_dominant) {
        double rate = rot_rate_get(st->rot_est);
        /* (2026-08-10n) 위 안전검사(rot_step_rad)가 쓴 want0 을 그대로 쓴다.
         * 예전에는 여기서 같은 식을 한 번 더 계산했는데, 두 곳이 조금이라도
         * 달라지면 '검사한 각도와 명령한 각도가 다른' 상태가 된다 -
         * slot_align 에서 구석 끼임을 만들었던 바로 그 버그다. */
        double want = nc->want0;
        /* 회전 펄스의 진짜 하한 = 정지마찰 돌파 시간 (2026-08-10p).
         * 이보다 짧으면 '탁' 소리만 나고 한 발도 안 돈다 - 명령을 낸 척만
         * 하는 스텝이라 시간만 버리고 각속도 실측도 못 쌓는다. */
        double brk_sec = (cfg->rot_breakaway_sec > cfg->sg_move_sec_min)
                         ? cfg->rot_breakaway_sec : cfg->sg_move_sec_min;
        /* 여유가 모자란 자리에서 도는 중이면 한 스텝을 최소로 - 스치더라도
         * 세게 밀지 않게 (2026-08-09e). 단 '안 도는 길이'로는 안 준다. */
        if (nc->rot_tight_space) want = brk_sec;
        /* '돌 수 있는 만큼만' 판정이 났으면 그 비율만큼 시간을 줄인다 (2026-08-10) */
        if (nc->rot_allow_frac < 1.0) {
            want *= nc->rot_allow_frac;
            if (want < brk_sec) want = brk_sec;
        }
        /* 안 돌고 있으면 펄스를 키운다 (2026-08-10p) */
        if (st->rot_kick_gain > 1.0) {
            want *= st->rot_kick_gain;
            if (want > cfg->sg_move_sec_max) want = cfg->sg_move_sec_max;
        }

        if (fabs(want - nc->move_sec) > 0.005) {
            printf("[main]   회전 스텝: %.0fms -> %.0fms (예상 %.1fdeg, "
                   "각속도 %.0fdeg/s%s, 탐색창 ±%.1fdeg)\n",
                   nc->move_sec * 1000.0, want * 1000.0,
                   want * rate * 180.0 / ANGLE_PI,
                   rate * 180.0 / ANGLE_PI,
                   st->rot_est->measured ? "-실측" : "-모델",
                   nav_slam->search_window_theta * 180.0 / ANGLE_PI);
            fflush(stdout);
        }
        /* 펄스를 키워도 안 돌면 속도까지 올린다 (2026-08-10p).
         * 제자리 회전은 차체 중심을 옮기지 않으므로, 속도를 올려도
         * 벽 쪽으로 더 밀고 들어가지는 않는다. */
        if (st->rot_no_motion >= 3) {
            double bo = 1.0 + 0.25 * (double)(st->rot_no_motion - 2);
            int cap = cfg->controller_max_speed;
            int nl = (int)lround(nc->out_left  * bo);
            int nr = (int)lround(nc->out_right * bo);
            if (nl >  cap) nl =  cap;
            if (nl < -cap) nl = -cap;
            if (nr >  cap) nr =  cap;
            if (nr < -cap) nr = -cap;
            if (nl != nc->out_left || nr != nc->out_right) {
                printf("[main] 아직도 안 돌아서 회전 속도를 (%d,%d) -> (%d,%d) 로 "
                       "올립니다\n", nc->out_left, nc->out_right, nl, nr);
                fflush(stdout);
                nc->out_left = nl; nc->out_right = nr;
            }
        }
        nc->move_sec = want;
        st->rot_prev_theta = ns->theta;
        st->rot_prev_sec = nc->move_sec;
        /* 왼바퀴가 더 빠르면 시계(theta 감소) - out_left/out_right 부호 규약 */
        st->rot_prev_dir = (nc->out_right > nc->out_left) ? +1.0 : -1.0;
        st->rot_pending = true;
    }
}


/* ---------- 연속 주행(stop-and-go 를 끈 경우) ----------
 * cfg->stop_and_go 가 false 일 때만 쓰는 단순 루프다. 실기 기본 설정은
 * stop_and_go=true 라 여기로 오지 않는다. 원본 뒷부분 그대로다. */
static bool nav__cruise_loop(const NavCtx *ctx, NavState *st) {
    FPGALink          *fpga     = ctx->fpga;
    LidarThread       *lidar    = ctx->lidar;
    DynamicNavigator  *nav      = ctx->nav;
    OccupancyGridSLAM *nav_slam = ctx->nav_slam;
    const RobotConfig *cfg      = ctx->cfg;
while (!g_stop_requested) {
    LidarScan scan;
    lidar_thread_get_latest(lidar, &scan);
    apply_lidar_frame_fix(&scan, cfg->lidar_mirror, cfg->lidar_yaw_offset_deg,
                          cfg->lidar_offset_forward_m);

    double x, y, theta;
    double odom_dx, odom_dy, odom_dth;
    robot_runner__predict_delta(fpga, cfg, nav_slam->theta, ctx->prev_raw_x, ctx->prev_raw_y, ctx->prev_raw_theta,
                                 &odom_dx, &odom_dy, &odom_dth);

    if (ctx->use_slam_rolling_map) {
        slam_update(nav_slam, &scan, odom_dx, odom_dy, odom_dth, &x, &y, &theta);
    } else {
        slam_localize_step(nav_slam, &scan, odom_dx, odom_dy, odom_dth, &x, &y, &theta);
    }

    ClusterResult cluster_result;
    cluster_lidar_scan(&scan, x, y, theta, 0.15, 2, 3.0, &cluster_result);
    DetectedObstacle detected[CLUSTER_MAX_CLUSTERS];
    int n_detected = cluster_result_to_detected(&cluster_result, detected, CLUSTER_MAX_CLUSTERS);

    WheelCmd cmd = dynnav_update(nav, x, y, theta, &scan,
                                  n_detected > 0 ? detected : NULL, n_detected);
    fpga_link_set_speed(fpga, cmd.left, cmd.right);

    printf("STATE navigate %d %.4f %.4f %.4f %d %d %d\n",
           st->step, x, y, theta, nav->current_wp_idx, nav->waypoints.count, cmd.done ? 1 : 0);
    printf("SCAN %d", scan.count);
    for (int i = 0; i < scan.count; i++) {
        double wa = theta + scan.readings[i].angle_deg * ANGLE_PI / 180.0;
        double d = scan.readings[i].dist_mm / 1000.0;
        printf(" %.3f %.3f", x + d * cos(wa), y + d * sin(wa));
    }
    printf("\n");
    fflush(stdout);

    if (st->step % (cfg->control_hz * 2) == 0) {
        long sent, recv, fail;
        fpga_link_get_stats(fpga, &sent, &recv, &fail);
        printf("[main] step=%d pose=(%.3f,%.3f,%.2f) L=%d R=%d wp=%d/%d "
               "lidar_scans=%ld lidar_err=%ld fpga_sent=%ld fpga_recv=%ld fpga_parse_fail=%ld\n",
               st->step, x, y, theta, cmd.left, cmd.right, nav->current_wp_idx, nav->waypoints.count,
               lidar->scan_count, lidar->error_count, sent, recv, fail);
    }

    if (cmd.done) {
        fpga_link_set_speed(fpga, 0, 0);
        if (nav->goal_unreachable) {   /* 위와 동일한 이유 - 연속주행 경로에도 적용 */
            printf("[main] *** 목적지 도달 실패: 경로가 막혔습니다. "
                   "현재 (%.3f,%.3f), 목표 (%.3f,%.3f) ***\n",
                   x, y, nav->goal_x, nav->goal_y);
            printf("[main] 안전을 위해 정지합니다. 장애물을 치우고 다시 실행하세요.\n");
            fflush(stdout);
            return false;
        }
        printf("[main] goal reached! step=%d, final pose=(%.3f,%.3f,%.2f)\n", st->step, x, y, theta);
        return true;
    }

    st->step++;
    struct timespec ts;
    ts.tv_sec = (time_t)ctx->control_period;
    ts.tv_nsec = (long)((ctx->control_period - ts.tv_sec) * 1e9);
    nanosleep(&ts, NULL);
}

fpga_link_set_speed(fpga, 0, 0);
return false;
}


/* ============================================================
 * navigate_one_leg: "현재 목표(nav에 이미 set_goal 되어있음) 하나에 도착할 때까지"
 * 제어루프를 도는 부분. run_robot()의 기존 단발성 주행과 run_shuttle()의 매 구간
 * 주행이 완전히 같은 로직이라 공용으로 뺌.
 * use_slam_rolling_map=true면 nav_slam이 계속 지도도 같이 갱신(SLAM 탐색후 내비게이션),
 * false면 위치추정만 함(고정맵/불러온맵 - slam_localize_step).
 * prev_raw_*는 오도메트리 델타 계산용으로 호출 사이에 계속 이어져야 해서 포인터로 받음.
 * 반환: 목표 도착하면 true, 중단신호(Ctrl+C) 받으면 false.
 * ============================================================ */
bool navigate_one_leg_inner(FPGALink *fpga, LidarThread *lidar, DynamicNavigator *nav,
                            OccupancyGridSLAM *nav_slam, const RobotConfig *cfg,
                            bool use_slam_rolling_map,
                            double *prev_raw_x, double *prev_raw_y, double *prev_raw_theta) {
    NavCtx ctx = { fpga, lidar, nav, nav_slam, cfg, use_slam_rolling_map,
                   prev_raw_x, prev_raw_y, prev_raw_theta, 1.0 / cfg->control_hz };
    NavState st;
    memset(&st, 0, sizeof(st));
    st.rot_kick_gain    = 1.0;
    st.last_global_step = -100;
    st.base_win_m   = nav_slam->search_window_m;
    st.base_step_m  = nav_slam->search_step_m;
    st.base_win_th  = nav_slam->search_window_theta;
    st.base_step_th = nav_slam->search_step_theta;

    /* ---------- Stop-and-go 주행 ----------
     * 멈춤 -> 깨끗한 스캔 -> 위치추정 -> 판단 -> 짧게 확실히 이동 -> 반복.
     * 자세한 이유는 RobotConfig.stop_and_go 주석 참고. */
    /* 회전 각속도 실측 추정기 - 모델이 틀려도 스텝이 탐색창을 못 넘게 자동 보정 */
    st.rot_est = rot_rate_shared(cfg, cfg->sg_step_speed);

    if (!cfg->stop_and_go) return nav__cruise_loop(&ctx, &st);

    /* 이동구간에는 듀티 사이클링(끊어보내기)을 끔 - 어차피 충분히 큰 속도를
     * 연속으로 주고 이동'시간'으로 양을 조절하므로, 끊으면 덜컹거리기만 함.
     * 정지에서 매번 출발하므로 킥은 그대로 유지(min_speed=0이 그 모드). */
    fpga_link_set_pulse_params(fpga, 0, cfg->controller_max_speed, 1, 2);

    /* PATCH (2026-08-06) 하이브리드 주행:
     * 매 스텝 멈추는 건 (a) 정지마찰과 (b) 이동 중 스캔 왜곡 때문이었는데, 실제로
     * 계산해보면 둘의 성격이 다름.
     *   - 직진: 라이다 1회전(102ms) 동안 13.6mm 이동 -> 라이다 자체 노이즈(30mm)
     *     보다 작아서 무시할 만함. 게다가 이미 굴러가는 중이면 정지마찰도 무관.
     *   - 제자리 회전: 1회전당 8.5~9.7도나 틀어짐 -> 스캔이 심하게 찌그러져서
     *     위치추정이 망가짐. 속도를 낮춰도 PWM 데드존 때문에 별로 안 줄어듦.
     * 그래서 "회전이 주된 동작일 때만 멈추고, 전진 위주일 때는 안 멈추고 계속
     * 굴러가는" 방식으로 바꿈. 좌우 바퀴 명령의 부호가 반대면(=제자리 회전)
     * 정지-관측-회전을 하고, 같은 부호면(=전진/완만한 곡선) 연속 주행함. */
    st.prev_was_rotation = true;

    while (!g_stop_requested) {
        NavStep ns;
        NavAction act = nav__observe(&ctx, &st, &ns);
        if (act == NAV_RETRY) continue;
        if (act == NAV_STOP)  break;

        act = nav__watch_lost(&ctx, &st, &ns);
        if (act == NAV_RETRY)  continue;
        if (act == NAV_FAILED) return false;

    /* 선속도 배율 학습 (2026-08-10). 직진 스텝이고, 매칭이 믿을 만하고,
     * 게이트가 개입하지 않았을 때만 - 그래야 표본이 오염되지 않는다. */
    if (!cfg->has_encoder && !st.prev_was_rotation
        && nav_slam->bad_match_streak == 0
        && nav_slam->last_match_quality > 0.60) {
        robot_runner__lin_scale_observe(hypot(ns.odom_dx, ns.odom_dy),
                                         hypot(ns.x - ns.slam_prev_x, ns.y - ns.slam_prev_y));
    }

    /* PATCH (2026-08-09g): 조기 종료 상자.
     * 슬롯 대기점처럼 '점을 정확히 찍는 것'이 목적이 아닌 목표에서는, 상자 안에
     * 들어온 순간 끝낸다. 나머지 정밀도는 slot_align() / slot_lateral_align()
     * 이 제자리 회전 + 직선 이동으로 훨씬 짧게 해결한다.
     * (사용자 신고: "슬롯 앞에서 찔끔찔끔 이동/회전 반복") */
    if (nav->finish_box_x > 0.0 && nav->finish_box_y > 0.0) {
        double ex = fabs(ns.x - nav->goal_x), ey = fabs(ns.y - nav->goal_y);
        if (ex <= nav->finish_box_x && ey <= nav->finish_box_y) {
            fpga_link_set_speed(fpga, 0, 0);
            printf("[main] 대기 구역 진입: 좌우 %+.0fmm (허용 %.0f), 전후 %+.0fmm "
                   "(허용 %.0f)\n"
                   "       나머지 정렬은 슬롯 정렬 루틴이 직접 하므로 여기서 "
                   "접근을 끝냅니다\n",
                   (ns.x - nav->goal_x) * 1000.0, nav->finish_box_x * 1000.0,
                   (ns.y - nav->goal_y) * 1000.0, nav->finish_box_y * 1000.0);
            fflush(stdout);
            robot_runner__emit_state("nav", st.step, ns.x, ns.y, ns.theta, &ns.scan);
            return true;
        }
    }

        if (st.rot_escape_cooldown > 0) st.rot_escape_cooldown--;

        nav__update_rot_rate(&ctx, &st, &ns);

        act = nav__watch_stuck(&ctx, &st, &ns);
        if (act == NAV_RETRY)  continue;
        if (act == NAV_FAILED) return false;

    /* 4) 판단 */
    ClusterResult cluster_result;
    cluster_lidar_scan(&ns.scan, ns.x, ns.y, ns.theta, 0.15, 2, 3.0, &cluster_result);
    DetectedObstacle detected[CLUSTER_MAX_CLUSTERS];
    int n_detected = cluster_result_to_detected(&cluster_result, detected, CLUSTER_MAX_CLUSTERS);

    WheelCmd cmd = dynnav_update(nav, ns.x, ns.y, ns.theta, &ns.scan,
                                  n_detected > 0 ? detected : NULL, n_detected);

    printf("STATE navigate %d %.4f %.4f %.4f %d %d %d\n",
           st.step, ns.x, ns.y, ns.theta, nav->current_wp_idx, nav->waypoints.count, cmd.done ? 1 : 0);
    printf("SCAN %d", ns.scan.count);
    for (int i = 0; i < ns.scan.count; i++) {
        double wa = ns.theta + ns.scan.readings[i].angle_deg * ANGLE_PI / 180.0;
        double d = ns.scan.readings[i].dist_mm / 1000.0;
        printf(" %.3f %.3f", ns.x + d * cos(wa), ns.y + d * sin(wa));
    }
    printf("\n");
    fflush(stdout);

    if (cmd.done) {
        fpga_link_set_speed(fpga, 0, 0);
        /* done이 곧 성공은 아님 - 막혀서 웨이포인트를 전부 건너뛴 경우에도
         * done이 뜸. 그 상태로 true를 돌려주면 호출부가 엉뚱한 위치에서
         * 슬롯 진입을 시도하므로(=벽으로 돌진), 반드시 구분해서 실패로 반환. */
        if (nav->goal_unreachable) {
            printf("[main] *** 목적지 도달 실패: 경로가 막혔습니다. "
                   "현재 (%.3f,%.3f), 목표 (%.3f,%.3f) ***\n",
                   ns.x, ns.y, nav->goal_x, nav->goal_y);
            printf("[main] 안전을 위해 정지합니다. 장애물을 치우고 다시 실행하세요.\n");
            fflush(stdout);
            return false;
        }
        printf("[main] goal reached! step=%d, final pose=(%.3f,%.3f,%.2f)\n", st.step, ns.x, ns.y, ns.theta);
        return true;
    }

        NavCmdPlan nc;
        memset(&nc, 0, sizeof(nc));
        nc.cmd = cmd;
        nc.rot_allow_frac = 1.0;
        act = nav__scale_command(&ctx, &st, &ns, &nc);
        if (act == NAV_RETRY) continue;

        nav__plan_rotation(&ctx, &st, &ns, &nc);

        if (nc.rotation_dominant && !nav->docking_mode) {
            act = nav__try_sweep(&ctx, &st, &ns, &nc);
            if (act == NAV_RETRY) continue;
        }

        if (nc.rotation_dominant) {
            act = nav__rotation_clearance(&ctx, &st, &ns, &nc);
            if (act == NAV_RETRY) continue;
        }

        if (nc.rotation_dominant) nav__rotation_command(&ctx, &st, &ns, &nc);

    fpga_link_set_speed(fpga, nc.out_left, nc.out_right);
    if (nc.rotation_dominant || !cfg->hybrid_cruise) {
        robot_runner__sleep_sec(nc.move_sec);          /* 짧게 돌고 다시 관측 */
    } else {
        robot_runner__sleep_sec(1.0 / cfg->control_hz);  /* 안 멈추고 계속 굴림 */
    }
    st.prev_was_rotation = nc.rotation_dominant;
    st.step++;
    }

    fpga_link_set_speed(fpga, 0, 0);
    return false;
}




/* 탐색창을 반드시 원복시키기 위한 래퍼 (2026-08-09h).
 * 본체는 회전 스텝마다 창을 바꾸므로, 어느 경로로 빠져나가든 여기서 되돌린다. */
bool navigate_one_leg(FPGALink *fpga, LidarThread *lidar, DynamicNavigator *nav,
                                     OccupancyGridSLAM *nav_slam, const RobotConfig *cfg,
                                     bool use_slam_rolling_map,
                                     double *prev_raw_x, double *prev_raw_y, double *prev_raw_theta) {
    SlamWinSave sv;
    sv.win_m = nav_slam->search_window_m;   sv.step_m  = nav_slam->search_step_m;
    sv.win_th = nav_slam->search_window_theta; sv.step_th = nav_slam->search_step_theta;
    sv.saved = true;
    bool ok = navigate_one_leg_inner(fpga, lidar, nav, nav_slam, cfg, use_slam_rolling_map,
                                      prev_raw_x, prev_raw_y, prev_raw_theta);
    slam_win_pop(nav_slam, &sv);
    return ok;
}
