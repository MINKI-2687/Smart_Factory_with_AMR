#include "robot_shuttle.h"
#include "robot_runner.h"

/* ============================================================
 * run_shuttle() 분해 (2026-08-11 리팩토링)
 *
 * 예전에는 734줄이 한 함수였고, 그 안의 왕복 루프가 536줄이었다.
 * 로직은 손대지 않고, 셔틀 전체가 공유하는 자원을 ShuttleCtx 로,
 * 이번 회차(A로 갈 차례인지 B로 갈 차례인지)의 값을 ShuttleLeg 로 묶은 뒤
 * 단계별 헬퍼로 잘랐다. 각 헬퍼의 주석은 원래 그 자리에 있던 것 그대로다.
 *
 * *** goto 두 개를 없앴다 ***
 * 원본에는 shuttle_arrived / shuttle_depart 라벨과 그리로 뛰는 goto 가
 * 있었다. 둘 다 앞으로만 뛰고 건너뛰는 구간이 한 덩어리라, 각각
 *     if (!skip_travel) { ...주행/주차... }
 *     if (!depart_now)  { ...트리거 대기... }
 * 로 바꿨다. 다른 부분과 달리 이건 줄을 그대로 옮긴 게 아니라 제어흐름을
 * 다시 쓴 것이므로, 나중에 의심이 들면 여기를 먼저 보면 된다.
 *
 * 제어흐름은 ShuttleAction 으로 돌려준다.
 *     SHUTTLE_GO      : 계속 진행
 *     SHUTTLE_QUIT    : 정상 종료 (중단 신호 등 - clean_stop 유지)
 *     SHUTTLE_ABORT   : 비정상 종료 (clean_stop = false)
 * ============================================================ */

/* 셔틀이 도는 동안 바뀌지 않는 자원과 설정 */
typedef struct {
    FPGALink          *fpga;
    LidarThread       *lidar;
    DynamicNavigator  *nav;
    OccupancyGridSLAM *nav_slam;
    RobotConfig       *cfg;
    RplidarReader     *reader;
    ShuttlePoint       point_a, point_b;
    const TriggerConfig *trigger_cfg;
    MapSource          map_source;
    bool               use_slam_rolling_map;
    /* 오도메트리 델타 계산용 - 구간을 넘어 이어져야 한다 */
    double prev_raw_x, prev_raw_y, prev_raw_theta;
} ShuttleCtx;

/* 이번 회차의 목적지와 그 파생값 */
typedef struct {
    ShuttlePoint    target;
    const char     *label;
    TriggerWaitKind wait_kind;
    double          cur_x, cur_y, cur_theta;
    double          target_theta_rad;
    double          nav_goal_x, nav_goal_y;
} ShuttleLeg;

typedef enum {
    SHUTTLE_GO = 0,
    SHUTTLE_QUIT,     /* 루프 탈출, clean_stop 유지 */
    SHUTTLE_ABORT,    /* 루프 탈출, clean_stop = false */
} ShuttleAction;

/* 주차 한 번의 시도 결과 */
typedef enum {
    PARK_OK = 0,
    PARK_RETRY,       /* 이번 시도 실패 - 다음 시도로 (원본의 continue) */
    PARK_INTERRUPT,   /* 중단 신호 (원본의 break) */
} ParkAttempt;


static bool shuttle__localize_start(ShuttleCtx *sh) {

    /* PATCH (2026-08-05): 움직이기 전에 (1) 라이다가 실제로 돌기 시작할 때까지 기다리고,
     * (2) 그 첫 스캔으로 시작 자세(특히 각도)를 전역탐색해서 확정함. 둘 다 안 하면
     * "start 좌표에 0도로 서 있다"는 틀린 전제로 주행이 시작돼서 pose가 계속 튐. */

        /* 지도가 앞으로 바뀌지 않는 모드(fixed/load)에서는, 주행 중 스캔매칭도 부드러운
         * 점수판을 쓰게 붙여줌. 매핑 모드(slam)에서는 지도가 매 스텝 갱신되므로 붙이지
         * 않음(붙이면 점수판이 낡아버림) - slam.h의 lf_field 주석 참고. */
        if (sh->map_source != MAP_SOURCE_SLAM) {
            sh->nav_slam->lf_field = slam_build_likelihood_field(sh->nav_slam);
            /* 정밀 탐색 전용 좁은 시그마 판도 같이 만든다 (2026-08-09n).
             * 넓은 판만 쓰면 봉우리가 뭉툭해서 몇 cm 어긋난 자세도 거의 같은 점수를
             * 받는다 - 그게 "모양은 맞는데 통째로 평행이동" 증상의 원인이었다. */
            sh->nav_slam->lf_field_fine =
                slam_build_likelihood_field_sigma(sh->nav_slam, g_slam_sigma_fine);
            if (sh->nav_slam->lf_field) {
                printf("[localize] 고정맵용 점수판 생성 완료 "
                       "(거친 탐색 sigma %.0fmm / 정밀 탐색 sigma %.0fmm%s)\n",
                       g_slam_sigma_coarse * 1000.0, g_slam_sigma_fine * 1000.0,
                       sh->nav_slam->lf_field_fine ? "" : " - 정밀판 생성 실패, 넓은 판만 사용");
            }
        }

        LidarScan init_scan;
        if (wait_for_lidar_ready(sh->lidar, 50, 10.0, &init_scan)) {
            apply_lidar_frame_fix(&init_scan, sh->cfg->lidar_mirror, sh->cfg->lidar_yaw_offset_deg,
                              sh->cfg->lidar_offset_forward_m);
            if (sh->cfg->auto_localize_start) {
                double lx, ly, lth;
                slam_global_localize(sh->nav_slam, &init_scan, sh->cfg->start_x, sh->cfg->start_y,
                                      true, &lx, &ly, &lth);
                sh->nav_slam->x = lx; sh->nav_slam->y = ly; sh->nav_slam->theta = lth;
            } else {
                sh->nav_slam->theta = sh->cfg->start_theta_deg * ANGLE_PI / 180.0;
                printf("[localize] 시작자세 수동지정: (%.3f, %.3f, %.1f deg)\n",
                       sh->nav_slam->x, sh->nav_slam->y, sh->cfg->start_theta_deg);
            }
        } else {
            fprintf(stderr, "[shuttle] 라이다 준비 실패 - 중단합니다\n");
            fpga_link_close(sh->fpga);
            lidar_thread_stop(sh->lidar);
            slam_free(sh->nav_slam);
            dynnav_free(sh->nav);
            return false;
        }
    return true;
}


static bool shuttle__detect_start_parked(ShuttleCtx *sh, int *leg) {

/* ============================================================
 * 시작할 때 이미 A/B 슬롯 안이면 '그 지점에 주차된 상태'로 본다 (2026-08-10h)
 *
 * *** 사용자 요청 ***
 *   "로봇이 A 또는 B 주차슬롯 내부에서 시작한다면, 각 슬롯에서의 트리거를
 *    기다리거나, 만약 트리거가 이미 충족되었다면 바로 후진해서 나가도록 해줘."
 *
 * 예전 동작: leg 가 무조건 0(=A)으로 시작하므로, A슬롯 안에서 켜면
 *   후진으로 슬롯을 빠져나옴 -> 다시 A로 A* 주행 -> A에 재주차 -> 그제서야 트리거 대기
 * 를 했다. 이미 A에 주차되어 있는데 나갔다가 다시 들어오는 완전한 낭비이고,
 * 그 재진입에서 정렬이 틀어지면 셔틀이 시작도 못 하고 실패했다.
 *
 * 이제는 시작 자세로 어느 슬롯에 있는지 판정해서, 그 지점을 '이번 목적지'로
 * 잡고 주행/주차 단계를 통째로 건너뛴 뒤 곧바로 트리거 대기로 넘어간다.
 * 다음 회차에서 반대편 지점이 목적지가 되고, 그때 루프 앞머리의 in_slot 후진이
 * 자연스럽게 슬롯 밖으로 빼낸다.
 * ============================================================ */
bool start_parked = false;
if (sh->cfg->slot_parking) {
    double sx, sy, sth;
    slam_get_pose(sh->nav_slam, &sx, &sy, &sth);
    const double START_INSIDE_MARGIN_M = 0.05;
    const double START_LANE_TOL_M = 0.15;
    if (sy < sh->cfg->slot_staging_y - START_INSIDE_MARGIN_M) {
        double da = fabs(sx - sh->point_a.x), db = fabs(sx - sh->point_b.x);
        if (da <= db && da < START_LANE_TOL_M) {
            *leg = 0; start_parked = true;
        } else if (db < START_LANE_TOL_M) {
            *leg = 1; start_parked = true;
        }
    }
    if (start_parked) {
        printf("[shuttle] 시작 자세 (%.3f, %.3f) 는 %s 슬롯 안입니다.\n"
               "          이미 그 지점에 주차된 것으로 보고, 주행/주차를 건너뛰고\n"
               "          곧바로 %s 출발 트리거를 확인합니다.\n",
               sx, sy, (*leg == 0) ? "A" : "B",
               (*leg == 0) ? "A지점(짐 얹힘)" : "B지점(짐 내려감)");
    } else {
        printf("[shuttle] 시작 자세 (%.3f, %.3f) 는 슬롯 밖입니다 - 평소대로 "
               "A지점부터 주행합니다\n", sx, sy);
    }
    fflush(stdout);
}
    return start_parked;
}


static ShuttleAction shuttle__exit_slot(ShuttleCtx *sh, ShuttleLeg *lg) {

    /* 슬롯 안에 있다면 먼저 후진해서 대기점까지 빠져나옴 - 슬롯 안에서는 회전이
     * 불가능하므로 A*를 돌리기 전에 반드시 나와야 함.
     *
     * BUGFIX (2026-08-08): 판정이 두 군데 틀려 있었음.
     *  (1) 조건이 그냥 (cur_y < slot_staging_y) 라서, 통로 한복판에서 시작해도
     *      y가 대기점보다 1cm만 낮으면 "슬롯 안"으로 오판했음.
     *      실기 로그: 자세 (0.730, 0.640)에서 대기점 0.650과 1cm 차이인데 후진 시도
     *      -> slot_drive의 방향 가드에 걸려 셔틀이 시작도 못 하고 중단됨.
     *      슬롯 깊이가 26cm이므로 "안에 있다"면 최소 5cm는 들어가 있어야 함.
     *  (2) 빠져나올 차선을 (leg == 0) ? point_b : point_a 로 '가정'했음. 첫 실행에서
     *      로봇이 어디서 시작할지는 알 수 없는데 무조건 반대쪽 슬롯에 있다고 친 것.
     *      실기 로그: 로봇 x=0.730인데 lane_x=1.275로 빠져나오려 했음.
     *      이제 실제 자세에서 가장 가까운 슬롯을 고르고, 어느 슬롯에도 안 걸리면
     *      슬롯 밖(통로)으로 보고 후진을 건너뜀. */
    bool in_slot = false;
    double slot_lane_x = 0.0, slot_lane_theta_deg = 0.0;
    if (sh->cfg->slot_parking) {
        const double SLOT_INSIDE_MARGIN_M = 0.05;  /* 이만큼은 들어가 있어야 '안' */
        const double SLOT_LANE_TOL_M = 0.15;       /* 차선에서 이 안쪽이어야 그 슬롯 */
        if (lg->cur_y < sh->cfg->slot_staging_y - SLOT_INSIDE_MARGIN_M) {
            double da = fabs(lg->cur_x - sh->point_a.x), db = fabs(lg->cur_x - sh->point_b.x);
            if (da <= db && da < SLOT_LANE_TOL_M) {
                in_slot = true; slot_lane_x = sh->point_a.x; slot_lane_theta_deg = sh->point_a.theta_deg;
            } else if (db < SLOT_LANE_TOL_M) {
                in_slot = true; slot_lane_x = sh->point_b.x; slot_lane_theta_deg = sh->point_b.theta_deg;
            }
        }
        if (!in_slot) {
            printf("[shuttle] 현재 자세 (%.3f, %.3f) 는 슬롯 밖 - 후진 없이 바로 주행합니다\n",
                   lg->cur_x, lg->cur_y);
            fflush(stdout);
        }
    }
    if (in_slot) {
        printf("[shuttle] 슬롯 안(차선 x=%.3f)에서 시작 - 대기점까지 후진합니다\n",
               slot_lane_x);
        fflush(stdout);
        if (!slot_drive(sh->fpga, sh->lidar, sh->nav_slam, sh->cfg, slot_lane_x,
                        slot_lane_theta_deg * ANGLE_PI / 180.0,
                        sh->cfg->slot_staging_y, false, "슬롯 후진", 0.0, NULL)) {
            /* ============================================================
             * PATCH (2026-08-10h): 한 번 실패했다고 셔틀을 끝내지 않는다.
             *
             * 슬롯에서 빠져나오는 건 '왔던 길 되짚기'라 물리적으로 막힐 이유가
             * 거의 없다. 실패했다면 대개 조향이 벽을 긁은 것이므로, 조향을
             * 아예 빼고(label 에 "재시도"를 넣으면 slot_drive 가 straight_out
             * 모드로 곧게 + 세게 민다) 한 번 더 시도한다.
             * 그래도 안 되면, 최소한 슬롯 밖으로는 나왔는지 보고 판단한다 -
             * 나왔으면 주행을 계속하는 게 맞고, 못 나왔을 때만 중단한다.
             * ============================================================ */
            printf("[shuttle] 슬롯 후진 실패 - 조향을 빼고 곧게 한 번 더 빼봅니다\n");
            fflush(stdout);
            slot_drive(sh->fpga, sh->lidar, sh->nav_slam, sh->cfg, slot_lane_x,
                       slot_lane_theta_deg * ANGLE_PI / 180.0,
                       sh->cfg->slot_staging_y, false, "슬롯 후진 재시도", 0.0, NULL);
            slam_get_pose(sh->nav_slam, &lg->cur_x, &lg->cur_y, &lg->cur_theta);
            if (lg->cur_y < sh->cfg->slot_staging_y - 0.10) {
                printf("[shuttle] 슬롯 밖으로 빠져나오지 못했습니다 "
                       "(y=%.3f, 대기점 %.3f) - 중단\n",
                       lg->cur_y, sh->cfg->slot_staging_y);
                return SHUTTLE_ABORT;
            }
            printf("[shuttle] 대기점까지는 못 갔지만 슬롯은 벗어났습니다 "
                   "(y=%.3f) - 주행을 계속합니다\n", lg->cur_y);
            fflush(stdout);
        }
        slam_get_pose(sh->nav_slam, &lg->cur_x, &lg->cur_y, &lg->cur_theta);
    }
    return SHUTTLE_GO;
}


static bool shuttle__is_direct_entry(const ShuttleCtx *sh, const ShuttleLeg *lg) {

    /* PATCH (2026-08-08): 이미 슬롯 바로 앞이면 A* 주행을 통째로 건너뜀.
     *
     * 요청: "목표점에 정확히 있지 않더라도 슬롯으로 바로 들어갈 수 있는 위치면
     *        그냥 바로 주차시켜라."
     *
     * 왜 필요한가: 슬롯 입구 근처는 좌우 벽이 가까워서 제자리 회전 여유가 거의 없음.
     * 그런데 A*는 로봇을 '대기점'이라는 특정 점으로 데려가려 하고, 그러려면 먼저
     * 웨이포인트 쪽으로 몸을 돌려야 함. 실기 로그: (0.190,0.535)에서 45스텝 동안
     * cmd=(-34,34)를 반복하는데 각도는 스텝당 0.5도씩밖에 안 변하고, 끼임 탈출을
     * 4번이나 시도하다 실패. 이미 슬롯 앞 20cm에 서 있었는데도 그랬음.
     * 어차피 슬롯은 -y 방향 직진으로 들어가므로, 차선 근처에 있고 슬롯보다 위에
     * 있기만 하면 대기점을 경유할 이유가 없음. 바로 정렬하고 들어가면 됨. */
    bool direct_entry = false;
    if (sh->cfg->slot_parking) {
        const double DIRECT_LANE_TOL_M = 0.12;  /* 차선에서 이 안쪽이면 바로 진입 */
        const double DIRECT_DEPTH_MIN_M = 0.08; /* 슬롯 안으로 너무 깊이 들어간 건 제외 */
        double lane_off = fabs(lg->cur_x - lg->target.x);
        bool above_slot = (lg->cur_y > lg->target.y + DIRECT_DEPTH_MIN_M);
        bool not_too_far = (lg->cur_y < sh->cfg->slot_staging_y + 0.20);
        if (lane_off <= DIRECT_LANE_TOL_M && above_slot && not_too_far) {
            direct_entry = true;
            printf("[shuttle] 이미 %s 슬롯 앞입니다 (차선오차 %+.0fmm, y=%.3f) "
                   "- 주행 없이 바로 진입합니다\n",
                   lg->label, (lg->cur_x - lg->target.x) * 1000.0, lg->cur_y);
            fflush(stdout);
        }
    }
    return direct_entry;
}


static ShuttleAction shuttle__travel_to_staging(ShuttleCtx *sh, ShuttleLeg *lg) {

    printf("[shuttle] %s로 이동 시작: (%.2f,%.2f) -> (%.2f,%.2f,%.1fdeg)\n",
           lg->label, lg->cur_x, lg->cur_y, lg->nav_goal_x, lg->nav_goal_y, lg->target.theta_deg);
    printf("TARGET %.4f %.4f %.4f\n", lg->nav_goal_x, lg->nav_goal_y, lg->target_theta_rad);
    bool planned = dynnav_set_goal(sh->nav, lg->cur_x, lg->cur_y, lg->nav_goal_x, lg->nav_goal_y,
                                    true, lg->target_theta_rad, NULL, NULL, 0);
    if (!planned) {
        printf("[shuttle] %s로 가는 A* 경로계획 실패 - 셔틀 중단\n  이유: %s\n",
               lg->label, dynnav_get_last_plan_fail_reason(sh->nav));
        return SHUTTLE_ABORT;
    }
    printf("WAYPOINTS %d", sh->nav->waypoints.count);
    for (int i = 0; i < sh->nav->waypoints.count; i++) {
        printf(" %.4f %.4f", sh->nav->waypoints.points[i].x, sh->nav->waypoints.points[i].y);
    }
    printf("\n");
    fflush(stdout);

    /* 슬롯 모드에서는 대기점을 mm 단위로 찍을 필요가 없다 - 상자 안에 들어오면
     * 끝내고 슬롯 정렬 루틴에 넘긴다 (2026-08-09g, DynamicNavigator.finish_box_* 주석 참고).
     * 슬롯 축이 -y 이므로 x 가 좌우오차, y 가 전후오차다. */
    sh->nav->finish_box_x = sh->cfg->slot_parking ? sh->cfg->slot_approach_x_tol : 0.0;
    sh->nav->finish_box_y = sh->cfg->slot_parking ? sh->cfg->slot_approach_y_tol : 0.0;

    bool reached = navigate_one_leg(sh->fpga, sh->lidar, sh->nav, sh->nav_slam, sh->cfg, sh->use_slam_rolling_map,
                                     &sh->prev_raw_x, &sh->prev_raw_y, &sh->prev_raw_theta);
    if (!reached) {
        /* 사용자 중단(Ctrl+C)인지, 경로가 막혀서 못 간 것인지 구분해서 알림.
         * 어느 쪽이든 슬롯 진입으로 넘어가지 않고 여기서 멈추는 게 핵심 - 도달하지
         * 못한 위치에서 진입하면 벽으로 돌진하게 됨. */
        if (sh->nav->goal_unreachable) {
            printf("[shuttle] 경로가 막혀 %s에 도달하지 못했습니다. 셔틀 종료\n", lg->label);
            return SHUTTLE_ABORT;
        }
            printf("[shuttle] 중단 신호 수신, 셔틀 종료\n");
            return SHUTTLE_QUIT;
    }
    return SHUTTLE_GO;
}


/* 재시도 전 복구: 슬롯 밖으로 빼내고, 접촉을 풀고, 필요하면 대기점까지 다시 접근. */
static ParkAttempt shuttle__recover_for_retry(ShuttleCtx *sh, ShuttleLeg *lg,
                                              int attempt, int max_tries) {

    const double RECOVER_ESCAPE_M = 0.04;

    printf("[shuttle] ===== %s 주차 재시도 %d/%d - 빼내고 다시 접근합니다 =====\n",
           lg->label, attempt + 1, max_tries);
    fflush(stdout);

    /* 1) 슬롯 안쪽이면 대기점 높이까지 후진해서 빼냄.
     * 실패해도 계속 진행 - 이미 밖이거나 조금 못 나온 것뿐일 수 있음. */
    double rx, ry, rth;
    slam_get_pose(sh->nav_slam, &rx, &ry, &rth);
    if (ry < sh->cfg->slot_staging_y - 0.03) {
        slot_drive(sh->fpga, sh->lidar, sh->nav_slam, sh->cfg, lg->target.x, lg->target_theta_rad,
                   sh->cfg->slot_staging_y, false, "재시도 후진", 0.0, NULL);
    }
    if (g_stop_requested) return PARK_INTERRUPT;

    /* 2) 아직 어딘가 닿아 있으면 접촉만 짧게 푼다 */
    robot_runner__escape_to_open(sh->fpga, sh->lidar, sh->cfg, RECOVER_ESCAPE_M, 0.0,
                                 sh->cfg->allow_crab);
    if (g_stop_requested) return PARK_INTERRUPT;

    /* 3) 대기점에서 많이 벗어났으면 A*로 다시 접근 */
    slam_get_pose(sh->nav_slam, &rx, &ry, &rth);
    double away = hypot(rx - lg->target.x, ry - sh->cfg->slot_staging_y);
    if (away > 0.06) {
        printf("[shuttle] 대기점에서 %.0fmm 벗어남 - A*로 다시 접근합니다\n",
               away * 1000.0);
        printf("TARGET %.4f %.4f %.4f\n",
               lg->target.x, sh->cfg->slot_staging_y, lg->target_theta_rad);
        fflush(stdout);
        if (dynnav_set_goal(sh->nav, rx, ry, lg->target.x, sh->cfg->slot_staging_y,
                            true, lg->target_theta_rad, NULL, NULL, 0)) {
            printf("WAYPOINTS %d", sh->nav->waypoints.count);
            for (int wi = 0; wi < sh->nav->waypoints.count; wi++)
                printf(" %.4f %.4f", sh->nav->waypoints.points[wi].x,
                       sh->nav->waypoints.points[wi].y);
            printf("\n");
            fflush(stdout);
            sh->nav->finish_box_x = sh->cfg->slot_approach_x_tol;
            sh->nav->finish_box_y = sh->cfg->slot_approach_y_tol;
            navigate_one_leg(sh->fpga, sh->lidar, sh->nav, sh->nav_slam, sh->cfg,
                             sh->use_slam_rolling_map,
                             &sh->prev_raw_x, &sh->prev_raw_y, &sh->prev_raw_theta);
        } else {
            printf("[shuttle] 대기점 재접근 경로계획 실패 - 현재 자리에서 그대로 재시도\n");
            fflush(stdout);
        }
    }
    if (g_stop_requested) return PARK_INTERRUPT;
    return PARK_OK;
}


/* 주차 한 번: 차선 맞추기 -> 각도 정렬 -> 좌우 정렬 -> 진입 게이트 -> 진입 -> 착석 -> 검사. */
static ParkAttempt shuttle__park_once(ShuttleCtx *sh, ShuttleLeg *lg,
                                      int attempt, int max_tries) {

    const double ENTER_LAT_MAX_M   = 0.060;
/* 10.0 -> 6.0 (2026-08-09e).
 * 슬롯에 낄 때까지 허용되는 임계 yaw 는 20.2도다
 * (130*cos(e)+255*sin(e) = 210mm 인 지점).
 * 그런데 진입 중에 경사벽이 밀기 한 번당 약 1.5도씩 각도를 키우므로
 * (실기 로그), 10도로 출발하면 7번 밀기 만에 임계각에 닿는다.
 * 슬롯 깊이를 다 내려가려면 20번 넘게 밀어야 하므로 그 전에 낀다.
 * 6도로 출발하면 임계각까지 9번, 게다가 8도에서 복구가 개입하므로
 * 훨씬 여유가 생긴다. slot_align 이 어차피 2.5도까지 맞춰주므로
 * 게이트를 6도로 조여도 통과율은 떨어지지 않는다. */
const double ENTER_ANGLE_MAX_D = 6.0;
    const double PARK_FAIL_MM      = 80.0;
    /* PATCH (2026-08-09i): 슬롯 쪽으로 돌기 '전에' 차선(x)을 맞춘다.
     * 지금 로봇은 통로를 따라 왔으므로 0도 또는 180도를 보고 있고,
     * 그 방향이 곧 x 축이다. 앞뒤로만 조금 움직이면 회전 없이 x 가 맞는다.
     * 예전에는 슬롯 쪽으로 먼저 돌고 나서 slot_lateral_align 이
     * 다시 90도씩 두 번 돌아 좌우를 맞췄다(약 5.6초 낭비). */
    slot_lane_trim(sh->fpga, sh->lidar, sh->nav_slam, sh->cfg, lg->target.x, 0.010,
                   &sh->prev_raw_x, &sh->prev_raw_y, &sh->prev_raw_theta);
    if (g_stop_requested) return PARK_INTERRUPT;

    /* PATCH (2026-08-09h): 정렬 전에 '돌 자리'부터 확보한다.
     * 슬롯 입구 근처(예: y=0.495)에서는 아래벽까지 45mm 뿐이라 90도
     * 제자리 회전이 물리적으로 불가능하다. 그 상태로 slot_align 을 부르면
     * 명령만 나가고 안 돌아서 '끼임 -> 탈출 -> 실패'를 반복하며 수십 초를
     * 버린다(실기 로그). 먼저 슬롯 밖으로 물러나 여유를 만든다. */
    {
        double rx2, ry2, rth2;
        slam_get_pose(sh->nav_slam, &rx2, &ry2, &rth2);
        double need_turn = normalize_angle(lg->target_theta_rad - rth2);
        double out_sign = (sh->cfg->slot_staging_y > lg->target.y) ? +1.0 : -1.0;
        robot_runner__ensure_rotation_room(sh->fpga, sh->lidar, sh->nav_slam, sh->cfg,
                                            need_turn, out_sign, 0.22,
                                            &sh->prev_raw_x, &sh->prev_raw_y,
                                            &sh->prev_raw_theta);
    }

    /* 진입 전 각도 정밀 정렬 - 슬롯이 좁아 1도당 7mm씩 밀리므로 필수 */
    if (!slot_align(sh->fpga, sh->lidar, sh->nav_slam, sh->cfg,
                    lg->target_theta_rad, sh->cfg->slot_align_tol_deg)) {
        printf("[shuttle] %s 진입 전 각도 정렬 실패 (시도 %d/%d)\n",
               lg->label, attempt + 1, max_tries);
        fflush(stdout);
        return PARK_RETRY;
    }

    /* 각도만 맞추면 좌우가 40mm씩 벌어진 채 들어가서 깔때기에 박힘 -> 좌우도 맞춤.
     * 다만 오차가 이미 여유(±20mm) 안이면 90도 회전 두 번은 낭비이므로 건너뜀. */
    /* 좌우 정렬(90도 회전 두 번짜리 기동)은 이제 '최후 수단'이다 (2026-08-09i).
     * 위 slot_lane_trim 이 통로를 보는 동안 회전 없이 x 를 맞췄고,
     * 남은 30mm 이내는 slot_drive 가 기울여 들어가며 흡수한다.
     * 그보다 크게 남았다는 건 차선 자체를 놓쳤다는 뜻이라 그때만 부른다. */
    /* 슬롯 정렬에 들어가기 전에 자세를 한 번 검증한다 (2026-08-10).
     * 실기 실패는 전부 이 지점 이후에 벌어졌는데, 길잃음 감시가
     * navigate_one_leg 안에만 있어서 여기는 무방비였다. */
    /* PATCH (2026-08-10f): 예전에는 반환값을 버렸다. 즉 "자세를 못
     * 믿겠습니다"를 찍고도 곧바로 정렬/진입으로 넘어갔다.
     * 이제 못 고치면 이번 시도를 접고 다시 접근한다 - 틀린 자세로
     * 슬롯에 밀어넣는 것보다 언제나 낫다. */
    if (!robot_runner__require_good_pose(sh->fpga, sh->lidar, sh->cfg, sh->nav_slam,
                                          "슬롯 정렬 시작 전 자세 검증")) {
        printf("[shuttle] %s 진입 전 자세를 못 믿어 이번 시도를 접습니다 "
               "(시도 %d/%d)\n", lg->label, attempt + 1, max_tries);
        fflush(stdout);
        return PARK_RETRY;
    }

    double entry_lat_err = 0.0;
    {
        double px0, py0, pth0;
        slam_get_pose(sh->nav_slam, &px0, &py0, &pth0);
        entry_lat_err = px0 - lg->target.x;
        if (fabs(entry_lat_err) <= 0.030) {
            printf("[shuttle] 좌우오차 %+.0fmm - 기울여 진입하며 흡수합니다 "
                   "(90도 회전 정렬 생략)\n", entry_lat_err * 1000.0);
            fflush(stdout);
        } else if (!slot_lateral_align(sh->fpga, sh->lidar, sh->nav_slam, sh->cfg,
                                        lg->target.x, lg->target_theta_rad)) {
            printf("[shuttle] %s 진입 전 좌우 정렬 실패 (시도 %d/%d)\n",
                   lg->label, attempt + 1, max_tries);
            fflush(stdout);
            /* PATCH (2026-08-10): 좌우 정렬 실패는 '차가 못 움직인 것'보다
             * '자세추정이 틀린 것'인 경우가 훨씬 많다. 실기 로그에서
             *     좌우 정렬 실패: 오차 +314mm (한계 60mm)
             * 가 났는데, 슬롯 폭이 210mm 이므로 314mm 는 애초에 슬롯 근처도
             * 아닌 값이다. 차가 그렇게 멀리 갈 리는 없고, 자세가 틀린 것이다.
             * 그러니 다시 접근하기 전에 지도 전체에서 위치를 다시 잡는다 -
             * 틀린 자세로 재접근하면 몇 번을 해도 같은 곳에서 실패한다. */
            fpga_link_set_speed(sh->fpga, 0, 0);
            robot_runner__relocalize_global(sh->lidar, sh->cfg, sh->nav_slam,
                                             "슬롯 진입 전 좌우 정렬 실패");
            return PARK_RETRY;
        } else {
            slam_get_pose(sh->nav_slam, &px0, &py0, &pth0);
            entry_lat_err = px0 - lg->target.x;
        }
    }

    /* 진입 직전 최종 안전 게이트 (2026-08-08g).
     * 여기까지 왔는데도 오차가 크면 슬롯이 아니라 벽으로 들어가게 된다.
     * 슬롯 폭 210mm / 차폭 130mm 이므로 좌우 여유는 편측 40mm,
     * 진입 깊이 40cm 동안 1도당 7mm 씩 밀리므로 각도는 10도 이내여야 한다.
     * (2026-08-09a: 여기서 셔틀을 끝내지 않고 이번 시도만 접는다) */
    {
        double gx2, gy2, gth2;
        slam_get_pose(sh->nav_slam, &gx2, &gy2, &gth2);
        double lat_e = gx2 - lg->target.x;
        double ang_e = normalize_angle(gth2 - lg->target_theta_rad) * 180.0 / ANGLE_PI;
        if (fabs(lat_e) > ENTER_LAT_MAX_M || fabs(ang_e) > ENTER_ANGLE_MAX_D) {
            printf("[shuttle] %s 진입 보류: 좌우오차 %+.0fmm (한계 %.0f), "
                   "각도오차 %+.1fdeg (한계 %.1f)\n"
                   "       이 상태로 밀면 슬롯이 아니라 벽으로 들어갑니다 (시도 %d/%d)\n",
                   lg->label, lat_e * 1000.0, ENTER_LAT_MAX_M * 1000.0,
                   ang_e, ENTER_ANGLE_MAX_D, attempt + 1, max_tries);
            fflush(stdout);
            /* PATCH (2026-08-10c): 게이트에 걸린 채로 그냥 재시도하면
             * 같은 자리에서 같은 오차가 또 난다(실기에서 3/3 전부 그랬다).
             * 각도오차로 걸렸다는 건 정렬 루틴이 '더는 못 줄인다'고 판단했다는
             * 뜻이고, 그 판단의 근거(자세추정)가 틀렸을 가능성이 높다.
             * 다시 접근하기 전에 지도 전체에서 위치를 잡는다 - 좌우 정렬
             * 실패 때 이미 쓰고 있던 것과 같은 처방이다. */
            fpga_link_set_speed(sh->fpga, 0, 0);
            robot_runner__relocalize_global(sh->lidar, sh->cfg, sh->nav_slam,
                                             "슬롯 진입 게이트 초과");
            return PARK_RETRY;
        }
    }

    /* 슬롯에 밀어넣기 직전, 마지막으로 한 번 더 확인한다 (2026-08-10f).
     * 여기서부터는 되돌리기 어려운 구간이라 가장 엄격해야 한다. */
    if (!robot_runner__require_good_pose(sh->fpga, sh->lidar, sh->cfg, sh->nav_slam,
                                          "슬롯 진입 직전 최종 확인")) {
        printf("[shuttle] %s 진입 직전 자세를 못 믿어 진입하지 않습니다 "
               "(시도 %d/%d)\n", lg->label, attempt + 1, max_tries);
        fflush(stdout);
        return PARK_RETRY;
    }

    bool touched = false;
    if (!slot_drive(sh->fpga, sh->lidar, sh->nav_slam, sh->cfg, lg->target.x, lg->target_theta_rad,
                    lg->target.y, true, "슬롯 진입", entry_lat_err, &touched)) {
        printf("[shuttle] %s 슬롯 진입 실패 (시도 %d/%d)\n",
               lg->label, attempt + 1, max_tries);
        fflush(stdout);
        return PARK_RETRY;
    }

    if (*robot_runner__slot_no_wall(sh->cfg)) {
        /* PATCH (2026-08-10g): 이건 '물리적 모순'이다 - 슬롯 깊이까지
         * 밀었는데 안쪽 벽이 없다는 건, 자세가 틀렸다는 확정 증거다.
         * 그러니 운동 예산으로 재위치추정을 막으면 안 된다.
         * (실기 로그: "전역 재위치추정 거부: 후보가 526mm 떨어져 있는데
         *  명령 이동은 71mm 뿐" - 일치도 94%->100% 인 정답을 막았다.
         *  94% 는 슬롯 근처에서 틀린 자세도 낼 수 있는 값이라, 그걸
         *  근거로 예산을 리셋한 것이 원인이었다.) */
        robot_runner__travel_acc_set(1.5);
        printf("[shuttle] %s 진입했다고 판단했지만 안쪽 벽에 닿지 않았습니다.\n"
               "       슬롯 안이라면 반드시 닿아야 하므로 주차로 인정하지 않고,\n"
               "       지도 전체에서 위치를 다시 잡은 뒤 재시도합니다 (시도 %d/%d)\n",
               lg->label, attempt + 1, max_tries);
        fflush(stdout);
        fpga_link_set_speed(sh->fpga, 0, 0);
        robot_runner__relocalize_global(sh->lidar, sh->cfg, sh->nav_slam,
                                         "슬롯 진입 후 벽 미접촉");
        return PARK_RETRY;
    }

    slot_seat(sh->fpga, sh->cfg, lg->target_theta_rad, touched);

    /* (2026-08-11a) 판정 직전에 라이다로 자세를 확정한다.
     * 여기가 "다 넣어놓고 도로 빼는" 사고가 나던 바로 그 지점이다. */
    bool phys_parked = robot_runner__slot_confirm_park(
                            sh->fpga, sh->lidar, sh->cfg, sh->nav_slam,
                            lg->target.x, lg->target_theta_rad, lg->target.y);

    double px, py, pth;
    slam_get_pose(sh->nav_slam, &px, &py, &pth);
    double err_mm = hypot(px - lg->target.x, py - lg->target.y) * 1000.0;
    /* BUGFIX (2026-08-07): 예전엔 오차가 얼마든 무조건 "주차 완료"를 찍었음
     * (실기 로그에 오차 1154.9mm인데 완료로 기록됨). 슬롯 깊이가 26cm이므로
     * 이보다 크게 벗어났으면 그건 주차가 아니라 실패임 - 여기서 걸러야
     * 다음 왕복에서 엉뚱한 자세로 후진하는 2차 사고를 막을 수 있음. */
    if (err_mm > PARK_FAIL_MM && phys_parked) {
        /* 스캔매칭 자세로는 오차가 크지만, 라이다 실측으로는 슬롯 안
         * 정위치다. 이런 경우 예전에는 빼내고 다시 넣었는데, 그게 바로
         * 사용자가 지적한 "잘 주차했는데 혼자 나가버리는" 동작이다.
         * 물리적 관측이 지도 위 좌표보다 우선한다. */
        printf("[shuttle] %s 스캔매칭 자세로는 오차 %.1fmm 지만, 라이다 실측으로는\n"
               "       슬롯 안 정위치입니다 - 빼내지 않고 주차로 인정합니다 "
               "(시도 %d/%d)\n",
               lg->label, err_mm, attempt + 1, max_tries);
        fflush(stdout);
        err_mm = 0.0;
    }
    if (err_mm > PARK_FAIL_MM) {
        printf("[shuttle] %s 주차 오차 초과: 목표(%.3f,%.3f) 실제(%.3f,%.3f) "
               "오차 %.1fmm (허용 %.0fmm), 각도 %.1fdeg (시도 %d/%d)\n",
               lg->label, lg->target.x, lg->target.y, px, py, err_mm, PARK_FAIL_MM,
               pth * 180.0 / ANGLE_PI, attempt + 1, max_tries);
        fflush(stdout);
        return PARK_RETRY;
    }

    printf("[shuttle] %s 주차 완료: 목표(%.3f,%.3f) 실제(%.3f,%.3f) "
           "오차 %.1fmm, 각도 %.1fdeg (시도 %d/%d)\n",
           lg->label, lg->target.x, lg->target.y, px, py, err_mm,
           pth * 180.0 / ANGLE_PI, attempt + 1, max_tries);
    fflush(stdout);

    return PARK_OK;
}


/* ============================================================
 * 대기점 도착 -> 슬롯 안으로 직진 진입
 *
 * PATCH (2026-08-09a): 실패를 '중단'이 아니라 '복구 후 재시도'로 바꿈.
 *
 * 예전 구조는 정렬 실패 / 좌우정렬 실패 / 진입 게이트 초과 / 진입 실패 /
 * 주차오차 초과 - 다섯 군데에서 전부 곧바로 break 해서 셔틀 전체를 끝냈음.
 * 그런데 이 실패들은 대부분 "이번 접근자세가 나빴다"는 뜻일 뿐이고, 사람이라면
 * 차를 빼서 다시 접근하지 시동을 끄지 않는다. 사용자 신고의 "계속 걸려서
 * 멈춰버린다"가 바로 이 구조 때문.
 *
 * 이제 실패하면 아래 순서로 복구하고 slot_max_tries 만큼 다시 시도한다.
 *   1) 슬롯 안쪽에 있으면 대기점 높이까지 후진해서 빼낸다
 *   2) 접촉이 남아 있으면 짧게 풀어준다(이동 상한 4cm - 준비자세 보호)
 *   3) 대기점에서 많이 벗어났으면 A*로 대기점까지 다시 접근한다
 *      (같은 자리에서 같은 각도로 다시 밀면 같은 곳에 또 걸리므로,
 *       '접근 자체를 다시 하는 것'이 핵심)
 * 마지막 시도까지 실패했을 때만 멈춘다.
 * ============================================================ */
static ShuttleAction shuttle__park(ShuttleCtx *sh, ShuttleLeg *lg) {

    int max_tries = (sh->cfg->slot_max_tries > 0) ? sh->cfg->slot_max_tries : 1;
    bool parked = false;

    for (int attempt = 0; attempt < max_tries && !parked && !g_stop_requested; attempt++) {
        if (attempt > 0
            && shuttle__recover_for_retry(sh, lg, attempt, max_tries) == PARK_INTERRUPT)
            break;
        ParkAttempt r = shuttle__park_once(sh, lg, attempt, max_tries);
        if (r == PARK_INTERRUPT) break;
        if (r == PARK_RETRY)     continue;
        parked = true;
    }

    if (!parked) {
if (g_stop_requested) {
    printf("[shuttle] 중단 신호 수신, 셔틀 종료\n");
} else {
    printf("[shuttle] *** %s 주차를 %d번 시도했지만 모두 실패했습니다. "
           "셔틀을 중단합니다. ***\n"
           "       확인할 것: (1) --slot-align-tol 이 2.5 이상인지,\n"
           "       (2) --lidar-yaw / --lidar-offset 이 실측값인지,\n"
           "       (3) --slot-staging 이 통로 정중앙인지,\n"
           "       (4) 슬롯 입구에 물리적 장애물이 없는지\n",
           lg->label, max_tries);
}
        fflush(stdout);
        return g_stop_requested ? SHUTTLE_QUIT : SHUTTLE_ABORT;
    }
    return SHUTTLE_GO;
}


static ShuttleAction shuttle__wait_and_depart(ShuttleCtx *sh, const ShuttleLeg *lg,
                                              bool skip_travel) {

    bool depart_now = false;
    printf("[shuttle] %s 도착 완료\n", lg->label);
    /* ============================================================
     * 시작할 때 이미 슬롯 안이었으면 '엣지'가 아니라 '현재 상태'로 판정한다
     * (2026-08-10h, trigger.h 의 trigger_poll_now() 주석 참고).
     *
     * trigger_wait() 는 "반대 상태를 한 번 거친 뒤 원하는 상태로 전이"를
     * 요구한다. 왕복 중에는 그게 맞지만(짐을 올렸다/내렸다는 사건을 잡아야
     * 하니까), 프로그램을 막 켠 순간은 그 사건이 있었는지 알 수가 없다.
     * 사람은 보통 짐을 올려둔 채로 실행하므로, 그때 엣지를 요구하면 짐을
     * 한 번 내렸다 다시 올려야만 출발하게 되어 이상하다.
     * 그래서 이 첫 회차에 한해, 이미 조건이 충족되어 있으면 바로 출발한다.
     * ============================================================ */
    if (skip_travel) {
        int now = trigger_poll_now(sh->trigger_cfg, lg->wait_kind);
        if (now == 1) {
            printf("[shuttle] %s 출발조건이 이미 충족되어 있습니다 - "
                   "기다리지 않고 바로 후진해서 나갑니다\n", lg->label);
            fflush(stdout);
            depart_now = true;
        }
        if (now == 0) {
            printf("[shuttle] %s 출발조건이 아직 아닙니다 - 트리거를 기다립니다\n",
                   lg->label);
            fflush(stdout);
        }
        /* now == -1 (Enter 트리거 등 상태를 읽을 수 없음) 이면 그냥 아래에서 대기 */
    }
    /* 트리거를 못 받는 상황(EOF 등)에서 예전엔 그냥 통과해서 출발해버렸음 */
    /* (리팩토링) 원본은 여기로 goto shuttle_depart 가 뛰어들어와 이 블록을
     * 통째로 건너뛰었다. depart_now 로 같은 일을 한다. */
    if (!depart_now && !trigger_wait(sh->trigger_cfg, lg->label, lg->wait_kind, &g_stop_requested)) {
        printf("[shuttle] %s 출발신호를 받지 못해 셔틀을 중단합니다\n", lg->label);
        return SHUTTLE_ABORT;
    }

    /* 트리거를 받은 뒤 잠시 대기 (2026-08-09n).
     * 시연에서 사람이 짐을 올리고 손을 빼는 시간이 필요하다. 1초 단위로 남은
     * 시간을 찍어서 관객도 언제 출발할지 알 수 있게 한다. 이 동안 바퀴는
     * 확실히 정지시켜 둔다. */
    if (sh->cfg->trigger_delay_sec > 0.0 && !g_stop_requested) {
        fpga_link_set_speed(sh->fpga, 0, 0);
        printf("[shuttle] 출발신호 확인 - %.1f초 후 출발합니다\n",
               sh->cfg->trigger_delay_sec);
        fflush(stdout);
        double left = sh->cfg->trigger_delay_sec;
        int last_shown = -1;
        while (left > 0.0 && !g_stop_requested) {
            int whole = (int)ceil(left);
            if (whole != last_shown) {
                printf("[shuttle] 출발까지 %d초...\n", whole);
                fflush(stdout);
                last_shown = whole;
            }
            double slice = (left > 0.1) ? 0.1 : left;
            robot_runner__sleep_sec(slice);
            left -= slice;
        }
        if (!g_stop_requested) {
            printf("[shuttle] 출발!\n");
            fflush(stdout);
        }
    }
    return SHUTTLE_GO;
}




/* ============================================================
 * run_shuttle: 지도를 한 번만 획득한 뒤(SLAM매핑/고정맵/파일불러오기 중 선택),
 * point_a <-> point_b를 무한 왕복. 각 지점 도착 후 trigger_wait()로 출발신호를
 * 기다림(A지점=짐 얹힘, B지점=짐 내려감). Ctrl+C 전까지 계속 돎.
 * 스마트팩토리 물류운송(적재/하차 지점 왕복) 용도.
 * ============================================================ */
int run_shuttle(int fpga_fd, int lidar_fd, RobotConfig *cfg,
                MapSource map_source, const char *map_load_path, const char *map_save_path,
                ShuttlePoint point_a, ShuttlePoint point_b,
                const TriggerConfig *trigger_cfg) {
    signal(SIGINT, robot_handle_sigint);
    g_stop_requested = 0;

    printf("[shuttle] starting FPGA link...\n");
    FPGALink fpga;
    fpga_link_init(&fpga, fpga_fd, cfg->controller_max_speed, cfg->pwm_safety_cap,
                    cfg->wheel_diameter, cfg->wheel_separation, cfg->ticks_per_rev, cfg->fpga_send_hz);
    fpga_link_set_encoder_invert(&fpga, cfg->encoder_invert_left, cfg->encoder_invert_right);
    fpga_link_start(&fpga);

    printf("[shuttle] starting lidar...\n");
    RplidarReader reader;
    rplidar_reader_init(&reader, lidar_fd, 2.0, 3);
    LidarThread lidar;
    lidar_thread_start(&lidar, &reader);
    printf("[shuttle] lidar scanning started\n");
    printf("[shuttle] acquiring map (source=%s)...\n",
           map_source == MAP_SOURCE_SLAM ? "slam" : map_source == MAP_SOURCE_LOAD ? "load" : "fixed");
    unsigned char *grid;
    int rows, cols;
    double final_x, final_y, final_theta;
    bool map_ok = acquire_map(&fpga, &lidar, cfg, map_source, map_load_path, map_save_path,
                               &grid, &rows, &cols, &final_x, &final_y, &final_theta);
    if (!map_ok) {
        printf("[shuttle] map acquisition failed or interrupted, cleaning up\n");
        fpga_link_close(&fpga);
        lidar_thread_stop(&lidar);
        return 1;
    }

    /* 움직이기 전에 A/B 좌표가 이 지도에서 말이 되는 값인지 확인 (2026-08-07).
     * 비트연산 | 을 쓴 이유: 두 지점 모두 검사해서 오류를 한꺼번에 보여주기 위함. */
    if ((!shuttle_validate_point(grid, rows, cols, cfg->resolution, cfg, point_a, "A지점")) |
        (!shuttle_validate_point(grid, rows, cols, cfg->resolution, cfg, point_b, "B지점"))) {
        fprintf(stderr, "[shuttle] 목적지 좌표가 잘못되어 시작하지 않습니다.\n");
        free(grid);
        fpga_link_close(&fpga);
        lidar_thread_stop(&lidar);
        return 1;
    }

    DynamicNavigator nav;
    /* pure pursuit 설정 반영 - 슬롯 진입/후진은 dynnav를 안 쓰므로 영향 없음 */
    dynnav_init(&nav, grid, rows, cols, cfg->resolution, cfg->robot_radius, 0.0, 0.0);
    nav.use_pure_pursuit = cfg->pure_pursuit;
    nav.lookahead_m = cfg->lookahead_m;

    /* 지도가 새로 매핑된 것이면(SLAM) 내비게이션 중에도 계속 지도를 다듬어가고,
     * 고정맵/불러온맵이면 지도는 고정하고 위치추정만 계속 보정함. */
    bool use_slam_rolling_map = (map_source == MAP_SOURCE_SLAM);
    OccupancyGridSLAM nav_slam;
    if (use_slam_rolling_map) {
        slam_init(&nav_slam, cfg->grid_size_x, cfg->grid_size_y, cfg->resolution, 0.0, 0.0,
                  final_x, final_y, final_theta, 0.15, 12.0, 0.03, 3.0);
    } else {
        slam_init_from_grid(&nav_slam, grid, rows, cols, cfg->resolution, 0.0, 0.0,
                             final_x, final_y, final_theta, 0.15, 12.0, 0.03, 3.0);
    }
    /* PATCH (2026-08-10): 운동 타당성 게이트를 켠다.
     * 예측이 명령 적분으로 실제 근처에 놓이게 됐으므로(predict_delta), 이제
     * "예측에서 70mm/12도 이상 떨어진 답"은 물리적으로 말이 안 되는 값이다.
     * 로그에서 맵을 회전시킨 도약들(136mm/10도, 114mm/12도)이 전부 여기 걸린다.
     * 이 두 수정은 반드시 한 쌍으로 있어야 한다 - 예측 없이 게이트만 켜면
     * 정상 이동까지 막히고, 게이트 없이 예측만 넣으면 도약이 그대로 통과한다. */
    nav_slam.max_jump_m   = 0.07;
    nav_slam.max_jump_rad = 12.0 * ANGLE_PI / 180.0;
    free(grid);

    /* (2026-08-11c) 존재하는 슬롯 차선을 등록한다. 슬롯 안에서 자세가 흔들릴 때
     * '어느 슬롯인지'를 가정하지 않고 채점으로 고르기 위한 후보 목록이다
     * (robot_runner__slot_identify 머리말 참고). */
    if (cfg->slot_parking) {
        double lanes[2] = { point_a.x, point_b.x };
        robot_runner__slot_lanes_set(lanes, 2);
        printf("[shuttle] 슬롯 차선 등록: x=%.3f, x=%.3f - 슬롯 안에서 자세가 틀어지면\n"
               "          이 둘을 라이다로 채점해서 어느 쪽인지 고릅니다\n",
               point_a.x, point_b.x);
        fflush(stdout);
    }

    ShuttleCtx sh;
    memset(&sh, 0, sizeof(sh));
    sh.fpga = &fpga;  sh.lidar = &lidar;  sh.nav = &nav;  sh.nav_slam = &nav_slam;
    sh.cfg = cfg;     sh.reader = &reader;
    sh.point_a = point_a;  sh.point_b = point_b;
    sh.trigger_cfg = trigger_cfg;
    sh.map_source = map_source;
    sh.use_slam_rolling_map = use_slam_rolling_map;

    if (!shuttle__localize_start(&sh)) return 1;

    robot_runner__init_prev_pose(&fpga, cfg->has_encoder,
                                 &sh.prev_raw_x, &sh.prev_raw_y, &sh.prev_raw_theta);

    printf("[shuttle] shuttle loop starting: A=(%.2f,%.2f,%.1fdeg) B=(%.2f,%.2f,%.1fdeg)\n",
           point_a.x, point_a.y, point_a.theta_deg, point_b.x, point_b.y, point_b.theta_deg);

    int leg = 0;  /* 0: 다음 목적지 A, 1: 다음 목적지 B */
    bool clean_stop = true;

    bool start_parked = cfg->slot_parking
                        ? shuttle__detect_start_parked(&sh, &leg)
                        : false;

    while (!g_stop_requested) {
        ShuttleLeg lg;
        memset(&lg, 0, sizeof(lg));
        lg.target = (leg == 0) ? sh.point_a : sh.point_b;
        lg.label = (leg == 0) ? "A지점" : "B지점";
        lg.wait_kind = (leg == 0) ? TRIGGER_WAIT_LOAD_PRESENT : TRIGGER_WAIT_LOAD_ABSENT;
        slam_get_pose(&nav_slam, &lg.cur_x, &lg.cur_y, &lg.cur_theta);
        lg.target_theta_rad = lg.target.theta_deg * ANGLE_PI / 180.0;

        /* 이번 회차만 주행/주차를 건너뛴다(시작할 때 이미 그 슬롯에 주차되어 있었음).
         * 플래그는 여기서 바로 내려서, 다음 회차부터는 평소대로 돌게 한다. */
        bool skip_travel = start_parked;
        start_parked = false;

        /* (리팩토링) 원본은 여기서 goto shuttle_arrived 로 주행/주차를 통째로
         * 건너뛰었다. 건너뛰던 구간이 한 덩어리라 그대로 if 로 감쌌다. */
        if (skip_travel) {
            printf("[shuttle] %s 에 이미 주차되어 있습니다 (pose=(%.3f,%.3f,%.1fdeg)) "
                   "- 주행/주차 생략\n",
                   lg.label, lg.cur_x, lg.cur_y, lg.cur_theta * 180.0 / ANGLE_PI);
            fflush(stdout);
            fpga_link_set_speed(sh.fpga, 0, 0);
        } else {
            /* 슬롯 주차 모드면 A*의 목적지는 슬롯 안이 아니라 입구 바깥 대기점 */
            lg.nav_goal_x = lg.target.x;
            lg.nav_goal_y = cfg->slot_parking ? cfg->slot_staging_y : lg.target.y;

            ShuttleAction act = SHUTTLE_GO;
            if (cfg->slot_parking) act = shuttle__exit_slot(&sh, &lg);
            if (act == SHUTTLE_ABORT) { clean_stop = false; break; }

            if (!shuttle__is_direct_entry(&sh, &lg)) {
                act = shuttle__travel_to_staging(&sh, &lg);
                if (act == SHUTTLE_ABORT) { clean_stop = false; break; }
                if (act == SHUTTLE_QUIT)  break;
            }

            if (cfg->slot_parking) {
                act = shuttle__park(&sh, &lg);
                if (act == SHUTTLE_ABORT) { clean_stop = false; break; }
                if (act == SHUTTLE_QUIT)  break;
            }
        }

        ShuttleAction act = shuttle__wait_and_depart(&sh, &lg, skip_travel);
        if (act == SHUTTLE_ABORT) { clean_stop = false; break; }
        if (act == SHUTTLE_QUIT)  break;

        leg = 1 - leg;
    }

    printf("[shuttle] cleaning up: stopping motors, stopping lidar, closing link\n");
    fpga_link_set_speed(&fpga, 0, 0);
    struct timespec ts = {0, 100 * 1000000L};
    nanosleep(&ts, NULL);
    fpga_link_close(&fpga);
    lidar_thread_stop(&lidar);
    slam_free(&nav_slam);
    dynnav_free(&nav);
    printf("[shuttle] shutdown complete\n");

    return clean_stop ? 0 : 1;
}
