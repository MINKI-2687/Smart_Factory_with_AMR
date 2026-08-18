#include "robot_pose_gate.h"
#include "robot_runner.h"


/* ============================================================
 * "이 자세를 믿고 움직여도 되는가" - 슬롯 구간 품질 관문 (2026-08-10f 신규)
 *
 * 지금까지 없던 것은 '감시'가 아니라 '거부권'이었다. check_lost() 는 한 번 재보고
 * 나쁘면 전역탐색을 시도했지만, 실패해도 호출부는 그냥 다음 줄로 넘어갔다.
 * 이 함수는 고칠 때까지 시도하고, 끝내 못 고치면 false 를 돌려 호출부가
 * '움직이지 않게' 만든다.
 *
 * 고치는 순서 (좁은 것부터 - 엉뚱한 곳으로 튈 위험이 작은 순서):
 *   1) 넓은 국소탐색  : 운동 예산 안에서만. A/B 슬롯 혼동 위험 없음.
 *   2) 전역탐색       : 지도 전체. 역시 운동 예산으로 제한됨.
 *   3) 매번 새 스캔으로 다시 측정 (스캔 한 장의 잡음에 속지 않기 위함)
 * ============================================================ */
bool robot_runner__require_good_pose(FPGALink *fpga, LidarThread *lidar,
                                                    const RobotConfig *cfg,
                                                    OccupancyGridSLAM *slam,
                                                    const char *where) {
    if (slam == NULL || slam->lf_field == NULL) return true;
    double need = cfg->min_match_quality;
    if (need <= 0.0) return true;              /* 감시 끔 */

    /* ============================================================
     * 방금 통과한 검사를 또 하지 않는다 (2026-08-10j)
     *
     * 진입 순서에 이 함수가 연달아 두 번 나온다.
     *     ... slot_align -> require_good_pose("슬롯 정렬 시작 전")
     *         -> (좌우오차가 30mm 이내면 slot_lateral_align 생략)
     *         -> 진입 게이트(새 스캔 안 씀) -> require_good_pose("진입 직전 최종")
     * 좌우 정렬이 생략되면 두 호출 사이에 바퀴가 한 번도 안 돈다. 그런데 이
     * 함수는 매번 정지 -> settle -> 새 스캔 2바퀴 -> 채점을 다시 하므로
     * 약 0.33초를 그냥 버린다(입력이 같으니 결과도 같다).
     *
     * 그래서 '마지막으로 통과한 자세와 시각'을 기억해 두고, 자세가 그대로이고
     * 얼마 지나지 않았으면 재측정을 건너뛴다. 조금이라도 움직였거나 시간이
     * 지났으면 예전처럼 다시 잰다 - 건너뛰는 조건이 "아무 일도 없었다"이므로
     * 감시가 느슨해지지 않는다.
     * ============================================================ */
    static double last_ok_x = 1e9, last_ok_y = 1e9, last_ok_th = 1e9;
    static double last_ok_t = -1e9;
    const double POSE_SAME_M   = 0.001;
    const double POSE_SAME_RAD = 0.004;    /* 약 0.2도 */
    const double POSE_CACHE_SEC = 1.5;
    {
        struct timespec now_ts;
        clock_gettime(CLOCK_MONOTONIC, &now_ts);
        double now = now_ts.tv_sec + now_ts.tv_nsec * 1e-9;
        if (now - last_ok_t < POSE_CACHE_SEC
            && fabs(slam->x - last_ok_x) < POSE_SAME_M
            && fabs(slam->y - last_ok_y) < POSE_SAME_M
            && fabs(normalize_angle(slam->theta - last_ok_th)) < POSE_SAME_RAD) {
            printf("[slam] %s: %.2f초 전 검사 이후 바퀴가 한 번도 안 돌았으므로 "
                   "재측정을 건너뜁니다\n", where, now - last_ok_t);
            fflush(stdout);
            return true;
        }
    }

    fpga_link_set_speed(fpga, 0, 0);
    robot_runner__sleep_sec(cfg->sg_settle_sec);

    for (int try_i = 0; try_i < 3 && !g_stop_requested; try_i++) {
        LidarScan scan;
        if (!robot_runner__take_scan(lidar, cfg, &scan, 2.0)) return false;
        double fit = robot_runner__map_fit(slam, &scan, slam->x, slam->y, slam->theta);
        if (fit >= need) {
            if (try_i > 0) {
                printf("[slam] %s: 자세를 다시 잡았습니다 - 일치도 %.0f%% (기준 %.0f%%). "
                       "이어서 진행합니다\n", where, fit * 100.0, need * 100.0);
                fflush(stdout);
            }
            if (fit >= 0.92) robot_runner__travel_acc_reset();
            /* 통과한 자세와 시각을 기억해 둔다 - 위 '재측정 건너뛰기' 참고 */
            {
                struct timespec ok_ts;
                clock_gettime(CLOCK_MONOTONIC, &ok_ts);
                last_ok_t  = ok_ts.tv_sec + ok_ts.tv_nsec * 1e-9;
                last_ok_x  = slam->x;
                last_ok_y  = slam->y;
                last_ok_th = slam->theta;
            }
            return true;
        }
        printf("[slam] *** %s: 일치도 %.0f%% < 기준 %.0f%% - 이 자세로는 움직이지 "
               "않습니다 (%d/3) ***\n"
               "       현재 자세 (%.3f,%.3f,%.0fdeg), 재탐색 허용범위 %.0fmm "
               "(그동안 명령한 이동 %.0fmm)\n",
               where, fit * 100.0, need * 100.0, try_i + 1,
               slam->x, slam->y, slam->theta * 180.0 / ANGLE_PI,
               robot_runner__reloc_budget() * 1000.0,
               (*robot_runner__travel_acc()) * 1000.0);
        fflush(stdout);

        if (robot_runner__relocalize_wide(lidar, cfg, slam, where)) continue;
        if (robot_runner__relocalize_global(lidar, cfg, slam, where)) continue;
        /* 둘 다 실패했으면 스캔 한 장을 더 받아 다시 재본다 - 스캔 잡음일 수 있다 */
        robot_runner__sleep_sec(cfg->sg_settle_sec);
    }

    printf("[slam] *** %s: 자세를 끝내 못 믿겠습니다. 여기서 멈춥니다. ***\n"
           "       확인할 것:\n"
           "       (1) 세트장에 지도에 없는 물체(손/짐/케이블)가 라이다에 보이는지\n"
           "       (2) --lidar-yaw 가 맞는지 (라이다 0도가 차체 정면인지)\n"
           "       (3) --min-match-quality (현재 %.2f)가 이 세트장에 너무 높지 않은지\n",
           where, need);
    fflush(stdout);
    return false;
}


/* ============================================================
 * 큰 이동 직후 위치 다시 잡기 (2026-08-10 신규)
 *
 * 끼임 탈출은 자세추정을 한 번도 갱신하지 않은 채 최대 225mm 를 이동한다.
 * 그 상태로 호출부가 slam_get_pose() 를 부르면 '탈출 전 자세'가 나오고,
 * 그걸로 경로를 다시 그리면 방금 빠져나온 자리로 되돌아가는 경로가 나온다
 * (= 사용자가 본 왕복의 마지막 고리). 게다가 다음 스텝의 탐색창은 ±100mm 뿐이라
 * 실제 이동량이 그보다 크면 정답이 창 밖이라 영영 못 따라간다.
 *
 * 그래서 탈출 직후 여기서 한 번, 넓은 창으로 위치를 다시 잡는다.
 *   - 탐색창을 ±0.22m / ±8도 로 넓힌다(이동은 컸지만 회전은 안 했으므로 각도는 좁게)
 *   - cmd_odom 에 쌓인 '명령한 이동량'을 예측으로 넘겨서 창의 중심을 맞춘다
 *   - 이 한 번만 운동 타당성 게이트를 끈다(큰 이동이 정상인 상황이므로)
 * ============================================================ */
void robot_runner__relocalize_after_move(LidarThread *lidar,
                                                        const RobotConfig *cfg,
                                                        OccupancyGridSLAM *slam) {
    if (slam == NULL) return;
    LidarScan scan;
    if (!robot_runner__take_scan(lidar, cfg, &scan, 2.0)) return;

    double win_m = slam->search_window_m,   step_m  = slam->search_step_m;
    double win_th = slam->search_window_theta, step_th = slam->search_step_theta;
    double jm = slam->max_jump_m, jr = slam->max_jump_rad;

    slam->search_window_m     = 0.22;
    slam->search_step_m       = 0.02;
    slam->search_window_theta = 8.0 * ANGLE_PI / 180.0;
    slam->search_step_theta   = 2.0 * ANGLE_PI / 180.0;
    slam->max_jump_m = 0.0;            /* 이 한 번은 게이트 해제 */
    slam->max_jump_rad = 0.0;

    double odx, ody, odth;
    robot_runner__cmd_odom_take(cfg, rot_rate_shared(cfg, cfg->sg_step_speed),
                                 slam->theta, &odx, &ody, &odth);

    double bx = slam->x, by = slam->y;
    double x, y, th;
    slam_localize_step(slam, &scan, odx, ody, odth, &x, &y, &th);

    slam->search_window_m     = win_m;   slam->search_step_m     = step_m;
    slam->search_window_theta = win_th;  slam->search_step_theta = step_th;
    slam->max_jump_m = jm;               slam->max_jump_rad = jr;
    slam->bad_match_streak = 0;

    printf("[main] 탈출 후 위치 재확인: (%.3f,%.3f) -> (%.3f,%.3f) "
           "[%.0fmm 반영, 명령상 %.0fmm, 품질 %.2f]\n",
           bx, by, x, y, hypot(x - bx, y - by) * 1000.0, hypot(odx, ody) * 1000.0,
           slam->last_match_quality);
    fflush(stdout);
}
