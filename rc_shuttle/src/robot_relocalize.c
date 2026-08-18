#include "robot_relocalize.h"
#include "robot_runner.h"


/* ============================================================
 * 주행 중 전역 재위치추정 (2026-08-10 신규)
 *
 * 사용자 질문: "라이다로 보이는 윤곽이 맵의 어느 부분인지 훤히 보이는데,
 *   1초쯤 멈춰도 되니까 제대로 맞추면 안 되나? 이게 스캔매칭 역할 아닌가?"
 *
 * 맞는 말이고, 못 맞춘 이유는 '능력'이 아니라 '범위'였다.
 * slam_localize_step() 이 쓰는 scan_match_correct_pose() 는 예측자세 주변
 *     위치 +-0.15m, 각도 +-12도 (회전 중에는 +-0.10m / +-25도)
 * 만 후보로 놓고 점수를 매긴다. 즉 90도 틀어진 자세는 애초에 '후보 목록에
 * 들어가지도 않는다'. 스캔이 아무리 또렷해도 평가조차 안 하니 찾을 수가 없다.
 * 이건 스캔매칭이 실패한 게 아니라, 우리가 국소 미세보정(local refinement)만
 * 시키고 전역 인식(global recognition)은 시킨 적이 없는 것이다.
 *
 * 넓은 재탐색(scan_match_recover)도 +-0.30m / +-25도뿐이라 90도 오차는
 * 원리적으로 복구 불가능하다. 반면 시동 때 쓰는 slam_global_localize() 는
 * 지도 전체 x 0~357도를 다 훑는다 - 정확히 사용자가 말한 그 동작이다.
 * 그런데 그걸 딱 한 번, 시동 때만 쓰고 주행 중에는 한 번도 안 썼다.
 *
 * 그래서 이 함수를 만든다: 길을 잃었다고 판단되면 멈추고, 깨끗한 스캔을 찍고,
 * 지도 전체를 훑어서 다시 잡는다. 약 0.6~1.2초 걸리지만, 틀린 자세로 계속
 * 움직이는 것보다 압도적으로 낫다.
 *
 * 안전장치: 전역탐색 결과가 지금 자세보다 '확실히' 좋을 때만 채택한다.
 * 이 지도는 A/B 슬롯 모양이 같아서 엉뚱한 슬롯으로 순간이동할 위험이 있는데,
 * 일치도가 뚜렷하게(+8%p) 좋아질 때만 받으면 그 위험을 억제할 수 있다.
 * ============================================================ */

/* 지금 자세에서의 스캔-지도 일치도(0~1). slam__match_quality 와 같은 척도. */
double robot_runner__map_fit(const OccupancyGridSLAM *s, const LidarScan *scan,
                                            double x, double y, double th) {
    if (s->lf_field == NULL || scan->count <= 0) return 1.0;
    double sc = slam_score_pose_lf(s, s->lf_field, x, y, th, scan);
    return sc / (double)scan->count;
}


/* 반환 true = 자세를 바꿨음. */
bool robot_runner__relocalize_global(LidarThread *lidar, const RobotConfig *cfg,
                                                    OccupancyGridSLAM *slam, const char *why) {
    if (slam == NULL || slam->lf_field == NULL) return false;

    /* 1) 완전히 멈추고 흔들림이 가라앉기를 기다린다 - 움직이며 찍은 스캔으로
     *    전역탐색을 하면 뭉개진 스캔이 엉뚱한 봉우리를 만든다. */
    robot_runner__sleep_sec(cfg->sg_settle_sec);
    LidarScan scan;
    if (!robot_runner__take_scan(lidar, cfg, &scan, 2.0)) return false;
    if (scan.count < 60) return false;

    double cx = slam->x, cy = slam->y, cth = slam->theta;
    double fit_now = robot_runner__map_fit(slam, &scan, cx, cy, cth);

    double map_w = slam->cols * slam->resolution;
    double map_h = slam->rows * slam->resolution;
    printf("[slam] *** 전역 재위치추정 시작 (%s) ***\n"
           "       현재 자세 (%.3f,%.3f,%.0fdeg) 일치도 %.0f%% - "
           "지도 전체 %.2fx%.2fm x 0~360도를 다시 훑습니다\n",
           why, cx, cy, cth * 180.0 / ANGLE_PI, fit_now * 100.0, map_w, map_h);
    fflush(stdout);

    /* 2) 거친 전역탐색: 5cm / 3도. 이미 만들어 둔 점수판을 재사용한다
     *    (시동 때처럼 새로 만들면 수백 ms 를 그냥 버린다). */
    PoseCandidate coarse = slam__search_poses(slam, slam->lf_field, &scan,
        slam->origin_x, slam->origin_x + map_w, 0.05,
        slam->origin_y, slam->origin_y + map_h,
        0.0, 357.0, 3.0, true);

    /* 3) 그 주변만 촘촘하게: 1cm / 0.5도 */
    double cdeg = coarse.theta * 180.0 / ANGLE_PI;
    PoseCandidate fine = slam__search_poses(slam, slam->lf_field, &scan,
        coarse.x - 0.05, coarse.x + 0.05, 0.01,
        coarse.y - 0.05, coarse.y + 0.05,
        cdeg - 4.0, cdeg + 4.0, 0.5, false);
    if (fine.score < coarse.score) fine = coarse;

    double fit_new = (scan.count > 0) ? fine.score / (double)scan.count : 0.0;

    /* 4) 채택 판정. '확실히' 좋을 때만 - 슬롯 두 개가 똑같이 생긴 지도라
     *    조금 나은 정도로 옮기면 엉뚱한 슬롯으로 순간이동할 수 있다. */
    const double ACCEPT_GAIN = 0.08;
    double moved   = hypot(fine.x - cx, fine.y - cy);
    double turned  = fabs(normalize_angle(fine.theta - cth)) * 180.0 / ANGLE_PI;

    /* 운동 예산 검사 (2026-08-10e) - robot_runner__travel_acc() 주석 참고.
     * 일치도만 보면 슬롯 입구에서 '슬롯 안 깊숙한 자세'가 더 좋게 나오는 함정이 있다.
     * 마지막으로 자세를 믿었던 순간부터 명령한 이동거리보다 멀리는 못 옮긴다. */
    double budget = robot_runner__reloc_budget();
    if (moved > budget) {
        printf("[slam] 전역 재위치추정 거부: 후보가 %.0fmm 떨어져 있는데 "
               "그동안 명령한 이동은 %.0fmm 뿐입니다 (허용 %.0fmm).\n"
               "       일치도 %.0f%% -> %.0f%% 로 좋아 보여도, 로봇은 그만큼 갈 수 "
               "없었으므로 다른 자리를 잘못 고른 것입니다. 자세를 유지합니다.\n",
               moved * 1000.0, (*robot_runner__travel_acc()) * 1000.0, budget * 1000.0,
               fit_now * 100.0, fit_new * 100.0);
        fflush(stdout);
        return false;
    }

    if (fit_new < fit_now + ACCEPT_GAIN) {
        printf("[slam] 전역 재위치추정 보류: 후보 (%.3f,%.3f,%.0fdeg) 일치도 %.0f%% 로 "
               "지금(%.0f%%)보다 뚜렷하게 낫지 않아 자세를 유지합니다\n",
               fine.x, fine.y, fine.theta * 180.0 / ANGLE_PI,
               fit_new * 100.0, fit_now * 100.0);
        fflush(stdout);
        return false;
    }

    printf("[slam] *** 전역 재위치추정 완료: (%.3f,%.3f,%.0fdeg) -> (%.3f,%.3f,%.0fdeg) ***\n"
           "       일치도 %.0f%% -> %.0f%%, %.0fmm / %.0fdeg 교정\n",
           cx, cy, cth * 180.0 / ANGLE_PI,
           fine.x, fine.y, fine.theta * 180.0 / ANGLE_PI,
           fit_now * 100.0, fit_new * 100.0, moved * 1000.0, turned);
    fflush(stdout);

    slam->x = fine.x; slam->y = fine.y; slam->theta = fine.theta;
    slam->bad_match_streak = 0;
    slam->relocalize_request = 0;
    slam->last_match_quality = fit_new;
    fpga_cmd_track_reset();      /* 멈춰 있었으므로 쌓인 명령 적분은 버린다 */
    robot_runner__travel_acc_reset();   /* 자세를 새로 확정했으므로 예산도 초기화 */
    return true;
}


/* ============================================================
 * 넓은 국소 재위치추정 (2026-08-10d 신규)
 *
 * 왜 전역탐색이 아니라 이것인가:
 * 슬롯 구간에서는 A/B 두 슬롯이 완전히 똑같이 생겨서, 지도 전체를 훑으면
 * '반대쪽 슬롯'으로 순간이동할 위험이 있다(이 코드베이스에서 실제로 겪은 사고).
 * 반면 실기에서 실제로 벌어지는 오차는 146~211mm 수준이다. 국소 스캔매칭의
 * 창(+-150mm)보다는 크고 슬롯 간격(1.05m)보다는 훨씬 작다.
 * 그래서 딱 그 사이(+-0.35m / +-30도)만 훑는다. 정답은 반드시 이 안에 있고,
 * 반대쪽 슬롯은 절대 후보에 들어오지 않는다.
 *
 * 채택 조건은 전역탐색과 같은 취지로 '뚜렷하게 좋을 때만'(+5%p).
 * ============================================================ */
bool robot_runner__relocalize_wide(LidarThread *lidar, const RobotConfig *cfg,
                                                  OccupancyGridSLAM *slam, const char *why) {
    if (slam == NULL || slam->lf_field == NULL) return false;
    LidarScan scan;
    if (!robot_runner__take_scan(lidar, cfg, &scan, 2.0)) return false;
    if (scan.count <= 0) return false;

    double cx, cy, cth;
    slam_get_pose(slam, &cx, &cy, &cth);
    double fit_now = robot_runner__map_fit(slam, &scan, cx, cy, cth);
    double cdeg = cth * 180.0 / ANGLE_PI;

    /* 탐색 반경은 '실제로 갈 수 있었던 거리'를 넘지 않게 한다 (2026-08-10e) */
    double WIN_M = robot_runner__reloc_budget();
    if (WIN_M > 0.35) WIN_M = 0.35;
    if (WIN_M < 0.08) WIN_M = 0.08;
    const double WIN_DEG = 30.0;
    PoseCandidate coarse = slam__search_poses(slam, slam->lf_field, &scan,
        cx - WIN_M, cx + WIN_M, 0.02,
        cy - WIN_M, cy + WIN_M,
        cdeg - WIN_DEG, cdeg + WIN_DEG, 2.0, false);
    PoseCandidate fine = slam__search_poses(slam, slam->lf_field, &scan,
        coarse.x - 0.02, coarse.x + 0.02, 0.005,
        coarse.y - 0.02, coarse.y + 0.02,
        coarse.theta * 180.0 / ANGLE_PI - 3.0,
        coarse.theta * 180.0 / ANGLE_PI + 3.0, 0.5, false);
    if (fine.score < coarse.score) fine = coarse;

    double fit_new = fine.score / (double)scan.count;
    double moved = hypot(fine.x - cx, fine.y - cy);
    if (fit_new < fit_now + 0.05) {
        printf("[slam] 넓은 재위치추정 보류(%s): 후보 일치도 %.0f%% 가 지금 %.0f%% 보다 "
               "뚜렷하게 낫지 않습니다\n", why, fit_new * 100.0, fit_now * 100.0);
        fflush(stdout);
        return false;
    }
    printf("[slam] *** 넓은 재위치추정(%s): (%.3f,%.3f,%.0fdeg) -> (%.3f,%.3f,%.0fdeg) ***\n"
           "       일치도 %.0f%% -> %.0f%%, %.0fmm 교정 (탐색범위 +-%.0fmm/+-%.0fdeg - "
           "반대쪽 슬롯은 후보에 없음)\n",
           why, cx, cy, cdeg, fine.x, fine.y, fine.theta * 180.0 / ANGLE_PI,
           fit_now * 100.0, fit_new * 100.0, moved * 1000.0,
           WIN_M * 1000.0, WIN_DEG);
    fflush(stdout);
    slam->x = fine.x; slam->y = fine.y; slam->theta = fine.theta;
    slam->last_match_quality = fit_new;
    slam->bad_match_streak = 0;
    slam->relocalize_request = 0;
    fpga_cmd_track_reset();
    robot_runner__travel_acc_reset();
    return true;
}
