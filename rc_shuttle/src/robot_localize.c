#include "robot_localize.h"
#include "robot_runner.h"


/* ------------------------------------------------------------
 * Likelihood field (스캔매칭용 부드러운 점수판)
 *
 * PATCH (2026-08-05 v5): 원래 slam_score_pose()는 "스캔 점이 점유셀에 정확히
 * 얹히면 +6, 빈 칸이면 -6, 지도 밖이면 0"이라 점수가 뚝뚝 끊김. 문제가 두 가지:
 *   (1) 5cm 격자에서 자세가 한 칸만 어긋나도 점수가 급락해서, 성긴 격자로 전역탐색을
 *       하면 참값 근처를 아예 못 짚음(실측: 6개 자세 중 2개만 복구).
 *   (2) 지도 밖(0점)이 빈 칸(-6점)보다 점수가 높아서, 로봇을 지도 가장자리로
 *       밀어내는 편향이 있음.
 * 그래서 "가장 가까운 벽까지의 거리"를 미리 계산해두고, 거리가 가까울수록 높은
 * 점수를 주는 부드러운 점수판을 만듦(가우시안). 점수가 연속적이라 성긴 격자로도
 * 참값 골짜기를 찾아갈 수 있고, 벽이 몇 cm 어긋나 있어도(우리 세트장처럼 손으로
 * 이어붙여 모서리가 정확히 90도가 아닌 경우) 급격히 무너지지 않음. 지도 밖은 0점
 * 이고 이제 그게 최저점이므로 가장자리 편향도 사라짐.
 * ------------------------------------------------------------ */

/* 각 칸에서 가장 가까운 점유셀까지의 거리(m)를 2-패스 체임퍼 방식으로 근사 계산한 뒤,
 * 가우시안으로 변환해 0~1 점수판을 만든다. 호출자가 free() 해야 함. */
/* 이 점수판으로 자세를 채점. 반환값은 0 ~ scan->count (전부 벽에 딱 맞으면 최대). */
double slam_score_pose_lf(const OccupancyGridSLAM *s, const double *field,
                                          double x, double y, double theta, const LidarScan *scan) {
    double score = 0.0;
    double ct = cos(theta), st = sin(theta);
    for (int i = 0; i < scan->count; i++) {
        double d = scan->readings[i].dist_mm / 1000.0;
        if (d <= 0.0) continue;
        double a = scan->readings[i].angle_deg * ANGLE_PI / 180.0;
        double ca = cos(a), sa = sin(a);
        /* 회전 = theta + a 를 덧셈정리로 전개 (매 점마다 cos/sin 재계산 안 하려고) */
        double ex = x + d * (ct * ca - st * sa);
        double ey = y + d * (st * ca + ct * sa);
        int r, c;
        slam_world_to_grid(s, ex, ey, &r, &c);
        if (slam_in_bounds(s, r, c)) score += field[slam_idx(s, r, c)];
    }
    return score;
}


/* 주어진 격자에서 로봇이 있을 수 있는 칸인가(=빈 공간인가) */
bool slam__cell_is_free(const OccupancyGridSLAM *s, double wx, double wy) {
    int r, c;
    slam_world_to_grid(s, wx, wy, &r, &c);
    if (!slam_in_bounds(s, r, c)) return false;
    return s->log_odds[slam_idx(s, r, c)] < 0.0;   /* 음수 = 비어있음 */
}


PoseCandidate slam__search_poses(const OccupancyGridSLAM *s, const double *field,
                                                 const LidarScan *scan,
                                                 double x0, double x1, double pos_step,
                                                 double y0, double y1,
                                                 double th0_deg, double th1_deg, double th_step_deg,
                                                 bool skip_occupied) {
    PoseCandidate best = { x0, y0, 0.0, -1e300 };
    for (double th = th0_deg; th <= th1_deg + 1e-9; th += th_step_deg) {
        double thr = th * ANGLE_PI / 180.0;
        for (double px = x0; px <= x1 + 1e-9; px += pos_step) {
            for (double py = y0; py <= y1 + 1e-9; py += pos_step) {
                if (skip_occupied && !slam__cell_is_free(s, px, py)) continue;
                double sc = slam_score_pose_lf(s, field, px, py, thr, scan);
                if (sc > best.score) {
                    best.score = sc; best.x = px; best.y = py; best.theta = thr;
                }
            }
        }
    }
    return best;
}


void slam_global_localize(const OccupancyGridSLAM *s, const LidarScan *scan,
                                          double hint_x, double hint_y, bool have_hint,
                                          double *out_x, double *out_y, double *out_theta) {
    double map_w = s->cols * s->resolution;
    double map_h = s->rows * s->resolution;

    printf("[localize] 시작자세 전역탐색 중... (지도 전체 %.2f x %.2f m, 위치+각도 모두)\n",
           map_w, map_h);
    fflush(stdout);

    double *field = slam_build_likelihood_field(s);
    if (!field) {
        fprintf(stderr, "[localize] 메모리 부족 - 힌트 위치를 그대로 사용합니다\n");
        *out_x = hint_x; *out_y = hint_y; *out_theta = 0.0;
        return;
    }

    /* 1단계: 지도 전체를 성긴 간격으로.
     * PATCH (2026-08-05 v9): 예전엔 격자 해상도(s->resolution) 간격으로 훑었는데,
     * 지도를 1cm 해상도로 올리면 후보 자세 수가 25배가 되어 몇 분씩 걸림. 탐색 간격은
     * 지도 해상도와 무관해야 하므로 5cm로 고정(부드러운 점수판이라 이 정도로 성겨도
     * 참값 골짜기를 찾아감 - 6/6 복구로 검증됨). 정밀도는 2단계에서 확보. */
    const double COARSE_STEP_M = 0.05;
    PoseCandidate coarse = slam__search_poses(s, field, scan,
        s->origin_x, s->origin_x + map_w, COARSE_STEP_M,
        s->origin_y, s->origin_y + map_h,
        0.0, 357.0, 3.0, true);

    /* 2단계: 그 주변만 촘촘하게 */
    double cdeg = coarse.theta * 180.0 / ANGLE_PI;
    PoseCandidate fine = slam__search_poses(s, field, scan,
        coarse.x - 0.05, coarse.x + 0.05, 0.01,
        coarse.y - 0.05, coarse.y + 0.05,
        cdeg - 4.0, cdeg + 4.0, 0.5, false);
    if (fine.score < coarse.score) fine = coarse;

    double best_deg = fine.theta * 180.0 / ANGLE_PI;
    while (best_deg < 0) best_deg += 360.0;
    while (best_deg >= 360.0) best_deg -= 360.0;

    /* 점수판이 0~1이므로, 점수/점개수 = 평균 일치도(1이면 완벽) */
    double fit = (scan->count > 0) ? fine.score / scan->count * 100.0 : 0.0;

    printf("[localize] 결과: (%.3f, %.3f, %.1f deg)  스캔-지도 일치도 %.0f%%\n",
           fine.x, fine.y, best_deg, fit);

    if (fit < 60.0) {
        printf("[localize] *** 경고: 일치도가 낮습니다(%.0f%%). 지도(set_map.txt)가 실제\n"
               "[localize]     세트장과 다르거나, 라이다 시야에 지도에 없는 물체가 있을 수\n"
               "[localize]     있습니다. live_view.py로 스캔과 지도를 겹쳐서 확인하세요. ***\n", fit);
    }
    if (have_hint) {
        double off = hypot(fine.x - hint_x, fine.y - hint_y);
        printf("[localize] --start (%.2f, %.2f) 로부터 %.2f m 떨어진 곳으로 판단했습니다.\n",
               hint_x, hint_y, off);
    }
    fflush(stdout);

    free(field);
    *out_x = fine.x; *out_y = fine.y; *out_theta = fine.theta;
}
