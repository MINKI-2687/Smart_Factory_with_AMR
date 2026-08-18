#include "slam.h"


int slam_idx(const OccupancyGridSLAM *s, int row, int col) {
    return row * s->cols + col;
}


bool slam_in_bounds(const OccupancyGridSLAM *s, int row, int col) {
    return row >= 0 && row < s->rows && col >= 0 && col < s->cols;
}


void slam_world_to_grid(const OccupancyGridSLAM *s, double x, double y,
                                       int *row, int *col) {
    *col = (int)((x - s->origin_x) / s->resolution);
    *row = (int)((y - s->origin_y) / s->resolution);
}


void slam_init(OccupancyGridSLAM *s, double size_x, double size_y, double resolution,
                              double origin_x, double origin_y,
                              double start_x, double start_y, double start_theta,
                              double search_window_m, double search_window_theta_deg,
                              double search_step_m, double search_step_theta_deg) {
    s->resolution = resolution;
    s->origin_x = origin_x;
    s->origin_y = origin_y;
    s->cols = (int)(size_x / resolution);
    s->rows = (int)(size_y / resolution);
    s->lf_field = NULL;
    s->lf_field_fine = NULL;
    s->last_match_quality = 1.0;
    s->max_jump_m = 0.0;          /* 0 = 게이트 끔 (호출부가 켜서 씀) */
    s->max_jump_rad = 0.0;
    s->bad_match_streak = 0;
    s->relocalize_request = 0;
    s->reloc_req_move_m = 0.0;
    s->wide_when_q_below = 0.70;
    s->log_odds = (double *)calloc((size_t)s->rows * s->cols, sizeof(double));
    s->x = start_x; s->y = start_y; s->theta = start_theta;
    s->has_scanned = false;
    s->search_window_m = search_window_m;
    s->search_window_theta = search_window_theta_deg * ANGLE_PI / 180.0;
    s->search_step_m = search_step_m;
    s->search_step_theta = search_step_theta_deg * ANGLE_PI / 180.0;
}


void slam_free(OccupancyGridSLAM *s) {
    free(s->lf_field);
    s->lf_field = NULL;
    free(s->lf_field_fine);
    s->lf_field_fine = NULL;
    free(s->log_odds);
    s->log_odds = NULL;
}


void slam_get_pose(const OccupancyGridSLAM *s, double *x, double *y, double *theta) {
    *x = s->x; *y = s->y; *theta = s->theta;
}


/* ---------------- 고정맵(이미 알고 있는 지도) 초기화 ----------------
 * "고정맵 모드"는 원래 순수 오도메트리(엔코더)만으로 위치를 추정했는데, 실측 검증 결과
 * 방 하나 가로지르는 정도의 이동에서도 이미 ~40cm급 드리프트가 쌓이는 것으로 확인됨
 * (엔코더 20틱/회전 해상도의 양자화 오차가 누적됨). 로봇 스스로는 "거의 정확히
 * 도착했다"고 믿는데 실제로는 수십cm 떨어진 곳에 있는, 겉으로 티도 안 나는 위험한
 * 실패였음. 지도를 이미 알고 있으면 그 지도에 라이다 스캔을 계속 대조(scan matching)
 * 해서 위치를 주기적으로 보정할 수 있으므로, 오도메트리 단독 사용은 근거가 없음.
 *
 * 0/1 이진 점유격자(raw_grid)를 log_odds로 미리 채워서 SLAM 인스턴스를 초기화.
 * 이미 확정된 지도이므로 각 칸을 극단값(완전 점유/완전 빈공간)으로 채워서, 이후
 * slam_localize_step()이 스캔매칭할 때 이 지도를 "고정된 사실"로 취급하게 함. */
void slam_init_from_grid(OccupancyGridSLAM *s, const unsigned char *raw_grid,
                                        int rows, int cols, double resolution,
                                        double origin_x, double origin_y,
                                        double start_x, double start_y, double start_theta,
                                        double search_window_m, double search_window_theta_deg,
                                        double search_step_m, double search_step_theta_deg) {
    s->resolution = resolution;
    s->origin_x = origin_x;
    s->origin_y = origin_y;
    s->rows = rows;
    s->cols = cols;
    s->lf_field = NULL;
    s->lf_field_fine = NULL;
    s->last_match_quality = 1.0;
    s->max_jump_m = 0.0;          /* 0 = 게이트 끔 (호출부가 켜서 씀) */
    s->max_jump_rad = 0.0;
    s->bad_match_streak = 0;
    s->relocalize_request = 0;
    s->reloc_req_move_m = 0.0;
    s->wide_when_q_below = 0.70;
    s->log_odds = (double *)malloc(sizeof(double) * (size_t)rows * cols);
    for (int i = 0; i < rows * cols; i++) {
        s->log_odds[i] = raw_grid[i] ? SLAM_LOG_ODDS_MAX : SLAM_LOG_ODDS_MIN;
    }
    s->x = start_x; s->y = start_y; s->theta = start_theta;
    s->has_scanned = true;   /* 지도가 이미 있으니 첫 스텝부터 바로 스캔매칭 보정 시도 */
    s->search_window_m = search_window_m;
    s->search_window_theta = search_window_theta_deg * ANGLE_PI / 180.0;
    s->search_step_m = search_step_m;
    s->search_step_theta = search_step_theta_deg * ANGLE_PI / 180.0;
}


int slam_bresenham(int r0, int c0, int r1, int c1,
                                  int *out_rows, int *out_cols, int max_points) {
    int count = 0;
    int dr = abs(r1 - r0), dc = abs(c1 - c0);
    int sr = (r0 < r1) ? 1 : -1;
    int sc = (c0 < c1) ? 1 : -1;
    int err = dr - dc;
    int r = r0, c = c0;
    while (true) {
        if (count < max_points) {
            out_rows[count] = r;
            out_cols[count] = c;
        }
        count++;
        if (r == r1 && c == c1) break;
        int e2 = 2 * err;
        if (e2 > -dc) { err -= dc; r += sr; }
        if (e2 < dr) { err += dr; c += sc; }
    }
    return count;
}

double g_slam_sigma_coarse = 0.06;
double g_slam_sigma_fine   = 0.04;

double *slam_build_likelihood_field(const OccupancyGridSLAM *s) {
    return slam_build_likelihood_field_sigma(s, g_slam_sigma_coarse);
}


double *slam_build_likelihood_field_sigma(const OccupancyGridSLAM *s,
                                                         double sigma_m) {
    int R = s->rows, C = s->cols;
    double res = s->resolution;
    double *dist = (double *)malloc(sizeof(double) * (size_t)R * C);
    if (!dist) return NULL;
    const double BIG = 1e9;

    for (int r = 0; r < R; r++)
        for (int c = 0; c < C; c++)
            dist[r * C + c] = (s->log_odds[slam_idx(s, r, c)] > 0.0) ? 0.0 : BIG;

    /* 정방향 패스 */
    for (int r = 0; r < R; r++) {
        for (int c = 0; c < C; c++) {
            double d = dist[r * C + c];
            if (r > 0)            d = fmin(d, dist[(r - 1) * C + c] + 1.0);
            if (c > 0)            d = fmin(d, dist[r * C + (c - 1)] + 1.0);
            if (r > 0 && c > 0)   d = fmin(d, dist[(r - 1) * C + (c - 1)] + 1.41421356);
            if (r > 0 && c < C-1) d = fmin(d, dist[(r - 1) * C + (c + 1)] + 1.41421356);
            dist[r * C + c] = d;
        }
    }
    /* 역방향 패스 */
    for (int r = R - 1; r >= 0; r--) {
        for (int c = C - 1; c >= 0; c--) {
            double d = dist[r * C + c];
            if (r < R-1)            d = fmin(d, dist[(r + 1) * C + c] + 1.0);
            if (c < C-1)            d = fmin(d, dist[r * C + (c + 1)] + 1.0);
            if (r < R-1 && c < C-1) d = fmin(d, dist[(r + 1) * C + (c + 1)] + 1.41421356);
            if (r < R-1 && c > 0)   d = fmin(d, dist[(r + 1) * C + (c - 1)] + 1.41421356);
            dist[r * C + c] = d;
        }
    }

    /* PATCH (2026-08-05 v8): 거리를 '칸 중심까지'로 재면 벽 셀 중심에서 0이 되는데,
     * 라이다가 실제로 맞히는 건 벽의 *표면*이고 표면은 셀 중심보다 반 칸(2.5cm) 앞임.
     * 보정 없이 쓰면 최적화가 스캔 끝점을 셀 중심 쪽으로 당기려고 로봇 자세를 통째로
     * 반 칸 밀어버림 - 실측으로 약 30mm의 계통 편향이 확인됐음. 반 칸을 빼서 벽
     * 표면에서 거리 0이 되도록 맞춤. */
    for (int i = 0; i < R * C; i++) {
        double dm = fmax(0.0, dist[i] - 0.5) * res;      /* 칸 단위 -> 미터, 표면 기준 */
        dist[i] = exp(-(dm * dm) / (2.0 * sigma_m * sigma_m));
    }
    return dist;
}


void slam__cache_scan(ScanTrigCache *c, const LidarScan *scan, int stride) {
    c->n = 0;
    if (stride < 1) stride = 1;
    for (int i = 0; i < scan->count; i += stride) {
        double dm = scan->readings[i].dist_mm;
        if (dm <= 0) continue;
        double a = scan->readings[i].angle_deg * ANGLE_PI / 180.0;
        c->ca[c->n] = cos(a);
        c->sa[c->n] = sin(a);
        c->d[c->n]  = dm / 1000.0;
        c->n++;
    }
}


/* 임의의 점수판(field)에서 이중선형 보간 샘플 (2026-08-09n) */
double slam__lf_sample_field(const OccupancyGridSLAM *s, const double *field,
                                            double wx, double wy) {
    double fc = (wx - s->origin_x) / s->resolution - 0.5;
    double fr = (wy - s->origin_y) / s->resolution - 0.5;
    int c0 = (int)floor(fc), r0 = (int)floor(fr);
    double tx = fc - c0, ty = fr - r0;
    double acc = 0.0;
    for (int dr = 0; dr <= 1; dr++) {
        for (int dc = 0; dc <= 1; dc++) {
            int r = r0 + dr, c = c0 + dc;
            double w = (dr ? ty : 1.0 - ty) * (dc ? tx : 1.0 - tx);
            if (r >= 0 && r < s->rows && c >= 0 && c < s->cols)
                acc += w * field[r * s->cols + c];
        }
    }
    return acc;
}


/* slam_score_pose()와 동일한 점수를 내되, 캐시된 삼각함수 값을 씀.
 * field 를 지정하면 그 점수판을 쓴다(거친 탐색=넓은 판, 정밀 탐색=좁은 판). */
double slam__score_cached_field(const OccupancyGridSLAM *s, const ScanTrigCache *c,
                                               const double *field,
                                               double x, double y, double theta) {
    double ct = cos(theta), st = sin(theta), score = 0.0;
    for (int i = 0; i < c->n; i++) {
        double d = c->d[i];
        double ex = x + d * (ct * c->ca[i] - st * c->sa[i]);
        double ey = y + d * (st * c->ca[i] + ct * c->sa[i]);
        if (field != NULL) {
            score += 12.0 * slam__lf_sample_field(s, field, ex, ey);
        } else {
            int r, cc;
            slam_world_to_grid(s, ex, ey, &r, &cc);
            if (slam_in_bounds(s, r, cc)) score += s->log_odds[slam_idx(s, r, cc)];
        }
    }
    return score;
}


double slam__score_cached(const OccupancyGridSLAM *s, const ScanTrigCache *c,
                                         double x, double y, double theta) {
    return slam__score_cached_field(s, c, s->lf_field, x, y, theta);
}


/* 매칭 품질: 점당 평균 우도(0~1). 정밀 점수판 기준으로 재야 의미가 있다
 * (넓은 판은 어긋나도 값이 잘 안 떨어져서 '잘 맞는 것처럼' 보인다). */
double slam__match_quality(const OccupancyGridSLAM *s, const ScanTrigCache *c,
                                          double x, double y, double theta) {
    const double *field = (s->lf_field_fine != NULL) ? s->lf_field_fine : s->lf_field;
    if (field == NULL || c->n <= 0) return 1.0;
    double sc = slam__score_cached_field(s, c, field, x, y, theta);
    return sc / (12.0 * (double)c->n);
}


void scan_match_correct_pose(const OccupancyGridSLAM *s,
                                      double predicted_x, double predicted_y, double predicted_theta,
                                      const LidarScan *scan,
                                      double *out_x, double *out_y, double *out_theta) {
    /* ============================================================
     * 오도메트리 벌점을 '점수 크기에 비례'하게 바꿈 (2026-08-09n) - 핵심 수정
     *
     * 사용자 신고: "모양은 맞는데 통째로 평행이동돼 있다."
     *
     * 이 지도는 기하학적으로 애매하다. 위쪽 통로는 긴 평행벽 두 개라 벽을 따라
     * 미끄러져도 스캔이 여전히 벽에 얹히고, A 슬롯과 B 슬롯은 모양이 똑같다.
     * 실측(합성스캔): 자세가 370mm 틀려도 점당 평균 우도가 0.93 이나 나온다.
     * 즉 "얼마나 잘 맞나"만으로는 틀린 자세를 구분할 수 없다.
     *
     * 이럴 때 유일한 판별 근거는 '연속성' - 로봇은 순간이동을 못 한다는 사실이다.
     * 그걸 반영하는 게 오도메트리 벌점(고무줄)인데, 상수가 완전히 잘못돼 있었다:
     *     점수 = sum(12 * 우도) 이므로 460점 스캔이면 최대 5520.
     *     그런데 0.3m 도약의 벌점은 40*0.09 = 3.6 뿐이다. 전체의 0.065%.
     * 고무줄이 있으나 마나였고, 그래서 매칭이 조금이라도 더 나은 엉뚱한 봉우리로
     * 슬금슬금 옮겨가 결국 통째로 평행이동한 자세에 눌러앉았다.
     *
     * 점수가 점 개수에 비례하므로 벌점도 점 개수에 비례해야 한다. 기준을 이렇게 잡았다.
     *     "0.3m 도약은 점당 평균 우도 0.10 만큼의 이득이 있어야만 허용"
     *     -> K * 0.09 = 12 * n * 0.10  ->  K = 13.3 * n
     * 각도도 같은 기준(25도 도약 = 우도 0.10):
     *     K_th * 0.19 = 12 * n * 0.10  ->  K_th = 6.3 * n
     * 정상 주행에서 한 스텝 이동은 크게 잡아도 60mm 이므로 벌점은
     * 13.3n * 0.0036 = 0.048n, 전체 12n 의 0.4% - 정상 움직임은 전혀 방해받지 않는다.
     * ============================================================ */
    const double MIN_SCORE_SPREAD = 1.0;
    const int COARSE_STRIDE = 3;   /* 거친 탐색은 3점당 1점만 사용 */

    double best_score = -1e300;
    double best_x = predicted_x, best_y = predicted_y, best_theta = predicted_theta;
    double min_raw = 1e300, max_raw = -1e300;

    int n_xy = (int)(2.0 * s->search_window_m / s->search_step_m + 1e-9) + 1;
    int n_th = (int)(2.0 * s->search_window_theta / s->search_step_theta + 1e-9) + 1;

    /* 제어루프는 단일 스레드이므로 static으로 두어 매 호출 스택 40KB 할당을 피함 */
    static ScanTrigCache cache;
    slam__cache_scan(&cache, scan, COARSE_STRIDE);
    const double stride_gain = (double)COARSE_STRIDE;   /* 점수 척도를 전체 점 기준으로 되돌림 */

    /* 벌점을 점 개수에 맞춘다(위 주석 참고). 거친 단계는 stride 로 점을 솎았지만
     * 점수에 stride_gain 을 곱해 전체 점 기준으로 되돌리므로, 여기서도 전체 점 수를 쓴다. */
    double n_pts = (double)cache.n * stride_gain;
    if (n_pts < 30.0) n_pts = 30.0;
    const double ODOM_PENALTY_POS   = 13.3 * n_pts;
    const double ODOM_PENALTY_THETA =  6.3 * n_pts;

    for (int ti = 0; ti < n_th; ti++) {
        double dtheta = -s->search_window_theta + ti * s->search_step_theta;
        for (int xi = 0; xi < n_xy; xi++) {
            double dx = -s->search_window_m + xi * s->search_step_m;
            for (int yi = 0; yi < n_xy; yi++) {
                double dy = -s->search_window_m + yi * s->search_step_m;
                double cand_x = predicted_x + dx;
                double cand_y = predicted_y + dy;
                double cand_theta = predicted_theta + dtheta;
                double raw_sc = slam__score_cached(s, &cache, cand_x, cand_y, cand_theta) * stride_gain;
                double penalty = ODOM_PENALTY_POS * (dx * dx + dy * dy)
                               + ODOM_PENALTY_THETA * (dtheta * dtheta);
                double sc = raw_sc - penalty;

                if (raw_sc < min_raw) min_raw = raw_sc;
                if (raw_sc > max_raw) max_raw = raw_sc;

                if (sc > best_score) {
                    best_score = sc;
                    best_x = cand_x; best_y = cand_y; best_theta = cand_theta;
                }
            }
        }
    }

    if ((max_raw - min_raw) < MIN_SCORE_SPREAD) {
        *out_x = predicted_x; *out_y = predicted_y; *out_theta = predicted_theta;
        return;
    }

    /* 정밀 단계: 5mm / 0.5도. 스캔 전체 점을 쓰고, **좁은 시그마 점수판**을 쓴다.
     * (2026-08-09n) 넓은 판으로 정밀 단계를 돌리면 봉우리가 뭉툭해서 몇 cm 어긋난
     * 자세도 거의 같은 점수를 받는다 - 그게 "모양은 맞는데 평행이동" 증상의 원인이었다. */
    const double *fine_field = (s->lf_field_fine != NULL) ? s->lf_field_fine : s->lf_field;
    if (s->lf_field != NULL) {
        const double FINE_STEP_M = 0.005;
        const double FINE_STEP_TH = 0.5 * ANGLE_PI / 180.0;
        slam__cache_scan(&cache, scan, 1);
        double fine_n = (double)cache.n;
        if (fine_n < 30.0) fine_n = 30.0;
        const double FINE_PEN_POS   = 13.3 * fine_n;
        const double FINE_PEN_THETA =  6.3 * fine_n;
        double cx = best_x, cy = best_y, cth = best_theta;
        best_score = -1e300;   /* 점수 척도(stride_gain)가 바뀌었으므로 재평가 */
        for (int ti = -6; ti <= 6; ti++) {
            double cand_theta = cth + ti * FINE_STEP_TH;
            double dtheta = cand_theta - predicted_theta;
            for (int xi = -6; xi <= 6; xi++) {
                double cand_x = cx + xi * FINE_STEP_M;
                double dx = cand_x - predicted_x;
                for (int yi = -6; yi <= 6; yi++) {
                    double cand_y = cy + yi * FINE_STEP_M;
                    double dy = cand_y - predicted_y;
                    double raw_sc = slam__score_cached_field(s, &cache, fine_field,
                                                             cand_x, cand_y, cand_theta);
                    double penalty = FINE_PEN_POS * (dx * dx + dy * dy)
                                   + FINE_PEN_THETA * (dtheta * dtheta);
                    double sc = raw_sc - penalty;
                    if (sc > best_score) {
                        best_score = sc;
                        best_x = cand_x; best_y = cand_y; best_theta = cand_theta;
                    }
                }
            }
        }
    }

    *out_x = best_x; *out_y = best_y; *out_theta = best_theta;
}


/* ============================================================
 * 위치를 잃었을 때 넓게 다시 찾기 (2026-08-09n 신규)
 *
 * 사용자 신고: "라이다 맵이 모양은 맞는데 통째로 평행이동돼 있다."
 *
 * 한 번 자세가 탐색창(±150mm) 밖으로 벗어나면 그 뒤로는 **영원히 못 돌아온다.**
 * 매 스텝 예측자세 주변만 보기 때문이다. 그래서 로봇은 틀린 자리를 목표로 믿고
 * 계속 이리저리 움직이게 된다.
 *
 * 이 함수는 매칭 품질(점당 평균 우도)이 낮을 때만 호출되며, 훨씬 넓은 범위를
 * 거친 격자로 훑어 봉우리를 다시 잡는다. 넓은 판(부드러움)으로 훑으므로 멀리서도
 * 끌려오고, 찾은 뒤에는 위의 정밀 단계가 좁은 판으로 확정한다.
 * ============================================================ */
bool scan_match_recover(const OccupancyGridSLAM *s, const LidarScan *scan,
                                       double cur_x, double cur_y, double cur_theta,
                                       double win_m, double win_theta_rad,
                                       double *out_x, double *out_y, double *out_theta,
                                       double *out_quality) {
    static ScanTrigCache cache;
    slam__cache_scan(&cache, scan, 2);
    if (cache.n <= 0) return false;

    const double STEP_M = 0.02;
    const double STEP_TH = 2.0 * ANGLE_PI / 180.0;
    int n_xy = (int)(2.0 * win_m / STEP_M + 1e-9) + 1;
    int n_th = (int)(2.0 * win_theta_rad / STEP_TH + 1e-9) + 1;

    double best = -1e300, bx = cur_x, by = cur_y, bth = cur_theta;
    for (int ti = 0; ti < n_th; ti++) {
        double cand_th = cur_theta - win_theta_rad + ti * STEP_TH;
        for (int xi = 0; xi < n_xy; xi++) {
            double cand_x = cur_x - win_m + xi * STEP_M;
            for (int yi = 0; yi < n_xy; yi++) {
                double cand_y = cur_y - win_m + yi * STEP_M;
                double sc = slam__score_cached_field(s, &cache, s->lf_field,
                                                      cand_x, cand_y, cand_th);
                if (sc > best) { best = sc; bx = cand_x; by = cand_y; bth = cand_th; }
            }
        }
    }
    /* 되찾은 자세를 좁은 판으로 다시 재서 품질을 돌려준다 */
    slam__cache_scan(&cache, scan, 1);
    double q = slam__match_quality(s, &cache, bx, by, bth);
    *out_x = bx; *out_y = by; *out_theta = bth;
    if (out_quality) *out_quality = q;
    return true;
}


/* slam_update()와 달리 지도는 절대 갱신하지 않고(slam_update_map 호출 안 함),
 * 오도메트리 예측 + 라이다 스캔매칭으로 위치추정만 계속 보정함. 고정맵 모드 전용. */
void slam_localize_step(OccupancyGridSLAM *s, const LidarScan *scan,
                                       double odom_dx, double odom_dy, double odom_dtheta,
                                       double *out_x, double *out_y, double *out_theta) {
    double predicted_x = s->x + odom_dx;
    double predicted_y = s->y + odom_dy;
    double predicted_theta = s->theta + odom_dtheta;

    /* 직전 스텝 일치도가 낮으면 이번 스텝만 탐색창을 넓힌다 (2026-08-10g).
     * wide_when_q_below 주석 참고 - '매번 제대로 다시 맞춘다'를 비용 없이 하는 방법. */
    double sw_m0 = s->search_window_m, sw_th0 = s->search_window_theta;
    double st_m0 = s->search_step_m,   st_th0 = s->search_step_theta;
    bool widened = false;
    if (s->lf_field != NULL && s->wide_when_q_below > 0.0
        && s->last_match_quality < s->wide_when_q_below) {
        s->search_window_m     *= 2.0;
        s->search_window_theta *= 2.0;
        s->search_step_m       *= 1.5;   /* 후보 수 폭증을 막으려고 간격도 키운다 */
        s->search_step_theta   *= 1.5;
        widened = true;
    }

    double cx, cy, cth;
    scan_match_correct_pose(s, predicted_x, predicted_y, predicted_theta, scan, &cx, &cy, &cth);

    if (widened) {
        s->search_window_m = sw_m0; s->search_window_theta = sw_th0;
        s->search_step_m   = st_m0; s->search_step_theta   = st_th0;
    }

    /* 매칭 품질 측정 + 필요하면 넓게 재탐색 (2026-08-09n) */
    static ScanTrigCache qcache;
    slam__cache_scan(&qcache, scan, 2);
    double q = slam__match_quality(s, &qcache, cx, cy, cth);

    /* ============================================================
     * (A) 운동 타당성 게이트 (2026-08-10 신규) - '맵이 통째로 회전'의 직접 차단
     *
     * 예측자세에서 한계 이상 떨어진 답은 두 가지로만 처리한다.
     *   1) 예측자세보다 확실히 좋지 않으면  -> 버리고 예측을 유지한다.
     *   2) 확실히 좋으면 -> 방향은 믿되 '한계까지만' 이동한다(비율 축소).
     *      한 번에 다 가지 않아도 다음 스텝들이 계속 같은 방향을 가리키면
     *      두세 스텝 만에 도착한다. 반대로 그 답이 틀렸다면 다음 스텝에서
     *      바로 부정되므로, 틀린 자세에 눌러앉는 일이 없어진다.
     * ============================================================ */
    bool gated = false;
    if (s->max_jump_m > 0.0 || s->max_jump_rad > 0.0) {
        double jump0    = hypot(cx - predicted_x, cy - predicted_y);
        double jump0_th = fabs(normalize_angle(cth - predicted_theta));
        /* 한계는 '고정값 + 이번에 움직인 양의 절반'이다.
         * 예측 오차는 움직인 양에 비례해서 커지므로(모델 배율 오차 30~50%),
         * 크게 움직인 스텝에는 그만큼 여유를 줘야 정상 보정까지 막지 않는다.
         * 반대로 거의 안 움직인 스텝에서는 한계가 고정값 그대로라 아주 빡빡하다
         * - '가만히 있었는데 136mm 튀었다'는 바로 그 경우를 잡는다. */
        double lim_m  = s->max_jump_m
                      + 0.5 * hypot(odom_dx, odom_dy);
        double lim_th = s->max_jump_rad
                      + 0.5 * fabs(odom_dtheta);
        /* 넓게 찾은 스텝(=이미 자세를 못 믿는 상태)에서는 한계를 2배로 연다
         * (2026-08-10g). 좁은 한계 그대로 두면 애써 찾은 정답을 다시 깎아버려서,
         * 넓게 본 의미가 없어진다. 대신 아래 '품질이 확실히 좋을 때만' 조건은
         * 그대로라 엉뚱한 자세로 튀지는 않는다. */
        if (widened) { lim_m *= 2.0; lim_th *= 2.0; }
        bool too_far = (s->max_jump_m   > 0.0 && jump0    > lim_m)
                    || (s->max_jump_rad > 0.0 && jump0_th > lim_th);
        if (too_far) {
            double qp = slam__match_quality(s, &qcache,
                                             predicted_x, predicted_y, predicted_theta);
            if (q < qp + SLAM_GATE_MARGIN) {
                printf("[slam] 도약 거부: %.0fmm/%.1fdeg (한계 %.0fmm/%.1fdeg) - "
                       "품질 %.2f 가 예측자세 %.2f 보다 낫지 않아 예측을 유지합니다\n",
                       jump0 * 1000.0, jump0_th * 180.0 / ANGLE_PI,
                       lim_m * 1000.0, lim_th * 180.0 / ANGLE_PI, q, qp);
                fflush(stdout);
                cx = predicted_x; cy = predicted_y; cth = predicted_theta;
                q = qp;
            } else {
                double k = 1.0;
                if (s->max_jump_m > 0.0 && jump0 > lim_m) {
                    double kk = lim_m / jump0;
                    if (kk < k) k = kk;
                }
                if (s->max_jump_rad > 0.0 && jump0_th > lim_th) {
                    double kk = lim_th / jump0_th;
                    if (kk < k) k = kk;
                }
                printf("[slam] 도약 제한: %.0fmm/%.1fdeg 중 %.0f%%만 반영합니다 "
                       "(품질 %.2f > 예측 %.2f)\n",
                       jump0 * 1000.0, jump0_th * 180.0 / ANGLE_PI, k * 100.0, q, qp);
                fflush(stdout);
                cx  = predicted_x + (cx - predicted_x) * k;
                cy  = predicted_y + (cy - predicted_y) * k;
                cth = predicted_theta + normalize_angle(cth - predicted_theta) * k;
                q   = slam__match_quality(s, &qcache, cx, cy, cth);
            }
            gated = true;
            s->bad_match_streak++;
        }
    }
    if (!gated && q >= SLAM_LOST_QUALITY) s->bad_match_streak = 0;

    /* (B) 재탐색 조건에 '연속 게이트'를 추가한다 (2026-08-10).
     * 품질 0.61~0.83 은 SLAM_LOST_QUALITY(0.45)를 넘어서 예전엔 재탐색이 아예 안 걸렸다.
     * 그런데 이 지도는 370mm 틀려도 품질이 0.93 나올 만큼 애매해서, 품질만으로는
     * '길을 잃었다'를 절대 알 수 없다. 도약이 연달아 거부된다는 것 자체가
     * "예측도 매칭도 서로 안 맞는다" = 길을 잃었다는 더 확실한 신호다. */
    if ((q < SLAM_LOST_QUALITY || s->bad_match_streak >= 4) && s->lf_field != NULL) {
        bool applied = false;
        double rx, ry, rth, rq = q;
        if (scan_match_recover(s, scan, cx, cy, cth, 0.30, 25.0 * ANGLE_PI / 180.0,
                                &rx, &ry, &rth, &rq) && rq > q + 0.03) {
            /* 되찾은 자리에서 정밀 단계를 한 번 더 돌려 확정 */
            double fx, fy, fth;
            scan_match_correct_pose(s, rx, ry, rth, scan, &fx, &fy, &fth);
            slam__cache_scan(&qcache, scan, 2);
            double fq = slam__match_quality(s, &qcache, fx, fy, fth);
            /* ============================================================
             * 재탐색 결과도 '운동 타당성'을 지켜야 한다 (2026-08-10)
             *
             * 실기 실패 로그:
             *     [slam] 위치 재탐색: 품질 0.72 -> 0.93,
             *            자세 (0.235,0.330,344deg) -> (0.235,0.190,358deg) [140mm 이동]
             * 이 한 줄이 통로에 있던 로봇을 '슬롯 안에 있다'고 믿게 만들었고,
             * 그 뒤 헛주차로 이어졌다. 품질만 보면 0.93 이라 완벽해 보이지만,
             * 제자리 회전 중에 140mm 를 순간이동하는 건 물리적으로 불가능하다.
             *
             * 문제의 본질: 재탐색은 '이미 틀어진 자세' 주변 +-0.30m 만 보므로,
             * 진짜 자리가 그 밖에 있으면 '틀린 자세들 중 제일 나은 것'을 고를 뿐이다.
             * 품질이 좋아졌다는 건 정답을 찾았다는 뜻이 아니다.
             *
             * 그래서 큰 교정은 여기서 조용히 적용하지 않고, 위층(robot_runner)에
             * "길을 잃었다"고 알려서 멈춘 뒤 지도 전체 x 0~360도 전역탐색을 하게 한다.
             * 그게 사용자가 요구한 "1초 멈춰도 되니 제대로 맞춰라" 이다.
             * ============================================================ */
            double rmove = hypot(fx - cx, fy - cy);
            double rlimit = (s->max_jump_m > 0.0) ? (s->max_jump_m * 1.5) : 1e9;
            if (fq > q && rmove <= rlimit) {
                printf("[slam] 위치 재탐색: 품질 %.2f -> %.2f, 자세 (%.3f,%.3f,%.0fdeg) "
                       "-> (%.3f,%.3f,%.0fdeg) [%.0fmm 이동]\n",
                       q, fq, cx, cy, cth * 180.0 / ANGLE_PI,
                       fx, fy, fth * 180.0 / ANGLE_PI, rmove * 1000.0);
                fflush(stdout);
                cx = fx; cy = fy; cth = fth; q = fq;
                applied = true;
            } else if (fq > q) {
                printf("[slam] 위치 재탐색 보류: %.0fmm 교정은 한 스텝에 불가능합니다 "
                       "(한계 %.0fmm). 조용히 옮기지 않고 전역 재탐색을 요청합니다\n",
                       rmove * 1000.0, rlimit * 1000.0);
                fflush(stdout);
                /* BUGFIX (2026-08-10d): 예전엔 bad_match_streak += 3 으로 신호를
                 * 보냈는데, 바로 아래에서 그 값을 0 으로 지워버렸고 위층도 그걸
                 * 읽지 않았다. 신호가 발신 즉시 소멸했다. 이제 전용 플래그에 남긴다. */
                s->relocalize_request++;
                s->reloc_req_move_m = rmove;
                applied = false;
            }
        }
        /* 재탐색이 실제로 반영됐을 때만 카운터를 리셋한다 (2026-08-10d).
         * 예전에는 무조건 0 으로 지워서, '계속 실패하고 있다'는 사실 자체가
         * 매 스텝 사라졌다 - 그래서 위 661줄의 streak 트리거도 사실상 죽어 있었다. */
        if (applied) s->bad_match_streak = 0;
    }
    s->last_match_quality = q;

    /* 진단 (2026-08-09n): 한 스텝에 자세가 크게 튀거나 품질이 낮으면 알린다.
     * 로봇은 순간이동을 못 하므로, 한 스텝 도약이 크다는 건 매칭이 엉뚱한 봉우리로
     * 옮겨갔다는 뜻이다. GUI 에서 "모양은 맞는데 평행이동" 으로 보이는 순간이
     * 이 로그와 일치하는지 확인하면 원인을 특정할 수 있다. */
    {
        double jump = hypot(cx - predicted_x, cy - predicted_y);
        double jump_th = fabs(normalize_angle(cth - predicted_theta));
        if (jump > 0.08 || q < 0.55) {
            printf("[slam] 주의: 한 스텝에 %.0fmm/%.1fdeg 이동, 매칭품질 %.2f "
                   "-> 자세 (%.3f,%.3f,%.0fdeg)\n",
                   jump * 1000.0, jump_th * 180.0 / ANGLE_PI, q,
                   cx, cy, cth * 180.0 / ANGLE_PI);
            fflush(stdout);
        }
    }

    s->x = cx; s->y = cy; s->theta = cth;
    *out_x = cx; *out_y = cy; *out_theta = cth;
}


void slam_update_map(OccupancyGridSLAM *s, double x, double y, double theta,
                                    const LidarScan *scan) {
    int r0, c0;
    slam_world_to_grid(s, x, y, &r0, &c0);

    int rows_buf[SLAM_MAX_RAY_LEN];
    int cols_buf[SLAM_MAX_RAY_LEN];

    for (int i = 0; i < scan->count; i++) {
        double dist_mm = scan->readings[i].dist_mm;
        if (dist_mm <= 0) continue;
        double dist_m = dist_mm / 1000.0;
        double world_angle = theta + scan->readings[i].angle_deg * ANGLE_PI / 180.0;
        double ex = x + dist_m * cos(world_angle);
        double ey = y + dist_m * sin(world_angle);
        int r1, c1;
        slam_world_to_grid(s, ex, ey, &r1, &c1);

        if (!slam_in_bounds(s, r0, c0) || !slam_in_bounds(s, r1, c1)) continue;

        int n = slam_bresenham(r0, c0, r1, c1, rows_buf, cols_buf, SLAM_MAX_RAY_LEN);
        if (n > SLAM_MAX_RAY_LEN) n = SLAM_MAX_RAY_LEN;
        if (n < 1) continue;

        for (int k = 0; k < n - 1; k++) {
            int idx = slam_idx(s, rows_buf[k], cols_buf[k]);
            double v = s->log_odds[idx] + SLAM_LOG_ODDS_FREE;
            if (v < SLAM_LOG_ODDS_MIN) v = SLAM_LOG_ODDS_MIN;
            if (v > SLAM_LOG_ODDS_MAX) v = SLAM_LOG_ODDS_MAX;
            s->log_odds[idx] = v;
        }
        int idx_end = slam_idx(s, rows_buf[n - 1], cols_buf[n - 1]);
        double v = s->log_odds[idx_end] + SLAM_LOG_ODDS_OCC;
        if (v < SLAM_LOG_ODDS_MIN) v = SLAM_LOG_ODDS_MIN;
        if (v > SLAM_LOG_ODDS_MAX) v = SLAM_LOG_ODDS_MAX;
        s->log_odds[idx_end] = v;
    }
}


void slam_update(OccupancyGridSLAM *s, const LidarScan *scan,
                                double odom_dx, double odom_dy, double odom_dtheta,
                                double *out_x, double *out_y, double *out_theta) {
    double predicted_x = s->x + odom_dx;
    double predicted_y = s->y + odom_dy;
    double predicted_theta = s->theta + odom_dtheta;

    double cx, cy, cth;
    if (!s->has_scanned) {
        cx = predicted_x; cy = predicted_y; cth = predicted_theta;
        s->has_scanned = true;
    } else {
        scan_match_correct_pose(s, predicted_x, predicted_y, predicted_theta, scan, &cx, &cy, &cth);
    }

    s->x = cx; s->y = cy; s->theta = cth;
    slam_update_map(s, cx, cy, cth, scan);

    *out_x = cx; *out_y = cy; *out_theta = cth;
}


unsigned char *slam_to_binary_grid(const OccupancyGridSLAM *s, double occ_threshold) {
    unsigned char *grid = (unsigned char *)malloc((size_t)s->rows * s->cols);
    for (int r = 0; r < s->rows; r++) {
        for (int c = 0; c < s->cols; c++) {
            double lo = s->log_odds[slam_idx(s, r, c)];
            double prob = 1.0 - 1.0 / (1.0 + exp(lo));
            grid[slam_idx(s, r, c)] = (prob >= occ_threshold) ? 1 : 0;
        }
    }
    return grid;
}
