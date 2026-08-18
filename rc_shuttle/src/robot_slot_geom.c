#include "robot_slot_geom.h"
#include "robot_runner.h"


bool robot_runner__grid_occ(const OccupancyGridSLAM *s, double wx, double wy) {
    int r, c;
    slam_world_to_grid(s, wx, wy, &r, &c);
    if (!slam_in_bounds(s, r, c)) return true;      /* 지도 밖은 벽으로 본다 */
    return s->log_odds[slam_idx(s, r, c)] > 0.0;
}


/* 슬롯의 폭 / 바닥 / 입구 / 반대쪽 끝을 격자에서 직접 잰다 (하드코딩 없음).
 *
 * probe_y 는 '슬롯 축 위 아무 데나'면 된다 - 슬롯 안이든 대기점이든 상관없다.
 * 먼저 lane_x 를 따라 아래로 내려가 안쪽 벽(슬롯 바닥)을 찾고, 거기서 조금 위를
 * 기준점으로 삼아 폭을 재기 때문이다. 그래서 진입(stop_y=주차 y)과
 * 이탈(stop_y=대기점 y) 양쪽에서 같은 함수를 쓸 수 있다. */
bool robot_runner__slot_map_geom(const OccupancyGridSLAM *s,
                                                double lane_x, double probe_y,
                                                SlotMapGeom *out) {
    if (s == NULL || out == NULL) return false;
    out->ok = false;
    double st = s->resolution * 0.5;
    if (st <= 0.0) return false;
    if (robot_runner__grid_occ(s, lane_x, probe_y)) return false;   /* 애초에 벽 속 */

    /* 1) 슬롯 바닥(안쪽 벽면) 찾기 - lane_x 를 따라 아래로 */
    double inner = probe_y;
    for (double d = st; d < 1.2; d += st) {
        if (!robot_runner__grid_occ(s, lane_x, probe_y - d)) inner = probe_y - d; else break;
    }
    /* 2) 바닥에서 조금 위(평행구간 한가운데가 되도록)를 기준으로 폭을 잰다 */
    double ref_y = inner + 3.0 * s->resolution;
    if (robot_runner__grid_occ(s, lane_x, ref_y)) return false;
    double xr = lane_x, xl = lane_x;
    for (double d = st; d < 0.8; d += st) {
        if (!robot_runner__grid_occ(s, lane_x + d, ref_y)) xr = lane_x + d; else break;
    }
    for (double d = st; d < 0.8; d += st) {
        if (!robot_runner__grid_occ(s, lane_x - d, ref_y)) xl = lane_x - d; else break;
    }
    double half = (xr - xl) * 0.5;
    if (half < 0.04 || half > 0.25) return false;      /* 슬롯이라기엔 이상한 폭 */

    /* 3) 평행구간의 끝(깔때기 시작)과 슬롯 반대쪽 끝(통로 바깥벽) */
    double mouth = ref_y, back = ref_y;
    for (double d = st; d < 2.0; d += st) {
        double yy = ref_y + d;
        if (robot_runner__grid_occ(s, lane_x, yy)) break;
        back = yy;
        double wr = 0.0, wl = 0.0;
        for (double e = st; e < 1.0; e += st) {
            if (!robot_runner__grid_occ(s, lane_x + e, yy)) wr = e; else break;
        }
        for (double e = st; e < 1.0; e += st) {
            if (!robot_runner__grid_occ(s, lane_x - e, yy)) wl = e; else break;
        }
        if (wr + wl <= 2.0 * half + 0.03) mouth = yy;
    }
    if (back <= mouth + 0.05) return false;            /* 슬롯 위가 안 열려 있다 */

    out->ok       = true;
    out->center_x = (xr + xl) * 0.5;
    out->half_w   = half;
    out->inner_y  = inner;
    out->mouth_y  = mouth;
    out->back_y   = back + s->resolution;   /* 마지막 자유셀 바깥이 벽면 */
    return true;
}


/* 한쪽 벽 점들을 직선(py = a*px + b)으로 적합해 벽의 몸통기준 기울기를 낸다.
 *
 * 벽은 슬롯 축과 나란하고 몸통 x축이 슬롯 축을 향하므로, 로봇이 축 대비 psi 만큼
 * 반시계로 돌아 있으면 몸통에서 본 벽 방향은 -psi 다. 즉 각도오차 = -atan(a).
 * (검산: 축이 -y, heading = -90+psi 일 때 R(-heading)*(0,1) = (-cos psi, sin psi),
 *  +x 쪽으로 뒤집으면 (cos psi, -sin psi) -> 기울기각 -psi.)
 *
 * *** 여기서 두 가지를 반드시 걸러야 한다 (합성스캔 실험으로 확인) ***
 *  (1) 각도로만 부채꼴을 자르면, 로봇이 기울어 있을 때 뒤쪽 광선이 슬롯 입구를 넘어
 *      '깔때기 경사벽'까지 닿는다. 경사벽은 15~20도로 기울어 있어서 직선적합을
 *      통째로 끌어당긴다. 실측: y=0.31 에서 참 -16도를 -9.7도로, y=0.34 에서
 *      참 -16도를 -3.9도로 읽었다(오차 12도!).
 *      -> 각도가 아니라 '벽을 따라 얼마나 멀리 보는가(|px|)'로 자른다. 이러면
 *         로봇이 기울어도 보는 구간의 길이가 변하지 않는다.
 *  (2) 그래도 섞여 들어오는 점은 잔차로 쳐낸다(2회 반복). 평행벽은 잔차가
 *      격자분해능(10mm) 수준이라, 8mm 를 넘는 점은 벽이 아니라고 봐도 된다.
 * 두 가지를 같이 넣으면 평행구간 안에서 오차가 1도 아래로 떨어진다(아래 실측표).
 *
 * along_m : 벽을 따라 앞뒤로 볼 거리(m). 짧으면 각도 분해능이 나빠지고,
 *           길면 깔때기까지 닿는다. 0.10 이 이 세트장의 절충점이다. */
bool robot_runner__wall_slope(const LidarScan *scan,
                                             double side,          /* +1 = 왼쪽 벽, -1 = 오른쪽 */
                                             double along_m, double max_perp_m,
                                             double *out_yaw_rad, int *out_n) {
    /* 제어루프는 단일 스레드다 - slam.h 의 ScanTrigCache 와 같은 이유로
     * 매 호출 10KB 스택 할당을 피하려고 static 으로 둔다. */
    static double px[LIDAR_MAX_READINGS], py[LIDAR_MAX_READINGS];
    static bool   use[LIDAR_MAX_READINGS];
    int n = 0;
    for (int i = 0; i < scan->count; i++) {
        double d = scan->readings[i].dist_mm / 1000.0;
        if (d <= 0.02 || d > 0.40) continue;
        double rad = scan->readings[i].angle_deg * ANGLE_PI / 180.0;
        double ax = d * cos(rad), ay = d * sin(rad);
        if (side > 0.0) { if (ay < 0.03 || ay > max_perp_m) continue; }
        else            { if (ay > -0.03 || ay < -max_perp_m) continue; }
        if (fabs(ax) > along_m) continue;        /* 벽을 따라 이만큼만 본다 */
        px[n] = ax; py[n] = ay; use[n] = true; n++;
        if (n >= LIDAR_MAX_READINGS) break;
    }
    if (out_n) *out_n = n;
    if (n < 10) return false;

    double a = 0.0, b = 0.0, rms = 0.0;
    int live = n;
    for (int pass = 0; pass < 3; pass++) {
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        double xmin = 1e9, xmax = -1e9;
        live = 0;
        for (int i = 0; i < n; i++) {
            if (!use[i]) continue;
            sx += px[i]; sy += py[i]; sxx += px[i] * px[i]; sxy += px[i] * py[i];
            if (px[i] < xmin) xmin = px[i];
            if (px[i] > xmax) xmax = px[i];
            live++;
        }
        if (live < 10) return false;
        if (xmax - xmin < 0.08) return false;    /* 벽을 따라 퍼진 길이가 모자람 */
        double den = (double)live * sxx - sx * sx;
        if (fabs(den) < 1e-9) return false;
        a = ((double)live * sxy - sx * sy) / den;
        b = (sy - a * sx) / (double)live;
        double rss = 0.0;
        for (int i = 0; i < n; i++) {
            if (!use[i]) continue;
            double e = py[i] - (a * px[i] + b);
            rss += e * e;
        }
        rms = sqrt(rss / (double)live);
        if (pass == 2) break;
        int dropped = 0;                          /* 벽이 아닌 점(깔때기 등) 쳐내기 */
        for (int i = 0; i < n; i++) {
            if (!use[i]) continue;
            if (fabs(py[i] - (a * px[i] + b)) > 0.008) { use[i] = false; dropped++; }
        }
        if (dropped == 0) break;
    }
    if (out_n) *out_n = live;
    if (rms > 0.006) return false;                /* 직선이 아니다 = 평행구간이 아니다 */
    double yaw = -atan(a);
    if (fabs(yaw) > 25.0 * ANGLE_PI / 180.0) return false;
    if (out_yaw_rad) *out_yaw_rad = yaw;
    return true;
}


/* x 와 각도를 '못 박고' y 만 1차원으로 훑는 제약 스캔매칭.
 * 3자유도에서 생기던 옆미끄러짐이 원천적으로 불가능해진다. */
double robot_runner__slot_depth_match(const OccupancyGridSLAM *s,
                                                     const LidarScan *scan,
                                                     double fix_x, double fix_th,
                                                     double y_lo, double y_hi,
                                                     double *out_fit) {
    const double *field = s->lf_field;
    double best_y = 0.5 * (y_lo + y_hi), best = -1e300;
    if (field == NULL || scan->count <= 0) { if (out_fit) *out_fit = 0.0; return best_y; }
    for (double yy = y_lo; yy <= y_hi + 1e-9; yy += 0.010) {
        double sc = slam_score_pose_lf(s, field, fix_x, yy, fix_th, scan);
        if (sc > best) { best = sc; best_y = yy; }
    }
    double c0 = best_y - 0.010, c1 = best_y + 0.010;
    for (double yy = c0; yy <= c1 + 1e-9; yy += 0.002) {
        double sc = slam_score_pose_lf(s, field, fix_x, yy, fix_th, scan);
        if (sc > best) { best = sc; best_y = yy; }
    }
    if (out_fit) *out_fit = best / (double)scan->count;
    return best_y;
}


/* 라이다만으로 슬롯 안 자세를 잰다. geom 은 지도에서 미리 읽어둔 슬롯 기하. */
void robot_runner__slot_measure(const OccupancyGridSLAM *s,
                                                 const LidarScan *scan,
                                                 const RobotConfig *cfg,
                                                 const SlotMapGeom *geom,
                                                 double heading_rad,
                                                 SlotLidarFix *out) {
    memset(out, 0, sizeof(*out));
    out->rear_y = -1.0;
    if (!geom->ok || scan->count < 40) return;

    double dl = robot_runner__sector_min(scan,  90.0, 20.0);
    double dr = robot_runner__sector_min(scan, -90.0, 20.0);
    out->d_left = dl; out->d_right = dr;
    out->width_m = dl + dr;
    if (dl < 0.02 || dr < 0.02 || dl > 0.25 || dr > 0.25) return;
    /* 폭이 슬롯 폭과 맞아야 '평행구간 안'이다. 깔때기(벌어짐)와 통로를 여기서 거른다.
     * 직선벽까지의 수직거리는 요(yaw)와 무관하므로 이 검사는 각도에 안 흔들린다. */
    if (fabs(out->width_m - 2.0 * geom->half_w) > 0.045) return;
    out->ok = true;
    out->lat_left_m = (dr - dl) * 0.5;

    double yl = 0.0, yr = 0.0;
    int nl = 0, nr = 0;
    bool okl = robot_runner__wall_slope(scan, +1.0, 0.10, 0.25, &yl, &nl);
    bool okr = robot_runner__wall_slope(scan, -1.0, 0.10, 0.25, &yr, &nr);
    /* *** 반드시 '양쪽 벽이 둘 다, 서로 같은 각도로' 잡힐 때만 믿는다 ***
     * 한쪽만 잡히면 그게 평행벽인지 깔때기 경사벽인지 구분할 방법이 없다.
     * 실측(합성스캔): 로봇이 깔때기 입구(y=0.34)에 걸쳐 있을 때 한쪽 벽만 잡혔고,
     * 그게 경사벽이라 참 -16도를 -1.3도로 읽었다(오차 15도).
     * 반면 평행구간에서는 두 벽이 물리적으로 나란하므로 두 추정이 반드시 일치한다.
     * 즉 '두 추정의 일치' 자체가 평행구간 안이라는 독립적인 증거다. */
    if (okl && okr && fabs(normalize_angle(yl - yr)) < 5.0 * ANGLE_PI / 180.0) {
        out->yaw_err_rad = 0.5 * (yl + yr);
        out->yaw_ok = true;
        out->n_wall = nl + nr;
    }

    /* 깊이: x/각도를 못 박고 y 만 훑는다 */
    double sgn = (sin(heading_rad) < 0.0) ? 1.0 : -1.0;
    double fix_x  = geom->center_x + out->lat_left_m * sgn;
    double fix_th = heading_rad + (out->yaw_ok ? out->yaw_err_rad : 0.0);
    double y_lo = geom->inner_y + 0.5 * cfg->body_length - 0.05;
    double y_hi = geom->mouth_y + 0.12;
    if (y_hi <= y_lo) y_hi = y_lo + 0.05;
    double fit = 0.0;
    double y_sm = robot_runner__slot_depth_match(s, scan, fix_x, fix_th, y_lo, y_hi, &fit);
    out->depth_y = y_sm;
    out->depth_fit = fit;

    /* 독립 검산: 뒤쪽 광선은 슬롯 입구를 지나 통로 바깥벽에 닿는다.
     * 이 값은 스캔매칭도 지도 점수판도 전혀 안 쓰는 완전히 다른 경로다. */
    double d_rear = robot_runner__sector_min(scan, 180.0, 8.0);
    if (d_rear < 9.0) {
        double cy = cos(out->yaw_ok ? out->yaw_err_rad : 0.0);
        if (cy > 0.5) out->rear_y = geom->back_y - d_rear * cy;
    }

    /* 깊이 채택 조건 (합성스캔 실측으로 정한 값들).
     *  - yaw_ok 필수: 각도를 못 박은 채로 y 를 훑으면 봉우리가 흐려져서 깔때기
     *    근처에서 최대 40mm 까지 틀렸다(실측표). 반대로 yaw_ok 인 구간에서는
     *    오차가 4mm 이내였다. 각도가 확인 안 되면 y 는 아예 건드리지 않는다.
     *  - 깔때기 근처(mouth 위)도 제외: 그 구간은 어차피 깊이를 정확히 알 필요가
     *    없고(계속 밀고 들어가면 된다), 추정만 나빠진다.
     *  - 후방 광선(완전히 독립적인 경로)과 80mm 이상 다르면 둘 중 하나가 틀린
     *    것이므로 채택하지 않는다. */
    bool sane = out->yaw_ok
                && (fit >= 0.50)
                && (y_sm >= y_lo - 1e-6) && (y_sm <= y_hi + 1e-6)
                && (y_sm <= geom->mouth_y - 0.02);
    if (sane && out->rear_y > 0.0 && fabs(out->rear_y - y_sm) > 0.08) sane = false;
    out->depth_ok = sane;
}


SlotLanes *robot_runner__slot_lanes(void) {
    static SlotLanes L = { 0, { 0.0 } };
    return &L;
}


/* '지금 하고 있는 슬롯 기동이 엉뚱한 슬롯을 전제로 하고 있다'는 신호.
 * robot_runner__slot_no_wall() 과 같은 방식(파일 스코프 static)으로 둔다. */
bool *robot_runner__slot_wrong_lane(void) {
    static bool v = false;
    return &v;
}


/* run_shuttle() 이 시작할 때 A/B 차선을 등록한다. 슬롯이 셋 이상이어도 그대로 됨. */
void robot_runner__slot_lanes_set(const double *xs, int n) {
    SlotLanes *L = robot_runner__slot_lanes();
    L->n = 0;
    for (int i = 0; i < n && L->n < SLOT_LANES_MAX; i++) {
        bool dup = false;
        for (int j = 0; j < L->n; j++) if (fabs(L->x[j] - xs[i]) < 0.05) dup = true;
        if (!dup) L->x[L->n++] = xs[i];
    }
}


/* ============================================================
 * 판별에는 '슬롯 밖을 본 점'만 쓴다 (2026-08-11c)
 *
 * 슬롯 안에서 찍은 스캔의 대부분은 바로 옆 평행벽(약 100mm)이다. 그런데 그 점들은
 * A슬롯이든 B슬롯이든 **완전히 똑같다** - 즉 '어느 슬롯인가'에 대한 정보량이 0이다.
 * 그런 점을 채점에 넣으면 두 후보의 점수에 같은 상수를 더하는 셈이라, 순위는
 * 그대로인데 '격차'만 희석된다.
 *
 * 시험에서 그대로 나왔다: 깊이 y=0.150 (가장 깊이 들어간 자리)에서
 *     전체 점으로 채점 -> 1등 100% / 2등 94~96% (격차 4~6%p, 문턱 미달로 보류)
 * 순위는 100% 맞는데 결론을 못 내는 상태다.
 *
 * 슬롯 밖을 본 점(통로 벽, 방 끝벽)만 남기면 상수항이 빠지고 격차가 되살아난다.
 * 문턱은 '슬롯 폭 + 여유' 로 잡는다 - 그보다 먼 점은 물리적으로 슬롯 벽일 수 없다.
 * 먼 점이 너무 적으면(깔때기 밖이 거의 안 보이면) 전체 점으로 되돌아간다.
 * ============================================================ */
double robot_runner__map_fit_far(const OccupancyGridSLAM *s,
                                                const LidarScan *scan,
                                                double x, double y, double th,
                                                double min_range_m, int *out_n) {
    /* 제어루프는 단일 스레드다 - 이 파일의 다른 곳(wall_slope 등)과 같은 이유로
     * 매 호출 스택에 8KB 를 잡지 않으려고 static 을 쓴다. */
    static LidarScan far;
    far.count = 0;
    for (int i = 0; i < scan->count && far.count < LIDAR_MAX_READINGS; i++) {
        if (scan->readings[i].dist_mm / 1000.0 < min_range_m) continue;
        far.readings[far.count++] = scan->readings[i];
    }
    if (out_n) *out_n = far.count;
    if (far.count < 25) return -1.0;          /* 표본 부족 - 호출부가 되돌아감 */
    return robot_runner__map_fit(s, &far, x, y, th);
}


bool robot_runner__slot_identify(const OccupancyGridSLAM *slam,
                                                const LidarScan *scan,
                                                const RobotConfig *cfg,
                                                double heading_rad, double probe_y,
                                                double prefer_lane_x,
                                                SlotPick *out) {
    const double SLOT_PICK_MIN_FIT     = 0.80;  /* 전체 스캔 기준 - 슬롯 안인가 */
    const double SLOT_PICK_FAR_MIN_FIT  = 0.70;  /* 판별용 먼 점 기준 - 1등의 절대 품질 */
    const double SLOT_PICK_MARGIN       = 0.06;  /* 1등과 2등의 최소 격차 */

    memset(out, 0, sizeof(*out));
    out->fit_best = -1.0; out->fit_second = -1.0;
    if (slam == NULL || slam->lf_field == NULL || scan == NULL || scan->count < 40)
        return false;

    SlotLanes *L = robot_runner__slot_lanes();
    if (L->n <= 0) return false;

    for (int i = 0; i < L->n; i++) {
        SlotMapGeom geom;
        if (!robot_runner__slot_map_geom(slam, L->x[i], probe_y, &geom)) continue;
        SlotLidarFix f;
        robot_runner__slot_measure(slam, scan, cfg, &geom, heading_rad, &f);
        if (!f.ok) continue;                 /* 이 슬롯 폭과 안 맞음 */
        out->n_cand++;
        out->any = true;

        double sgn = (sin(heading_rad) < 0.0) ? 1.0 : -1.0;
        double px  = geom.center_x + f.lat_left_m * sgn;
        double pth = f.yaw_ok ? (heading_rad + f.yaw_err_rad) : heading_rad;
        /* ============================================================
         * 채점용 깊이와 확정용 깊이를 나눈다 (중요).
         *
         * 채점은 언제나 f.depth_y (그 슬롯 축 위에서 1차원으로 훑은 최적 깊이)로 한다.
         * 여기서 slam->y 로 물러서면 안 된다 - 지금 못 믿겠다고 판단한 바로 그 값이라,
         * 두 후보 모두 똑같이 나쁜 y 로 채점되어 점수차가 뭉개진다.
         * 실제로 그렇게 짰다가 시험에서 90건 중 34건이 '판정 보류'로 빠졌다
         * (순위는 전부 맞았는데 격차가 6%p 문턱을 못 넘었다).
         * 통계로 말하면 y 는 관심 없는 성가신 모수(nuisance parameter)이므로,
         * 후보마다 각자 최적 y 로 최대화한 뒤 비교하는 것이 옳다.
         *
         * 반대로 '자세로 확정할 깊이'는 예전 그대로 보수적으로 간다 -
         * depth_ok 가 아니면 y 는 건드리지 않는다.
         * ============================================================ */
        double score_y  = f.depth_y;
        double commit_y = f.depth_ok ? f.depth_y : slam->y;
        double fit_all  = robot_runner__map_fit(slam, scan, px, score_y, pth);
        /* 슬롯 벽보다 먼 점만으로 채점 - 위 robot_runner__map_fit_far 머리말 참고 */
        int n_far = 0;
        double far_min = 2.0 * geom.half_w + 0.10;
        double fit = robot_runner__map_fit_far(slam, scan, px, score_y, pth,
                                                far_min, &n_far);
        if (fit < 0.0) { fit = fit_all; n_far = 0; }   /* 먼 점이 모자라면 전체로 */

        /* 동점일 때는 '명령한 차선'을 아주 살짝 우대한다 (사용자 제안 2번).
         * 0.005 는 실측 격차(0.16 이상)에 비해 무시할 만한 크기라, 물리적 증거가
         * 있을 때는 절대 결과를 뒤집지 못한다 - 진짜 동점일 때만 작동한다. */
        if (prefer_lane_x > -1e8 && fabs(L->x[i] - prefer_lane_x) < 0.05) fit += 0.005;

        if (fit > out->fit_best) {
            out->fit_second = out->fit_best;
            out->fit_best = fit;
            out->fit_all = fit_all;
            out->n_far = n_far;
            out->lane_x = L->x[i];
            out->px = px; out->py = commit_y; out->pth = pth;
            out->fix = f;
        } else if (fit > out->fit_second) {
            out->fit_second = fit;
        }
    }

    if (!out->any) return false;
    /* 두 관문을 모두 넘어야 결론을 낸다.
     *  - fit_all  : '정말로 슬롯 안에 있는 자세인가' (전체 스캔 기준 건전성)
     *  - fit_best : '어느 슬롯인가' (판별용 먼 점 기준, 2등과의 격차 포함) */
    out->decisive = (out->fit_all >= SLOT_PICK_MIN_FIT)
                    && (out->fit_best >= SLOT_PICK_FAR_MIN_FIT)
                    && (out->n_cand == 1 || (out->fit_best - out->fit_second) >= SLOT_PICK_MARGIN);
    return out->decisive;
}


/* ============================================================
 * 재정박: 라이다 실측이 스캔매칭과 어긋나면 자세를 실측 쪽으로 다시 박는다.
 *
 * 사용자 요구 그대로다 - "슬롯 안에서 맵이 틀어지면 스캔매칭 불일치인 걸 알고
 * 꼭 다시 스캔매칭을 해야 한다." 여기서 하는 것이 정확히 그것이고, 다만
 * 3자유도로 다시 하면 같은 함정에 또 빠지므로 x/각도를 라이다로 못 박은 뒤
 * 남은 1자유도만 다시 맞춘다.
 *
 * 안전장치: 지금 자세가 이 슬롯에서 SNAP_MAX_M 이상 떨어져 있으면 손대지 않는다.
 * 두 슬롯이 1.05m 간격으로 똑같이 생겼으므로, 그만큼 벌어졌다면 '반대쪽 슬롯일
 * 수도 있는' 상황이라 조용히 옮기면 안 되고 전역 재위치추정이 맞다.
 * 반환 true = 자세를 바꿨음.
 * ============================================================ */
bool robot_runner__slot_reanchor(OccupancyGridSLAM *slam,
                                                const LidarScan *scan,
                                                const RobotConfig *cfg,
                                                double lane_x, double heading_rad,
                                                double park_y, const char *why,
                                                SlotLidarFix *out_fix,
                                                double *out_dx, double *out_dy) {
    const double SNAP_MAX_M   = 0.35;   /* 이보다 멀면 다른 슬롯일 수 있다 */
    const double LAT_TRIG_M   = 0.020;  /* 좌우가 이만큼 어긋나면 다시 박는다 */
    const double YAW_TRIG_DEG = 4.0;
    const double DEP_TRIG_M   = 0.035;

    if (out_dx) *out_dx = 0.0;
    if (out_dy) *out_dy = 0.0;
    SlotLidarFix fix;
    memset(&fix, 0, sizeof(fix));
    if (out_fix) *out_fix = fix;
    if (!cfg->slot_lidar_fix || slam == NULL || slam->lf_field == NULL) return false;

    SlotMapGeom geom;
    if (!robot_runner__slot_map_geom(slam, lane_x, park_y, &geom)) return false;
    robot_runner__slot_measure(slam, scan, cfg, &geom, heading_rad, &fix);
    if (out_fix) *out_fix = fix;
    if (!fix.ok) return false;

    double sgn = (sin(heading_rad) < 0.0) ? 1.0 : -1.0;
    double new_x  = geom.center_x + fix.lat_left_m * sgn;
    double new_th = fix.yaw_ok ? (heading_rad + fix.yaw_err_rad) : slam->theta;
    double new_y  = fix.depth_ok ? fix.depth_y : slam->y;

    double dx = new_x - slam->x;
    double dy = new_y - slam->y;
    double dth = normalize_angle(new_th - slam->theta);

    if (fabs(slam->x - geom.center_x) > SNAP_MAX_M || fabs(dx) > SNAP_MAX_M) {
        /* ============================================================
         * (2026-08-11c) 예전에는 여기서 그냥 포기했다. 이제는 검증한다.
         * robot_runner__slot_identify() 머리말 참고 - 이 지도에서 A/B 구분은
         * 일치도 격차 16~31%p 로 뚜렷하게 된다.
         * ============================================================ */
        SlotPick pick;
        bool decided = robot_runner__slot_identify(slam, scan, cfg, heading_rad,
                                                    park_y, lane_x, &pick);
        if (decided) {
            bool same_lane = (fabs(pick.lane_x - lane_x) < 0.05);
            printf("[slot] 라이다-스캔매칭 불일치 %.0fmm (%s) - 어느 슬롯인지 직접 검증했습니다.\n"
                   "       후보 %d개 채점: 1등 차선 x=%.3f 일치도 %.0f%%",
                   fabs(dx) * 1000.0, why, pick.n_cand, pick.lane_x,
                   pick.fit_best * 100.0);
            if (pick.n_cand > 1)
                printf(" / 2등 %.0f%% (격차 %.0f%%p)",
                       pick.fit_second * 100.0,
                       (pick.fit_best - pick.fit_second) * 100.0);
            printf("\n");
            if (same_lane) {
                printf("       => 지금 들어와 있는 슬롯이 맞습니다. 자세 (%.3f,%.3f,%.1fdeg)"
                       " -> (%.3f,%.3f,%.1fdeg) 로 다시 박습니다\n",
                       slam->x, slam->y, slam->theta * 180.0 / ANGLE_PI,
                       pick.px, pick.py, pick.pth * 180.0 / ANGLE_PI);
                fflush(stdout);
                if (out_dx) *out_dx = pick.px - slam->x;
                if (out_dy) *out_dy = pick.py - slam->y;
                slam->x = pick.px; slam->y = pick.py; slam->theta = pick.pth;
                slam->bad_match_streak = 0;
                slam->relocalize_request = 0;
                slam->last_match_quality = pick.fit_best;
                if (out_fix) *out_fix = pick.fix;
                fpga_cmd_track_reset();
                robot_runner__travel_acc_reset();
                return true;
            }
            /* 다른 슬롯이 이겼다 = 지금 진행 중인 기동의 전제가 틀렸다.
             * 자세는 증거대로 고쳐 놓되, 이 기동은 호출부가 접어야 한다. */
            printf("       => *** 지금 있는 곳은 명령한 차선(x=%.3f)이 아니라 x=%.3f 슬롯입니다 ***\n"
                   "       자세만 바로잡고 이번 기동은 중단합니다 (호출부가 다시 계획합니다)\n",
                   lane_x, pick.lane_x);
            fflush(stdout);
            slam->x = pick.px; slam->y = pick.py; slam->theta = pick.pth;
            slam->bad_match_streak = 0;
            slam->relocalize_request = 0;
            slam->last_match_quality = pick.fit_best;
            fpga_cmd_track_reset();
            robot_runner__travel_acc_reset();
            *robot_runner__slot_wrong_lane() = true;
            if (out_fix) *out_fix = pick.fix;
            return true;
        }
        printf("[slot] 라이다-스캔매칭 불일치가 %.0fmm 나 됩니다 (%s).\n"
               "       후보 슬롯 %d개를 채점했지만 1등 %.0f%% / 2등 %.0f%% 로 격차가 모자라\n"
               "       결론을 못 냈습니다 - 전역 재위치추정에 맡깁니다\n",
               fabs(dx) * 1000.0, why, pick.n_cand,
               pick.fit_best * 100.0, pick.fit_second * 100.0);
        fflush(stdout);
        slam->relocalize_request++;
        return false;
    }
    if (fabs(dx) < LAT_TRIG_M && fabs(dy) < DEP_TRIG_M
        && fabs(dth) * 180.0 / ANGLE_PI < YAW_TRIG_DEG) {
        return false;                    /* 어긋남이 라이다 잡음 수준 - 그냥 둔다 */
    }

    printf("[slot] *** 라이다-스캔매칭 불일치 -> 자세를 다시 박습니다 (%s) ***\n"
           "       좌벽 %.3fm 우벽 %.3fm (합 %.3f, 슬롯폭 %.3f) -> 치우침 %+.0fmm"
           "%s\n"
           "       자세 (%.3f,%.3f,%.1fdeg) -> (%.3f,%.3f,%.1fdeg)  "
           "[좌우 %+.0fmm, 깊이 %+.0fmm, 각도 %+.1fdeg]\n",
           why, fix.d_left, fix.d_right, fix.width_m, 2.0 * geom.half_w,
           fix.lat_left_m * 1000.0,
           fix.yaw_ok ? "" : " (벽 직선적합 실패 - 각도는 그대로 둡니다)",
           slam->x, slam->y, slam->theta * 180.0 / ANGLE_PI,
           new_x, new_y, new_th * 180.0 / ANGLE_PI,
           dx * 1000.0, dy * 1000.0, dth * 180.0 / ANGLE_PI);
    if (fix.depth_ok) {
        printf("       깊이는 x/각도를 라이다로 고정하고 y 만 1차원으로 다시 맞춘 값입니다 "
               "(일치도 %.0f%%%s)\n", fix.depth_fit * 100.0,
               (fix.rear_y > 0.0) ? "" : ", 후방검산 없음");
        if (fix.rear_y > 0.0)
            printf("       후방 광선 독립검산 y=%.3f (차이 %.0fmm)\n",
                   fix.rear_y, fabs(fix.rear_y - new_y) * 1000.0);
    } else {
        printf("       깊이(y)는 확신이 없어 건드리지 않았습니다 "
               "(1차원 재매칭 일치도 %.0f%%)\n", fix.depth_fit * 100.0);
    }
    fflush(stdout);

    slam->x = new_x; slam->y = new_y; slam->theta = new_th;
    slam->bad_match_streak = 0;
    slam->relocalize_request = 0;
    slam->last_match_quality = robot_runner__map_fit(slam, scan, new_x, new_y, new_th);
    robot_runner__travel_acc_reset();    /* 자세를 새로 확정했으므로 예산도 초기화 */
    if (out_dx) *out_dx = dx;
    if (out_dy) *out_dy = dy;
    return true;
}


SlotCtx *robot_runner__slot_ctx(void) {
    static SlotCtx c = { false, 0.0, 0.0, 0.0 };
    return &c;
}


void robot_runner__slot_ctx_set(double lane_x, double probe_y, double axis_rad) {
    SlotCtx *c = robot_runner__slot_ctx();
    c->set = true; c->lane_x = lane_x; c->probe_y = probe_y; c->axis_rad = axis_rad;
}


/* 문맥이 채워져 있을 때만 재정박한다. 안 채워져 있으면 아무 일도 하지 않는다. */
bool robot_runner__slot_reanchor_ctx(OccupancyGridSLAM *slam,
                                                    const LidarScan *scan,
                                                    const RobotConfig *cfg,
                                                    const char *why, SlotLidarFix *out_fix) {
    SlotCtx *c = robot_runner__slot_ctx();
    if (!c->set) {
        if (out_fix) memset(out_fix, 0, sizeof(*out_fix));
        return false;
    }
    double dx = 0.0, dy = 0.0;
    return robot_runner__slot_reanchor(slam, scan, cfg, c->lane_x, c->axis_rad,
                                        c->probe_y, why, out_fix, &dx, &dy);
}
