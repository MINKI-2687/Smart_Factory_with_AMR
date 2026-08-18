#include "robot_sense.h"
#include "robot_runner.h"


/* ============================================================
 * 끼임 탈출: 라이다로 빈 공간을 찾아 그쪽으로 확실히 빠져나옴 (2026-08-08)
 *
 * 예전 탈출 동작은 "짧게(0.25초) 앞뒤로 번갈아 밀기"였음. 벽에 살짝 스친 정도면
 * 그걸로 풀리지만, 차체가 구석에 물린 경우엔 어느 쪽이 빈 공간인지 모른 채
 * 절반의 확률로 벽 쪽으로 더 밀어붙이게 되고, 이동량도 2cm 남짓이라 부족했음.
 * 실기 로그: (0.155,0.670)에서 탈출 4번을 다 쓰고도 못 빠져나와 150스텝 만에 포기.
 * (x=0.155는 로봇 반경 0.16보다 왼쪽 벽에 가까움 = 이미 벽에 박힌 자세)
 *
 * 지금은 스캔에서 정면/후면 여유를 직접 재서 넓은 쪽으로, 여유가 허락하는 만큼
 * (최대 12cm) 한 번에 밀어냄. 조향은 걸지 않음 - 물린 상태에서 조향하면 더 물림.
 * ============================================================ */
/* 스캔에서 [center-half, center+half] 방향 부채꼴의 최소 거리(m). 점이 없으면 9.9 */
double robot_runner__sector_min(const LidarScan *scan,
                                               double center_deg, double half_deg) {
    double best = 9.9;
    for (int i = 0; i < scan->count; i++) {
        double d = scan->readings[i].dist_mm / 1000.0;
        if (d <= 0.03) continue;
        double a = normalize_angle_deg(scan->readings[i].angle_deg - center_deg);
        if (fabs(a) <= half_deg && d < best) best = d;
    }
    return best;
}


/* 사방 최소거리가 '어느 방향'에서 나오는지 (차체기준 도). 2026-08-09c
 * 이게 없어서 예전 탈출은 "앞이 막혔나 뒤가 막혔나"만 보고 앞뒤로만 움직였음.
 * 실기 로그에서 걸린 상황이 정확히 이것: 막힌 건 왼쪽 벽(옆)인데 앞뒤로만
 * 왔다갔다 하니 사방최소가 117mm <-> 98mm 를 오갈 뿐 영영 해결이 안 됐음. */
double robot_runner__min_clearance_dir(const LidarScan *scan,
                                                      double *out_dist) {
    double best = 9.9, best_dir = 0.0;
    for (int i = 0; i < scan->count; i++) {
        double d = scan->readings[i].dist_mm / 1000.0;
        if (d > 0.03 && d < best) { best = d; best_dir = scan->readings[i].angle_deg; }
    }
    if (out_dist) *out_dist = best;
    return best_dir;
}


/* ============================================================
 * 두 스캔이 얼마나 달라졌는가 (2026-08-10e 신규)
 *
 * *** '로봇이 안 움직인 것'과 '자세추정만 멈춘 것'을 구분하는 유일한 관측 ***
 *
 * 실기 로그(통로):
 *   step=11..47 pose=(0.217,0.552) 가 36스텝 동안 고정, 매번 전진 명령
 *   [main] 자세가 5스텝째 안 움직이지만 차체는 막히지 않았습니다 ... 펄스를 3.4배로
 *   ...
 *   [slam] 전역 재위치추정 완료: (0.352,0.712) -> (0.840,0.760) 일치도 62% -> 99%
 * 즉 로봇은 그동안 실제로 60cm 이상 달리고 있었고, 멈춰 있던 건 자세추정이었다.
 * 그런데 코드는 '자세가 안 변한다'만 보고 '바퀴가 안 돈다'로 해석해서 펄스를
 * 계속 키웠다 - 눈을 감은 채 더 세게 달린 셈이라 상황을 더 나쁘게 만들었다.
 *
 * 둘은 원시 스캔으로 확실히 구분된다.
 *   - 바퀴가 안 돌면   : 스캔이 거의 그대로다 (평균 변화 몇 mm)
 *   - 자세만 멈췄으면  : 스캔이 크게 달라진다 (수십~수백 mm)
 * 스캔은 자세추정과 무관한 날것의 관측이므로 이 판정은 절대 속지 않는다.
 *
 * 2도 빈으로 각 방향 최근접거리를 뽑아, 양쪽 다 값이 있는 빈에서 평균 절대차(m).
 * 비교 가능한 빈이 너무 적으면 -1 을 돌려 '판단 불가'를 알린다.
 * ============================================================ */
double robot_runner__scan_change(const LidarScan *a, const LidarScan *b) {
    enum { NB = 180 };                 /* 2도 빈 */
    double ra[NB], rb[NB];
    for (int i = 0; i < NB; i++) { ra[i] = -1.0; rb[i] = -1.0; }
    for (int i = 0; i < a->count; i++) {
        double d = a->readings[i].dist_mm / 1000.0;
        if (d <= 0.03) continue;
        int k = (int)((normalize_angle_deg(a->readings[i].angle_deg) + 180.0) / 2.0);
        if (k < 0) k = 0;
        if (k >= NB) k = NB - 1;
        if (ra[k] < 0.0 || d < ra[k]) ra[k] = d;
    }
    for (int i = 0; i < b->count; i++) {
        double d = b->readings[i].dist_mm / 1000.0;
        if (d <= 0.03) continue;
        int k = (int)((normalize_angle_deg(b->readings[i].angle_deg) + 180.0) / 2.0);
        if (k < 0) k = 0;
        if (k >= NB) k = NB - 1;
        if (rb[k] < 0.0 || d < rb[k]) rb[k] = d;
    }
    double sum = 0.0;
    int n = 0;
    for (int i = 0; i < NB; i++) {
        if (ra[i] < 0.0 || rb[i] < 0.0) continue;
        sum += fabs(ra[i] - rb[i]);
        n++;
    }
    if (n < 30) return -1.0;
    return sum / n;
}


/* ============================================================
 * 제자리 회전 가능 여부를 '직사각형' 기준으로 판정 (2026-08-09f)
 *
 * 왜 바꾸나 (= 사용자 신고 "통로에서 앞으로 빨리 가다가 휙 뒤로 가기를 반복"):
 *
 * 예전 검사는 이랬다.
 *     if (robot_runner__min_clearance(&scan) < cfg->robot_radius + 0.01) -> 끼임 탈출
 * 즉 "사방 어느 방향으로든 회전반경(143mm)만큼 비어 있어야 돈다"였다. 그런데
 * 이 통로는 높이가 0.395m 라 중심이 정중앙에서 5cm만 벗어나도 이 조건이 깨진다.
 * 그러면 robot_runner__escape_to_open() 이 앞/뒤로 최대 12cm 를 sg_step_speed(34)로
 * 밀어내는데 - 그게 화면에서 보이던 "휙" 이다 - 앞뒤로 움직여도 좌우 여유는
 * 1mm 도 안 바뀌므로 조건이 그대로고, 다시 회전을 시도하면 또 같은 판정이 난다.
 *
 * 그런데 이 조건 자체가 과하게 보수적이다. 143mm 는 차체 대각선 반경, 즉
 * "360도 다 돌 때" 필요한 값이다. 실제로 한 스텝에 도는 각도는 스캔매칭 탐색창
 * 제한 때문에 8.4도뿐이다. 옆벽 방향으로 차체가 뻗은 길이는 반폭 w=65mm 이고,
 * 8.4도 도는 동안 최대치를 잡아도 70mm 남짓이다.
 * 즉 필요한 값이 143mm 가 아니라 70mm 라, 통로에서 5cm 벗어난 정도로는
 * 애초에 문제가 되지 않았다. 검사가 틀려서 멀쩡한 상황을 위험으로 오판하고 있었다.
 *
 * 반대로 90도 회전처럼 큰 회전에서는 회전 도중 차체의 '긴 쪽'이 그 벽을 향하게
 * 되므로 필요값이 반길이 L=127.5mm 까지 올라간다 - 예전 143mm 와 사실상 같다.
 * 즉 안전 기준은 그대로 두고 과잉판정만 없앤다.
 *
 * 이 함수는 스캔의 모든 점에 대해 "이번 스텝에 돌 각도만큼 도는 동안 그 방향으로
 * 필요한 최대 돌출량"을 직접 계산해서 실제 거리와 비교한다. 90도 회전처럼 큰
 * 회전에서는 e 의 최대가 L(127.5mm)이 되므로 예전 값과 사실상 같아진다 -
 * 즉 안전은 그대로 두고 과잉판정만 없앤다.
 * ============================================================ */
/* 차체기준 방향 phi 로, 중심에서 차체 외곽까지의 거리(m).
 * 직사각형이므로 '지지함수 |L*cos|+|w*sin|' 이 아니라 '광선-사각형 교점'이 맞다.
 * 지지함수는 그 방향의 최대 돌출(모서리)이라 옆방향에서 2배 가까이 과대평가된다.
 *   phi=90도 -> 이 함수는 w(65mm), 지지함수는 65mm  (같음)
 *   phi=60도 -> 이 함수는 75mm,   지지함수는 120mm  (지지함수가 과대) */
double robot_runner__body_reach(double phi, double half_len, double half_w) {
    double c = fabs(cos(phi)), sn = fabs(sin(phi));
    double rx = (c  > 1e-9) ? (half_len / c)  : 1e9;
    double ry = (sn > 1e-9) ? (half_w  / sn) : 1e9;
    return (rx < ry) ? rx : ry;
}


/* dtheta_rad: 이번에 돌 각도(부호 포함, + 반시계).
 * out_min_gap: 가장 빠듯한 지점의 여유(m). out_dir_deg: 그 방향(차체기준 도).
 * 반환 true 면 margin 이상 여유를 두고 돌 수 있음. */
bool robot_runner__rotation_is_safe(const LidarScan *scan, double dtheta_rad,
                                                   double half_len, double half_w,
                                                   double margin,
                                                   double *out_min_gap, double *out_dir_deg,
                                                   double *out_need_m) {
    const int NSTEP = 8;
    double dt = dtheta_rad / NSTEP;
    double worst = 1e9, worst_dir = 0.0, worst_need = 0.0;
    for (int i = 0; i < scan->count; i++) {
        double d = scan->readings[i].dist_mm / 1000.0;
        if (d <= 0.03) continue;
        double a = scan->readings[i].angle_deg * ANGLE_PI / 180.0;
        /* 차체가 +dt*k 돌면 고정 장애물은 차체기준으로 -dt*k 돈다 */
        double need = 0.0;
        for (int k = 0; k <= NSTEP; k++) {
            double e = robot_runner__body_reach(a - dt * k, half_len, half_w);
            if (e > need) need = e;
        }
        double gap = d - need;
        if (gap < worst) {
            worst = gap; worst_dir = scan->readings[i].angle_deg; worst_need = need;
        }
    }
    if (worst > 1e8) { worst = 9.9; worst_need = 0.0; }   /* 점이 하나도 없음 */
    if (out_min_gap) *out_min_gap = worst;
    if (out_dir_deg) *out_dir_deg = worst_dir;
    if (out_need_m)  *out_need_m  = worst_need;
    return worst >= margin;
}


/* ============================================================
 * "돌 수 있는 만큼만 돈다" - 부분 회전 (2026-08-10 신규)
 *
 * 사용자 지적: "각도를 틀어서 이동하면 되지 않나?"  -> 맞다. 그리고 지금까지
 * 못 한 이유는 검사가 전부-아니면-전무(all-or-nothing) 였기 때문이다.
 *
 * 실기 로그:
 *     회전 여유 부족: -126deg 방향 138mm < 필요 141mm (이번 스텝 26.2deg 회전 기준)
 *     연속 회전 보류: +65deg 를 한 번에 돌기엔 -120deg 방향 여유가 7mm 모자랍니다
 * 3mm, 7mm 가 모자라서 회전을 통째로 포기하고 앞뒤 왕복 탈출로 갔다. 그런데
 * 141mm 는 '26.2도를 다 돌 때' 필요한 값이다. 20도만 돌면 필요값이 130mm 로 줄어
 * 지금 있는 138mm 로 충분하다. 즉 돌 수 있었는데 안 돈 것이다.
 *
 * 이 함수는 요청한 회전량 중 안전하게 돌 수 있는 비율을 이분법으로 찾는다.
 * 매 스텝 조금씩이라도 목표 각도에 가까워지므로, 왕복 없이 자연스럽게 풀린다.
 * (needed 는 각도에 대해 단조증가라 이분법이 성립한다 - body_reach 의 최대값을
 *  구간 [a-dt, a] 에서 취하므로 구간이 넓어질수록 커지기만 한다.)
 * ============================================================ */
double robot_runner__max_safe_rotation_frac(const LidarScan *scan,
                                                           double dtheta_rad,
                                                           double half_len, double half_w,
                                                           double margin) {
    double g = 0.0, d = 0.0, n = 0.0;
    if (robot_runner__rotation_is_safe(scan, dtheta_rad, half_len, half_w, margin, &g, &d, &n))
        return 1.0;
    double lo = 0.0, hi = 1.0;
    for (int i = 0; i < 6; i++) {           /* 1/64 해상도면 충분 */
        double mid = 0.5 * (lo + hi);
        if (robot_runner__rotation_is_safe(scan, dtheta_rad * mid, half_len, half_w,
                                            margin, &g, &d, &n)) lo = mid;
        else hi = mid;
    }
    return lo;
}


/* ============================================================
 * 슬롯/깔때기 안에서 '어느 쪽이 막혔는지'를 라이다로 직접 판단 (2026-08-09a)
 *
 * 왜 필요한가 (= 사용자가 본 두 번째 증상의 근본원인):
 * 예전 깔때기 복구는 회전 방향을 오직 SLAM 좌우오차(x - lane_x)로 정했음. 그런데
 *   (1) 슬롯 평행구간의 좌우 여유는 편측 9.5mm 인데 스캔매칭 좌우 오차는 10~20mm,
 *   (2) 깔때기에 물린 순간에는 스캔이 벽에 눌려 매칭 자체가 편향됨.
 * 즉 "차체가 실제로 어느 쪽 경사벽에 걸렸는가"와 "SLAM이 계산한 좌우오차의 부호"가
 * 자주 반대로 나옴 -> 걸린 쪽으로 더 틀어버림 -> 더 세게 물림(신고 증상 그대로).
 *
 * 접촉면은 라이다로 직접 보인다. 차체 기준 좌/우 앞쪽 부채꼴의 최소거리를 비교하면
 * "지금 어느 쪽 코가 벽에 붙어 있는지"를 SLAM 없이 알 수 있고, 이건 물린 상태에서도
 * 신뢰할 수 있는 관측이다. 스캔 각도는 apply_lidar_frame_fix() 를 거친 뒤라
 * 0도 = 차체 정면, +90도 = 차체 왼쪽 규약이다.
 * ============================================================ */

/* 차체 앞쪽 좌/우 여유(m). left = +y(왼쪽), right = -y(오른쪽) */
void robot_runner__front_side_clearance(const LidarScan *scan,
                                                       double *out_left, double *out_right) {
    if (out_left)  *out_left  = robot_runner__sector_min(scan,  35.0, 25.0);
    if (out_right) *out_right = robot_runner__sector_min(scan, -35.0, 25.0);
}


/* 어느 쪽이 막혔나. 반환 +1 = 왼쪽이 더 막힘, -1 = 오른쪽이 더 막힘, 0 = 판단 불가.
 * DECIDE_M 이상 차이가 나야 판단함(그보다 작으면 라이다 노이즈 수준). */
int robot_runner__blocked_side(const LidarScan *scan,
                                              double *out_l, double *out_r) {
    const double DECIDE_M = 0.030;
    const double NEAR_M   = 0.30;   /* 이보다 멀면 '걸릴 만한 벽'이 아님 */
    double l, r;
    robot_runner__front_side_clearance(scan, &l, &r);
    if (out_l) *out_l = l;
    if (out_r) *out_r = r;
    if (l > NEAR_M && r > NEAR_M) return 0;      /* 양쪽 다 멀면 걸린 게 아님 */
    if (fabs(l - r) < DECIDE_M) return 0;        /* 대칭이면 옆이 아니라 정면이 막힌 것 */
    return (l < r) ? +1 : -1;
}


/* 슬롯 평행구간(좌우 벽이 나란함) 안에서 좌우 어긋남을 라이다로 직접 측정.
 * 반환 true면 *out_off_m 유효. 부호: + 면 차체가 슬롯 중심보다 '왼쪽(+y)'으로 치우침.
 *
 * 원리: 좌우 벽까지 거리를 d_l, d_r 이라 하면 슬롯 반폭 H, 차체 반폭 w 에 대해
 *   d_l = H - w - e,  d_r = H - w + e   ->   e = (d_r - d_l) / 2
 * H 와 w 를 몰라도 되고, 스캔매칭이 틀려도 상관없는 게 핵심.
 * 유효성: 양쪽 벽이 둘 다 가까울 때(<= MAX_M)만 인정 - 통로에서는 false 가 나온다. */
bool robot_runner__slot_lateral_from_scan(const LidarScan *scan,
                                                         double *out_off_m) {
    const double MAX_M = 0.25;
    double dl = robot_runner__sector_min(scan,  90.0, 20.0);
    double dr = robot_runner__sector_min(scan, -90.0, 20.0);
    if (dl > MAX_M || dr > MAX_M) return false;
    if (dl < 0.005 || dr < 0.005) return false;
    if (out_off_m) *out_off_m = (dr - dl) * 0.5;
    return true;
}


/* 지도상의 x 오차를 '차체 기준 좌우오차(왼쪽 +)'로 변환.
 * 이 프로젝트의 슬롯은 -y 방향으로 들어가는 세로 슬롯이라 heading 이 ±90도 계열이다.
 * heading 이 0/180도(가로 슬롯)면 lane_x 개념 자체가 성립하지 않으므로 false 를
 * 돌려 lane 제어를 끄게 한다. 부호 검산:
 *   heading=-90 -> 차체 왼쪽 = 월드 +x -> lat_left = (x - lane_x)
 *   heading=+90 -> 차체 왼쪽 = 월드 -x -> lat_left = -(x - lane_x)           */
bool robot_runner__lane_offset_body(double x, double lane_x,
                                                   double heading_rad, double *out_left_m) {
    double s = sin(heading_rad);
    if (fabs(s) < 0.5) return false;          /* 세로 슬롯이 아님 */
    if (out_left_m) *out_left_m = (x - lane_x) * ((s < 0.0) ? 1.0 : -1.0);
    return true;
}
