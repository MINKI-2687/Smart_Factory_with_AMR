#include "path_follow.h"


/* 점 p 에서 선분 ab 까지의 최근접점과 거리제곱. t 는 [0,1] 로 잘린 투영계수. */
double path_follow__seg_closest(double px, double py,
                                               double ax, double ay,
                                               double bx, double by,
                                               double *out_t,
                                               double *out_x, double *out_y) {
    double vx = bx - ax, vy = by - ay;
    double len2 = vx * vx + vy * vy;
    double t;
    if (len2 < 1e-12) {
        t = 0.0;
    } else {
        t = ((px - ax) * vx + (py - ay) * vy) / len2;
        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;
    }
    double cx = ax + vx * t, cy = ay + vy * t;
    if (out_t) *out_t = t;
    if (out_x) *out_x = cx;
    if (out_y) *out_y = cy;
    double dx = px - cx, dy = py - cy;
    return dx * dx + dy * dy;
}


/* 경로 위 (seg, t) 지점에서 앞쪽으로 호길이 L 만큼 나아간 점.
 * 경로 끝을 넘으면 마지막 점을 돌려준다. */
void path_follow__advance(const Path *p, int seg, double t, double L,
                                         double *out_x, double *out_y) {
    int n = p->count;
    double cx = p->points[seg].x + (p->points[seg + 1].x - p->points[seg].x) * t;
    double cy = p->points[seg].y + (p->points[seg + 1].y - p->points[seg].y) * t;
    double remain = L;
    for (int i = seg; i < n - 1; i++) {
        double ex = p->points[i + 1].x, ey = p->points[i + 1].y;
        double d = hypot(ex - cx, ey - cy);
        if (d >= remain) {
            double k = (d > 1e-9) ? (remain / d) : 0.0;
            *out_x = cx + (ex - cx) * k;
            *out_y = cy + (ey - cy) * k;
            return;
        }
        remain -= d;
        cx = ex; cy = ey;
    }
    *out_x = p->points[n - 1].x;
    *out_y = p->points[n - 1].y;
}


/* 경로 추종 기하 일괄 계산.
 *   hint_seg   : 직전 호출의 seg_index (뒤로 크게 되돌아가는 것을 막는 용도). 처음엔 0.
 *   base_L     : 기본 전방주시 거리(m)
 *   max_L      : 전방주시 거리 상한(m). 보통 base_L 의 3배 정도.
 * 반환 false 면 경로가 비어 있어 추종할 수 없음. */
bool path_follow_track(const Path *p, int hint_seg,
                                      double x, double y,
                                      double base_L, double max_L,
                                      PathTrack *out) {
    out->valid = false;
    if (p == NULL || p->count == 0) return false;

    if (p->count == 1) {
        out->valid = true;
        out->seg_index = 0;
        out->proj_x = p->points[0].x;
        out->proj_y = p->points[0].y;
        out->cross_track = 0.0;
        out->path_heading = atan2(p->points[0].y - y, p->points[0].x - x);
        out->look_x = p->points[0].x;
        out->look_y = p->points[0].y;
        out->look_dist = hypot(p->points[0].x - x, p->points[0].y - y);
        out->remain = out->look_dist;
        return true;
    }

    /* 1) 최근접 구간 찾기.
     * 뒤로 되돌아가는 것은 한 구간까지만 허용한다 - 경로가 U자로 접혀 있을 때
     * 반대편 구간이 더 가깝게 잡혀 진행이 역전되는 것을 막기 위함. */
    int start = hint_seg - 1;
    if (start < 0) start = 0;
    if (start > p->count - 2) start = p->count - 2;

    int    best_i = start;
    double best_d2 = 1e300, best_t = 0.0, best_x = 0.0, best_y = 0.0;
    for (int i = start; i < p->count - 1; i++) {
        double t, cx, cy;
        double d2 = path_follow__seg_closest(x, y,
                                             p->points[i].x, p->points[i].y,
                                             p->points[i + 1].x, p->points[i + 1].y,
                                             &t, &cx, &cy);
        if (d2 < best_d2) {
            best_d2 = d2; best_i = i; best_t = t; best_x = cx; best_y = cy;
        }
    }

    /* 2) 접선 방향과 부호 있는 수직거리.
     * cross = tx*ry - ty*rx  (r = 투영점 -> 로봇). + 면 로봇이 진행방향 왼쪽. */
    double sx = p->points[best_i].x, sy = p->points[best_i].y;
    double ex = p->points[best_i + 1].x, ey = p->points[best_i + 1].y;
    double seg_len = hypot(ex - sx, ey - sy);
    double tx = (seg_len > 1e-9) ? (ex - sx) / seg_len : 1.0;
    double ty = (seg_len > 1e-9) ? (ey - sy) / seg_len : 0.0;
    double rx = x - best_x, ry = y - best_y;

    out->valid = true;
    out->seg_index = best_i;
    out->proj_x = best_x;
    out->proj_y = best_y;
    out->cross_track = tx * ry - ty * rx;
    out->path_heading = atan2(ty, tx);

    /* 3) 남은 경로길이 */
    double remain = hypot(ex - best_x, ey - best_y);
    for (int i = best_i + 1; i < p->count - 1; i++)
        remain += hypot(p->points[i + 1].x - p->points[i].x,
                        p->points[i + 1].y - p->points[i].y);
    out->remain = remain;

    /* 4) 벗어난 거리에 비례해 전방주시 거리를 늘림 (이 파일 머리말 참고) */
    double d = fabs(out->cross_track);
    double L = base_L;
    if (PATH_FOLLOW_CROSS_GAIN * d > L) L = PATH_FOLLOW_CROSS_GAIN * d;
    if (max_L > base_L && L > max_L) L = max_L;
    out->look_dist = L;

    path_follow__advance(p, best_i, best_t, L, &out->look_x, &out->look_y);
    return true;
}
