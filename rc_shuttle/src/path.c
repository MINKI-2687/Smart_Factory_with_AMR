#include "path.h"


void path_init(Path *p) {
    p->points = NULL;
    p->count = 0;
    p->capacity = 0;
}


void path_push(Path *p, double x, double y) {
    if (p->count >= p->capacity) {
        p->capacity = (p->capacity == 0) ? 16 : p->capacity * 2;
        p->points = (Point2D *)realloc(p->points, sizeof(Point2D) * p->capacity);
    }
    p->points[p->count].x = x;
    p->points[p->count].y = y;
    p->count++;
}


void path_free(Path *p) {
    free(p->points);
    p->points = NULL;
    p->count = 0;
    p->capacity = 0;
}


void path_simplify(const Path *path, double min_dist, Path *out) {
    path_init(out);
    if (path->count < 2) {
        for (int i = 0; i < path->count; ++i) path_push(out, path->points[i].x, path->points[i].y);
        return;
    }
    path_push(out, path->points[0].x, path->points[0].y);
    for (int i = 1; i < path->count; ++i) {
        Point2D last = out->points[out->count - 1];
        Point2D cur = path->points[i];
        double d = hypot(cur.x - last.x, cur.y - last.y);
        if (d >= min_dist) {
            path_push(out, cur.x, cur.y);
        }
    }
    Point2D last_out = out->points[out->count - 1];
    Point2D last_in = path->points[path->count - 1];
    if (last_out.x != last_in.x || last_out.y != last_in.y) {
        path_push(out, last_in.x, last_in.y);
    }
}
