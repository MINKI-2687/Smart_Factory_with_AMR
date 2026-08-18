#ifndef PATH_H
#define PATH_H

#include <math.h>
#include <stdlib.h>

typedef struct { double x, y; } Point2D;

typedef struct {
    Point2D *points;
    int count;
    int capacity;
} Path;
void path_init(Path *p);
void path_push(Path *p, double x, double y);
void path_free(Path *p);
void path_simplify(const Path *path, double min_dist, Path *out);

#endif /* PATH_H */
