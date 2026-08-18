#include "smoothing.h"


/* Python PathSmoother.smooth()와 동일 로직 (Sebastian Thrun 방식 경사하강법 스무딩).
 * 점 개수는 그대로 두고, 각 점을 이웃 쪽으로 끌어당겨서 꺾이는 각도를 완만하게 만든다. */
void path_smooth(const Path *path, double weight_data, double weight_smooth,
                                double tolerance, int max_iterations, Path *out) {
    path_init(out);
    if (path->count < 3) {
        for (int i = 0; i < path->count; ++i) path_push(out, path->points[i].x, path->points[i].y);
        return;
    }

    int n = path->count;
    double (*original)[2] = malloc(sizeof(double[2]) * n);
    double (*newpath)[2] = malloc(sizeof(double[2]) * n);
    for (int i = 0; i < n; ++i) {
        original[i][0] = path->points[i].x; original[i][1] = path->points[i].y;
        newpath[i][0]  = path->points[i].x; newpath[i][1]  = path->points[i].y;
    }

    double error1 = 1e300, error2 = 0.0;
    int iterations = 0;
    while (fabs(error1 - error2) >= tolerance && iterations < max_iterations) {
        error1 = error2;
        error2 = 0.0;
        for (int i = 1; i < n - 1; ++i) {
            for (int k = 0; k < 2; ++k) {
                double d1 = weight_data * (original[i][k] - newpath[i][k]);
                double d2 = weight_smooth * (newpath[i + 1][k] + newpath[i - 1][k] - 2 * newpath[i][k]);
                newpath[i][k] += d1 + d2;
                error2 += d1 * d1 + d2 * d2;
            }
        }
        iterations++;
    }

    for (int i = 0; i < n; ++i) path_push(out, newpath[i][0], newpath[i][1]);
    free(original);
    free(newpath);
}
