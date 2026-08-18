#include "obstacle_list.h"


void obstacle_list_init(ObstacleList *ol) {
    ol->items = NULL; ol->count = 0; ol->capacity = 0;
}


void obstacle_list_free(ObstacleList *ol) {
    free(ol->items); ol->items = NULL; ol->count = 0; ol->capacity = 0;
}


bool obstacle_equal(Obstacle a, Obstacle b) {
    return a.x == b.x && a.y == b.y && a.radius == b.radius;
}


bool obstacle_list_contains(const ObstacleList *ol, Obstacle o) {
    for (int i = 0; i < ol->count; ++i) {
        if (obstacle_equal(ol->items[i], o)) return true;
    }
    return false;
}


bool obstacle_list_add_if_new(ObstacleList *ol, Obstacle o) {
    if (obstacle_list_contains(ol, o)) return false;
    if (ol->count >= ol->capacity) {
        ol->capacity = (ol->capacity == 0) ? 8 : ol->capacity * 2;
        ol->items = (Obstacle *)realloc(ol->items, sizeof(Obstacle) * ol->capacity);
    }
    ol->items[ol->count++] = o;
    return true;
}


/* Python _make_grid와 동일: 장애물 목록 -> 0/1 격자(raw, 팽창 전) 생성. 호출부에서 free 필요.
 * PATCH (2026-08-03): 방 테두리(벽)를 항상 명시적으로 포함시킴. A* 자체는 원래도
 * "격자 밖=막힘"으로 암묵 처리해서 벽이 없어도 동작했지만, 라이다 기반 위치보정
 * (slam_localize_step)을 쓰려면 지도에 벽이 없으면 심각한 역효과가 남 - 라이다는
 * 실제로 벽에서 반사되는데 지도에 벽이 "빈공간"으로 기록돼 있으면, 스캔매칭이
 * 정답 위치에 오히려 감점을 주고 엉뚱한 위치를 더 그럴듯하다고 오판함(실측으로
 * 수미터급 발산 확인됨). 벽을 넣어주는 것만으로 위치추정오차가 40cm대에서 3cm대로
 * 개선됨(실측 확인됨). */
unsigned char *make_grid(double size_x, double size_y, double resolution,
                                        double origin_x, double origin_y,
                                        const ObstacleList *obstacles, int *out_rows, int *out_cols) {
    int cols = (int)(size_x / resolution);
    int rows = (int)(size_y / resolution);
    unsigned char *grid = (unsigned char *)calloc((size_t)rows * cols, 1);

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            double wx = c * resolution + resolution / 2.0 + origin_x;
            double wy = r * resolution + resolution / 2.0 + origin_y;
            for (int i = 0; i < obstacles->count; ++i) {
                double ox = obstacles->items[i].x, oy = obstacles->items[i].y, rad = obstacles->items[i].radius;
                if (hypot(wx - ox, wy - oy) <= rad) {
                    grid[r * cols + c] = 1;
                    break;
                }
            }
        }
    }

    for (int c = 0; c < cols; ++c) {
        grid[0 * cols + c] = 1;
        grid[(rows - 1) * cols + c] = 1;
    }
    for (int r = 0; r < rows; ++r) {
        grid[r * cols + 0] = 1;
        grid[r * cols + (cols - 1)] = 1;
    }

    *out_rows = rows;
    *out_cols = cols;
    return grid;
}
