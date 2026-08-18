#include "grid_map.h"


int grid_idx(const OccupancyGridMap *m, int row, int col) {
    return row * m->cols + col;
}


bool grid_in_bounds(const OccupancyGridMap *m, int row, int col) {
    return row >= 0 && row < m->rows && col >= 0 && col < m->cols;
}


bool grid_is_free(const OccupancyGridMap *m, int row, int col) {
    return grid_in_bounds(m, row, col) && m->grid[grid_idx(m, row, col)] == 0;
}


bool grid_is_blocked(const OccupancyGridMap *m, int row, int col) {
    return !grid_in_bounds(m, row, col) || m->grid[grid_idx(m, row, col)] == 1;
}


/* 원본(팽창 전) 격자로부터 각 칸의 '가장 가까운 장애물까지 거리(m)'를 2-패스 체임퍼로
 * 근사 계산. slam.h의 likelihood field와 같은 방식이지만 가우시안 변환은 하지 않고
 * 거리 그대로 둔다(비용 계산에 직접 쓰기 위함). */
double *grid__build_clearance(const unsigned char *raw_grid,
                                             int rows, int cols, double resolution) {
    double *d = (double *)malloc(sizeof(double) * (size_t)rows * cols);
    if (!d) return NULL;
    const double BIG = 1e9;
    for (int i = 0; i < rows * cols; ++i) d[i] = raw_grid[i] ? 0.0 : BIG;

    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c) {
            double v = d[r * cols + c];
            if (r > 0)              v = fmin(v, d[(r-1)*cols + c] + 1.0);
            if (c > 0)              v = fmin(v, d[r*cols + (c-1)] + 1.0);
            if (r > 0 && c > 0)     v = fmin(v, d[(r-1)*cols + (c-1)] + 1.41421356);
            if (r > 0 && c < cols-1)v = fmin(v, d[(r-1)*cols + (c+1)] + 1.41421356);
            d[r * cols + c] = v;
        }
    for (int r = rows - 1; r >= 0; --r)
        for (int c = cols - 1; c >= 0; --c) {
            double v = d[r * cols + c];
            if (r < rows-1)             v = fmin(v, d[(r+1)*cols + c] + 1.0);
            if (c < cols-1)             v = fmin(v, d[r*cols + (c+1)] + 1.0);
            if (r < rows-1 && c < cols-1) v = fmin(v, d[(r+1)*cols + (c+1)] + 1.41421356);
            if (r < rows-1 && c > 0)    v = fmin(v, d[(r+1)*cols + (c-1)] + 1.41421356);
            d[r * cols + c] = v;
        }
    for (int i = 0; i < rows * cols; ++i) d[i] *= resolution;   /* 칸 -> m */
    return d;
}


void grid_init(OccupancyGridMap *m, const unsigned char *raw_grid,
                              int rows, int cols, double resolution, double robot_radius,
                              double origin_x, double origin_y) {
    m->rows = rows;
    m->cols = cols;
    m->resolution = resolution;
    m->origin_x = origin_x;
    m->origin_y = origin_y;
    m->grid = (unsigned char *)malloc(sizeof(unsigned char) * rows * cols);
    memcpy(m->grid, raw_grid, sizeof(unsigned char) * rows * cols);

    /* 여유거리 지도는 '팽창 전' 원본 기준으로 만든다(팽창 후엔 정보가 이미 뭉개짐).
     * 선호 여유는 robot_radius + 6cm - 통로 정중앙이면 대개 이 값을 넘으므로,
     * 지나갈 수 있는 좁은 곳에서는 벌점을 물면서도 길은 계속 존재한다(soft 제약). */
    m->clearance = grid__build_clearance(raw_grid, rows, cols, resolution);
    m->clearance_pref_m = robot_radius + 0.06;
    m->clearance_weight = 4.0;

    int inflate_cells = (int)ceil(robot_radius / resolution);
    if (inflate_cells <= 0) return;

    unsigned char *inflated = (unsigned char *)malloc(sizeof(unsigned char) * rows * cols);
    memcpy(inflated, m->grid, sizeof(unsigned char) * rows * cols);

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            if (m->grid[grid_idx(m, r, c)] != 1) continue;
            for (int dr = -inflate_cells; dr <= inflate_cells; ++dr) {
                for (int dc = -inflate_cells; dc <= inflate_cells; ++dc) {
                    if (dr * dr + dc * dc > inflate_cells * inflate_cells) continue;
                    int nr = r + dr, nc = c + dc;
                    if (grid_in_bounds(m, nr, nc)) {
                        inflated[grid_idx(m, nr, nc)] = 1;
                    }
                }
            }
        }
    }
    free(m->grid);
    m->grid = inflated;
}


void grid_free(OccupancyGridMap *m) {
    free(m->grid);
    m->grid = NULL;
    free(m->clearance);
    m->clearance = NULL;
}


/* 한 칸을 지날 때의 비용 배수. 여유가 clearance_pref_m 이상이면 1.0(벌점 없음),
 * 여유가 0이면 1+clearance_weight. 그 사이는 제곱으로 부드럽게 이어짐. */
double grid_cost_multiplier(const OccupancyGridMap *m, int row, int col) {
    if (m->clearance == NULL || m->clearance_pref_m <= 0.0) return 1.0;
    double cl = m->clearance[grid_idx(m, row, col)];
    if (cl >= m->clearance_pref_m) return 1.0;
    double t = (m->clearance_pref_m - cl) / m->clearance_pref_m;   /* 0~1 */
    return 1.0 + m->clearance_weight * t * t;
}


void world_to_grid(const OccupancyGridMap *m, double x, double y, int *row, int *col) {
    *col = (int)((x - m->origin_x) / m->resolution);
    *row = (int)((y - m->origin_y) / m->resolution);
}


void grid_to_world(const OccupancyGridMap *m, int row, int col, double *x, double *y) {
    *x = col * m->resolution + m->resolution / 2.0 + m->origin_x;
    *y = row * m->resolution + m->resolution / 2.0 + m->origin_y;
}


void path_grid_to_world(const OccupancyGridMap *m, const Path *grid_path, Path *world_path) {
    path_init(world_path);
    for (int i = 0; i < grid_path->count; ++i) {
        int row = (int)grid_path->points[i].x;
        int col = (int)grid_path->points[i].y;
        double x, y;
        grid_to_world(m, row, col, &x, &y);
        path_push(world_path, x, y);
    }
}
