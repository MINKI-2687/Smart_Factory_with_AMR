#include "astar.h"


double astar_heuristic(int r1, int c1, int r2, int c2) {
    return hypot((double)(r1 - r2), (double)(c1 - c2));
}


bool astar_search(const OccupancyGridMap *m,
                                 int start_row, int start_col, int goal_row, int goal_col,
                                 Path *out_grid_path) {
    path_init(out_grid_path);

    if (!grid_in_bounds(m, start_row, start_col) || !grid_in_bounds(m, goal_row, goal_col))
        return false;
    /* 시작점은 안전성을 검사하지 않는다: 로봇이 실제로 그 위치에 있다는 사실은
     * 이미 벌어진 물리적 사실이므로, "안전지대인지"를 A*가 판단할 이유가 없다.
     * (재탐색 시 로봇이 장애물 팽창영역 경계에 바짝 붙어있는 경우가 흔한데,
     *  이때 시작점을 막힘으로 처리하면 재탐색 자체가 항상 실패하는 문제가 생김) */
    if (!grid_is_free(m, goal_row, goal_col)) return false;

    int n = m->rows * m->cols;
    double *g_score = (double *)malloc(sizeof(double) * n);
    int *came_from = (int *)malloc(sizeof(int) * n);
    bool *visited = (bool *)malloc(sizeof(bool) * n);
    for (int i = 0; i < n; ++i) { g_score[i] = -1; came_from[i] = -1; visited[i] = false; }

    MinHeap open;
    heap_init(&open);
    int start_idx = grid_idx(m, start_row, start_col);
    int goal_idx = grid_idx(m, goal_row, goal_col);
    g_score[start_idx] = 0;
    heap_push(&open, 0, start_row, start_col);

    bool found = false;

    while (!heap_empty(&open)) {
        HeapNode current = heap_pop(&open);
        int cur_idx = grid_idx(m, current.row, current.col);

        if (cur_idx == goal_idx) { found = true; break; }
        if (visited[cur_idx]) continue;
        visited[cur_idx] = true;

        for (int k = 0; k < 8; ++k) {
            int dr = ASTAR_MOVES[k][0], dc = ASTAR_MOVES[k][1];
            int nr = current.row + dr, nc = current.col + dc;
            if (!grid_is_free(m, nr, nc)) continue;

            if (dr != 0 && dc != 0) {
                if (grid_is_blocked(m, current.row + dr, current.col) ||
                    grid_is_blocked(m, current.row, current.col + dc)) {
                    continue;
                }
            }

            int n_idx = grid_idx(m, nr, nc);
            if (visited[n_idx]) continue;

            /* 이동비용에 '여유거리 벌점'을 곱한다 (2026-08-08f).
             * 통과 가능/불가능만 보던 예전 비용은 팽창 경계에 붙은 최단경로를 골랐고,
             * 그 경로 위에서는 제자리 회전 시 차체 모서리가 벽에 걸렸다
             * (grid_map.h 의 clearance 주석 참고). 벌점은 유한값이라, 좁은 길밖에
             * 없으면 여전히 그 길을 고른다 - 경로가 사라지지는 않는다. */
            double cost = hypot((double)dr, (double)dc) * grid_cost_multiplier(m, nr, nc);
            double tentative_g = g_score[cur_idx] + cost;

            if (g_score[n_idx] < 0 || tentative_g < g_score[n_idx]) {
                g_score[n_idx] = tentative_g;
                double f = tentative_g + astar_heuristic(nr, nc, goal_row, goal_col);
                heap_push(&open, f, nr, nc);
                came_from[n_idx] = cur_idx;
            }
        }
    }

    if (found) {
        Path reversed;
        path_init(&reversed);
        int idx = goal_idx;
        path_push(&reversed, (double)(idx / m->cols), (double)(idx % m->cols));
        while (came_from[idx] != -1) {
            idx = came_from[idx];
            path_push(&reversed, (double)(idx / m->cols), (double)(idx % m->cols));
        }
        for (int i = reversed.count - 1; i >= 0; --i) {
            path_push(out_grid_path, reversed.points[i].x, reversed.points[i].y);
        }
        path_free(&reversed);
    }

    free(g_score);
    free(came_from);
    free(visited);
    heap_free(&open);
    return found;
}
