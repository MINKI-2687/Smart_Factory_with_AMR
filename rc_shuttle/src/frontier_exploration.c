#include "frontier_exploration.h"


double frontier__prob(const OccupancyGridSLAM *s, int r, int c) {
    return 1.0 / (1.0 + exp(-s->log_odds[r * s->cols + c]));
}


int frontier_find_cells(const OccupancyGridSLAM *s,
                                       double free_prob, double occ_prob, double weak_occ_upper,
                                       double *out_x, double *out_y, int max_cells) {
    int n = 0;
    static const int dr[4] = {-1, 1, 0, 0};
    static const int dc[4] = {0, 0, -1, 1};

    for (int r = 0; r < s->rows && n < max_cells; r++) {
        for (int c = 0; c < s->cols && n < max_cells; c++) {
            if (frontier__prob(s, r, c) >= free_prob) continue;
            for (int k = 0; k < 4; k++) {
                int nr = r + dr[k], nc = c + dc[k];
                if (nr < 0 || nr >= s->rows || nc < 0 || nc >= s->cols) continue;
                double np_ = frontier__prob(s, nr, nc);
                bool is_unknown = (np_ > free_prob && np_ < occ_prob);
                bool is_weak_occupied = (np_ >= occ_prob && np_ < weak_occ_upper);
                if (is_unknown || is_weak_occupied) {
                    out_x[n] = c * s->resolution + s->resolution / 2.0;
                    out_y[n] = r * s->resolution + s->resolution / 2.0;
                    n++;
                    break;
                }
            }
        }
    }
    return n;
}


int frontier_cluster_cells(const double *xs, const double *ys, int n_cells,
                                          double cluster_dist, FrontierCluster *out_clusters,
                                          int max_clusters) {
    static bool visited[FRONTIER_MAX_CELLS];
    static int queue[FRONTIER_MAX_CELLS];
    for (int i = 0; i < n_cells; i++) visited[i] = false;

    int n_clusters = 0;
    double cd2 = cluster_dist * cluster_dist;

    for (int seed = 0; seed < n_cells; seed++) {
        if (visited[seed]) continue;
        if (n_clusters >= max_clusters) break;

        int qhead = 0, qtail = 0;
        queue[qtail++] = seed;
        visited[seed] = true;

        while (qhead < qtail) {
            int idx = queue[qhead++];
            for (int j = 0; j < n_cells; j++) {
                if (visited[j]) continue;
                double dx = xs[j] - xs[idx], dy = ys[j] - ys[idx];
                if (dx * dx + dy * dy < cd2) {
                    visited[j] = true;
                    if (qtail < FRONTIER_MAX_CELLS) queue[qtail++] = j;
                }
            }
        }

        /* 컴포넌트(queue[0..qtail))를 FRONTIER_MAX_CLUSTER_CELLS 칸씩 나눠서 각각
         * 별도 클러스터로 만듦 (작은 컴포넌트는 그냥 청크 1개 = 기존과 동일 동작) */
        for (int chunk_start = 0; chunk_start < qtail; chunk_start += FRONTIER_MAX_CLUSTER_CELLS) {
            if (n_clusters >= max_clusters) break;
            int chunk_end = chunk_start + FRONTIER_MAX_CLUSTER_CELLS;
            if (chunk_end > qtail) chunk_end = qtail;

            double sum_x = 0, sum_y = 0;
            int count = 0;
            for (int k = chunk_start; k < chunk_end; k++) {
                int idx = queue[k];
                sum_x += xs[idx]; sum_y += ys[idx]; count++;
            }
            out_clusters[n_clusters].x = sum_x / count;
            out_clusters[n_clusters].y = sum_y / count;
            out_clusters[n_clusters].size = count;
            n_clusters++;
        }
    }
    return n_clusters;
}


void frontier_explorer_init(FrontierExplorer *fe, int max_miss) {
    fe->has_target = false;
    fe->target_x = 0; fe->target_y = 0;
    fe->miss_count = 0;
    fe->max_miss = max_miss;
    fe->steps_on_current_target = 0;
    fe->stuck_timeout_steps = 600;
    fe->avoid_history_count = 0;
    fe->avoid_history_next = 0;
}


void frontier_explorer__remember(FrontierExplorer *fe, double x, double y) {
    fe->avoid_history_x[fe->avoid_history_next] = x;
    fe->avoid_history_y[fe->avoid_history_next] = y;
    fe->avoid_history_next = (fe->avoid_history_next + 1) % FRONTIER_EXPLORER_HISTORY;
    if (fe->avoid_history_count < FRONTIER_EXPLORER_HISTORY) fe->avoid_history_count++;
}


bool frontier_explorer_step(FrontierExplorer *fe, const OccupancyGridSLAM *s,
                                           double robot_x, double robot_y, bool reached_current) {
    bool need_new = reached_current || !fe->has_target;

    if (fe->has_target && !reached_current) {
        fe->steps_on_current_target++;
        if (fe->steps_on_current_target >= fe->stuck_timeout_steps) need_new = true;
    }

    if (!need_new) return false;

    if (fe->has_target) frontier_explorer__remember(fe, fe->target_x, fe->target_y);

    static double cell_x[FRONTIER_MAX_CELLS];
    static double cell_y[FRONTIER_MAX_CELLS];
    static FrontierCluster clusters[FRONTIER_MAX_CLUSTERS];
    int n_cells = frontier_find_cells(s, 0.45, 0.6, 0.8, cell_x, cell_y, FRONTIER_MAX_CELLS);
    int n_clusters = (n_cells > 0)
        ? frontier_cluster_cells(cell_x, cell_y, n_cells, 0.15, clusters, FRONTIER_MAX_CLUSTERS)
        : 0;

    bool found = false;
    double best_dist = 1e300;
    double best_x = 0, best_y = 0;
    for (int i = 0; i < n_clusters; i++) {
        if (clusters[i].size < 3) continue;
        bool blocked = false;
        for (int h = 0; h < fe->avoid_history_count; h++) {
            double adx = clusters[i].x - fe->avoid_history_x[h];
            double ady = clusters[i].y - fe->avoid_history_y[h];
            if (adx * adx + ady * ady <= 0.45 * 0.45) { blocked = true; break; }
        }
        if (blocked) continue;
        double dx = clusters[i].x - robot_x, dy = clusters[i].y - robot_y;
        double d = dx * dx + dy * dy;
        if (!found || d < best_dist) { found = true; best_dist = d; best_x = clusters[i].x; best_y = clusters[i].y; }
    }

    if (found) {
        fe->target_x = best_x; fe->target_y = best_y; fe->has_target = true;
        fe->miss_count = 0;
        fe->steps_on_current_target = 0;
        return true;
    }
    fe->miss_count++;
    fe->steps_on_current_target = 0;
    return false;
}


bool frontier_explorer_done(const FrontierExplorer *fe) {
    return fe->miss_count >= fe->max_miss;
}
