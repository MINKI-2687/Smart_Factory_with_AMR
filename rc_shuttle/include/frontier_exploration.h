#ifndef FRONTIER_EXPLORATION_H
#define FRONTIER_EXPLORATION_H

#include <math.h>
#include <stdbool.h>
#include "slam.h"

#define FRONTIER_MAX_CELLS 4000
#define FRONTIER_MAX_CLUSTERS 200

typedef struct {
    double x, y;
    int size;
} FrontierCluster;
double frontier__prob(const OccupancyGridSLAM *s, int r, int c);
int frontier_find_cells(const OccupancyGridSLAM *s,
                         double free_prob, double occ_prob, double weak_occ_upper,
                         double *out_x, double *out_y, int max_cells);

/* PATCH (2026-08-03): 연결된 프론티어 컴포넌트 하나가 너무 커지면(예: 크게 뚫린 방의
 * 경계선 전체가 체인으로 이어진 경우) FRONTIER_MAX_CLUSTER_CELLS 칸씩 끊어서 여러 개의
 * 하위 대표점으로 나눔. 원래는 컴포넌트 전체를 평균내서 대표점 1개만 뽑았는데, 그러면
 * "방 전체 경계선 = 클러스터 1개"가 되어버려서:
 *   1) 실제로는 여러 방향에 갈 곳이 남아있는데 전부 뭉뚱그려진 점 하나로만 안내됨
 *   2) 그 대표점이 이미 방문한 곳(회피이력) 근처에 걸리면, 다른 대안이 하나도 없어서
 *      "탐사 끝"으로 조기 오판(miss_count 누적) - 실측으로 재현/확인된 버그
 * (참고: BFS 방문 순서(queue)는 서로 인접한 칸을 이어서 담기 때문에, 이 순서 그대로
 *  청크로 자르면 각 청크는 여전히 공간적으로 뭉쳐있는 구간이 됨) */
#define FRONTIER_MAX_CLUSTER_CELLS 20
int frontier_cluster_cells(const double *xs, const double *ys, int n_cells,
                            double cluster_dist, FrontierCluster *out_clusters,
                            int max_clusters);

#define FRONTIER_EXPLORER_HISTORY 8

typedef struct {
    double target_x, target_y;
    bool has_target;
    int miss_count;
    int max_miss;
    int steps_on_current_target;
    int stuck_timeout_steps;
    double avoid_history_x[FRONTIER_EXPLORER_HISTORY];
    double avoid_history_y[FRONTIER_EXPLORER_HISTORY];
    int avoid_history_count;
    int avoid_history_next;
} FrontierExplorer;
void frontier_explorer_init(FrontierExplorer *fe, int max_miss);
void frontier_explorer__remember(FrontierExplorer *fe, double x, double y);
bool frontier_explorer_step(FrontierExplorer *fe, const OccupancyGridSLAM *s,
                             double robot_x, double robot_y, bool reached_current);
bool frontier_explorer_done(const FrontierExplorer *fe);

#endif /* FRONTIER_EXPLORATION_H */
