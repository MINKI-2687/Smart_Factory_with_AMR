#ifndef GRID_MAP_H
#define GRID_MAP_H

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "path.h"

typedef struct {
    int rows, cols;
    double resolution;
    double origin_x, origin_y;
    unsigned char *grid;

    /* ---- 여유거리(clearance) 지도 (2026-08-08f 추가) ----
     * 각 칸에서 가장 가까운 '진짜' 장애물까지의 거리(m). 팽창 전 원본 격자 기준.
     * A*가 "지나갈 수만 있으면 된다"가 아니라 "가능하면 한가운데로"를 고르게 하는 데 씀.
     *
     * 왜 필요한가: 팽창격자는 통과 가능/불가능만 알려주므로, A*는 최단경로를 찾다가
     * 팽창 경계에 바짝 붙은 길을 고르는 게 정상이다. 폭 40cm 통로에서 robot_radius
     * 0.14로 팽창하면 통과 가능한 띠가 y=0.59~0.70인데, A*는 그 중 아무 데나(예: 0.59)
     * 지나가도 된다고 본다. 그런데 차체 반대각선이 0.143m라 y=0.59에서 제자리 회전을
     * 하면 뒤 모서리가 y=0.447까지 내려가 벽(0.45)에 걸린다. 실기에서 통로 주행 중
     * 벽에 걸려 못 빠져나온 원인이 이것.
     * 여유거리를 비용에 넣으면 같은 통로에서 y=0.6475(정중앙, 여유 0.1975m)를 고른다. */
    double *clearance;          /* rows*cols, 단위 m. NULL이면 비용 가산 없음 */
    double clearance_pref_m;    /* 이만큼 여유가 있으면 벌점 0 */
    double clearance_weight;    /* 벌점 최대 배율 (여유 0일 때 이동비용 x(1+weight)) */
} OccupancyGridMap;
int grid_idx(const OccupancyGridMap *m, int row, int col);
bool grid_in_bounds(const OccupancyGridMap *m, int row, int col);
bool grid_is_free(const OccupancyGridMap *m, int row, int col);
bool grid_is_blocked(const OccupancyGridMap *m, int row, int col);
double *grid__build_clearance(const unsigned char *raw_grid,
                               int rows, int cols, double resolution);
void grid_init(OccupancyGridMap *m, const unsigned char *raw_grid,
                int rows, int cols, double resolution, double robot_radius,
                double origin_x, double origin_y);
void grid_free(OccupancyGridMap *m);
double grid_cost_multiplier(const OccupancyGridMap *m, int row, int col);
void world_to_grid(const OccupancyGridMap *m, double x, double y, int *row, int *col);
void grid_to_world(const OccupancyGridMap *m, int row, int col, double *x, double *y);
void path_grid_to_world(const OccupancyGridMap *m, const Path *grid_path, Path *world_path);

#endif /* GRID_MAP_H */
