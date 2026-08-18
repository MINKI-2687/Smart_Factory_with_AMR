#ifndef ASTAR_H
#define ASTAR_H

#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include "grid_map.h"
#include "min_heap.h"

/* ============================================================
 * A* 탐색 (Python AStarPlanner.search와 동일 로직, 대각선 모퉁이깎기 방지 포함)
 * 이제 이 파일엔 진짜 A* 알고리즘 자체만 있음. Path/격자/힙은 각자 파일로 분리됨
 * (path.h, grid_map.h, min_heap.h).
 * 반환: grid_path(격자좌표 (row,col) 나열), 실패 시 grid_path.count == 0
 * ============================================================ */
static const int ASTAR_MOVES[8][2] = {
    {-1,0}, {1,0}, {0,-1}, {0,1}, {-1,-1}, {-1,1}, {1,-1}, {1,1}
};
double astar_heuristic(int r1, int c1, int r2, int c2);
bool astar_search(const OccupancyGridMap *m,
                   int start_row, int start_col, int goal_row, int goal_col,
                   Path *out_grid_path);

#endif /* ASTAR_H */
