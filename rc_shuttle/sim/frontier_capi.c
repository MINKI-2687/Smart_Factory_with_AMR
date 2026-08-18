/* frontier_capi.c
 * frontier_exploration.h(순수 C, header-only)를 파이썬 ctypes에서 부를 수 있게 감싸는 래퍼.
 * slam_capi.c의 OccupancyGridSLAM 핸들을 그대로 재사용함(같은 slam_capi_create로 만든 걸 씀).
 *
 * 컴파일: gcc -std=c11 -O2 -shared -fPIC -o libfrontier.so frontier_capi.c -lm
 */
#include <stdlib.h>
#include "frontier_exploration.h"

/* 반환: 목표를 찾았으면 1(+out_x/out_y 채움), 못 찾았으면(탐사 끝) 0 */
int frontier_capi_pick_target(void *slam_handle, double robot_x, double robot_y,
                               int min_cluster_size,
                               int has_avoid_point, double avoid_x, double avoid_y, double avoid_radius,
                               double *out_x, double *out_y) {
    OccupancyGridSLAM *s = (OccupancyGridSLAM *)slam_handle;
    bool found = frontier_pick_target(s, robot_x, robot_y, min_cluster_size,
                                       has_avoid_point != 0, avoid_x, avoid_y, avoid_radius,
                                       out_x, out_y);
    return found ? 1 : 0;
}

/* ---- FrontierExplorer 상태머신 - 도착시 즉시 근처제외 재선정까지 포함한 완결형 API ---- */
void *frontier_capi_explorer_create(int max_miss) {
    FrontierExplorer *fe = (FrontierExplorer *)malloc(sizeof(FrontierExplorer));
    frontier_explorer_init(fe, max_miss);
    return fe;
}

void frontier_capi_explorer_destroy(void *handle) {
    free(handle);
}

int frontier_capi_explorer_update(void *handle, void *slam_handle,
                                   double robot_x, double robot_y, int reached_current) {
    FrontierExplorer *fe = (FrontierExplorer *)handle;
    OccupancyGridSLAM *s = (OccupancyGridSLAM *)slam_handle;
    return frontier_explorer_update(fe, s, robot_x, robot_y, reached_current != 0) ? 1 : 0;
}

int frontier_capi_explorer_done(void *handle) {
    return frontier_explorer_done((FrontierExplorer *)handle) ? 1 : 0;
}

void frontier_capi_explorer_get_target(void *handle, double *out_x, double *out_y) {
    FrontierExplorer *fe = (FrontierExplorer *)handle;
    *out_x = fe->target_x; *out_y = fe->target_y;
}

int frontier_capi_explorer_has_target(void *handle) {
    return ((FrontierExplorer *)handle)->has_target ? 1 : 0;
}
