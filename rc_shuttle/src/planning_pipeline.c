#include "planning_pipeline.h"


/* Python PathPlanningPipeline.plan()과 동일 로직:
 *   A* 탐색 -> world 좌표 변환 -> simplify -> (옵션) smooth -> 스무딩 안전성 체크(불안전하면 되돌림)
 *
 * enable_smoothing이 false면 simplify까지만 하고 반환.
 * 성공 시 out에 최종 웨이포인트를 채우고 true 반환, 실패(경로 없음)면 false.
 */
bool planning_pipeline_plan(const OccupancyGridMap *map,
                                           double start_x, double start_y,
                                           double goal_x, double goal_y,
                                           double simplify_min_dist,
                                           bool enable_smoothing,
                                           double smooth_weight_data, double smooth_weight_smooth,
                                           double smooth_tolerance, int smooth_max_iterations,
                                           Path *out) {
    int sr, sc, gr, gc;
    world_to_grid(map, start_x, start_y, &sr, &sc);
    world_to_grid(map, goal_x, goal_y, &gr, &gc);

    Path grid_path;
    bool found = astar_search(map, sr, sc, gr, gc, &grid_path);
    if (!found) {
        path_free(&grid_path);
        path_init(out);
        return false;
    }

    Path world_path;
    path_grid_to_world(map, &grid_path, &world_path);
    path_free(&grid_path);

    Path simplified;
    path_simplify(&world_path, simplify_min_dist, &simplified);
    path_free(&world_path);

    if (!enable_smoothing) {
        *out = simplified;  /* 소유권 이전 (out을 이후 path_free 하면 됨) */
        return true;
    }

    Path smoothed;
    path_smooth(&simplified, smooth_weight_data, smooth_weight_smooth,
                smooth_tolerance, smooth_max_iterations, &smoothed);

    /* 스무딩된 점이 장애물(팽창영역) 안으로 들어갔으면 simplify 결과값으로 되돌림 */
    path_init(out);
    for (int i = 0; i < smoothed.count; ++i) {
        int row, col;
        world_to_grid(map, smoothed.points[i].x, smoothed.points[i].y, &row, &col);
        if (grid_is_free(map, row, col)) {
            path_push(out, smoothed.points[i].x, smoothed.points[i].y);
        } else {
            path_push(out, simplified.points[i].x, simplified.points[i].y);
        }
    }

    path_free(&simplified);
    path_free(&smoothed);
    return true;
}
