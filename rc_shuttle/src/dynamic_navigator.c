#include "dynamic_navigator.h"


void dynnav__world_to_grid(const DynamicNavigator *nav, double x, double y,
                                          int *row, int *col) {
    *col = (int)((x - nav->origin_x) / nav->resolution);
    *row = (int)((y - nav->origin_y) / nav->resolution);
}


bool dynnav__in_bounds(const DynamicNavigator *nav, int row, int col) {
    return row >= 0 && row < nav->rows && col >= 0 && col < nav->cols;
}


bool dynnav_register_unexpected_obstacle(DynamicNavigator *nav,
                                                         const double *xs, const double *ys, int n) {
    bool changed = false;
    for (int i = 0; i < n; i++) {
        int row, col;
        dynnav__world_to_grid(nav, xs[i], ys[i], &row, &col);
        if (dynnav__in_bounds(nav, row, col)) {
            int idx = row * nav->cols + col;
            if (nav->known_grid[idx] != 1) {
                nav->known_grid[idx] = 1;
                changed = true;
            }
        }
    }
    return changed;
}


void dynnav_init(DynamicNavigator *nav, const unsigned char *raw_grid,
                                int rows, int cols, double resolution, double robot_radius,
                                double origin_x, double origin_y) {
    nav->resolution = resolution;
    nav->robot_radius = robot_radius;
    nav->origin_x = origin_x;
    nav->origin_y = origin_y;
    nav->rows = rows;
    nav->cols = cols;

    nav->known_grid = (unsigned char *)malloc((size_t)rows * cols);
    memcpy(nav->known_grid, raw_grid, (size_t)rows * cols);

    nav->map_valid = false;

    goal_pid_init_default(&nav->goal_ctrl, 0.06);
    obstacle_avoidance_init_default(&nav->avoider);
    safe_navigator_init(&nav->local_ctrl, &nav->goal_ctrl, &nav->avoider);

    docking_init(&nav->docking_ctrl);
    nav->docking_ctrl.max_linear_speed = nav->goal_ctrl.max_linear_speed;
    nav->docking_ctrl.max_angular_speed = nav->goal_ctrl.max_angular_speed;
    nav->docking_radius = 0.4;
    nav->goal_unreachable = false;
    nav->use_pure_pursuit = false;   /* 호출부에서 켜도록 - 슬롯 정밀구간에서는 끄는 게 안전 */
    nav->lookahead_m = 0.22;
    nav->lookahead_max_m = 0.66;
    memset(&nav->track, 0, sizeof(nav->track));
    nav->finish_box_x = 0.0;
    nav->finish_box_y = 0.0;
    nav->docking_mode = false;

    path_init(&nav->waypoints);
    nav->current_wp_idx = 0;
    nav->steps_on_current_wp = 0;
    nav->skip_after_steps = 600;

    nav->replan_detect_radius = 1.5;
    nav->max_replans = 3;
    nav->replan_count = 0;
    nav->has_goal_theta = false;
    nav->replanned_this_call = false;
    nav->last_plan_fail_reason[0] = '\0';
}


void dynnav_free(DynamicNavigator *nav) {
    free(nav->known_grid);
    nav->known_grid = NULL;
    if (nav->map_valid) grid_free(&nav->map);
    path_free(&nav->waypoints);
}


/* Restore cells that are ONLY blocked because of robot_radius inflation (not because
 * there's an actual obstacle there) within a small radius of the given world point.
 * Used for both the start point (robot may already be physically touching a wall/
 * obstacle safety margin after mapping or a replan) and the goal point (a user- or
 * factory-chosen dock point is often placed flush against a wall on purpose). Cells
 * that are genuinely occupied in the RAW (pre-inflation) known_grid stay blocked. */
void dynnav__rescue_inflation_near(DynamicNavigator *nav, double x, double y) {
    int row, col;
    dynnav__world_to_grid(nav, x, y, &row, &col);
    int rescue_cells = (int)ceil(nav->robot_radius / nav->resolution) + 1;
    for (int dr = -rescue_cells; dr <= rescue_cells; dr++) {
        for (int dc = -rescue_cells; dc <= rescue_cells; dc++) {
            int rr = row + dr, rc = col + dc;
            if (!dynnav__in_bounds(nav, rr, rc)) continue;
            int idx = rr * nav->cols + rc;
            if (nav->known_grid[idx] == 0) {
                nav->map.grid[idx] = 0;
            }
        }
    }
}


/* A*계획이 실패했을 때, 왜 실패했는지 사람이 읽을 수 있는 문자열로 진단해서
 * nav->last_plan_fail_reason에 채워넣음. 원인을 순서대로 좁혀나감:
 *   1) 출발점이 지도 범위 밖
 *   2) 목표점이 지도 범위 밖
 *   3) 목표칸이 (팽창 면제 후에도) 막혀있음 - 진짜 장애물이 그 자리를 덮고 있다는 뜻
 *   4) 출발점 주변 8칸이 전부 막혀서 첫 걸음도 못 뗌 (팽창면제로도 못 푼 경우 -
 *      실제로 사방이 진짜 장애물/벽으로 둘러싸인 경우)
 *   5) 위 전부 아니면: 출발점과 목표점 자체는 멀쩡한데 그 사이를 잇는 경로가 없음
 *      (지도가 두 영역으로 완전히 분리돼있을 가능성) */
void dynnav__diagnose_plan_failure(DynamicNavigator *nav, double start_x, double start_y) {
    int start_row, start_col, goal_row, goal_col;
    dynnav__world_to_grid(nav, start_x, start_y, &start_row, &start_col);
    dynnav__world_to_grid(nav, nav->goal_x, nav->goal_y, &goal_row, &goal_col);

    if (!dynnav__in_bounds(nav, start_row, start_col)) {
        snprintf(nav->last_plan_fail_reason, sizeof(nav->last_plan_fail_reason),
                 "start point (%.3f, %.3f) is outside the map bounds (%d x %d cells, resolution=%.3f -> "
                 "map covers 0..%.3f x 0..%.3f)",
                 start_x, start_y, nav->rows, nav->cols, nav->resolution,
                 nav->cols * nav->resolution, nav->rows * nav->resolution);
        return;
    }
    if (!dynnav__in_bounds(nav, goal_row, goal_col)) {
        snprintf(nav->last_plan_fail_reason, sizeof(nav->last_plan_fail_reason),
                 "goal point (%.3f, %.3f) is outside the map bounds (%d x %d cells, resolution=%.3f -> "
                 "map covers 0..%.3f x 0..%.3f)",
                 nav->goal_x, nav->goal_y, nav->rows, nav->cols, nav->resolution,
                 nav->cols * nav->resolution, nav->rows * nav->resolution);
        return;
    }
    if (!grid_is_free(&nav->map, goal_row, goal_col)) {
        int raw_idx = goal_row * nav->cols + goal_col;
        bool raw_blocked = nav->known_grid[raw_idx] != 0;
        snprintf(nav->last_plan_fail_reason, sizeof(nav->last_plan_fail_reason),
                 "goal cell (%.3f, %.3f) is blocked even after inflation-rescue - %s",
                 nav->goal_x, nav->goal_y,
                 raw_blocked
                     ? "a real obstacle/wall actually occupies that cell in the raw map (pick a different goal)"
                     : "still marked blocked despite the raw map being free there (unexpected - check "
                       "robot_radius/rescue radius, or resolution mismatch if this map was loaded from a file)");
        return;
    }

    int free_neighbors = 0;
    for (int dr = -1; dr <= 1; dr++) {
        for (int dc = -1; dc <= 1; dc++) {
            if (dr == 0 && dc == 0) continue;
            if (grid_is_free(&nav->map, start_row + dr, start_col + dc)) free_neighbors++;
        }
    }
    if (free_neighbors == 0) {
        snprintf(nav->last_plan_fail_reason, sizeof(nav->last_plan_fail_reason),
                 "start point (%.3f, %.3f) has zero free neighboring cells - the robot is completely "
                 "boxed in by real obstacles/walls (not just inflation) and can't take a single step",
                 start_x, start_y);
        return;
    }

    snprintf(nav->last_plan_fail_reason, sizeof(nav->last_plan_fail_reason),
             "start (%.3f, %.3f) and goal (%.3f, %.3f) are both individually reachable cells, but no "
             "path connects them - the map is likely split into disconnected regions (a wall/obstacle "
             "fully separates the two areas)",
             start_x, start_y, nav->goal_x, nav->goal_y);
}


bool dynnav__plan_internal(DynamicNavigator *nav, double start_x, double start_y) {
    if (nav->map_valid) grid_free(&nav->map);
    grid_init(&nav->map, nav->known_grid, nav->rows, nav->cols,
              nav->resolution, nav->robot_radius, nav->origin_x, nav->origin_y);
    nav->map_valid = true;

    /* PATCH (2026-08-04): Rescue inflation-only-blocked cells around BOTH the start point
     * AND the goal point, not just the start.
     *
     * Originally this only rescued the start point (robot ending mapping/replanning right
     * up against a wall/obstacle safety margin, blocking all 8 neighbor cells so A* couldn't
     * take a single step). But the exact same problem happens at the GOAL: if a user picks a
     * goal point very close to a wall (e.g. via map_picker_shuttle_gui.py clicking near a
     * corner), inflation blocks the goal cell itself, and astar_search() requires
     * grid_is_free(goal) to succeed - so planning fails immediately with no fallback
     * (confirmed via real test: goal picked at (0.12, 2.87), ~0.12m from a wall, robot_radius
     * =0.10m -> goal cell swallowed by inflation -> "A* planning failed").
     *
     * This is a legitimate real-world case too: a factory loading/unloading dock is often
     * placed flush against a wall on purpose, not a mapping artifact. So both cases deserve
     * the same treatment: only exempt cells that are actually free in the RAW (pre-inflation)
     * map - real obstacles stay blocked either way. */
    dynnav__rescue_inflation_near(nav, start_x, start_y);
    dynnav__rescue_inflation_near(nav, nav->goal_x, nav->goal_y);

    Path new_waypoints;
    bool ok = planning_pipeline_plan(&nav->map, start_x, start_y, nav->goal_x, nav->goal_y,
                                      0.15, true, 0.5, 0.3, 1e-4, 1000, &new_waypoints);
    if (!ok) {
        path_free(&new_waypoints);
        dynnav__diagnose_plan_failure(nav, start_x, start_y);
        return false;
    }
    path_free(&nav->waypoints);
    nav->waypoints = new_waypoints;
    nav->current_wp_idx = 0;
    nav->steps_on_current_wp = 0;
    nav->track.valid = false;   /* 새 경로 - 예전 투영결과를 그대로 쓰면 안 됨 */
    return true;
}


/* dynnav_set_goal() 또는 재탐색이 false를 반환했을 때만 이 문자열이 유효함. */
const char *dynnav_get_last_plan_fail_reason(const DynamicNavigator *nav) {
    return nav->last_plan_fail_reason;
}


bool dynnav_set_goal(DynamicNavigator *nav, double start_x, double start_y,
                                    double goal_x, double goal_y,
                                    bool has_goal_theta, double goal_theta,
                                    const double *known_xs, const double *known_ys, int n_known) {
    nav->goal_unreachable = false;
    if (known_xs != NULL && n_known > 0) {
        dynnav_register_unexpected_obstacle(nav, known_xs, known_ys, n_known);
    }
    nav->goal_x = goal_x;
    nav->goal_y = goal_y;
    nav->has_goal_theta = has_goal_theta;
    nav->goal_theta = goal_theta;
    nav->replan_count = 0;
    nav->docking_mode = false;

    return dynnav__plan_internal(nav, start_x, start_y);
}


bool dynnav__replan_from(DynamicNavigator *nav, double cur_x, double cur_y) {
    return dynnav__plan_internal(nav, cur_x, cur_y);
}


/* ------------------------------------------------------------
 * Pure pursuit: 경로 위에서 로봇보다 lookahead_m 만큼 앞선 점을 목표로 삼음.
 *
 * PATCH (2026-08-06): 기존에는 waypoints[current_wp_idx]를 직접 조준하고, 그 점에
 * goal_tolerance(0.06m) 안으로 들어와야 다음 웨이포인트로 넘어갔음. 그래서 웨이포인트
 * 마다 "정확히 그 점에 도달 -> 다음 점 방향으로 회전"을 반복하게 되고, 특히 코너에서
 * 목표 방향이 급격히 꺾이면서 angle_error가 커져(>60도면 전진속도 0) 제자리 회전만
 * 하다가 다시 출발하는 멈칫거림이 생김.
 *
 * 전방주시점은 경로를 따라 항상 일정 거리 앞에 있으므로 목표 방향이 연속적으로 변함
 * -> 조향이 부드러워지고 멈출 일이 없음. lookahead가 클수록 부드럽지만 코너를 크게
 * 잘라먹고, 작을수록 경로를 정확히 따르지만 다시 뻣뻣해짐.
 * ------------------------------------------------------------ */
/* PATCH (2026-08-09e): 위 방식을 path_follow.h 의 '투영 + 적응형 전방주시'로 교체.
 *
 * 예전 코드의 문제는 두 가지였다.
 *  (1) 전방주시 거리가 0.22m 로 고정이라, 경로에서 옆으로 d 만큼 벗어나면 목표
 *      방위각이 접선에서 asin(d/0.22) 만큼 꺾였다(d=0.10m -> 27도, 0.15m -> 43도).
 *      safe_navigator 는 30도를 넘으면 선속도를 0으로 두고 제자리 회전만 하는데,
 *      높이 0.40m 통로에서는 제자리 회전 여유가 안 나와 '끼임 탈출'이 걸리고,
 *      탈출은 앞뒤로만 움직여 좌우 여유를 전혀 못 바꾼다 -> 무한 왕복.
 *      (사용자 신고: "경로에서 벗어나면 크게 직진/후진만 반복")
 *  (2) current_wp_idx 가 단조증가라, 한 번 지나친 뒤에는 되돌아올 방법이 없었다.
 *
 * 이제 매 스텝 경로 위 최근접점을 다시 찾고, 벗어난 거리에 비례해 전방주시를
 * 늘려서 목표 방위각이 접선에서 최대 21.8도만 꺾이게 한다. 제자리 회전을 요구하지
 * 않으므로 통로에서 비스듬히 완만하게 경로로 복귀한다. */
Point2D dynnav__lookahead_point(DynamicNavigator *nav, double x, double y, double L) {
    Point2D p;
    int n = nav->waypoints.count;
    if (n <= 0) { p.x = x; p.y = y; return p; }

    int hint = nav->current_wp_idx;
    if (hint < 0) hint = 0;
    if (hint > n - 1) hint = n - 1;

    double maxL = (nav->lookahead_max_m > L) ? nav->lookahead_max_m : (3.0 * L);
    if (!path_follow_track(&nav->waypoints, hint, x, y, L, maxL, &nav->track)) {
        p = nav->waypoints.points[n - 1];
        return p;
    }

    /* 진행상황 표시용 인덱스도 투영 위치에 맞춰 갱신(GUI/정체판정이 이 값을 씀).
     * 예전처럼 '도달해야 넘어가는' 값이 아니라 '지금 어느 구간을 지나는가'가 된다. */
    int idx = nav->track.seg_index + 1;
    if (idx > n - 1) idx = n - 1;
    if (idx < 0) idx = 0;
    nav->current_wp_idx = idx;

    p.x = nav->track.look_x;
    p.y = nav->track.look_y;
    return p;
}


WheelCmd dynnav_update(DynamicNavigator *nav, double x, double y, double theta,
                                      const LidarScan *scan,
                                      const DetectedObstacle *detected, int n_detected) {
    WheelCmd result;
    nav->replanned_this_call = false;

    if (nav->current_wp_idx >= nav->waypoints.count) {
        result.left = 0; result.right = 0; result.done = true;
        return result;
    }

    double dist_to_final_goal = hypot(x - nav->goal_x, y - nav->goal_y);

    /* PATCH (2026-08-09e): 도킹 전환 판정을 '웨이포인트 인덱스'가 아니라 '남은 경로
     * 길이'로 바꿈. 인덱스는 정체 복구에서 건너뛰어질 수 있어서, 경로 중간인데도
     * 마지막 웨이포인트로 표시돼 엉뚱한 자리에서 도킹이 시작되는 경우가 있었다. */
    bool near_path_end = nav->track.valid
                            ? (nav->track.remain <= nav->docking_radius)
                            : (nav->current_wp_idx >= nav->waypoints.count - 1);

    bool should_dock = nav->has_goal_theta && near_path_end
                        && (dist_to_final_goal < nav->docking_radius);

    if (nav->docking_mode || should_dock) {
        if (!nav->docking_mode) {
            nav->docking_mode = true;
            docking_reset(&nav->docking_ctrl);
        }
        WheelCmd cmd = docking_compute(&nav->docking_ctrl, x, y, theta,
                                        nav->goal_x, nav->goal_y, nav->goal_theta);
        if (cmd.done) {
            nav->current_wp_idx = nav->waypoints.count;
            result.left = 0; result.right = 0; result.done = true;
            return result;
        }
        cmd.done = false;
        return cmd;
    }

    Point2D target;
    int prev_wp_idx = nav->current_wp_idx;
    if (nav->use_pure_pursuit) {
        target = dynnav__lookahead_point(nav, x, y, nav->lookahead_m);
    } else {
        target = nav->waypoints.points[nav->current_wp_idx];
    }
    WheelCmd cmd = safe_navigator_compute(&nav->local_ctrl, x, y, theta, target.x, target.y, scan);

    /* PATCH (2026-08-09e): 정체 카운터를 '투영 구간이 바뀌면' 리셋한다.
     * 새 pure pursuit 은 current_wp_idx 를 매 스텝 투영 위치로 다시 계산하므로,
     * 예전처럼 cmd.done 을 기다리면 카운터가 영영 안 줄고 600스텝 뒤에 멀쩡한
     * 주행이 '정체'로 오판돼 goal_unreachable 로 끊겼다. 이제 이 카운터는
     * "경로 진행이 실제로 멈춘 스텝 수"라는 원래 의미를 갖는다. */
    if (nav->use_pure_pursuit && nav->current_wp_idx != prev_wp_idx) {
        nav->steps_on_current_wp = 0;
    }
    nav->steps_on_current_wp += 1;

    /* pure pursuit에서는 전방주시점이 항상 앞에 있어서 safe_navigator의 done이
     * 안 뜨므로(=웨이포인트 전진이 안 일어남), 마지막 점 근처 도달을 따로 판정.
     * 최종 접근/정지는 아래 도킹 단계가 담당하므로 여기서는 '경로 끝에 왔다'만 알림. */
    if (nav->use_pure_pursuit) {
        bool at_last = nav->track.valid
                          ? (nav->track.remain <= nav->local_ctrl.goal_controller->goal_tolerance)
                          : (nav->current_wp_idx >= nav->waypoints.count - 1);
        double d_end = hypot(x - nav->goal_x, y - nav->goal_y);
        if (at_last && d_end < nav->local_ctrl.goal_controller->goal_tolerance) {
            nav->current_wp_idx = nav->waypoints.count;
            result.left = 0; result.right = 0; result.done = true;
            return result;
        }
    }

    bool stuck = (!cmd.done) && (nav->steps_on_current_wp > nav->skip_after_steps);

    if (stuck) {
        if (nav->replan_count < nav->max_replans && detected != NULL && n_detected > 0) {
            bool newly_found = false;
            for (int i = 0; i < n_detected; i++) {
                double ox = 0, oy = 0;
                for (int k = 0; k < detected[i].count; k++) { ox += detected[i].xs[k]; oy += detected[i].ys[k]; }
                if (detected[i].count > 0) { ox /= detected[i].count; oy /= detected[i].count; }
                if (hypot(x - ox, y - oy) < nav->replan_detect_radius) {
                    if (dynnav_register_unexpected_obstacle(nav, detected[i].xs, detected[i].ys, detected[i].count))
                        newly_found = true;
                }
            }
            if (newly_found && dynnav__replan_from(nav, x, y)) {
                nav->replan_count += 1;
                nav->replanned_this_call = true;
                goal_pid_reset(&nav->goal_ctrl);
                result.left = 0; result.right = 0; result.done = false;
                return result;
            }
        }

        nav->current_wp_idx += 1;
        nav->steps_on_current_wp = 0;
        if (nav->current_wp_idx >= nav->waypoints.count) {
            /* 막혀서 웨이포인트를 전부 건너뛴 상태. 실제로 목적지 근처면 도착으로 봐도
             * 되지만, 멀리 떨어져 있으면 '도착'이 아니라 '포기'이므로 반드시 구분해서
             * 알려야 함 - 위 goal_unreachable 주석 참고. */
            if (dist_to_final_goal > DYNNAV_ARRIVAL_MAX_ERR_M) {
                nav->goal_unreachable = true;
            }
            result.left = 0; result.right = 0; result.done = true;
            return result;
        }
        cmd.done = false;
        return cmd;
    }

    if (cmd.done) {
        nav->current_wp_idx += 1;
        nav->steps_on_current_wp = 0;
        if (nav->current_wp_idx >= nav->waypoints.count) {
            result.left = 0; result.right = 0; result.done = true;
            return result;
        }
    }

    cmd.done = false;
    return cmd;
}
