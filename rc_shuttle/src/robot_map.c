#include "robot_map.h"
#include "robot_runner.h"


bool run_slam_mapping_phase(FPGALink *fpga, LidarThread *lidar, const RobotConfig *cfg,
                                           unsigned char **out_grid, int *out_rows, int *out_cols,
                                           double *out_x, double *out_y, double *out_theta) {
    printf("[main][SLAM] mapping phase start (frontier exploration to build map)\n");
    printf("GOAL %.4f %.4f\n", cfg->goal_x, cfg->goal_y);
    fflush(stdout);

    OccupancyGridSLAM slam;
    slam_init(&slam, cfg->grid_size_x, cfg->grid_size_y, cfg->resolution, 0.0, 0.0,
              cfg->start_x, cfg->start_y, 0.0, 0.15, 12.0, 0.03, 3.0);

    /* 프론티어 추적 전용 - "정밀 도착"이 아니라 "라이다로 보일 만큼만 가까워지면 충분"이므로
     * 넉넉한 허용오차를 씀(목적지 도킹용 0.15와는 다른 목적) */
    GoalPIDController goal_ctrl;
    goal_pid_init_default(&goal_ctrl, 0.4);
    ObstacleAvoidance avoider;
    obstacle_avoidance_init_default(&avoider);
    SafeNavigator local_ctrl;
    safe_navigator_init(&local_ctrl, &goal_ctrl, &avoider);

    double prev_raw_x, prev_raw_y, prev_raw_theta;
    robot_runner__init_prev_pose(fpga, cfg->has_encoder, &prev_raw_x, &prev_raw_y, &prev_raw_theta);

    /* ============================================================
     * PATCH (2026-08-03): 기존에는 여기서 has_target/target_x/target_y/miss_count/
     * stuck_count/avoid_x/avoid_y를 손으로 직접 관리하면서 30스텝마다 블라인드로
     * frontier_pick_target()을 재호출했음. 이 방식의 근본 문제:
     *   1) "실제로 목표에 도착했는지"를 한 번도 확인하지 않음 - 그냥 주기적으로
     *      재계산만 하고, 새 목표가 이전 목표와 비슷하면(<0.05m) "갇혔다"고 추측만 함.
     *   2) 회피지점을 avoid_x/avoid_y 단 1개만 기억 (매번 덮어씀) -> A<->B 핑퐁 가능.
     *   3) 핑퐁 감지까지 FRONTIER_STUCK_LIMIT(5)*RECHECK_INTERVAL(30) = 150스텝(7.5초)
     *      이나 걸림.
     * frontier_exploration.h에는 이 문제를 풀려고 이미 만들어둔 FrontierExplorer
     * 상태머신(히스토리 8개, 실제 도착여부 기반, stuck_timeout)이 있었는데 여기서는
     * 안 쓰고 있었음. 아래는 FrontierExplorer를 실제로 사용하도록 교체한 버전.
     * ============================================================ */
    const int FRONTIER_MAX_MISS = 3;              /* 이 횟수만큼 연속으로 새 프론티어를
                                                     * 못 찾아야 "진짜 탐사 끝" */
    const double FRONTIER_REACH_TOLERANCE = 0.4;  /* goal_ctrl의 허용오차(0.4)와 일치시킴 -
                                                     * "도착판정"의 기준이 서로 다르면 안 됨 */

    FrontierExplorer fe;
    frontier_explorer_init(&fe, FRONTIER_MAX_MISS);
    /* 기본값 600스텝(30초)은 이 3x3m 방에는 다소 길어서, 갇힘 감지를 10초로 단축.
     * 필요하면 시나리오별로 튜닝 가능. */
    fe.stuck_timeout_steps = cfg->control_hz * 10;

    double control_period = 1.0 / cfg->control_hz;
    int step;
    for (step = 0; step < cfg->slam_mapping_max_steps; step++) {
        if (g_stop_requested) {
            printf("[main][SLAM] stop signal received during mapping phase\n");
            break;
        }

        double odom_dx, odom_dy, odom_dth;
        robot_runner__predict_delta(fpga, cfg, slam.theta, &prev_raw_x, &prev_raw_y, &prev_raw_theta,
                                     &odom_dx, &odom_dy, &odom_dth);

        LidarScan scan;
        lidar_thread_get_latest(lidar, &scan);
        apply_lidar_frame_fix(&scan, cfg->lidar_mirror, cfg->lidar_yaw_offset_deg,
                              cfg->lidar_offset_forward_m);

        if (step == 1) {
            for (int i = 0; i < 5 && i < scan.count; i++)
                fprintf(stderr, "(angle%.1f,dist%.1fmm) ", scan.readings[i].angle_deg, scan.readings[i].dist_mm);
            fprintf(stderr, "\n");
        }
        double sx, sy, sth;
        if (step == 0) {
            slam_update_map(&slam, slam.x, slam.y, slam.theta, &scan);
            sx = slam.x; sy = slam.y; sth = slam.theta;
        } else {
            slam_update(&slam, &scan, odom_dx, odom_dy, odom_dth, &sx, &sy, &sth);
        }

        /* --- 실제 도착 여부를 매 스텝 거리로 판정 (핵심 수정) --- */
        double dist_to_target = fe.has_target ? hypot(sx - fe.target_x, sy - fe.target_y) : 1e300;
        bool reached_current = fe.has_target && dist_to_target < FRONTIER_REACH_TOLERANCE;

        bool target_changed = frontier_explorer_step(&fe, &slam, sx, sy, reached_current);
        if (target_changed) {
            fprintf(stderr,
                    "[FRONTIER] step=%d NEW target=(%.3f,%.3f) prev_reached=%d prev_dist=%.3f "
                    "avoid_history_count=%d miss_count=%d\n",
                    step, fe.target_x, fe.target_y, reached_current, dist_to_target,
                    fe.avoid_history_count, fe.miss_count);
        }

        bool exploration_done = frontier_explorer_done(&fe);
        if (exploration_done) {
            printf("[main][SLAM] mapping phase: no more frontiers (step=%d), map complete\n", step);
            break;
        }

        if (fe.has_target) {
            WheelCmd cmd = safe_navigator_compute(&local_ctrl, sx, sy, sth, fe.target_x, fe.target_y, &scan);
            fpga_link_set_speed(fpga, cmd.left, cmd.right);
        } else {
            fpga_link_set_speed(fpga, 0, 0);  /* 다음 스텝에 새 목표가 나올 때까지 잠시 대기 */
        }

        printf("STATE mapping %d %.4f %.4f %.4f %.4f %.4f\n", step, sx, sy, sth, fe.target_x, fe.target_y);
        printf("SCAN %d", scan.count);
        for (int i = 0; i < scan.count; i++) {
            double wa = sth + scan.readings[i].angle_deg * ANGLE_PI / 180.0;
            double d = scan.readings[i].dist_mm / 1000.0;
            printf(" %.3f %.3f", sx + d * cos(wa), sy + d * sin(wa));
        }
        printf("\n");
        if (step % 20 == 0) {
            printf("MAP %d %d %.4f ", slam.rows, slam.cols, slam.resolution);
            for (int r = 0; r < slam.rows; r++)
                for (int c = 0; c < slam.cols; c++)
                    putchar((1.0 / (1.0 + exp(-slam.log_odds[r * slam.cols + c])) >= cfg->slam_occ_threshold) ? '1' : '0');
            printf("\n");
        }
        fflush(stdout);

        if (step % (cfg->control_hz * 2) == 0) {
            fprintf(stderr,
                    "[main][SLAM] step=%d pose=(%.3f,%.3f,%.2f) target=(%.2f,%.2f) dist=%.3f "
                    "reached=%d has_target=%d steps_on_target=%d miss=%d\n",
                    step, sx, sy, sth, fe.target_x, fe.target_y, dist_to_target,
                    reached_current, fe.has_target, fe.steps_on_current_target, fe.miss_count);
        }

        struct timespec ts;
        ts.tv_sec = (time_t)control_period;
        ts.tv_nsec = (long)((control_period - ts.tv_sec) * 1e9);
        nanosleep(&ts, NULL);
    }
    if (step >= cfg->slam_mapping_max_steps) {
        printf("[main][SLAM] mapping phase: timed out (using map built so far)\n");
    }

    fpga_link_set_speed(fpga, 0, 0);
    *out_grid = slam_to_binary_grid(&slam, cfg->slam_occ_threshold);
    *out_rows = slam.rows; *out_cols = slam.cols;
    slam_get_pose(&slam, out_x, out_y, out_theta);
    slam_free(&slam);
    return true;
}


/* ============================================================
 * acquire_map: SLAM 매핑 / 고정맵 구성 / 파일에서 불러오기 - 세 가지 지도 획득 방식을
 * 한 곳에 모음. run_robot()과 run_shuttle()이 둘 다 이걸 씀(로직 중복 방지).
 * map_save_path가 NULL이 아니면(SLAM/FIXED로 새로 만든 경우) 그 경로에 저장까지 함
 * (LOAD 모드는 이미 파일에서 왔으니 다시 저장 안 함).
 * 반환: 성공하면 true(+out_* 채움), 실패/중단 신호 받으면 false.
 * ============================================================ */
void robot_runner__print_map_line(const unsigned char *grid, int rows, int cols,
                                                  double resolution) {
    printf("MAP %d %d %.4f ", rows, cols, resolution);
    for (int i = 0; i < rows * cols; i++) putchar(grid[i] ? '1' : '0');
    printf("\n");
    fflush(stdout);
}


bool acquire_map(FPGALink *fpga, LidarThread *lidar, RobotConfig *cfg,
                                MapSource map_source, const char *map_load_path,
                                const char *map_save_path,
                                unsigned char **out_grid, int *out_rows, int *out_cols,
                                double *out_final_x, double *out_final_y, double *out_final_theta) {
    if (map_source == MAP_SOURCE_LOAD) {
        double loaded_resolution;
        unsigned char *grid = map_load(map_load_path, out_rows, out_cols, &loaded_resolution);
        if (!grid) return false;
        /* PATCH (2026-08-05 v9): 예전에는 경고만 찍고 cfg->resolution을 계속 썼는데,
         * 그러면 1cm 해상도 지도를 넣었을 때 5cm로 착각해서 좌표계가 통째로 5배
         * 틀어짐. 파일에 적힌 해상도가 진실이므로 그걸로 덮어씀. */
        if (loaded_resolution != cfg->resolution) {
            printf("[acquire_map] 맵 파일의 resolution(%.4f)을 사용합니다 (설정값 %.4f는 무시)\n",
                   loaded_resolution, cfg->resolution);
            cfg->resolution = loaded_resolution;
        }
        if (0) {
            fprintf(stderr, "[acquire_map] 경고: 불러온 맵의 resolution(%.4f)이 cfg->resolution(%.4f)과 "
                            "다릅니다 - A*/스캔매칭 계산이 맞지 않을 수 있습니다\n",
                    loaded_resolution, cfg->resolution);
        }
        *out_grid = grid;
        /* 파일에서 불러온 경우, 로봇이 실제로 지금 어디 있는지는 --start 값을 그대로
         * 믿는 수밖에 없음(매핑을 안 했으니 SLAM이 알아낸 pose가 없음) - 반드시
         * 로봇의 실제 물리적 현재 위치와 일치시켜서 넣어야 함. */
        *out_final_x = cfg->start_x; *out_final_y = cfg->start_y; *out_final_theta = 0.0;
        robot_runner__print_map_line(grid, *out_rows, *out_cols, loaded_resolution);
        return true;
    }

    if (map_source == MAP_SOURCE_SLAM) {
        double fx, fy, fth;
        run_slam_mapping_phase(fpga, lidar, cfg, out_grid, out_rows, out_cols, &fx, &fy, &fth);
        if (g_stop_requested) return false;
        *out_final_x = fx; *out_final_y = fy; *out_final_theta = fth;
        if (map_save_path) map_save(map_save_path, *out_grid, *out_rows, *out_cols, cfg->resolution);
        robot_runner__print_map_line(*out_grid, *out_rows, *out_cols, cfg->resolution);
        return true;
    }

    /* MAP_SOURCE_FIXED */
    ObstacleList olist;
    obstacle_list_init(&olist);
    for (int i = 0; i < cfg->n_known_obstacles; i++)
        obstacle_list_add_if_new(&olist, cfg->known_obstacles[i]);
    int rows, cols;
    unsigned char *grid = make_grid(cfg->grid_size_x, cfg->grid_size_y, cfg->resolution, 0.0, 0.0,
                                     &olist, &rows, &cols);
    obstacle_list_free(&olist);
    *out_grid = grid; *out_rows = rows; *out_cols = cols;
    *out_final_x = cfg->start_x; *out_final_y = cfg->start_y; *out_final_theta = 0.0;
    if (map_save_path) map_save(map_save_path, grid, rows, cols, cfg->resolution);
    robot_runner__print_map_line(grid, rows, cols, cfg->resolution);
    return true;
}
