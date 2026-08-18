#ifndef MAPPING_SESSION_H
#define MAPPING_SESSION_H

#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include "slam.h"
#include "frontier_exploration.h"
#include "goal_pid.h"
#include "obstacle_avoidance.h"
#include "safe_navigator.h"
#include "dynamic_navigator.h"
#include "lidar_clustering.h"
#include "detected_obstacle.h"
#include "graph_slam.h"

/* ============================================================
 * MappingSession: "매핑(SLAM+프론티어 탐색, 그래프SLAM 루프클로저 보정) -> 지도확정
 * -> A*계획 -> 내비게이션(회피+도킹)" 전체 파이프라인을 하나로 캡슐화한 상태머신.
 *
 * 왜 이 모듈이 필요한가:
 *   robot_runner.h(run_slam_mapping_phase + run_robot)가 실기용으로 이미 이 파이프라인을
 *   구현하고 있는데, 파이썬 시뮬레이션(live_scenario_gui.py)이 이걸 안 쓰고 파이썬으로
 *   따로 재구현하면서 서로 로직이 갈라지는 문제가 실제로 있었음(예: 프론티어 회피이력을
 *   파이썬 쪽에서는 1개만 기억, C의 FrontierExplorer는 8개 기억 - 같은 버그를 두 번 고쳐야
 *   하고 실제로 한쪽만 고쳐진 채 방치되는 사고가 남). 이 모듈은 robot_runner.h와 동일한
 *   C 함수(frontier_explorer_step, safe_navigator_compute, dynnav_update 등)를 그대로
 *   재사용해서, "결정 로직"이 오직 여기 한 곳에만 존재하게 함. 파이썬 쪽은 이 세션에
 *   스캔+오도메트리를 넣고 나온 바퀴명령을 시뮬레이터 물리에 적용하기만 하면 됨(로직 없음).
 *
 * 그래프SLAM 통합(2026-08-04): 실측 노이즈(라이다 노이즈+오도메트리 양자화+PWM데드존)를
 * 반영해서 매핑을 돌려보니, 매핑 단계(slam_update - 위치추정과 지도작성을 동시에 함)의
 * 오차가 초반 스텝부터 자기강화되며 누적되는 것이 실측으로 확인됨 (320스텝만에 드리프트
 * 0.42m, 방의 20%도 채 못 탐사하고 "탐사 끝"으로 조기종료 - 국소 스캔매칭만으로는 이런
 * 자기모순적 드리프트를 못 잡음). graph_slam.h(랜드마크 재관측 기반 전체궤적 재최적화,
 * 즉 루프클로저)를 매핑 단계에 실제로 연결해서 이 문제를 완화함 - 매핑이 끝나는 시점에
 * 한 번 그래프 최적화를 돌려서 지도와 최종 pose를 다시 계산함.
 *
 * 랜드마크 필터링 주의사항(중요): cluster_lidar_scan()은 라이다 스캔을 "각도순으로
 * 인접한 점끼리" 묶는데(모든 쌍 비교가 아님), 벽처럼 길게 이어진 표면은 인접점 사이에
 * 큰 틈이 안 생겨서 벽 전체가 거대한 클러스터 하나로 뭉칠 수 있음 - frontier_cluster_cells
 * 에서 발견했던 "거대 클러스터 병합" 버그와 같은 근본 패턴이 여기서도 나타날 수 있음.
 * GraphSLAM의 랜드마크는 "같은 자리에서 계속 재관측되는 고정점"이어야 의미가 있는데,
 * 벽 뭉치의 중심점은 로봇이 보는 각도가 바뀔 때마다 위치가 흔들리는 가짜 점이라 랜드마크
 * 매칭 전제 자체가 깨짐 - 이런 클러스터를 그대로 랜드마크로 넣으면 그래프 최적화에
 * 쓰레기 제약을 넣는 셈이라 결과가 오히려 나빠짐. 그래서 아래
 * mapping_session__extract_landmark_observations()가 "공간적으로 작고 뚜렷한"
 * 클러스터(원형 장애물 등)만 골라서 랜드마크로 씀 - 벽처럼 퍼진 클러스터는 제외함. */

#define GSLAM_LANDMARK_MAX_EXTENT_M 0.6  /* 이보다 넓게 퍼진 클러스터는 벽으로 간주하고
                                            랜드마크 후보에서 제외 (원형 장애물 반경
                                            0.15~0.3m 정도를 감안한 여유값) */
#define GSLAM_STEP_INTERVAL 10  /* 매 스텝(20Hz)마다 그래프 노드를 추가하면 노드 수가
                                   너무 빨리 늘어나서(GSLAM_MAX_POSES=3000 캡을 금방
                                   채움) 최종 solve()가 O(포즈수^3)라 지나치게 느려짐
                                   (수천 포즈 기준 수십 초~수 분급) - 10스텝(0.5초)마다만
                                   그래프 노드를 추가해서 같은 매핑 시간 동안 노드 수를
                                   1/10로 줄임(=solve 비용 최대 1000배 절감) */
#define GSLAM_MOTION_NOISE (0.02 * 0.02)
#define GSLAM_MEASUREMENT_NOISE (0.05 * 0.05)
#define GSLAM_LANDMARK_MATCH_THRESHOLD 0.25

/* 클러스터 중 "공간적으로 작고 뚜렷한"(벽이 아닌) 것만 걸러서 로봇기준 상대
 * (각도,거리) 랜드마크 관측치로 변환. 반환: 채운 개수. */
static inline int mapping_session__extract_landmark_observations(
        const ClusterResult *cr, double robot_x, double robot_y, double robot_theta,
        double *out_angles_rad, double *out_dists_m, int max_out) {
    int n = 0;
    for (int i = 0; i < cr->n_clusters && n < max_out; i++) {
        const LidarCluster *c = &cr->clusters[i];
        if (c->count < 1) continue;

        double x0 = c->xs[0], y0 = c->ys[0];
        double x1 = c->xs[c->count - 1], y1 = c->ys[c->count - 1];
        double extent = hypot(x1 - x0, y1 - y0);
        if (extent > GSLAM_LANDMARK_MAX_EXTENT_M) continue;  /* 벽으로 추정, 제외 */

        double cx = 0, cy = 0;
        for (int k = 0; k < c->count; k++) { cx += c->xs[k]; cy += c->ys[k]; }
        cx /= c->count; cy /= c->count;

        double rel_x = cx - robot_x, rel_y = cy - robot_y;
        double dist = hypot(rel_x, rel_y);
        double world_angle = atan2(rel_y, rel_x);
        double rel_angle = atan2(sin(world_angle - robot_theta), cos(world_angle - robot_theta));

        out_angles_rad[n] = rel_angle;
        out_dists_m[n] = dist;
        n++;
    }
    return n;
}

typedef enum {
    SESSION_PHASE_MAPPING = 0,
    SESSION_PHASE_NAVIGATE = 1,
    SESSION_PHASE_DONE = 2,
    SESSION_PHASE_PLAN_FAILED = 3,  /* A* 계획 실패 - 더 진행 불가. 호출부가 반드시 확인해야 함
                                      * (예전 파이썬 GUI가 set_goal() 반환값을 안 봐서
                                      * "조용히 즉시 DONE"이 나던 문제를 여기서 막음) */
    SESSION_PHASE_MAP_READY = 4     /* 매핑은 끝났지만 아직 목표를 안 정함 - GUI에서 완성된
                                      * 지도를 보고 사용자가 클릭으로 목표를 고를 수 있게
                                      * 잠시 멈춰있는 상태 (mapping_session_init_mapping_only로
                                      * 시작했을 때만 거치는 단계) */
} SessionPhase;

typedef struct {
    /* ---- 설정값 ---- */
    double resolution;
    double robot_radius;
    double occ_threshold;
    double goal_x, goal_y;
    bool has_goal_theta;
    double goal_theta;          /* 라디안 */
    int max_mapping_steps;
    double frontier_reach_tolerance;
    bool localize_only;         /* true면 NAVIGATE 단계에서 지도 갱신 없이 위치추정만
                                  * 라이다로 보정(고정맵 모드). false면 기존처럼 계속
                                  * slam_update로 지도도 같이 갱신(SLAM 탐색 후 내비게이션). */
    bool auto_navigate_after_mapping;  /* true(기존 기본동작): 매핑 끝나자마자 자동으로
                                         * goal_x/y로 A*계획하고 NAVIGATE 진입.
                                         * false: 매핑만 끝내고 SESSION_PHASE_MAP_READY에서
                                         * 멈춤 - GUI에서 사용자가 지도를 보고 목표를 고른 뒤
                                         * mapping_session_start_navigation()을 불러야 진행됨. */

    /* ---- 매핑 단계 상태 ---- */
    OccupancyGridSLAM slam;
    FrontierExplorer frontier;
    GoalPIDController frontier_goal_ctrl;
    ObstacleAvoidance frontier_avoider;
    SafeNavigator frontier_nav;
    int mapping_step_count;
    GraphSlam gslam;
    bool gslam_initialized;      /* mapping_session_free()가 gslam_free()를 안전하게
                                   * 호출할지 판단하는 플래그 (고정맵 초기화 경로는
                                   * gslam을 아예 안 씀) */
    bool used_graph_slam_correction;  /* 이번 매핑에서 실제로 그래프SLAM 보정이
                                        * 적용됐는지(디버그/GUI 표시용) */

    /* ---- 내비게이션 단계 상태 ---- */
    DynamicNavigator nav;
    bool nav_created;

    /* ---- 공용 상태 ---- */
    SessionPhase phase;
    double est_x, est_y, est_theta;  /* 최신 추정 pose */
} MappingSession;

static inline void mapping_session_init(MappingSession *ms,
                                         double size_x, double size_y, double resolution,
                                         double start_x, double start_y, double start_theta,
                                         double goal_x, double goal_y,
                                         bool has_goal_theta, double goal_theta_rad,
                                         double robot_radius, double occ_threshold,
                                         int frontier_max_miss, int max_mapping_steps) {
    ms->resolution = resolution;
    ms->robot_radius = robot_radius;
    ms->occ_threshold = occ_threshold;
    ms->goal_x = goal_x; ms->goal_y = goal_y;
    ms->has_goal_theta = has_goal_theta;
    ms->goal_theta = goal_theta_rad;
    ms->max_mapping_steps = max_mapping_steps;
    ms->frontier_reach_tolerance = 0.4;

    slam_init(&ms->slam, size_x, size_y, resolution, 0.0, 0.0,
              start_x, start_y, start_theta, 0.15, 12.0, 0.03, 3.0);

    frontier_explorer_init(&ms->frontier, frontier_max_miss);

    goal_pid_init_default(&ms->frontier_goal_ctrl, ms->frontier_reach_tolerance);
    obstacle_avoidance_init_default(&ms->frontier_avoider);
    safe_navigator_init(&ms->frontier_nav, &ms->frontier_goal_ctrl, &ms->frontier_avoider);

    ms->mapping_step_count = 0;
    gslam_init(&ms->gslam, start_x, start_y,
               GSLAM_MOTION_NOISE, GSLAM_MEASUREMENT_NOISE, GSLAM_LANDMARK_MATCH_THRESHOLD);
    ms->gslam_initialized = true;
    ms->used_graph_slam_correction = false;
    ms->nav_created = false;
    ms->phase = SESSION_PHASE_MAPPING;
    ms->localize_only = false;
    ms->auto_navigate_after_mapping = true;
    ms->est_x = start_x; ms->est_y = start_y; ms->est_theta = start_theta;
}

/* ---- 매핑전용 초기화 ----
 * 목표를 아직 모를 때(GUI에서 지도를 보여준 뒤 사용자가 클릭으로 고르게 하고 싶을 때)
 * 씀. 매핑이 끝나면 자동으로 A*계획하지 않고 SESSION_PHASE_MAP_READY에서 멈춤 -
 * 이후 mapping_session_start_navigation()을 호출해서 실제 목표를 넣어줘야 진행됨. */
static inline void mapping_session_init_mapping_only(MappingSession *ms,
                                                       double size_x, double size_y, double resolution,
                                                       double start_x, double start_y, double start_theta,
                                                       double robot_radius, double occ_threshold,
                                                       int frontier_max_miss, int max_mapping_steps) {
    mapping_session_init(ms, size_x, size_y, resolution, start_x, start_y, start_theta,
                          0.0, 0.0, false, 0.0, robot_radius, occ_threshold,
                          frontier_max_miss, max_mapping_steps);
    ms->auto_navigate_after_mapping = false;
}

/* ---- 고정맵 모드 초기화 ----
 * 매핑(프론티어 탐색) 단계를 아예 건너뛰고 바로 NAVIGATE 단계로 시작함. known_grid는
 * 이미 확정된 지도(0/1, 반드시 벽 포함 - obstacle_list.h의 make_grid가 자동으로 넣어줌)를
 * A* 계획과 라이다 위치보정 양쪽에 그대로 씀. 오도메트리만으로는 방 하나 가로지르는
 * 정도의 이동에서도 ~40cm급 드리프트가 쌓이는 게 실측으로 확인됐기 때문에, 고정맵
 * 모드에서도 반드시 라이다 스캔매칭으로 위치를 계속 보정함(지도 자체는 갱신 안 함). */
static inline bool mapping_session_init_fixed_map(MappingSession *ms,
                                                    const unsigned char *known_grid,
                                                    int rows, int cols, double resolution,
                                                    double start_x, double start_y, double start_theta,
                                                    double goal_x, double goal_y,
                                                    bool has_goal_theta, double goal_theta_rad,
                                                    double robot_radius) {
    ms->resolution = resolution;
    ms->robot_radius = robot_radius;
    ms->occ_threshold = 0.6;  /* 고정맵 모드에선 안 쓰이지만 일관성을 위해 채워둠 */
    ms->goal_x = goal_x; ms->goal_y = goal_y;
    ms->has_goal_theta = has_goal_theta;
    ms->goal_theta = goal_theta_rad;
    ms->max_mapping_steps = 0;
    ms->frontier_reach_tolerance = 0.4;
    ms->localize_only = true;
    ms->auto_navigate_after_mapping = true;  /* 이 초기화 경로는 애초에 MAPPING단계를
                                                * 거치지 않아서 실질적으로 안 쓰이지만,
                                                * 필드를 항상 초기화해두기 위함 */
    ms->mapping_step_count = 0;
    ms->gslam_initialized = false;   /* 고정맵 경로는 매핑을 아예 안 해서 그래프SLAM 불필요 */
    ms->used_graph_slam_correction = false;
    ms->est_x = start_x; ms->est_y = start_y; ms->est_theta = start_theta;

    slam_init_from_grid(&ms->slam, known_grid, rows, cols, resolution, 0.0, 0.0,
                         start_x, start_y, start_theta, 0.15, 12.0, 0.03, 3.0);

    dynnav_init(&ms->nav, known_grid, rows, cols, resolution, robot_radius, 0.0, 0.0);
    ms->nav_created = true;

    bool ok = dynnav_set_goal(&ms->nav, start_x, start_y, goal_x, goal_y,
                               has_goal_theta, goal_theta_rad, NULL, NULL, 0);
    ms->phase = ok ? SESSION_PHASE_NAVIGATE : SESSION_PHASE_PLAN_FAILED;
    return ok;
}

static inline void mapping_session_free(MappingSession *ms) {
    slam_free(&ms->slam);
    if (ms->nav_created) dynnav_free(&ms->nav);
    if (ms->gslam_initialized) gslam_free(&ms->gslam);
}

/* 지도 확정(log_odds -> 이진격자) + DynamicNavigator 생성 + A*계획 + 내비게이션용
 * 로컬라이제이션 SLAM 재초기화. mapping_session_step()의 자동전환과
 * mapping_session_start_navigation()의 수동전환이 둘 다 이 로직을 공유함. */
static inline void mapping_session__finalize_map_and_plan(MappingSession *ms,
                                                            double sx, double sy, double sth,
                                                            double goal_x, double goal_y,
                                                            bool has_goal_theta, double goal_theta_rad) {
    ms->goal_x = goal_x; ms->goal_y = goal_y;
    ms->has_goal_theta = has_goal_theta;
    ms->goal_theta = goal_theta_rad;

    unsigned char *grid = slam_to_binary_grid(&ms->slam, ms->occ_threshold);
    dynnav_init(&ms->nav, grid, ms->slam.rows, ms->slam.cols, ms->resolution,
                ms->robot_radius, 0.0, 0.0);
    free(grid);
    ms->nav_created = true;

    bool ok = dynnav_set_goal(&ms->nav, sx, sy, goal_x, goal_y,
                               has_goal_theta, goal_theta_rad, NULL, NULL, 0);
    if (!ok) {
        ms->phase = SESSION_PHASE_PLAN_FAILED;
        return;
    }

    /* 내비게이션 전용 로컬라이제이션 SLAM: 최종 pose에서 새로 시작
     * (robot_runner.h의 run_robot()과 동일한 방식) */
    double grid_size_x = ms->nav.cols * ms->resolution;
    double grid_size_y = ms->nav.rows * ms->resolution;
    slam_free(&ms->slam);
    slam_init(&ms->slam, grid_size_x, grid_size_y, ms->resolution, 0.0, 0.0,
              sx, sy, sth, 0.15, 12.0, 0.03, 3.0);

    ms->phase = SESSION_PHASE_NAVIGATE;
}

/* 매핑전용 세션(mapping_session_init_mapping_only)이 SESSION_PHASE_MAP_READY에서
 * 멈춰있을 때, GUI에서 사용자가 고른 목표로 실제 내비게이션을 시작시킴.
 * 반환: 계획 성공하면 true(+NAVIGATE로 전환), 실패하면 false(+PLAN_FAILED). */
static inline bool mapping_session_start_navigation(MappingSession *ms,
                                                      double goal_x, double goal_y,
                                                      bool has_goal_theta, double goal_theta_rad) {
    if (ms->phase != SESSION_PHASE_MAP_READY) return false;
    double sx, sy, sth;
    slam_get_pose(&ms->slam, &sx, &sy, &sth);
    mapping_session__finalize_map_and_plan(ms, sx, sy, sth, goal_x, goal_y, has_goal_theta, goal_theta_rad);
    return ms->phase == SESSION_PHASE_NAVIGATE;
}

/* 그래프SLAM(루프클로저) solve+rebuild를 시도하는 "체크포인트" - 프론티어가 새 목표를
 * 고를 때마다(적당한 빈도의 자연스러운 체크포인트) 매핑 도중에도, 그리고 매핑이 완전히
 * 끝나는 시점에도 이 함수를 재사용함. 지도(log_odds)뿐 아니라 "현재 pose로 믿는 값"도
 * 같이 갱신해야 함 - 위치만 바로잡고 이미 그려둔 지도는 옛날(드리프트된) 좌표계 그대로
 * 두면, 그 다음 스캔매칭이 "보정된 위치"와 "안 보정된 지도"를 비교하게 돼서 오히려 더
 * 혼란스러워짐. 실패하면(아직 재관측된 랜드마크가 부족해서 특이행렬 등) 아무것도 안 하고
 * 조용히 리턴 - 그 시점까지의 raw 추정치를 계속 씀.
 * (참고: FrontierExplorer의 avoid_history는 절대좌표를 그대로 저장하고 있어서, 이 보정이
 * 좌표를 크게 뒤바꾸면 회피이력이 살짝 안 맞을 수 있음 - 자주 체크포인트를 찍어서
 * 한 번에 바뀌는 폭 자체를 작게 유지하는 것으로 완화함) */
static inline void mapping_session__try_graph_slam_correction(MappingSession *ms,
                                                                double *sx, double *sy) {
    double *corrected_pose_x = (double *)malloc(sizeof(double) * GSLAM_MAX_POSES);
    double *corrected_pose_y = (double *)malloc(sizeof(double) * GSLAM_MAX_POSES);
    double *corrected_lm_x = (double *)malloc(sizeof(double) * GSLAM_MAX_LANDMARKS);
    double *corrected_lm_y = (double *)malloc(sizeof(double) * GSLAM_MAX_LANDMARKS);
    if (!corrected_pose_x || !corrected_pose_y || !corrected_lm_x || !corrected_lm_y) {
        free(corrected_pose_x); free(corrected_pose_y); free(corrected_lm_x); free(corrected_lm_y);
        return;
    }

    bool solve_ok = gslam_solve(&ms->gslam, corrected_pose_x, corrected_pose_y,
                                 corrected_lm_x, corrected_lm_y);
    if (solve_ok) {
        bool rebuild_ok = gslam_rebuild_grid(&ms->gslam, &ms->slam);
        if (rebuild_ok) {
            ms->used_graph_slam_correction = true;
            int last = ms->gslam.n_poses - 1;
            /* 각도(theta)는 graph_slam.h가 따로 풀지 않으므로(위치 x,y만 최적화)
             * 로컬 SLAM의 기존 추정치를 그대로 둠. */
            ms->slam.x = *sx = corrected_pose_x[last];
            ms->slam.y = *sy = corrected_pose_y[last];
        }
    }
    free(corrected_pose_x); free(corrected_pose_y); free(corrected_lm_x); free(corrected_lm_y);
}

/* 한 스텝 실행.
 * scan: 이번 스텝 라이다 스캔
 * odom_dx/dy/dtheta: 직전 스텝 이후의 오도메트리 변화량(라디안)
 * 출력: out_left/right = 이번 스텝 바퀴명령, out_done = 도킹까지 완료(또는 계획실패로 중단)
 */
static inline void mapping_session_step(MappingSession *ms, const LidarScan *scan,
                                         double odom_dx, double odom_dy, double odom_dtheta,
                                         int *out_left, int *out_right, bool *out_done) {
    *out_left = 0; *out_right = 0; *out_done = false;

    if (ms->phase == SESSION_PHASE_MAPPING) {
        double sx, sy, sth;
        if (ms->mapping_step_count == 0) {
            slam_update_map(&ms->slam, ms->slam.x, ms->slam.y, ms->slam.theta, scan);
            sx = ms->slam.x; sy = ms->slam.y; sth = ms->slam.theta;
        } else {
            slam_update(&ms->slam, scan, odom_dx, odom_dy, odom_dtheta, &sx, &sy, &sth);
        }
        ms->est_x = sx; ms->est_y = sy; ms->est_theta = sth;

        /* 그래프SLAM 노드 추가 (throttle: GSLAM_STEP_INTERVAL 스텝마다 한 번만 -
         * 이유는 파일 상단 GSLAM_STEP_INTERVAL 주석 참고). step 0은 아직 이동이 없어서
         * (odom_dx/dy가 정의상 이번 스텝의 "직전 스텝 이후 변화량"인데 첫 스텝엔 의미
         * 없음) 건너뜀. */
        if (ms->mapping_step_count > 0 && ms->mapping_step_count % GSLAM_STEP_INTERVAL == 0) {
            ClusterResult gcr;
            cluster_lidar_scan(scan, sx, sy, sth, 0.15, 2, 3.0, &gcr);
            double lm_angles[CLUSTER_MAX_CLUSTERS], lm_dists[CLUSTER_MAX_CLUSTERS];
            int n_lm = mapping_session__extract_landmark_observations(
                &gcr, sx, sy, sth, lm_angles, lm_dists, CLUSTER_MAX_CLUSTERS);
            gslam_step(&ms->gslam, odom_dx, odom_dy, lm_angles, lm_dists, n_lm, sth, scan);
        }

        double dist_to_target = ms->frontier.has_target
            ? hypot(sx - ms->frontier.target_x, sy - ms->frontier.target_y) : 1e300;
        bool reached = ms->frontier.has_target && dist_to_target < ms->frontier_reach_tolerance;
        bool target_changed = frontier_explorer_step(&ms->frontier, &ms->slam, sx, sy, reached);

        /* PATCH (2026-08-04): 매핑이 완전히 끝날 때 딱 한 번만 보정하면, 짧게 조기종료된
         * 탐사는 애초에 "재관측할 루프"를 만들 기회조차 없어서 보정이 무력함(실측 확인됨:
         * used_graph_slam_correction=true인데도 드리프트가 그대로였음). 대신 프론티어가
         * 새 목표를 고를 때마다(=탐사가 진행되는 자연스러운 체크포인트) 그때그때 보정을
         * 시도해서, 드리프트가 크게 불어나기 전에 계속 바로잡아줌 - 이러면 프론티어
         * 자신의 "어디를 이미 가봤는지" 판단(회피이력 비교)에 쓰이는 좌표 자체가 계속
         * 정확하게 유지돼서, 드리프트로 인한 조기종료를 애초에 줄여줄 수 있음. */
        if (target_changed) {
            mapping_session__try_graph_slam_correction(ms, &sx, &sy);
            ms->est_x = sx; ms->est_y = sy;
        }

        bool mapping_timed_out = (ms->mapping_step_count + 1) >= ms->max_mapping_steps;
        if (frontier_explorer_done(&ms->frontier) || mapping_timed_out) {
            /* 매핑이 끝나는 이 시점에 마지막으로 한 번 더 보정 시도(도중에 못 걸린
             * 마지막 구간의 관측치까지 반영하기 위함). 실패해도(랜드마크 부족 등)
             * 원래(비보정) 지도를 그대로 씀 - 원본 파이썬 GUI의 폴백 동작과 동일. */
            mapping_session__try_graph_slam_correction(ms, &sx, &sy);

            if (!ms->auto_navigate_after_mapping) {
                /* 목표를 아직 모름 - 지도만 확정된 상태로 멈춤. GUI가 지도를 보여주고
                 * 사용자가 고른 뒤 mapping_session_start_navigation()을 불러야 함.
                 * (지도 자체는 이미 ms->slam.log_odds에 있으니 get_prob_grid()로
                 * 계속 조회 가능 - 여기서 따로 안 지워도 됨) */
                ms->phase = SESSION_PHASE_MAP_READY;
                *out_done = true;
                return;
            }
            mapping_session__finalize_map_and_plan(ms, sx, sy, sth,
                                                    ms->goal_x, ms->goal_y,
                                                    ms->has_goal_theta, ms->goal_theta);
            *out_done = (ms->phase == SESSION_PHASE_PLAN_FAILED);
            return;
        }

        if (ms->frontier.has_target) {
            WheelCmd cmd = safe_navigator_compute(&ms->frontier_nav, sx, sy, sth,
                                                   ms->frontier.target_x, ms->frontier.target_y, scan);
            *out_left = cmd.left; *out_right = cmd.right;
        }
        ms->mapping_step_count++;
        return;
    }

    if (ms->phase == SESSION_PHASE_NAVIGATE) {
        double sx, sy, sth;
        if (ms->localize_only) {
            slam_localize_step(&ms->slam, scan, odom_dx, odom_dy, odom_dtheta, &sx, &sy, &sth);
        } else {
            slam_update(&ms->slam, scan, odom_dx, odom_dy, odom_dtheta, &sx, &sy, &sth);
        }
        ms->est_x = sx; ms->est_y = sy; ms->est_theta = sth;

        ClusterResult cr;
        cluster_lidar_scan(scan, sx, sy, sth, 0.15, 2, 3.0, &cr);
        DetectedObstacle detected[CLUSTER_MAX_CLUSTERS];
        int n_det = cluster_result_to_detected(&cr, detected, CLUSTER_MAX_CLUSTERS);

        WheelCmd cmd = dynnav_update(&ms->nav, sx, sy, sth, scan, n_det > 0 ? detected : NULL, n_det);
        *out_left = cmd.left; *out_right = cmd.right; *out_done = cmd.done;
        if (cmd.done) ms->phase = SESSION_PHASE_DONE;
        return;
    }

    /* SESSION_PHASE_DONE / SESSION_PHASE_PLAN_FAILED: 더 할 일 없음 */
    *out_done = true;
}

/* ============================================================
 * mapping_session_retarget: 셔틀 왕복(A<->B) 용. 이미 DONE(한 목표에 도킹 완료)
 * 상태인 세션을 재사용해서 "지금 위치에서 새 목표로" A*를 다시 계획하고 NAVIGATE로
 * 되돌림. 지도(nav의 known_grid, 위치추정용 SLAM 인스턴스)는 전부 그대로 유지되고
 * 목표만 바뀜 - 매 구간마다 세션을 새로 만들 필요가 없음.
 *
 * robot_runner.h의 run_shuttle()과 동일한 발상이지만, 저건 하드웨어(FPGALink/
 * LidarThread)에 직접 묶여있어서 시뮬레이션에 못 씀 - 이 함수가 그 자리를 대신해서
 * 파이썬 시뮬레이션에서도 A<->B 왕복을 테스트할 수 있게 해줌.
 *
 * 호출 전제조건: ms->phase == SESSION_PHASE_DONE (한 번 도킹까지 끝난 상태).
 * 반환: 계획 성공하면 true(+phase=NAVIGATE로 전환), 실패하면 false(+phase=PLAN_FAILED). */
static inline bool mapping_session_retarget(MappingSession *ms,
                                             double new_goal_x, double new_goal_y,
                                             bool has_goal_theta, double new_goal_theta_rad) {
    ms->goal_x = new_goal_x;
    ms->goal_y = new_goal_y;
    ms->has_goal_theta = has_goal_theta;
    ms->goal_theta = new_goal_theta_rad;

    bool ok = dynnav_set_goal(&ms->nav, ms->est_x, ms->est_y, new_goal_x, new_goal_y,
                               has_goal_theta, new_goal_theta_rad, NULL, NULL, 0);
    ms->phase = ok ? SESSION_PHASE_NAVIGATE : SESSION_PHASE_PLAN_FAILED;
    return ok;
}

#endif /* MAPPING_SESSION_H */
