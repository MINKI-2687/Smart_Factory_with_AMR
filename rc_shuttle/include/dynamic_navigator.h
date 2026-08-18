#ifndef DYNAMIC_NAVIGATOR_H
#define DYNAMIC_NAVIGATOR_H

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "astar.h"
#include "planning_pipeline.h"
#include "goal_pid.h"
#include "obstacle_avoidance.h"
#include "path_follow.h"

/* 이 거리보다 멀리 떨어진 채 웨이포인트가 소진되면 '도착'이 아니라 '실패'로 본다. */
#define DYNNAV_ARRIVAL_MAX_ERR_M 0.15
#include "safe_navigator.h"
#include "docking.h"
#include "detected_obstacle.h"

typedef struct {
    double resolution, robot_radius, origin_x, origin_y;
    int rows, cols;
    unsigned char *known_grid;

    OccupancyGridMap map;
    bool map_valid;

    GoalPIDController goal_ctrl;
    ObstacleAvoidance avoider;
    SafeNavigator local_ctrl;

    DockingController docking_ctrl;
    double docking_radius;

    /* pure pursuit 사용 여부와 전방주시 거리. dynnav__lookahead_point() 주석 참고.
     * false면 기존처럼 웨이포인트를 하나씩 정확히 찍고 감(정밀하지만 코너에서 멈칫함). */
    /* 목적지에 실제로는 도달하지 못했는데 웨이포인트가 소진된 경우 true.
     * PATCH (2026-08-06): 원래는 막혀서 웨이포인트를 전부 건너뛴 경우에도 done=true를
     * 돌려줘서, 호출부(run_shuttle)가 "도착했다"고 믿고 그 자리에서 슬롯 진입을
     * 시도했음 - 실기라면 엉뚱한 위치에서 벽으로 돌진하는 위험한 실패였음(시뮬레이션
     * 에서 목적지 x=0.225인데 x=0.93에서 '도착' 보고하는 것으로 확인). 도착과 포기를
     * 구분할 수단이 없어서 생긴 문제라, 명시적인 실패 신호를 추가함. */
    bool goal_unreachable;

    bool use_pure_pursuit;
    double lookahead_m;
    /* 경로에서 벗어났을 때 전방주시 거리를 늘릴 수 있는 상한 (2026-08-09e).
     * path_follow.h 머리말 참고 - 이게 있어야 "옆으로 벗어남"이 "제자리 회전 요구"로
     * 번역되지 않는다. dynnav_init 에서 lookahead_m 의 3배로 잡음. */
    double lookahead_max_m;
    PathTrack track;          /* 직전 dynnav_update 의 경로추종 기하 (로그/판정용) */

    /* 조기 종료 상자 (2026-08-09g).
     * >0 이면 "목표점 기준 이 상자 안에 들어오면 그 자리에서 도착으로 친다".
     *
     * 왜: 슬롯 대기점은 '정확한 점'일 필요가 없다. 진입에 실제로 필요한 건
     * (1) 슬롯 축과 나란한 자세, (2) 차선(lane_x)과 맞는 좌우위치 두 가지이고,
     * 이건 slot_align() / slot_lateral_align() 이 훨씬 직접적으로(제자리 회전 +
     * 직선 이동) 해결한다. 그런데 도킹 컨트롤러는 mm 단위로 x, y, 자세를 모두
     * 맞추려고 회전-직진-후진을 반복해서, 사용자가 본 "슬롯 앞에서 찔끔찔끔"이 됐다.
     * 상자 안에 들어오는 순간 넘겨버리면 그 구간이 통째로 사라진다. */
    double finish_box_x, finish_box_y;

    bool docking_mode;

    Path waypoints;
    int current_wp_idx;
    int steps_on_current_wp;
    int skip_after_steps;

    double replan_detect_radius;
    int max_replans;
    int replan_count;
    double goal_x, goal_y;
    bool has_goal_theta;
    double goal_theta;

    bool replanned_this_call;
    char last_plan_fail_reason[256];  /* A*계획 실패 시 사람이 읽을 수 있는 원인 진단 문자열.
                                        * 계획 성공하면 내용이 이전 실패 때 값 그대로 남아있을
                                        * 수 있음 - 반드시 dynnav_set_goal()의 반환값이 false일
                                        * 때만 읽어야 함(dynnav_get_last_plan_fail_reason 참고). */
} DynamicNavigator;
void dynnav__world_to_grid(const DynamicNavigator *nav, double x, double y,
                            int *row, int *col);
bool dynnav__in_bounds(const DynamicNavigator *nav, int row, int col);
bool dynnav_register_unexpected_obstacle(DynamicNavigator *nav,
                                           const double *xs, const double *ys, int n);
void dynnav_init(DynamicNavigator *nav, const unsigned char *raw_grid,
                  int rows, int cols, double resolution, double robot_radius,
                  double origin_x, double origin_y);
void dynnav_free(DynamicNavigator *nav);
void dynnav__rescue_inflation_near(DynamicNavigator *nav, double x, double y);
void dynnav__diagnose_plan_failure(DynamicNavigator *nav, double start_x, double start_y);
bool dynnav__plan_internal(DynamicNavigator *nav, double start_x, double start_y);
const char *dynnav_get_last_plan_fail_reason(const DynamicNavigator *nav);
bool dynnav_set_goal(DynamicNavigator *nav, double start_x, double start_y,
                      double goal_x, double goal_y,
                      bool has_goal_theta, double goal_theta,
                      const double *known_xs, const double *known_ys, int n_known);
bool dynnav__replan_from(DynamicNavigator *nav, double cur_x, double cur_y);
Point2D dynnav__lookahead_point(DynamicNavigator *nav, double x, double y, double L);
WheelCmd dynnav_update(DynamicNavigator *nav, double x, double y, double theta,
                        const LidarScan *scan,
                        const DetectedObstacle *detected, int n_detected);

#endif /* DYNAMIC_NAVIGATOR_H */
