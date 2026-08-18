#include "obstacle_avoidance.h"


void obstacle_avoidance_init(ObstacleAvoidance *o,
                                            double danger_distance, double repulsive_gain,
                                            double tangent_gain) {
    o->danger_distance = danger_distance;
    o->repulsive_gain = repulsive_gain;
    o->tangent_gain = tangent_gain;
}


void obstacle_avoidance_init_default(ObstacleAvoidance *o) {
    obstacle_avoidance_init(o, 0.1, 15, 8);
}


void compute_repulsion(const ObstacleAvoidance *o, const LidarScan *scan,
                                      double robot_theta, double *repel_x, double *repel_y) {
    *repel_x = 0.0; *repel_y = 0.0;
    for (int i = 0; i < scan->count; ++i) {
        double dist_m = scan->readings[i].dist_mm / 1000.0;
        if (dist_m <= 0 || dist_m > o->danger_distance) continue;

        double world_angle = robot_theta + scan->readings[i].angle_deg * ANGLE_PI / 180.0;
        double strength = o->repulsive_gain * (1.0 / dist_m - 1.0 / o->danger_distance);
        *repel_x += -strength * cos(world_angle);
        *repel_y += -strength * sin(world_angle);

        double tangent_angle = world_angle + ANGLE_PI / 2.0;
        double tangent_strength = o->tangent_gain * (1.0 / dist_m - 1.0 / o->danger_distance);
        *repel_x += -tangent_strength * cos(tangent_angle);
        *repel_y += -tangent_strength * sin(tangent_angle);
    }

    /* 목표유도력(attract_x/y)은 항상 크기가 정확히 1로 정규화되는데, 여기 회피력은
     * 거리가 아주 가까워지면 1/dist 항 때문에 수십~수백까지 커질 수 있었음. 그러면
     * 방향 결정(atan2)이 회피력에 완전히 압도돼서, 장애물 근처에만 가면 각도오차가
     * 커져 속도가 확 줄어들며 진동하는 문제가 있었음. 목표유도력과 비슷한 수준(최대 3배)
     * 으로 캡핑해서, "방향에 영향은 주되 완전히 압도하진 않게" 함. */
    double mag = hypot(*repel_x, *repel_y);
    const double REPEL_MAX_MAG = 3.0;
    if (mag > REPEL_MAX_MAG) {
        *repel_x = *repel_x / mag * REPEL_MAX_MAG;
        *repel_y = *repel_y / mag * REPEL_MAX_MAG;
    }
}
