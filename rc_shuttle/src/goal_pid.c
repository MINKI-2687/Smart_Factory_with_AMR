#include "goal_pid.h"


void goal_pid_init(GoalPIDController *g,
                                  double kp_linear, double ki_linear, double kd_linear,
                                  double kp_angular, double ki_angular, double kd_angular,
                                  double goal_tolerance,
                                  double max_linear_speed, double max_angular_speed,
                                  double integral_limit_linear,
                                  double integral_limit_angular) {
    double il_lin = (integral_limit_linear >= 0.0) ? integral_limit_linear : max_linear_speed;
    double il_ang = (integral_limit_angular >= 0.0) ? integral_limit_angular : max_angular_speed;
    pid_init(&g->linear_pid, kp_linear, ki_linear, kd_linear, il_lin);
    pid_init(&g->angular_pid, kp_angular, ki_angular, kd_angular, il_ang);
    g->goal_tolerance = goal_tolerance;
    g->max_linear_speed = max_linear_speed;
    g->max_angular_speed = max_angular_speed;
}


void goal_pid_init_default(GoalPIDController *g, double goal_tolerance) {
    goal_pid_init(g, 60, 0, 0, 40, 0, 0, goal_tolerance, 50, 30, -1, -1);
}


void goal_pid_reset(GoalPIDController *g) {
    pid_reset(&g->linear_pid);
    pid_reset(&g->angular_pid);
}


void goal_pid_compute(GoalPIDController *g, double error_linear, double error_angular,
                                     double *linear_speed, double *angular_speed) {
    *linear_speed = pid_compute(&g->linear_pid, error_linear);
    *angular_speed = pid_compute(&g->angular_pid, error_angular);
}
