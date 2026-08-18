#ifndef GOAL_PID_H
#define GOAL_PID_H

#include "pid.h"

typedef struct {
    PID linear_pid;
    PID angular_pid;
    double goal_tolerance;
    double max_linear_speed;
    double max_angular_speed;
} GoalPIDController;
void goal_pid_init(GoalPIDController *g,
                    double kp_linear, double ki_linear, double kd_linear,
                    double kp_angular, double ki_angular, double kd_angular,
                    double goal_tolerance,
                    double max_linear_speed, double max_angular_speed,
                    double integral_limit_linear,
                    double integral_limit_angular);
void goal_pid_init_default(GoalPIDController *g, double goal_tolerance);
void goal_pid_reset(GoalPIDController *g);
void goal_pid_compute(GoalPIDController *g, double error_linear, double error_angular,
                       double *linear_speed, double *angular_speed);

#endif /* GOAL_PID_H */
