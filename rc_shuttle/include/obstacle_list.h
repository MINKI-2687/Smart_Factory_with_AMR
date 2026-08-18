#ifndef OBSTACLE_LIST_H
#define OBSTACLE_LIST_H

#include <math.h>
#include <stdlib.h>
#include <stdbool.h>

typedef struct { double x, y, radius; } Obstacle;

typedef struct {
    Obstacle *items;
    int count;
    int capacity;
} ObstacleList;
void obstacle_list_init(ObstacleList *ol);
void obstacle_list_free(ObstacleList *ol);
bool obstacle_equal(Obstacle a, Obstacle b);
bool obstacle_list_contains(const ObstacleList *ol, Obstacle o);
bool obstacle_list_add_if_new(ObstacleList *ol, Obstacle o);
unsigned char *make_grid(double size_x, double size_y, double resolution,
                          double origin_x, double origin_y,
                          const ObstacleList *obstacles, int *out_rows, int *out_cols);

#endif /* OBSTACLE_LIST_H */
