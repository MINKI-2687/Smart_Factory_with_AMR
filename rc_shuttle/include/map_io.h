#ifndef MAP_IO_H
#define MAP_IO_H

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
bool map_save(const char *path, const unsigned char *grid,
               int rows, int cols, double resolution);
unsigned char *map_load(const char *path, int *out_rows, int *out_cols,
                         double *out_resolution);

#endif /* MAP_IO_H */
