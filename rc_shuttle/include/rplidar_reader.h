#ifndef RPLIDAR_READER_H
#define RPLIDAR_READER_H
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif

#include <unistd.h>
#include <time.h>
#include <string.h>
#include <stdbool.h>
#include "rplidar_c1.h"
#include "lidar_types.h"

typedef struct {
    int fd;
    double health_check_timeout;
    int max_reset_retries;
    bool scanning;
} RplidarReader;
void rplidar_reader_init(RplidarReader *r, int fd,
                          double health_check_timeout, int max_reset_retries);
bool rplidar__read_exact(RplidarReader *r, uint8_t *buf, int n, double timeout);

typedef enum {
    RPLIDAR_OK = 0,
    RPLIDAR_ERR_TIMEOUT,
    RPLIDAR_ERR_PROTOCOL,
    RPLIDAR_ERR_HARDWARE_FAIL,
} RplidarStatus;
RplidarStatus rplidar_check_health(RplidarReader *r, RplidarHealth *out_health);
RplidarStatus rplidar_start_scan(RplidarReader *r);
void rplidar_stop_scan(RplidarReader *r);
RplidarStatus rplidar_read_one_full_scan(RplidarReader *r, double timeout,
                                           LidarScan *out,
                                           bool (*should_continue)(void *), void *ctx);

#endif /* RPLIDAR_READER_H */
