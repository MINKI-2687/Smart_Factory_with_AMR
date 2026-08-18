#include "rplidar_reader.h"


void rplidar_reader_init(RplidarReader *r, int fd,
                                        double health_check_timeout, int max_reset_retries) {
    r->fd = fd;
    r->health_check_timeout = health_check_timeout;
    r->max_reset_retries = max_reset_retries;
    r->scanning = false;
}


bool rplidar__read_exact(RplidarReader *r, uint8_t *buf, int n, double timeout) {
    int got = 0;
    struct timespec start; clock_gettime(CLOCK_MONOTONIC, &start);
    while (got < n) {
        struct timespec now; clock_gettime(CLOCK_MONOTONIC, &now);
        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_nsec - start.tv_nsec) / 1e9;
        if (elapsed > timeout) return false;

        ssize_t rd = read(r->fd, buf + got, (size_t)(n - got));
        if (rd > 0) {
            got += (int)rd;
        } else {
            struct timespec ts = {0, 2 * 1000000L};
            nanosleep(&ts, NULL);
        }
    }
    return true;
}


RplidarStatus rplidar_check_health(RplidarReader *r, RplidarHealth *out_health) {
    uint8_t req[8];
    int n = rplidar_build_get_health(req);
    if (write(r->fd, req, n) < 0) return RPLIDAR_ERR_TIMEOUT;

    uint8_t desc_raw[7];
    if (!rplidar__read_exact(r, desc_raw, 7, r->health_check_timeout)) return RPLIDAR_ERR_TIMEOUT;

    RplidarDescriptor desc;
    if (!rplidar_parse_descriptor(desc_raw, &desc) || desc.length != 3) return RPLIDAR_ERR_PROTOCOL;

    uint8_t health_raw[3];
    if (!rplidar__read_exact(r, health_raw, 3, r->health_check_timeout)) return RPLIDAR_ERR_TIMEOUT;

    rplidar_parse_health(health_raw, out_health);
    return RPLIDAR_OK;
}


RplidarStatus rplidar_start_scan(RplidarReader *r) {
    for (int attempt = 0; attempt <= r->max_reset_retries; attempt++) {
        RplidarHealth health;
        RplidarStatus st = rplidar_check_health(r, &health);
        if (st != RPLIDAR_OK) return st;

        if (health.status != RPLIDAR_HEALTH_ERROR) break;

        if (attempt >= r->max_reset_retries) return RPLIDAR_ERR_HARDWARE_FAIL;

        uint8_t reset_req[8];
        int n = rplidar_build_reset(reset_req);
        if (write(r->fd, reset_req, n) < 0) return RPLIDAR_ERR_TIMEOUT;
        struct timespec ts = {0, 500 * 1000000L};
        nanosleep(&ts, NULL);
    }

    uint8_t scan_req[8];
    int n = rplidar_build_scan(scan_req);
    if (write(r->fd, scan_req, n) < 0) return RPLIDAR_ERR_TIMEOUT;

    uint8_t desc_raw[7];
    if (!rplidar__read_exact(r, desc_raw, 7, r->health_check_timeout)) return RPLIDAR_ERR_TIMEOUT;

    RplidarDescriptor desc;
    if (!rplidar_parse_descriptor(desc_raw, &desc) || desc.length != 5) return RPLIDAR_ERR_PROTOCOL;

    r->scanning = true;
    return RPLIDAR_OK;
}


void rplidar_stop_scan(RplidarReader *r) {
    uint8_t buf[8];
    int n = rplidar_build_stop(buf);
    if (write(r->fd, buf, n) < 0) { /* 종료 중이라 무시 */ }
    struct timespec ts1 = {0, 1 * 1000000L};
    nanosleep(&ts1, NULL);

    n = rplidar_build_motor_speed_ctrl(0, buf);
    if (write(r->fd, buf, n) < 0) { /* 무시 */ }

    r->scanning = false;
}


RplidarStatus rplidar_read_one_full_scan(RplidarReader *r, double timeout,
                                                         LidarScan *out,
                                                         bool (*should_continue)(void *), void *ctx) {
    out->count = 0;
    struct timespec start; clock_gettime(CLOCK_MONOTONIC, &start);
    bool seen_first_new_scan = false;

    while (true) {
        if (should_continue != NULL && !should_continue(ctx)) return RPLIDAR_OK;

        struct timespec now; clock_gettime(CLOCK_MONOTONIC, &now);
        double elapsed = (now.tv_sec - start.tv_sec) + (now.tv_nsec - start.tv_nsec) / 1e9;
        if (elapsed > timeout) return RPLIDAR_ERR_TIMEOUT;

        uint8_t raw[5];
        if (!rplidar__read_exact(r, raw, 5, 0.2)) continue;

        RplidarSample sample;
        rplidar_parse_sample(raw, &sample);
        if (!sample.valid) continue;

        if (sample.new_scan) {
            if (seen_first_new_scan) break;
            seen_first_new_scan = true;
        }

        if (seen_first_new_scan) {
            if (out->count < LIDAR_MAX_READINGS) {
                out->readings[out->count].angle_deg = sample.angle_deg;
                out->readings[out->count].dist_mm = sample.distance_mm;
                out->count++;
            }
        }
    }
    return RPLIDAR_OK;
}
