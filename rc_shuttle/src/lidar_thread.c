#include "lidar_thread.h"


bool lidar_thread_should_continue(void *ctx) {
    LidarThread *lt = (LidarThread *)ctx;
    return lt->running;
}


void *lidar_thread_loop(void *arg) {
    LidarThread *lt = (LidarThread *)arg;
    while (lt->running) {
        LidarScan scan;
        RplidarStatus st = rplidar_read_one_full_scan(lt->reader, 2.0, &scan,
                                                        lidar_thread_should_continue, lt);
        if (!lt->running) break;
        if (st == RPLIDAR_OK) {
            pthread_mutex_lock(&lt->lock);
            lt->latest_scan = scan;
            lt->scan_count++;
            pthread_mutex_unlock(&lt->lock);
        } else {
            pthread_mutex_lock(&lt->lock);
            lt->error_count++;
            pthread_mutex_unlock(&lt->lock);
            struct timespec ts = {0, 50 * 1000000L};
            nanosleep(&ts, NULL);
        }
    }
    return NULL;
}


void lidar_thread_start(LidarThread *lt, RplidarReader *reader) {
    lt->reader = reader;
    pthread_mutex_init(&lt->lock, NULL);
    lt->latest_scan.count = 0;
    lt->running = true;
    lt->scan_count = 0;
    lt->error_count = 0;
    rplidar_start_scan(reader);
    pthread_create(&lt->thread, NULL, lidar_thread_loop, lt);
}


void lidar_thread_get_latest(LidarThread *lt, LidarScan *out) {
    pthread_mutex_lock(&lt->lock);
    *out = lt->latest_scan;
    pthread_mutex_unlock(&lt->lock);
}


void lidar_thread_stop(LidarThread *lt) {
    lt->running = false;
    pthread_join(lt->thread, NULL);
    rplidar_stop_scan(lt->reader);
    pthread_mutex_destroy(&lt->lock);
}
