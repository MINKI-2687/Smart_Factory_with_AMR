#ifndef RPLIDAR_C1_H
#define RPLIDAR_C1_H

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

/* ============================================================
 * 명령어 코드 (Python rplidar_c1.py와 동일)
 * ============================================================ */
#define RPLIDAR_CMD_STOP            0x25
#define RPLIDAR_CMD_RESET           0x40
#define RPLIDAR_CMD_SCAN            0x20
#define RPLIDAR_CMD_GET_INFO        0x50
#define RPLIDAR_CMD_GET_HEALTH      0x52
#define RPLIDAR_CMD_GET_SAMPLERATE  0x59
#define RPLIDAR_CMD_MOTOR_SPEED_CTRL 0xA8

#define RPLIDAR_SYNC_BYTE 0xA5
#define RPLIDAR_DESCRIPTOR_SYNC2 0x5A

#define RPLIDAR_HEALTH_GOOD 0
#define RPLIDAR_HEALTH_WARNING 1
#define RPLIDAR_HEALTH_ERROR 2
int rplidar_build_request(uint8_t command, const uint8_t *payload, int payload_len,
                           uint8_t *out);
int rplidar_build_stop(uint8_t *out);
int rplidar_build_reset(uint8_t *out);
int rplidar_build_scan(uint8_t *out);
int rplidar_build_get_health(uint8_t *out);
int rplidar_build_motor_speed_ctrl(uint16_t rpm, uint8_t *out);

/* ============================================================
 * 응답 서술자(Response Descriptor) 파싱 - 7바이트 고정
 * ============================================================ */
typedef struct {
    uint32_t length;
    uint8_t send_mode;
    uint8_t data_type;
} RplidarDescriptor;
bool rplidar_parse_descriptor(const uint8_t *data7, RplidarDescriptor *out);

/* ============================================================
 * GET_HEALTH 응답 파싱 - 3바이트
 * ============================================================ */
typedef struct {
    uint8_t status;
    uint16_t error_code;
} RplidarHealth;
void rplidar_parse_health(const uint8_t *data3, RplidarHealth *out);

/* ============================================================
 * Standard SCAN 측정 샘플 파싱 - 5바이트/포인트
 * ============================================================ */
typedef struct {
    bool new_scan;
    int quality;
    double angle_deg;
    double distance_mm;
    bool valid;
} RplidarSample;
void rplidar_parse_sample(const uint8_t *data5, RplidarSample *out);

#endif /* RPLIDAR_C1_H */
