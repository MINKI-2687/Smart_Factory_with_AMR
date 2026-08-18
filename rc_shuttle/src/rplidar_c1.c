#include "rplidar_c1.h"


/* ============================================================
 * 요청 패킷 생성
 * 페이로드 없는 요청은 딱 2바이트(sync+command)뿐, 체크섬조차 없음
 * (문서 실제 예시 "A5 25" 등을 보고 확인된 사실 - Optional Section이
 *  PayloadSize+Payload+Checksum을 통째로 묶고 있었음)
 * 반환: 실제로 만들어진 바이트 수
 * ============================================================ */
int rplidar_build_request(uint8_t command, const uint8_t *payload, int payload_len,
                                         uint8_t *out) {
    if (payload == NULL || payload_len == 0) {
        out[0] = RPLIDAR_SYNC_BYTE;
        out[1] = command;
        return 2;
    }
    out[0] = RPLIDAR_SYNC_BYTE;
    out[1] = command;
    out[2] = (uint8_t)payload_len;
    memcpy(out + 3, payload, payload_len);
    uint8_t checksum = 0;
    for (int i = 0; i < 3 + payload_len; i++) checksum ^= out[i];
    out[3 + payload_len] = checksum;
    return 3 + payload_len + 1;
}


int rplidar_build_stop(uint8_t *out) {
    return rplidar_build_request(RPLIDAR_CMD_STOP, NULL, 0, out);
}

int rplidar_build_reset(uint8_t *out) {
    return rplidar_build_request(RPLIDAR_CMD_RESET, NULL, 0, out);
}

int rplidar_build_scan(uint8_t *out) {
    return rplidar_build_request(RPLIDAR_CMD_SCAN, NULL, 0, out);
}

int rplidar_build_get_health(uint8_t *out) {
    return rplidar_build_request(RPLIDAR_CMD_GET_HEALTH, NULL, 0, out);
}

int rplidar_build_motor_speed_ctrl(uint16_t rpm, uint8_t *out) {
    uint8_t payload[2] = { (uint8_t)(rpm & 0xFF), (uint8_t)((rpm >> 8) & 0xFF) };
    return rplidar_build_request(RPLIDAR_CMD_MOTOR_SPEED_CTRL, payload, 2, out);
}


bool rplidar_parse_descriptor(const uint8_t *data7, RplidarDescriptor *out) {
    if (data7[0] != RPLIDAR_SYNC_BYTE || data7[1] != RPLIDAR_DESCRIPTOR_SYNC2) return false;
    uint32_t raw32 = (uint32_t)data7[2] | ((uint32_t)data7[3] << 8) |
                      ((uint32_t)data7[4] << 16) | ((uint32_t)data7[5] << 24);
    out->length = raw32 & 0x3FFFFFFF;
    out->send_mode = (raw32 >> 30) & 0x3;
    out->data_type = data7[6];
    return true;
}


void rplidar_parse_health(const uint8_t *data3, RplidarHealth *out) {
    out->status = data3[0];
    out->error_code = (uint16_t)data3[1] | ((uint16_t)data3[2] << 8);
}


void rplidar_parse_sample(const uint8_t *data5, RplidarSample *out) {
    uint8_t byte0 = data5[0], byte1 = data5[1], byte2 = data5[2], byte3 = data5[3], byte4 = data5[4];

    int s = byte0 & 0x1;
    int s_inv = (byte0 >> 1) & 0x1;
    int quality = byte0 >> 2;

    int c = byte1 & 0x1;
    int angle_low7 = byte1 >> 1;
    int angle_high8 = byte2;
    int angle_q6 = (angle_high8 << 7) | angle_low7;

    int distance_q2 = (int)byte3 | ((int)byte4 << 8);

    bool checkbits_ok = (s != s_inv) && (c == 1);
    bool is_measurement_valid = distance_q2 != 0;  /* 0이면 "측정 무효" (스펙 명시) */

    out->new_scan = (s != 0);
    out->quality = quality;
    out->angle_deg = angle_q6 / 64.0;
    out->distance_mm = distance_q2 / 4.0;
    out->valid = checkbits_ok && is_measurement_valid;
}
