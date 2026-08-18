#include "angle_utils.h"


/* 각도를 -pi ~ +pi 범위로 정규화 */
double normalize_angle(double angle) {
    return atan2(sin(angle), cos(angle));
}


/* 각도(도)를 -180 ~ +180 범위로 정규화 */
double normalize_angle_deg(double deg) {
    while (deg >= 180.0) deg -= 360.0;
    while (deg < -180.0) deg += 360.0;
    return deg;
}
