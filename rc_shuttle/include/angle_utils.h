#ifndef ANGLE_UTILS_H
#define ANGLE_UTILS_H

#include <math.h>

/* M_PI는 컴파일러/헤더 include 순서에 따라 정의 여부가 갈릴 수 있어서(glibc feature macro 이슈),
 * 아예 직접 정의해서 그 문제를 피한다. */
#define ANGLE_PI 3.14159265358979323846
double normalize_angle(double angle);
double normalize_angle_deg(double deg);

#endif /* ANGLE_UTILS_H */
