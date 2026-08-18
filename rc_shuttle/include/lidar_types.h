#ifndef LIDAR_TYPES_H
#define LIDAR_TYPES_H

/* Python의 [(angle_deg, dist_mm), ...] 리스트에 대응.
 * 실시간 임베디드 환경에선 가변배열보다 고정 크기 버퍼가 일반적이라 이렇게 둠.
 * RPLidar C1은 각도분해능 약 0.72도라 한 바퀴에 최대 ~500개 포인트가 나올 수 있어서
 * 여유있게 600으로 잡음. */
#define LIDAR_MAX_READINGS 600

typedef struct {
    double angle_deg;  /* 실제 라이다는 소수점 각도(예: 123.4375도)라 int로 깎으면 정밀도 손실됨 */
    double dist_mm;
} LidarReading;

typedef struct {
    LidarReading readings[LIDAR_MAX_READINGS];
    int count;
} LidarScan;

#endif /* LIDAR_TYPES_H */
