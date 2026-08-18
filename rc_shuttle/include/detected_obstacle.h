#ifndef DETECTED_OBSTACLE_H
#define DETECTED_OBSTACLE_H

/* 재탐색 트리거 시 "새로 감지된 장애물" 하나를 나타내는 순수 인터페이스 타입.
 * points는 (x,y) 쌍이 xs[i],ys[i]로 저장된 배열.
 *
 * dynamic_navigator.h(재탐색을 받는 쪽)와 lidar_clustering.h(재탐색을 만드는 쪽)가
 * 이 타입 하나만 공유하면 되는데, 예전엔 lidar_clustering.h가 이 타입 하나 때문에
 * dynamic_navigator.h 전체(A*+PID+도킹)를 통째로 include해야 했음 - 이 파일로 분리해서
 * 그 불필요한 의존을 없앰. */
typedef struct {
    const double *xs;
    const double *ys;
    int count;
} DetectedObstacle;

#endif /* DETECTED_OBSTACLE_H */
