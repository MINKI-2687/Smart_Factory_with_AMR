#ifndef WHEEL_CMD_H
#define WHEEL_CMD_H

#include <stdbool.h>

/* 좌/우 바퀴 속도 명령 + 도착(완료) 여부. 여러 컨트롤러(Docking, SafeNavigator 등)가 공유. */
typedef struct {
    int left;
    int right;
    bool done;
} WheelCmd;

#endif /* WHEEL_CMD_H */
