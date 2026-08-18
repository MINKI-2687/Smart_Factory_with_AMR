#ifndef PATH_FOLLOW_H
#define PATH_FOLLOW_H

#include <math.h>
#include <stdbool.h>
#include "path.h"
#include "angle_utils.h"

/* ============================================================
 * 경로추종 기하 계산 (2026-08-09e 신규)
 *
 * 왜 새로 만들었나 (= 사용자 신고 "통로에서 경로를 벗어나면 크게 앞뒤로만 왔다갔다"):
 *
 * 예전 pure pursuit(dynnav__lookahead_point)는 이렇게 동작했다.
 *     while (웨이포인트[i] 까지 거리 < L) i++;     // L = 0.22 고정
 *     목표 = 웨이포인트[i]
 * 여기에 두 가지 구조적 결함이 있었다.
 *
 *  (1) '경로까지의 수직거리'라는 개념 자체가 없다. 로봇이 경로에서 옆으로 d 만큼
 *      벗어나면, 전방주시점은 여전히 경로 위 한 점이므로 그 점을 향한 방위각이
 *      경로 접선에서 asin(d/L) 만큼 꺾인다. d=0.10m, L=0.22m 면 27도,
 *      d=0.15m 면 43도다.
 *  (2) safe_navigator_compute() 는 방위각 오차가 30도를 넘으면 선속도를 0으로
 *      두고 '제자리 회전'만 한다. 그런데 이 통로는 높이가 0.40m 뿐이고 차체
 *      회전반경이 0.143m 라, 중심이 통로 정중앙에서 5cm만 벗어나도 제자리 회전
 *      여유(robot_radius+0.01)가 안 나온다.
 *      -> navigate_one_leg 의 '회전 여유 부족' 검사가 걸린다
 *      -> robot_runner__escape_to_open() 이 최대 12cm 씩 앞/뒤로 밀어낸다
 *      -> 앞뒤로 움직여봐야 좌우 여유는 1mm도 안 변하므로 다시 같은 판정
 *      -> 전진/후진 무한 반복. 이것이 화면에서 보이던 "크게 앞뒤로 왔다갔다".
 *
 * 즉 근본원인은 탈출 로직이 아니라 "경로에서 조금 벗어났을 뿐인데 제자리 회전을
 * 요구한 것"이다. 사람이 운전하듯 비스듬히 천천히 붙으면 되는 상황이었다.
 *
 * 고친 방법: 전방주시 거리를 옆으로 벗어난 거리에 비례해 늘린다.
 *     L_eff = clamp(max(L_base, LOOKAHEAD_CROSS_GAIN * |cross_track|), L_base, L_max)
 * PATH_FOLLOW_CROSS_GAIN = 2.5 이면 방위각이 접선에서 최대 atan(1/2.5)=21.8도까지만
 * 꺾이므로, 30도 문턱을 원리적으로 넘지 않는다. 벗어난 거리가 클수록 목표점이
 * 멀어져 완만하게 붙고(=사용자가 요청한 "천천히 경로에 맞춰가기"), 경로 위에
 * 올라오면 L_base 로 돌아와 다시 정확히 따라간다.
 *
 * 덤으로 얻는 것들:
 *   - 경로까지의 수직거리(cross_track)와 접선방향(path_heading)이 생겨서
 *     상위에서 로그/판정에 쓸 수 있다.
 *   - 남은 경로길이(remain)를 알 수 있어 도킹 전환 시점을 거리 기준으로
 *     정확히 잡을 수 있다(예전엔 웨이포인트 인덱스로 판단해서, 인덱스가
 *     건너뛰어지면 엉뚱한 시점에 도킹으로 넘어갔다).
 *   - 투영점을 매번 다시 찾으므로 웨이포인트를 '지나쳤다가 되돌아오는' 경우에도
 *     복구된다(예전 코드는 인덱스가 단조증가라 한 번 지나치면 영영 뒤를 봤다).
 * ============================================================ */

#define PATH_FOLLOW_CROSS_GAIN 2.5

typedef struct {
    bool   valid;
    int    seg_index;      /* 투영된 구간의 시작 웨이포인트 인덱스 */
    double proj_x, proj_y; /* 경로 위 최근접점 */
    double cross_track;    /* 부호 있는 수직거리(m). + = 로봇이 진행방향 기준 왼쪽 */
    double path_heading;   /* 그 지점의 경로 접선 방향(rad) */
    double look_x, look_y; /* 전방주시점 */
    double look_dist;      /* 실제로 사용한 전방주시 거리(m) */
    double remain;         /* 투영점부터 경로 끝까지 남은 길이(m) */
} PathTrack;
double path_follow__seg_closest(double px, double py,
                                 double ax, double ay,
                                 double bx, double by,
                                 double *out_t,
                                 double *out_x, double *out_y);
void path_follow__advance(const Path *p, int seg, double t, double L,
                           double *out_x, double *out_y);
bool path_follow_track(const Path *p, int hint_seg,
                        double x, double y,
                        double base_L, double max_L,
                        PathTrack *out);

#endif /* PATH_FOLLOW_H */
