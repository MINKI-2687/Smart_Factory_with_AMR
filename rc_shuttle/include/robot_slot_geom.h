#ifndef ROBOT_SLOT_GEOM_H
#define ROBOT_SLOT_GEOM_H

#include "robot_config.h"

/* ============================================================
 * 슬롯 안에서는 '라이다 실측'이 진실이다 (2026-08-11a 신규)
 *
 * ---- 실기 로그로 확인된 증상 (사용자 신고 1번) ----
 *   [slot] step=27 pose=(1.272,0.214,630.9deg) 좌우오차=-0.006m(라이다) cmd=(34,34)
 *   [slam] 도약 제한: 136mm/1.5deg 중 78%만 반영합니다 (품질 0.99 > 예측 0.51)
 *   [slam] 주의: 한 스텝에 106mm/1.2deg 이동, 매칭품질 0.89 -> 자세 (1.099,0.196,640deg)
 *   [slot] 슬롯 진입 완료: pose=(1.049, 0.164, 634.9deg)
 *   [shuttle] B지점 주차 오차 초과: ... 오차 226.7mm (허용 80mm)
 *   [shuttle] ===== B지점 주차 재시도 2/3 - 빼내고 다시 접근합니다 =====
 *
 * 로봇은 눈으로 보면 B슬롯에 제대로 들어가 있었다. 움직인 것은 차가 아니라
 * 자세추정이다. x 가 한 스텝에 176mm 미끄러졌고, 그 값으로 주차오차를 재니
 * 226mm 가 나와서 '다 넣어 놓고 도로 빼는' 최악의 동작이 나왔다.
 *
 * ---- 왜 슬롯 안에서 스캔매칭이 미끄러지나 (구조적인 문제다) ----
 * (1) likelihood field 는 '가장 가까운 점유셀까지의 거리'를 가우시안으로 바꾼 판이라,
 *     벽 *속* 깊숙한 점도 거리 0 = 만점(1.0)을 받는다. 슬롯 안에서는 스캔의 대부분이
 *     좌우 벽 두 줄뿐인데, 자세를 옆으로 176mm 밀면 한쪽 벽의 점들이 가운데 큰 벽
 *     덩어리(x 0.33~1.17) 속으로 통째로 들어가 전부 만점을 받는다. 그래서
 *     '품질 0.99' 라는 완벽해 보이는 오답이 만들어진다.
 * (2) 슬롯 축(y) 방향은 긴 평행벽이라 원리적으로 관측이 안 된다(통로 구멍 문제).
 *     사진 속 y=0.938(지도 밖!) 도 같은 원인이다.
 * => 슬롯 안에서 3자유도 스캔매칭을 그대로 믿으면 안 된다.
 *
 * ---- 반대로 라이다 원시값은 슬롯 안에서 가장 정확하다 ----
 *   좌우: 양쪽 벽까지 거리 d_l, d_r -> 치우침 e = (d_r - d_l)/2.
 *         슬롯 폭도 차폭도 몰라도 되고 지도/자세와 완전히 무관하다.
 *   각도: 좌/우 벽 점들을 직선으로 최소제곱 적합하면 그 기울기가 곧 각도오차다.
 *         벽이 슬롯 축과 나란하므로 몸통기준 기울기 phi 에 대해 각도오차 = -phi.
 *   깊이: x 와 각도를 위 실측값으로 '못 박고' y 만 1차원으로 훑으면, 슬롯 입구
 *         모서리와 통로 안쪽 벽이 유일한 봉우리를 만든다. 3자유도일 때 생기던
 *         미끄러짐이 원천적으로 사라진다. = 사용자가 말한 "다시 스캔매칭".
 *
 * 그래서 슬롯 평행구간 안이라고 라이다가 확인해 주면, 그 실측값으로 자세를 다시
 * 박고 그 위에서 판단한다. 이 재정박은 '슬롯 평행구간' 안에서만 동작한다 -
 * 통로에서는 양쪽 벽이 멀거나 폭이 안 맞아서 아래 ok 조건이 성립하지 않는다.
 * ============================================================ */

/* 지도에서 읽어낸 슬롯의 기하 (하드코딩 없이 매번 격자에서 직접 잰다) */
typedef struct {
    bool   ok;
    double center_x;    /* 슬롯 중심선 x */
    double half_w;      /* 슬롯 반폭 (벽면 기준) */
    double inner_y;     /* 안쪽 벽면 y (슬롯 바닥) */
    double mouth_y;     /* 평행구간이 끝나고 깔때기가 시작되는 y */
    double back_y;      /* 슬롯 반대쪽 끝 = 통로 바깥벽 y (후방 광선 검산용) */
} SlotMapGeom;

/* 라이다만으로 잰 슬롯 안 자세 */
typedef struct {
    bool   ok;            /* 슬롯 평행구간 안이라고 확신하는가 */
    double d_left, d_right;
    double width_m;       /* d_left + d_right (= 슬롯 폭이어야 함) */
    double lat_left_m;    /* 차체 기준 좌우 치우침 (+ = 왼쪽으로 치우침) */
    bool   yaw_ok;
    double yaw_err_rad;   /* 슬롯 축 대비 각도오차 - 스캔매칭과 완전히 독립 */
    int    n_wall;
    bool   depth_ok;
    double depth_y;       /* 1차원 스캔매칭으로 되찾은 y */
    double depth_fit;     /* 그 자리의 점당 평균 우도 (0~1) */
    double rear_y;        /* 후방 광선으로 독립 계산한 y (검산용, <0 이면 없음) */
} SlotLidarFix;
bool robot_runner__grid_occ(const OccupancyGridSLAM *s, double wx, double wy);
bool robot_runner__slot_map_geom(const OccupancyGridSLAM *s,
                                  double lane_x, double probe_y,
                                  SlotMapGeom *out);
bool robot_runner__wall_slope(const LidarScan *scan,
                               double side,          /* +1 = 왼쪽 벽, -1 = 오른쪽 */
                               double along_m, double max_perp_m,
                               double *out_yaw_rad, int *out_n);
double robot_runner__slot_depth_match(const OccupancyGridSLAM *s,
                                       const LidarScan *scan,
                                       double fix_x, double fix_th,
                                       double y_lo, double y_hi,
                                       double *out_fit);
void robot_runner__slot_measure(const OccupancyGridSLAM *s,
                                   const LidarScan *scan,
                                   const RobotConfig *cfg,
                                   const SlotMapGeom *geom,
                                   double heading_rad,
                                   SlotLidarFix *out);

/* ============================================================
 * 어느 슬롯에 있는지 '가정하지 말고 검증한다' (2026-08-11c 신규)
 *
 * *** 사용자 신고 2번 ***
 *   [slot] 라이다-스캔매칭 불일치가 604mm 나 됩니다 (슬롯 진입 중).
 *          이 거리면 '반대쪽 슬롯일 가능성'을 배제할 수 없어 조용히 옮기지 않고,
 *          전역 재위치추정에 맡깁니다
 * 이 메시지가 매 스텝 반복되면서 자세가 끝내 안 고쳐졌다. 이유가 둘이었다.
 *
 *  (1) 여기서 '맡긴다'고 한 전역 재위치추정이 실제로는 아무 데서도 안 일어났다.
 *      slam->relocalize_request 를 올리기만 했는데, 그 요청을 읽는 곳은
 *      slot_align_inner() 와 navigate_one_leg_inner() 두 군데뿐이고
 *      slot_drive() / slot_confirm_park() 에는 없었다. 즉 슬롯 진입~주차 구간
 *      전체가 요청을 받아주는 사람이 없는 상태였다.
 *  (2) 애초에 '반대쪽 슬롯일 가능성'을 검사한 적이 없다. 거리가 멀다는 이유만으로
 *      포기했을 뿐이다.
 *
 * ---- (2)에 대해: 라이다로 A/B 구분이 되는가? 된다. 그것도 아주 뚜렷하게 ----
 * 이 지도(set_map.txt)는 좌우 완전 대칭이라 '슬롯 안쪽 벽 세 면'만 보면 A와 B가
 * 똑같다. 그래서 예전 판단이 나온 것이고, 거기까지는 맞다.
 * 그런데 로봇은 슬롯 안에서도 뒤쪽(입구 쪽)으로 통로와 방 끝벽을 본다.
 * A슬롯(x=0.225)에서는 왼쪽 끝벽이 175mm, 오른쪽 끝벽이 1225mm 다. B슬롯이면
 * 정확히 반대다. 이 비대칭은 스캔 한 장에 그대로 찍힌다.
 *
 * 실제로 이 지도로 합성스캔 실험을 돌려 봤다(잡음 10mm, 좌우 오프셋 ±20mm,
 * 각도오차 ±8도, 깊이 y=0.15~0.47 전 구간):
 *      맞는 슬롯 일치도 0.989~0.993  /  반대쪽 슬롯 0.68~0.83
 *      차이 16~31%p  (가장 나쁜 조건에서도 16%p)
 * 즉 '못 고른다'가 아니라 '고르려는 시도를 한 번도 안 했다'가 정답이었다.
 *
 * 그래서 이 함수는 후보 슬롯마다
 *      x = 그 슬롯 중심 + 라이다로 잰 좌우 치우침
 *      각도 = 슬롯 축 + 라이다로 잰 각도오차
 *      y = 그 x/각도를 못 박고 1차원으로 다시 맞춘 깊이
 * 로 '완전한 자세 하나'를 만들고, 점수판으로 채점한다. 1등이 2등보다
 * 뚜렷하게(SLOT_PICK_MARGIN) 좋을 때만 결론을 낸다.
 *
 * 사용자가 제안한 '직전에 어느 슬롯으로 향했는지 플래그'는 여기서 동점일 때만
 * 쓴다(= 명령한 차선을 유지). 단독으로 쓰면 위험하다는 사용자의 판단이 맞다 -
 * 자세가 이미 틀어진 상황이라 '내가 뭘 명령했는지'가 '로봇이 어디 있는지'를
 * 보장하지 않기 때문이다. 물리적 증거를 먼저 보고, 증거가 비길 때만 명령을 믿는다.
 * ============================================================ */
#define SLOT_LANES_MAX 8

typedef struct {
    int    n;
    double x[SLOT_LANES_MAX];
} SlotLanes;
SlotLanes *robot_runner__slot_lanes(void);
bool *robot_runner__slot_wrong_lane(void);
void robot_runner__slot_lanes_set(const double *xs, int n);

typedef struct {
    bool   decisive;      /* 1등이 2등보다 뚜렷하게 좋아서 결론을 낼 수 있는가 */
    bool   any;           /* 후보가 하나라도 '슬롯 평행구간 안'으로 측정됐는가 */
    int    n_cand;
    double lane_x;        /* 1등 슬롯의 차선 */
    double fit_best, fit_second;   /* 판별용(먼 점만) 일치도 */
    double fit_all;       /* 1등 후보의 전체 스캔 일치도 (건전성 확인용) */
    int    n_far;         /* 판별에 실제로 쓰인 점 수 */
    double px, py, pth;   /* 1등 슬롯 기준으로 복원한 자세 */
    SlotLidarFix fix;     /* 1등 슬롯에서의 라이다 실측 */
} SlotPick;
double robot_runner__map_fit_far(const OccupancyGridSLAM *s,
                                  const LidarScan *scan,
                                  double x, double y, double th,
                                  double min_range_m, int *out_n);
bool robot_runner__slot_identify(const OccupancyGridSLAM *slam,
                                  const LidarScan *scan,
                                  const RobotConfig *cfg,
                                  double heading_rad, double probe_y,
                                  double prefer_lane_x,
                                  SlotPick *out);
bool robot_runner__slot_reanchor(OccupancyGridSLAM *slam,
                                  const LidarScan *scan,
                                  const RobotConfig *cfg,
                                  double lane_x, double heading_rad,
                                  double park_y, const char *why,
                                  SlotLidarFix *out_fix,
                                  double *out_dx, double *out_dy);

/* ============================================================
 * '지금 다루고 있는 슬롯'을 기억해 두는 곳 (2026-08-11a)
 *
 * 슬롯 안에서 도는 하위 루틴(slot_fix_heading 등)은 lane_x / 슬롯 축을 인자로
 * 받지 않는다. 그렇다고 함수 시그니처를 줄줄이 바꾸면 호출부가 10군데라
 * 오히려 위험하므로, 이 코드베이스가 이미 쓰는 방식(robot_runner__slot_no_wall)
 * 과 같이 파일 스코프 static 에 담아 둔다. slot_drive() 진입 시 한 번 채운다.
 * ============================================================ */
typedef struct {
    bool   set;
    double lane_x;
    double probe_y;     /* 슬롯 축 위 아무 점 (지도에서 바닥을 찾는 출발점) */
    double axis_rad;    /* 슬롯 축 heading */
} SlotCtx;
SlotCtx *robot_runner__slot_ctx(void);
void robot_runner__slot_ctx_set(double lane_x, double probe_y, double axis_rad);
bool robot_runner__slot_reanchor_ctx(OccupancyGridSLAM *slam,
                                      const LidarScan *scan,
                                      const RobotConfig *cfg,
                                      const char *why, SlotLidarFix *out_fix);

#endif /* ROBOT_SLOT_GEOM_H */
