#ifndef SLAM_H
#define SLAM_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "angle_utils.h"
#include "lidar_types.h"

#define SLAM_LOG_ODDS_FREE (-0.4)
#define SLAM_LOG_ODDS_OCC   (0.85)
#define SLAM_LOG_ODDS_MIN  (-6.0)
#define SLAM_LOG_ODDS_MAX   (6.0)
#define SLAM_MAX_RAY_LEN   1024

typedef struct {
    int rows, cols;
    double resolution;
    double origin_x, origin_y;
    double *log_odds;

    double x, y, theta;
    bool has_scanned;

    double search_window_m;
    double search_window_theta;
    double search_step_m;
    double search_step_theta;

    /* PATCH (2026-08-05 v5): NULL이 아니면 slam_score_pose()가 이 "부드러운 점수판"을
     * 사용함(각 칸 = 가장 가까운 벽까지의 거리를 가우시안으로 변환한 0~1 값).
     * 고정맵/불러온맵처럼 지도가 변하지 않는 경우에만 robot_runner.h에서 만들어 붙임.
     * 매핑 모드(slam_update로 지도가 매 스텝 바뀜)에서는 NULL로 두어 기존 동작 유지.
     * 이유: 손으로 이어붙인 세트장은 벽이 몇 cm씩 어긋나 있어서, "셀에 정확히 맞으면
     * +6, 아니면 -6"인 딱딱한 점수로는 자세가 한 칸만 틀어져도 점수가 급락해 매칭이
     * 계속 튐. 거리 기반 점수는 부드러워서 이런 오차에 강함. */
    double *lf_field;
    /* 정밀 탐색 전용 좁은 시그마 점수판 (2026-08-09n). NULL 이면 lf_field 를 그대로 씀. */
    double *lf_field_fine;
    /* 직전 매칭 품질: 점당 평균 우도(0~1). 1에 가까울수록 스캔이 벽에 잘 얹혀 있음.
     * 0.5 아래면 '자기 위치를 잃었다'는 신호. */
    double last_match_quality;

    /* ============================================================
     * 운동 타당성 게이트 (2026-08-10 신규)
     *
     * 사용자 신고: "가로 통로를 지나다가 갑자기 맵이 통째로 회전한다."
     * 실기 로그(연속 3스텝):
     *     한 스텝에 136mm/10.0deg 이동, 매칭품질 0.61 -> 자세 (1.225,0.540,320deg)
     *     한 스텝에 138mm/ 6.0deg 이동, 매칭품질 0.75 -> 자세 (1.270,0.410,326deg)
     *     한 스텝에 114mm/12.0deg 이동, 매칭품질 0.83 -> 자세 (1.200,0.320,338deg)
     *
     * 이 값들은 전부 '경고만 찍고 그대로 채택'됐다. 로봇은 한 스텝(0.2~0.4초)에
     * 136mm 를 가면서 동시에 10도를 돌 수 없다 - 물리적으로 불가능한 자세를
     * 스캔매칭이 골랐고, 코드는 그걸 막을 근거가 없어서 받아들였다.
     * 그렇게 한 번 틀어진 자세가 다음 스텝의 예측이 되어 오차가 눈덩이처럼 불었다.
     *
     * 이제는 "예측에서 이만큼 이상 떨어진 답은, 예측보다 확실히 더 좋을 때만
     * 그것도 한계까지만 받아들인다"로 바꾼다. 로봇은 순간이동을 못 한다는 사실이
     * 이 지도(평행벽 + 똑같이 생긴 슬롯 2개)에서 유일하게 믿을 수 있는 정보다.
     *
     * 0 이면 검사하지 않음(기존 테스트/시뮬레이션 코드는 그대로 동작).
     * ============================================================ */
    double max_jump_m;      /* 한 스텝에 허용할 최대 위치 도약(m) */
    double max_jump_rad;    /* 한 스텝에 허용할 최대 각도 도약(rad) */
    int    bad_match_streak; /* 게이트에 연속으로 걸린 횟수 */
    /* ============================================================
     * 전역 재위치추정 요청 플래그 (2026-08-10d 신규) *** 죽어 있던 신호 ***
     *
     * 아래 slam_localize_step() 은 '재탐색이 정답을 찾았지만 한 스텝에 옮기기엔
     * 너무 큰 교정'일 때
     *     [slam] 위치 재탐색 보류: 146mm 교정은 한 스텝에 불가능합니다 (한계 105mm).
     *            조용히 옮기지 않고 전역 재탐색을 요청합니다
     * 를 찍고 bad_match_streak += 3 으로 위층에 신호를 보내려 했다. 그런데
     *   (1) 바로 다음 줄에서 bad_match_streak = 0 으로 스스로 지워버렸고,
     *   (2) 애초에 robot_runner 쪽에서 bad_match_streak 를 '전역탐색 트리거'로
     *       읽는 코드가 한 줄도 없었다(학습 금지 가드로만 쓰였다).
     * 즉 스캔매칭은 정답을 찾아 놓고, 옮기지 못한다고 알렸는데, 그 말을 들은
     * 사람이 아무도 없었다. 실기에서 자세가 200mm 틀어진 채로 계속 달린 이유다.
     *
     * 이제 별도 플래그로 확실히 남기고, robot_runner 가 이 값을 보고 멈춰서
     * 지도 전체 전역탐색을 돌린 뒤 스스로 지운다.
     * ============================================================ */
    int    relocalize_request;   /* >0 이면 위층이 전역 재위치추정을 해야 함 */
    double reloc_req_move_m;     /* 그때 필요하다고 본 교정량(m) - 로그용 */
    /* ============================================================
     * 일치도가 낮으면 그 스텝만 탐색창을 넓힌다 (2026-08-10g 신규)
     *
     * 사용자 요청: "맵이 돌아간 것 같아도 스캔매칭을 다시 안 하냐. 매번 시켜야지."
     * 맞는 지적이고, 지금까지 못 한 이유는 '능력'이 아니라 '범위'였다.
     * 매 스텝 스캔매칭은 하고 있었지만 예측자세 주변 ±150mm/±12도만 봤다.
     * 자세가 그보다 더 틀어지면 정답이 후보에 없으므로, 아무리 여러 번 돌려도
     * 같은 오답을 다시 고를 뿐이다(실기에서 그 상태로 수십 스텝을 갔다).
     *
     * 그렇다고 항상 넓게 보면 후보 수가 5배가 되어 제어 주기를 못 맞춘다.
     * 그래서 '일치도가 낮은 스텝에만' 창을 2배로 넓힌다. 자세가 멀쩡할 때는
     * 비용이 0 이고, 틀어진 순간에만 값을 치른다.
     * 0 이면 이 기능을 끈다. ============================================================ */
    double wide_when_q_below;    /* 이 일치도 미만이면 그 스텝 탐색창을 넓힘 */
} OccupancyGridSLAM;

/* 도약을 채택하려면 예측자세보다 점당 평균 우도가 이만큼은 좋아야 한다.
 * 0.03 은 라이다 잡음 수준의 차이라 '우연히 조금 나은' 봉우리를 걸러낸다. */
#define SLAM_GATE_MARGIN 0.03
int slam_idx(const OccupancyGridSLAM *s, int row, int col);
bool slam_in_bounds(const OccupancyGridSLAM *s, int row, int col);
void slam_world_to_grid(const OccupancyGridSLAM *s, double x, double y,
                         int *row, int *col);
void slam_init(OccupancyGridSLAM *s, double size_x, double size_y, double resolution,
                double origin_x, double origin_y,
                double start_x, double start_y, double start_theta,
                double search_window_m, double search_window_theta_deg,
                double search_step_m, double search_step_theta_deg);
void slam_free(OccupancyGridSLAM *s);
void slam_get_pose(const OccupancyGridSLAM *s, double *x, double *y, double *theta);
void slam_init_from_grid(OccupancyGridSLAM *s, const unsigned char *raw_grid,
                          int rows, int cols, double resolution,
                          double origin_x, double origin_y,
                          double start_x, double start_y, double start_theta,
                          double search_window_m, double search_window_theta_deg,
                          double search_step_m, double search_step_theta_deg);
int slam_bresenham(int r0, int c0, int r1, int c1,
                    int *out_rows, int *out_cols, int max_points);

/* PATCH (2026-08-05 v8): 점수판을 칸 단위로 읽으면 5cm 격자로 양자화돼서, 자세를
 * 아무리 촘촘히 탐색해도 점수가 계단처럼 변해 그 이하 정밀도가 안 나옴. 주변 4칸을
 * 이중선형 보간해서 읽으면 점수가 연속적으로 변하므로 mm 단위 자세 차이도 점수에
 * 반영됨 - 주차 정밀도를 올리려면 필수. */
/* ============================================================
 * 점수판 시그마 - 넓은 것과 좁은 것 두 장 (2026-08-09n)
 *
 * 사용자 신고: "라이다 맵이 모양은 맞는데 위치가 평행이동돼 있다. 자기 위치를
 * 잃어버린 것 같다."  -> 정확히 시그마가 너무 넓을 때 나오는 증상이다.
 *
 * 점수판은 각 칸에 '가장 가까운 벽까지의 거리'를 exp(-d^2/2sigma^2) 로 바꿔 넣은 것이다.
 * sigma=0.06 이면 6cm 어긋나도 점수가 60% 밖에 안 떨어진다. 세트장 벽이 몇 cm씩
 * 어긋나 있어도 매칭이 안 튄다는 장점이 있지만, 대신 **봉우리가 뭉툭해져서 위치를
 * 정밀하게 못 잡는다.** 특히 이 맵의 위쪽 통로는 긴 평행벽 두 개라, 벽을 따라
 * 미끄러지는 방향(x)으로는 점수 변화가 거의 없다. 그래서 모양은 맞는데 통째로
 * 평행이동한 자세가 최고점으로 뽑힌다.
 *
 * 해결: 표준적인 다중해상도 방식으로 점수판을 두 장 만든다.
 *   - 넓은 것(sigma 0.06): 거친 탐색용. 기울기가 완만해 멀리서도 봉우리로 끌려온다.
 *   - 좁은 것(sigma 0.02): 정밀 탐색용. 봉우리가 뾰족해 mm 단위로 자세를 확정한다.
 * 거친 단계에서 넓은 판으로 대략을 잡고, 정밀 단계에서 좁은 판으로 확정하면
 * 강건함과 정밀도를 동시에 얻는다.
 * ============================================================ */
#define LF_SIGMA_M      0.06   /* 거친 탐색용 - 벽 오차에 관대 */
/* 실행 옵션(--slam-sigma)으로 바꿀 수 있게 전역으로 뺌 (2026-08-09n) */
extern double g_slam_sigma_coarse;
extern double g_slam_sigma_fine;
#define LF_SIGMA_FINE_M 0.04   /* 정밀 탐색용 - 아래 주석의 실측 참고 */
/* 점당 평균 우도가 이 값 아래로 떨어지면 '위치를 잃었다'고 보고 넓게 재탐색한다.
 * 잘 맞을 때는 0.75~0.95, 10cm 어긋나면 0.3 아래로 떨어진다(좁은 시그마 0.02 기준). */
#define SLAM_LOST_QUALITY 0.45

double *slam_build_likelihood_field_sigma(const OccupancyGridSLAM *s,
                                                         double sigma_m);
double *slam_build_likelihood_field(const OccupancyGridSLAM *s);
double *slam_build_likelihood_field_sigma(const OccupancyGridSLAM *s,
                                           double sigma_m);

/* 순수 스캔매칭 함수 - 이름에 "slam"이 안 들어간 이유: SLAM 매핑 중(slam_update)에도,
 * 고정맵 위치추정 전용(slam_localize_step)에도 똑같이 쓰임 - 지도를 "새로 만드는 중"인지
 * "이미 확정된 지도"인지는 이 함수 입장에서 상관없음(log_odds 배열 하나 보고 점수만 매길 뿐).
 * 오도메트리 페널티(고무줄): 예측위치에서 멀어질수록 감점, 지도가 비어있거나
 * 특징이 부실할 때 근거 없이 엉뚱한 후보로 튀는 걸 방지함.
 * 신뢰도확인: 후보들 간 점수차가 거의 없으면(구분 안 되면) 예측위치를 그대로 씀. */
/* ============================================================
 * 스캔 삼각함수 캐시 (PATCH 2026-08-08c: 스캔매칭 속도)
 *
 * 기존 slam_score_pose()는 "후보자세마다, 그 안에서 점마다" cos/sin을 다시 계산했음.
 * 한 번 위치추정에 쓰이는 후보자세는
 *     거친 탐색 11 x 11 x 9 = 1089개  +  정밀 탐색 13^3 = 2197개 = 3286개
 * 이고 스캔이 460점이면 cos/sin 호출이 약 300만 번. 회전각을
 *     cos(theta + a) = cos(theta)cos(a) - sin(theta)sin(a)
 * 로 전개하면 점별 cos(a)/sin(a)는 스캔당 한 번만 구하면 되고, 자세별로는
 * cos(theta)/sin(theta) 두 번만 있으면 됨.
 * 거친 탐색은 어차피 3cm/3도 격자라 점을 3개당 1개만 써도 봉우리 위치가 안 변함.
 *
 * 실측(set_map.txt 90x150 @1cm, 합성스캔 460점, x86):
 *     기존 67.6ms  ->  24.5ms  (2.8배), 결과 자세는 완전히 동일
 * ============================================================ */
typedef struct {
    double ca[LIDAR_MAX_READINGS], sa[LIDAR_MAX_READINGS], d[LIDAR_MAX_READINGS];
    int n;
} ScanTrigCache;
void slam__cache_scan(ScanTrigCache *c, const LidarScan *scan, int stride);
double slam__lf_sample_field(const OccupancyGridSLAM *s, const double *field,
                              double wx, double wy);
double slam__score_cached_field(const OccupancyGridSLAM *s, const ScanTrigCache *c,
                                 const double *field,
                                 double x, double y, double theta);
double slam__score_cached(const OccupancyGridSLAM *s, const ScanTrigCache *c,
                           double x, double y, double theta);
double slam__match_quality(const OccupancyGridSLAM *s, const ScanTrigCache *c,
                            double x, double y, double theta);
void scan_match_correct_pose(const OccupancyGridSLAM *s,
                        double predicted_x, double predicted_y, double predicted_theta,
                        const LidarScan *scan,
                        double *out_x, double *out_y, double *out_theta);
bool scan_match_recover(const OccupancyGridSLAM *s, const LidarScan *scan,
                         double cur_x, double cur_y, double cur_theta,
                         double win_m, double win_theta_rad,
                         double *out_x, double *out_y, double *out_theta,
                         double *out_quality);
void slam_localize_step(OccupancyGridSLAM *s, const LidarScan *scan,
                         double odom_dx, double odom_dy, double odom_dtheta,
                         double *out_x, double *out_y, double *out_theta);
void slam_update_map(OccupancyGridSLAM *s, double x, double y, double theta,
                      const LidarScan *scan);
void slam_update(OccupancyGridSLAM *s, const LidarScan *scan,
                  double odom_dx, double odom_dy, double odom_dtheta,
                  double *out_x, double *out_y, double *out_theta);
unsigned char *slam_to_binary_grid(const OccupancyGridSLAM *s, double occ_threshold);

#endif /* SLAM_H */
