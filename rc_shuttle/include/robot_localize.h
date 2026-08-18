#ifndef ROBOT_LOCALIZE_H
#define ROBOT_LOCALIZE_H

#include "robot_config.h"

/* ============================================================
 * 시작 자세 전역 탐색 (위치 + 각도 전부).
 *
 * PATCH (2026-08-05 v4): 이전 버전은 각도만 360도 뒤지고 위치는 --start 주변
 * ±0.10m만 봤음. 그런데 사람이 로봇을 cm 단위로 정확히 놓아줄 수가 없어서, 실제
 * 위치가 그 창을 벗어나면 올바른 위치를 못 찾고 "각도를 엉뚱하게 틀어서" 억지로
 * 점수를 맞춰버림. 실기 로그 3회에서 결과가 (0.25,0.20,40deg) / (0.25,0.15,28deg) /
 * (0.20,0.10,168deg)로 나왔는데, 이 중 두 번의 x가 정확히 탐색 한계(+0.10)에
 * 붙어 있었음 - 전형적인 "더 가고 싶은데 막힌" 신호이고, 그 대가로 각도가 매번
 * 다르게 나온 것.
 *
 * 그래서 위치도 지도 전체를 훑도록 바꿈. 전수조사는 비싸므로 2단계로:
 *   1단계(성김): 위치 0.10m 간격 x 각도 4도 간격으로 지도 전체
 *   2단계(촘촘): 1단계 최적점 주변만 위치 0.02m x 각도 1도로 정밀화
 * 벽/장애물 안쪽 칸은 로봇이 있을 수 없으므로 아예 건너뜀 - 속도도 벌고 말도 안 되는
 * 답도 걸러짐.
 *
 * 이제 --start는 "반드시 맞아야 하는 값"이 아니라 참고용 힌트가 됨(어긋나면 경고만).
 * ============================================================ */
typedef struct {
    double x, y, theta, score;
} PoseCandidate;
double slam_score_pose_lf(const OccupancyGridSLAM *s, const double *field,
                            double x, double y, double theta, const LidarScan *scan);
bool slam__cell_is_free(const OccupancyGridSLAM *s, double wx, double wy);
PoseCandidate slam__search_poses(const OccupancyGridSLAM *s, const double *field,
                                   const LidarScan *scan,
                                   double x0, double x1, double pos_step,
                                   double y0, double y1,
                                   double th0_deg, double th1_deg, double th_step_deg,
                                   bool skip_occupied);
void slam_global_localize(const OccupancyGridSLAM *s, const LidarScan *scan,
                            double hint_x, double hint_y, bool have_hint,
                            double *out_x, double *out_y, double *out_theta);

#endif /* ROBOT_LOCALIZE_H */
