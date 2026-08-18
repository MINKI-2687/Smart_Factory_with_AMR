/* ============================================================
 * test_encoder_ab.c - 엔코더 유무에 따른 "제자리 회전 추적 성능" A/B 테스트
 *
 * 무엇을 재는가
 * -------------
 * 실기에서 나온 증상 "90도만 돌면 되는데 270도를 돌아버림"의 원인이
 * '스캔매칭 각도 탐색창(±12도) < 한 스텝 회전량(최대 22.6도)' 인지 확인하고,
 * (a) 엔코더 추가, (b) 회전 스텝 클램프 두 대책의 효과를 각각 분리해서 측정함.
 *
 * 어떻게
 * ------
 *  - 실제 지도 파일(set_map.txt)을 그대로 읽어서, 그 격자에 레이캐스팅해서
 *    라이다 스캔을 합성함(각도 해상도/노이즈/최대거리를 실기와 비슷하게).
 *  - slot_align()과 똑같은 제어식으로 제자리 회전을 시킴
 *    (각속도 = 2*speed*0.165/80/0.160, move_sec을 오차에 비례).
 *  - 매 스텝 slam_localize_step()에 오도메트리를 넣거나(엔코더 O) 0을 넣고(엔코더 X)
 *    "추정 각도"와 "진짜 각도"가 얼마나 벌어지는지, 그리고 제어기가 언제 멈추는지를 봄.
 *  - 엔코더 모델에는 실제 하드웨어 한계를 넣음:
 *      * 분해능 20틱/rev -> 차동회전 7.43도/틱 (양자화)
 *      * 제자리 회전 시 바퀴 슬립(기본 15%)
 *
 * 이 테스트가 알려주지 못하는 것 (중요)
 * ------------------------------------
 *  - 지도 정합도. 여기선 "지도 == 실물"이라고 가정하므로, 폼보드가 우그러져서
 *    생기는 편향(bias)은 절대 안 나타남. 이 테스트에서 오차 0이 나와도 실기의
 *    절대 정확도가 보장되는 게 아님.
 *  - 라이다 스캔 왜곡(회전 중 1회전 102ms 동안 뭉개짐), 모터 비선형/데드존,
 *    통신 지연/유실.
 * 즉 이건 "추정기와 제어기의 상호작용"만 떼어내서 보는 시험대임. 마침 270도 증상이
 * 딱 그 범주라서 유효함.
 *
 * 빌드:  gcc -std=c11 -O2 -o test_encoder_ab test_encoder_ab.c -lm
 * 실행:  ./test_encoder_ab [맵파일]      (기본 set_map.txt)
 * ============================================================ */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "angle_utils.h"
#include "slam.h"
#include "map_io.h"

/* ---- 실기 설정과 동일하게 맞춘 상수들 ---- */
#define SPEED            34      /* sg_step_speed */
#define CONTROLLER_MAX   80
#define MAX_WHEEL_MPS    0.165
#define WHEEL_SEP        0.160
#define WHEEL_DIAM       0.066
#define TICKS_PER_REV    20
#define MOVE_SEC_MIN     0.07
#define MOVE_SEC_MAX     0.45
#define SEARCH_WIN_M     0.15
#define SEARCH_WIN_DEG   12.0
#define LIDAR_NOISE_MM   30.0    /* 1시그마 */
#define LIDAR_BEAMS      360
#define ALIGN_TOL_DEG    1.0     /* cfg->slot_align_tol_deg 와 동일 */
#define N_TRIALS         20      /* 노이즈 시드를 바꿔가며 반복 - 1회 결과는 운에 좌우됨 */
#define LIDAR_MAX_M      3.0

static double rot_rate(void) {   /* rad/s */
    return 2.0 * SPEED * MAX_WHEEL_MPS / CONTROLLER_MAX / WHEEL_SEP;
}

/* ---- 재현 가능한 난수 (테스트마다 같은 노이즈를 써야 A/B 비교가 공정함) ---- */
static unsigned long g_rng = 12345;
static double urand(void) {
    g_rng = g_rng * 6364136223846793005ULL + 1442695040888963407ULL;
    return (double)((g_rng >> 33) & 0x7FFFFFFF) / (double)0x7FFFFFFF;
}
static double gauss(void) {
    double u1 = urand(), u2 = urand();
    if (u1 < 1e-12) u1 = 1e-12;
    return sqrt(-2.0 * log(u1)) * cos(2.0 * ANGLE_PI * u2);
}

/* ---- 격자에 레이캐스팅해서 스캔 합성 ---- */
static double grid_raycast(const unsigned char *g, int rows, int cols, double res,
                            double x, double y, double ang) {
    double step = res * 0.4;
    double dx = cos(ang) * step, dy = sin(ang) * step;
    double px = x, py = y;
    for (double d = 0.0; d < LIDAR_MAX_M; d += step) {
        px += dx; py += dy;
        int c = (int)(px / res), r = (int)(py / res);
        if (r < 0 || r >= rows || c < 0 || c >= cols) return d;
        if (g[r * cols + c]) return d;
    }
    return LIDAR_MAX_M;
}

static void synth_scan(LidarScan *scan, const unsigned char *g, int rows, int cols,
                        double res, double x, double y, double theta,
                        double beam_keep, double noise_mult) {
    scan->count = 0;
    for (int i = 0; i < LIDAR_BEAMS && scan->count < LIDAR_MAX_READINGS; i++) {
        double a_deg = -180.0 + 360.0 * i / LIDAR_BEAMS;
        double d = grid_raycast(g, rows, cols, res, x, y, theta + a_deg * ANGLE_PI / 180.0);
        if (d >= LIDAR_MAX_M - 1e-6) continue;          /* 미탐지 빔은 버림 */
        if (urand() > beam_keep) continue;              /* 스캔 품질 저하 모델 */
        double d_mm = d * 1000.0 + gauss() * LIDAR_NOISE_MM * noise_mult;
        if (d_mm < 50.0) continue;
        scan->readings[scan->count].angle_deg = a_deg;
        scan->readings[scan->count].dist_mm = d_mm;
        scan->count++;
    }
}

typedef struct {
    bool   has_encoder;
    bool   clamp_rotation;
    double clamp_frac;        /* 탐색창 대비 허용 스텝 비율 */
    double slip;              /* 제자리 회전 시 바퀴 슬립 비율 (0.15 = 15%) */
    double beam_keep;         /* 스캔 품질: 살아있는 빔 비율 (1.0 = 정상) */
    double noise_mult;        /* 라이다 노이즈 배수 */
    const char *name;
} Scenario;

typedef struct {
    bool   converged;
    int    steps;
    double true_rotated_deg;   /* 몸통이 실제로 돈 총 각도(절대값 누적) */
    double final_err_deg;      /* 목표 대비 실제 자세 오차 */
    double max_est_err_deg;    /* 추정각 vs 실제각 최대 괴리 */
    double last_cmd_rot;       /* 직전 스텝에 실제로 돈 양(rad) - 오도메트리 입력용 */
} Result;

static Result run_case(const unsigned char *grid, int rows, int cols, double res,
                        double start_x, double start_y,
                        double start_theta, double goal_theta, const Scenario *sc,
                        unsigned long seed) {
    Result out = {0};
    g_rng = seed;   /* 시나리오 간 비교는 같은 시드로 - 공정한 A/B */

    OccupancyGridSLAM slam;
    slam_init_from_grid(&slam, grid, rows, cols, res, 0.0, 0.0,
                         start_x, start_y, start_theta,
                         SEARCH_WIN_M, SEARCH_WIN_DEG, 0.03, 3.0);

    double true_theta = start_theta;
    double tick_residual = 0.0;                  /* 엔코더 양자화 잔차 */
    double quantum = 2.0 * (ANGLE_PI * WHEEL_DIAM / TICKS_PER_REV) / WHEEL_SEP;  /* rad/tick */
    const int MAX_STEPS = 60;

    for (int step = 0; step < MAX_STEPS; step++) {
        LidarScan scan;
        synth_scan(&scan, grid, rows, cols, res, start_x, start_y, true_theta,
                    sc->beam_keep, sc->noise_mult);

        /* 오도메트리 델타: 직전 스텝에서 실제로 돈 양을 엔코더가 본 값 */
        double odom_dth = 0.0;
        if (sc->has_encoder) {
            double measured = out.last_cmd_rot * (1.0 - sc->slip) + tick_residual;
            double n = trunc(measured / quantum);           /* 틱은 정수 단위로만 보임 */
            odom_dth = n * quantum;
            tick_residual = measured - odom_dth;
        }

        double ex, ey, eth;
        slam_localize_step(&slam, &scan, 0.0, 0.0, odom_dth, &ex, &ey, &eth);

        double est_gap = fabs(normalize_angle(eth - true_theta)) * 180.0 / ANGLE_PI;
        if (est_gap > out.max_est_err_deg) out.max_est_err_deg = est_gap;

        /* --- slot_align과 동일한 제어 --- */
        double err = normalize_angle(eth - goal_theta);
        if (fabs(err) * 180.0 / ANGLE_PI <= ALIGN_TOL_DEG) {
            out.converged = true;
            out.steps = step;
            break;
        }
        double sec = fabs(err) / rot_rate();
        if (sec < MOVE_SEC_MIN) sec = MOVE_SEC_MIN;
        if (sec > MOVE_SEC_MAX) sec = MOVE_SEC_MAX;
        if (sc->clamp_rotation) {
            double frac = sc->clamp_frac;
            double max_sec = (SEARCH_WIN_DEG * ANGLE_PI / 180.0) * frac / rot_rate();
            if (max_sec < MOVE_SEC_MIN) max_sec = MOVE_SEC_MIN;
            if (sec > max_sec) sec = max_sec;
        }

        double rot = rot_rate() * sec * (err > 0 ? -1.0 : 1.0);   /* err>0이면 시계로 */
        rot *= (1.0 - sc->slip);                                   /* 슬립으로 덜 돎 */
        true_theta = normalize_angle(true_theta + rot);
        out.true_rotated_deg += fabs(rot) * 180.0 / ANGLE_PI;
        out.last_cmd_rot = rot;
        out.steps = step + 1;
    }

    out.final_err_deg = fabs(normalize_angle(true_theta - goal_theta)) * 180.0 / ANGLE_PI;
    slam_free(&slam);
    return out;
}

int main(int argc, char **argv) {
    const char *map_path = (argc > 1) ? argv[1] : "set_map.txt";
    int rows, cols; double res;
    unsigned char *grid = map_load(map_path, &rows, &cols, &res);
    if (!grid) { fprintf(stderr, "맵 로드 실패: %s\n", map_path); return 1; }
    printf("맵: %s (%dx%d, %.3fm/셀 = %.2f x %.2f m)\n\n",
           map_path, rows, cols, res, cols * res, rows * res);

    printf("회전 각속도 = %.3f rad/s (%.1f deg/s)\n", rot_rate(), rot_rate() * 180 / ANGLE_PI);
    printf("한 스텝 최대 회전 = %.1f deg (move_sec_max %.2fs)\n",
           rot_rate() * MOVE_SEC_MAX * 180 / ANGLE_PI, MOVE_SEC_MAX);
    printf("스캔매칭 탐색창 = ±%.1f deg\n", SEARCH_WIN_DEG);
    printf("엔코더 분해능 = %.2f deg/틱\n\n",
           2.0 * (ANGLE_PI * WHEEL_DIAM / TICKS_PER_REV) / WHEEL_SEP * 180 / ANGLE_PI);

    /* A지점 슬롯 대기점 근처에서 목표 자세(-90도)로 돌리는 상황 */
    double sx = 0.225, sy = 0.65;
    double goal = -90.0 * ANGLE_PI / 180.0;

    /* 표1: 라이다 스캔이 깨끗한 이상적 조건 */
    Scenario clean[] = {
        { false, false, 0.6, 0.15, 1.00, 1.0, "엔코더X, 클램프X (원래코드)" },
        { false, true,  0.6, 0.15, 1.00, 1.0, "엔코더X, 클램프O(0.6)" },
        { true,  false, 0.6, 0.15, 1.00, 1.0, "엔코더O, 클램프X" },
        { true,  true,  1.5, 0.15, 1.00, 1.0, "엔코더O, 클램프O(frac 1.5)" },
        { true,  true,  1.0, 0.15, 1.00, 1.0, "엔코더O, 클램프O(frac 1.0)" },
        { true,  true,  0.8, 0.15, 1.00, 1.0, "엔코더O, 클램프O(frac 0.8)" },
        { true,  true,  0.6, 0.15, 1.00, 1.0, "엔코더O, 클램프O(frac 0.6)" },
    };

    /* 표2: 스캔 품질이 나쁜 조건 - 빔 25%만 살고 노이즈 3배.
     * 슬롯 안처럼 보이는 게 벽뿐이거나, 벽을 잘라서 특징이 줄었거나,
     * 라이다가 첫 회전을 못 끝낸 초반 구간을 흉내낸 것. */
    Scenario degraded[] = {
        { false, true,  0.6, 0.15, 0.25, 3.0, "엔코더X, 클램프O" },
        { true,  true,  0.6, 0.15, 0.25, 3.0, "엔코더O, 클램프O" },
    };

    double starts[] = { 0.0, 90.0, 179.0 };   /* 목표까지 90 / 180 / 269도 */

    for (int table = 0; table < 2; table++) {
        Scenario *cases = table == 0 ? clean : degraded;
        size_t n_cases = table == 0 ? sizeof(clean)/sizeof(clean[0])
                                     : sizeof(degraded)/sizeof(degraded[0]);
        printf("############ 표%d: %s ############\n\n", table + 1,
               table == 0 ? "라이다 스캔 정상" : "라이다 스캔 열화 (빔 25%, 노이즈 3배)");

        for (size_t si = 0; si < sizeof(starts)/sizeof(starts[0]); si++) {
            double st = starts[si] * ANGLE_PI / 180.0;
            double need = fabs(normalize_angle(st - goal)) * 180.0 / ANGLE_PI;
            printf("=== 시작 %.0fdeg -> 목표 -90deg (최단 %.0fdeg) ===\n", starts[si], need);
            printf("%-30s %8s %8s %9s %10s\n",
                   "시나리오", "실제회전", "평균오차", "최악오차", "실패(>30도)");
            for (size_t ci = 0; ci < n_cases; ci++) {
                double sum_rot = 0, sum_err = 0, worst = 0;
                int n_bad = 0;
                for (int t = 0; t < N_TRIALS; t++) {
                    Result r = run_case(grid, rows, cols, res, sx, sy, st, goal,
                                         &cases[ci], 1000u + 7919u * (unsigned long)t);
                    sum_rot += r.true_rotated_deg;
                    sum_err += r.final_err_deg;
                    if (r.final_err_deg > worst) worst = r.final_err_deg;
                    if (r.final_err_deg > 30.0) n_bad++;
                }
                printf("%-30s %8.0f %8.1f %9.1f %8d/%d\n",
                       cases[ci].name, sum_rot / N_TRIALS, sum_err / N_TRIALS,
                       worst, n_bad, N_TRIALS);
            }
            printf("\n");
        }
    }

    free(grid);
    return 0;
}
