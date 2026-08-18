#include "robot_motion.h"
#include "robot_runner.h"


/* 주어진 컨트롤러 속도에서의 제자리 회전 각속도(rad/s).
 *
 * BUGFIX (2026-08-08d): 예전 식은 speed/controller_max_speed 를 그대로 속도비로 썼음:
 *     rate = 2 * (speed/80) * 0.165 / 0.160
 * 그런데 실제로 모터에 나가는 값은 speed_to_pwm()이 '데드존 재매핑'을 거친 PWM임:
 *     PWM = PWM_DEADZONE + speed/80 * (pwm_safety_cap - PWM_DEADZONE)
 *         = 170 + 34/80 * (255-170) = 206      (speed=34 기준)
 * 즉 speed 34는 "최대의 42.5%"가 아니라 "PWM의 80.8%"임. 그래서 예전 모델은
 * 실제 회전속도를 약 절반으로 과소평가하고 있었음.
 *
 * 왜 이제서야 문제가 됐나: 예전에는 회전 스텝 시간이 sg_move_sec_min(0.07s)에 눌려
 * 있어서 이 모델이 클램프에 한 번도 안 걸렸음. 2026-08-08c 패치로 스텝 시간을
 * 클램프 한계까지 늘리자마자, "10.8도 돌 것"이라고 계산한 215ms 스텝이 실제로는
 * 20도 넘게 돌아서 스캔매칭 탐색창(±12도)을 넘겨버렸음
 * -> 매칭이 창 가장자리에서 포화 -> 추정 각도가 회전을 못 따라감
 * -> GUI에서 맵이 로봇과 같이 회전 (--no-encoder 인데도).
 *
 * 이제 실제 나가는 PWM 기준으로 계산함. 모터 특성(무부하 vs 부하, 스크럽 마찰)까지는
 * 모델링할 수 없으므로, 아래 RotRateEst 로 주행 중 실측해서 계속 보정함. */
double robot_runner__rotation_rate(const RobotConfig *cfg, int speed) {
    int s = abs(speed);
    if (s < 1) s = 1;
    char sign;
    int pwm;
    speed_to_pwm(s, cfg->controller_max_speed, cfg->pwm_safety_cap, &sign, &pwm);
    double duty = (double)pwm / (double)cfg->pwm_safety_cap;
    if (duty <= 0.0) duty = 1.0 / (double)cfg->pwm_safety_cap;
    double wheel_mps = duty * ROBOT_MAX_WHEEL_MPS;
    return 2.0 * wheel_mps / cfg->wheel_separation;
}


/* ============================================================
 * BUGFIX (2026-08-09a): 선속도 모델에도 PWM 데드존이 빠져 있었음
 *
 * 회전 각속도는 2026-08-08d에 speed_to_pwm() 기준으로 고쳤는데, "몇 mm를 밀지"를
 * 이동시간으로 환산하는 두 곳(끼임 탈출 / 깔때기 후진)은 여전히 옛 모델을 쓰고 있었음:
 *     v = speed / controller_max_speed * ROBOT_MAX_WHEEL_MPS
 * 실제로 모터에 나가는 값은
 *     PWM = 170 + speed/80 * (255-170)      (speed=34 -> 206, speed=28 -> 200)
 * 이므로 실제 속도는 위 모델의 약 1.9~2.2배임.
 *
 * 결과(= 사용자가 본 증상 그대로):
 *   - "12cm만 빠져나온다"는 탈출이 실제로는 23cm를 가서 반대편 벽에 그대로 박음
 *     -> 탈출한다면서 오히려 더 심하게 걸리거나, 주차 준비자세에서 한참 멀어짐
 *   - "15mm만 물러난다"는 깔때기 복구가 실제로는 30mm 이상 물러나 깊이를 계속 까먹음
 * 두 곳 모두 이 함수를 쓰도록 바꿈.
 * ============================================================ */
double robot_runner__linear_speed(const RobotConfig *cfg, int speed) {
    int s = abs(speed);
    if (s < 1) s = 1;
    char sign;
    int pwm;
    speed_to_pwm(s, cfg->controller_max_speed, cfg->pwm_safety_cap, &sign, &pwm);
    double duty = (double)pwm / (double)cfg->pwm_safety_cap;
    if (duty <= 0.0) duty = 1.0 / (double)cfg->pwm_safety_cap;
    return duty * ROBOT_MAX_WHEEL_MPS;
}


/* ============================================================
 * 선속도 자가보정 (2026-08-09c) - 실기 로그에서 잡힌 2.2배 오차
 *
 * 실측 로그:
 *     "후진으로 78mm 빠져나갑니다 (0.59s)"  ->  pose y 0.465 -> 0.640 = 실제 175mm
 *     "전진으로 67mm 빠져나갑니다 (0.46s)"  ->  pose y 0.640 -> 0.470 = 실제 170mm
 * 즉 명령한 거리의 2.2~2.5배를 감. 데드존 재매핑은 이미 반영했으므로 남은 원인은
 * ROBOT_MAX_WHEEL_MPS(0.165) 상수 자체가 이 차량과 안 맞는 것뿐임
 * (실제로는 0.37 m/s 근처로 보임).
 *
 * 회전 각속도는 이미 주행 중 실측으로 보정하고 있는데(rot_rate_*), 선속도만
 * 상수를 그대로 믿고 있었음. 같은 방식으로 실측 보정을 넣는다.
 *
 * 측정 방법: 탈출 직선이동 전후로 '진행방향 부채꼴 최소거리'를 재면, 그 차이가
 * 곧 실제로 간 거리다. SLAM이 필요 없고 벽 하나만 보이면 되므로 어디서든 쓸 수 있다.
 * 비율은 [0.3, 4.0]으로 제한하고 EMA로 천천히 반영한다(한 번의 오측정에 안 흔들리게).
 * ============================================================ */
double *robot_runner__lin_scale(void) {
    static double scale = 1.0;   /* 모델 대비 실제 속도 배율 */
    return &scale;
}


/* 배율이 실측으로 몇 번 갱신됐는지. 아직 덜 익었으면 좁은 곳에서 보수적으로 움직인다. */
int *robot_runner__lin_scale_n(void) {
    static int n = 0;
    return &n;
}


void robot_runner__lin_scale_update(double commanded_m, double actual_m) {
    if (commanded_m < 0.010 || actual_m < 0.002) return;   /* 너무 짧으면 잡음 */
    double ratio = actual_m / commanded_m;
    if (ratio < 0.4 || ratio > 3.0) return;                /* 오측정으로 판단 */
    (*robot_runner__lin_scale_n())++;
    double *s = robot_runner__lin_scale();
    double before = *s;
    /* 천천히 반영. 한 번의 오측정으로 배율이 튀면 이동시간이 짧아져서 오히려
     * 정지마찰을 못 이기는 쪽으로 망가지므로, 상한을 3.0으로 조인다 (2026-08-09d) */
    *s = 0.8 * (*s) + 0.2 * ((*s) * ratio);
    if (*s < 0.5) *s = 0.5;
    if (*s > 3.0) *s = 3.0;
    if (fabs(*s - before) > 0.05) {
        printf("[main] 선속도 보정: 명령 %.0fmm -> 실제 %.0fmm (x%.2f), "
               "배율 %.2f -> %.2f\n",
               commanded_m * 1000.0, actual_m * 1000.0, ratio, before, *s);
        fflush(stdout);
    }
}


/* ============================================================
 * 주행 중 선속도 배율 학습 (2026-08-10 신규)
 *
 * 위 lin_scale_update() 는 '끼임 탈출'에서만 호출된다. 탈출이 한 번도 안 일어나면
 * 배율이 영영 1.0 에 머무는데, 이 차량의 참값은 2.0 근처다(로그에서 확인).
 * 그러면 명령 적분 예측이 실제의 절반이 되고, 스캔매칭이 매번 나머지 절반을
 * '도약'으로 메꿔야 해서 운동 타당성 게이트와 싸우게 된다.
 *
 * 주행 중에는 매 스텝 '예측한 이동량'과 '스캔매칭이 본 이동량'을 둘 다 알고 있으므로
 * 그 비율로 배율을 배우면 된다. 탈출 때의 부채꼴 거리 측정보다 표본이 훨씬 많고
 * 조건도 깨끗하다(직진 스텝, 매칭 품질 양호, 게이트 미작동일 때만 학습).
 * ============================================================ */
void robot_runner__lin_scale_observe(double predicted_m, double observed_m) {
    if (predicted_m < 0.020) return;            /* 너무 짧으면 잡음이 지배 */
    double ratio = observed_m / predicted_m;
    if (ratio < 0.3 || ratio > 3.5) return;     /* 오측정으로 판단 */
    double *sc = robot_runner__lin_scale();
    double before = *sc;
    *sc = (*sc) * (0.85 + 0.15 * ratio);        /* 천천히 반영 */
    if (*sc < 0.5) *sc = 0.5;
    if (*sc > 3.0) *sc = 3.0;
    (*robot_runner__lin_scale_n())++;
    if (fabs(*sc - before) > 0.08) {
        printf("[main] 선속도 배율(주행 중 학습): 예측 %.0fmm -> 실측 %.0fmm (x%.2f), "
               "배율 %.2f -> %.2f\n",
               predicted_m * 1000.0, observed_m * 1000.0, ratio, before, *sc);
        fflush(stdout);
    }
}



/* 거리/속도 -> 이동 시간. 최소·최대 클램프까지 한 곳에 모았다 (2026-08-11 리팩토링).
 * move_sec_for() 와 move_sec_for_safe() 가 이 네 줄을 그대로 복사해 쓰고 있었다.
 * 둘의 진짜 차이는 속도 v 에 곱하는 guard 뿐이므로 v 를 인자로 받는다. */
static double robot_runner__move_sec_at(const RobotConfig *cfg, double dist_m,
                                        double cap_sec, double v) {
    double sec = (v > 1e-6) ? (dist_m / v) : 0.0;
    if (sec <= 0.0) return 0.0;
    if (sec < cfg->sg_move_sec_min) sec = cfg->sg_move_sec_min;
    if (cap_sec > 0.0 && sec > cap_sec) sec = cap_sec;
    return sec;
}

/* 거리(m)를 이동시간(초)으로. cap_sec 가 0보다 크면 상한으로 씀.
 *
 * BUGFIX (2026-08-09d): 정지마찰 하한이 빠져 있었음.
 * 선속도 자가보정으로 배율이 2.7까지 올라가면 15mm 이동이 0.04초가 되는데,
 * 그건 킥(20ms)+유지(20ms) 수준이라 바퀴가 정지마찰을 못 이기고 그대로 멈춰 있음.
 * "명령은 나갔는데 물리적으로 하나도 안 움직이는" 상태가 됨 - fpga_link.h 의
 * PWM 데드존 주석과 정확히 같은 함정이다. 시간축에도 같은 하한을 둔다. */
double robot_runner__move_sec_for(const RobotConfig *cfg, int speed,
                                                 double dist_m, double cap_sec) {
    return robot_runner__move_sec_at(cfg, dist_m, cap_sec,
                                     robot_runner__linear_speed(cfg, speed)
                                         * (*robot_runner__lin_scale()));
}


/* ============================================================
 * 좁은 곳 전용 보수적 환산 (2026-08-10 신규)
 *
 * 왜 필요한가 (= 사용자 신고 "너무 확확 움직여서 오히려 더 나빠진다"):
 * 실기 로그에서 탈출 이동은 명령의 1.4~1.7배가 나갔다.
 *     명령 120mm -> 실제 178mm (x1.48)
 *     명령  72mm -> 실제 124mm (x1.72)
 * 배율 자가보정이 1.10 -> 1.21 -> 1.37 -> 1.50 -> 1.62 로 '수렴하는 중'이었기
 * 때문이다(참값은 2.0 부근). 수렴 자체는 정상이지만, 수렴하는 동안 매번
 * 50% 씩 더 나가버리는 게 문제다. 벽 앞 40mm 여유를 두고 밀었는데 60mm 를 가면
 * 그대로 들이받고, 그 충격으로 자세가 틀어져서 또 탈출을 해야 한다 - 무한 왕복.
 *
 * 좁은 곳에서 넘치는 것과 모자라는 것은 대가가 전혀 다르다. 모자라면 한 번 더
 * 밀면 그만이지만, 넘치면 벽에 박는다. 그래서 배율이 덜 익었을 때는 일부러
 * '실제가 더 빠를 것'이라고 가정해 시간을 짧게 준다.
 * ============================================================ */
double robot_runner__move_sec_for_safe(const RobotConfig *cfg, int speed,
                                                      double dist_m, double cap_sec) {
    int n = *robot_runner__lin_scale_n();
    double guard = (n >= 3) ? 1.15 : 1.80;
    return robot_runner__move_sec_at(cfg, dist_m, cap_sec,
                                     robot_runner__linear_speed(cfg, speed)
                                         * (*robot_runner__lin_scale()) * guard);
}


/* ============================================================
 * GUI 텔레메트리 (2026-08-09a)
 *
 * 문제: STATE/SCAN 줄을 navigate_one_leg() 와 slot_drive() 에서만 찍고 있었음.
 * 그래서 slot_align() / slot_lateral_align() / 끼임 탈출처럼 "제자리에서 오래
 * 돌고 있는" 구간에서는 GUI로 아무것도 안 나감 -> 로봇은 잘 움직이는데 화면만
 * 멈춰 보임(사용자 신고 증상). 정렬은 최대 40스텝 x 0.5초라 20초 넘게 침묵할 수 있고,
 * 좌우 정렬까지 들어가면 몇 분씩 멈춰 보임.
 *
 * 이제 "관측 -> 위치추정"을 한 모든 곳에서 이 함수를 부른다. phase 문자열만 다르고
 * 형식은 기존 STATE 줄과 완전히 동일하므로 GUI 파서 형식은 그대로다
 * (다만 live_shuttle_gui.py 가 kind 를 mapping/navigate 만 받고 있었으므로 같이 고침).
 * ============================================================ */
void robot_runner__emit_state(const char *phase, int step,
                                             double x, double y, double theta,
                                             const LidarScan *scan) {
    printf("STATE %s %d %.4f %.4f %.4f 0 0 0\n", phase, step, x, y, theta);
    if (scan && scan->count > 0) {
        printf("SCAN %d", scan->count);
        for (int i = 0; i < scan->count; i++) {
            double wa = theta + scan->readings[i].angle_deg * ANGLE_PI / 180.0;
            double d = scan->readings[i].dist_mm / 1000.0;
            printf(" %.3f %.3f", x + d * cos(wa), y + d * sin(wa));
        }
        printf("\n");
    }
    fflush(stdout);   /* 파이프는 블록버퍼라 이게 없으면 GUI가 뭉텅이로 늦게 받음 */
}



/* 탈출용 '천천히' 속도. PWM = 170 + speed/80*85 이므로
 *   speed 34 -> PWM 206 (기존, 빠름) / speed 22 -> PWM 193 (느리지만 데드존 위)
 * 데드존(170) 바로 위인 171~180 은 실기에서 거의 안 도는 것이 확인됐으므로
 * 그보다는 충분히 위로 잡는다. 그래도 안 움직이면 아래에서 원래 속도로 재시도한다. */
int robot_runner__escape_speed(const RobotConfig *cfg) {
    int sp = (cfg->sg_step_speed * 2) / 3;
    if (sp < cfg->min_arc_speed + 4) sp = cfg->min_arc_speed + 4;
    if (sp > cfg->sg_step_speed) sp = cfg->sg_step_speed;
    return sp;
}
