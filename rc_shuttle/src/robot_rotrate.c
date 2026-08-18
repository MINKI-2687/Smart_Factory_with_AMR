#include "robot_rotrate.h"
#include "robot_runner.h"


void rot_rate_init(RotRateEst *e, const RobotConfig *cfg, int speed) {
    e->rate = robot_runner__rotation_rate(cfg, speed);
    e->rate_model = e->rate;
    /* 물리 상한: PWM을 최대로 줘도 이보다 빠를 수 없음. 모델 불확실성 2배까지 허용. */
    e->rate_max = 2.0 * (2.0 * ROBOT_MAX_WHEEL_MPS / cfg->wheel_separation);
    /* 하한 0.2 -> 0.4 (2026-08-08g): 실기에서 벽에 걸려 1~2도만 돈 스텝들을 학습해
     * 추정이 하한(19deg/s)까지 떨어졌고, 그 값으로 계산한 '최소 회전 스텝'이 1.3도가
     * 되는 바람에 129도나 남은 상태를 '하드웨어 한계'로 오판했음. */
    e->rate_min = 0.4 * e->rate;
    if (e->rate_min < 1e-3) e->rate_min = 1e-3;
    e->measured = false;
    e->n_update = 0;
}


void rot_rate__clip(RotRateEst *e) {
    if (e->rate < e->rate_min) e->rate = e->rate_min;
    if (e->rate > e->rate_max) e->rate = e->rate_max;
}


double rot_rate_get(const RotRateEst *e) {
    return (e->rate > 1e-3) ? e->rate : 1e-3;
}

RotRateEst g_rot_est;
bool g_rot_est_ready = false;
RotRateEst *rot_rate_shared(const RobotConfig *cfg, int speed) {
    if (!g_rot_est_ready) { rot_rate_init(&g_rot_est, cfg, speed); g_rot_est_ready = true; }
    return &g_rot_est;
}


/* 한 번의 최소 이동시간으로 낼 수 있는 회전량(도). 이보다 작은 각도오차는 이 하드웨어로
 * 물리적으로 못 만듦 - 정지마찰 때문에 sg_move_sec_min 보다 짧게 줄 수 없기 때문. */
double rot_rate_min_step_deg(const RotRateEst *e, const RobotConfig *cfg) {
    return rot_rate_get(e) * cfg->sg_move_sec_min * 180.0 / ANGLE_PI;
}


/* 회전 스텝 직후 호출.
 *   observed_rad  : 스캔매칭이 본 자세변화의 절대값
 *   commanded_sec : 그 스텝에 실제로 준 이동시간
 *   window_rad    : 스캔매칭 각도 탐색창(단측) */
void rot_rate_update(RotRateEst *e, double observed_rad_signed,
                                    double predicted_rad_signed,
                                    double commanded_sec, double window_rad,
                                    double commanded_dir) {
    double observed_rad = fabs(observed_rad_signed);
    if (commanded_sec < 0.02 || window_rad <= 0.0) return;

    /* ============================================================
     * 포화는 '스캔매칭 보정량'으로 판단한다 (2026-08-10q)  *** 엔코더 장착 대응 ***
     *
     * 탐색창은 '예측자세 주변 ±window' 를 훑는 범위다. 즉 창에서 잘릴 수 있는 것은
     * 전체 회전량이 아니라 '예측과 실제의 차이(보정량)' 뿐이다.
     *   보정량 = 관측된 자세변화 - 오도메트리가 예측한 자세변화
     *
     * 엔코더가 없을 때는 예측이 명령적분(모델)이라 보정량이 곧 회전량에 가까워서
     * 둘을 구분할 필요가 없었다. 그런데 엔코더를 붙이면
     *   - clamp_rotation_sec_rate 의 안전계수가 0.7 -> 2.0 이 되어
     *     한 스텝에 창의 2배(50도)까지 돈다
     *   - 예측(엔코더)이 실제와 거의 같으므로 보정량은 몇 도뿐이다
     * 이 상태에서 '관측 >= 창의 80%' 로 포화를 판정하면, 정상적으로 잘 돈
     * 50도짜리 스텝이 전부 '포화'로 오분류되어 진짜 학습(정상 경로)이 통째로
     * 건너뛰어진다. 각속도 추정이 모델값에 붙박이는 셈이다.
     *
     * 보정량으로 보면 두 설정 모두에서 옳다.
     *   엔코더 없음: 예측이 틀리면 보정량이 커져 -> 포화 감지 O
     *   엔코더 있음: 예측이 맞으면 보정량이 작아 -> 정상 학습 O
     * ============================================================ */
    double corr_rad = fabs(normalize_angle(observed_rad_signed - predicted_rad_signed));

    /* ============================================================
     * 가짜 관측은 '부호'로 걸러낸다 (2026-08-10n)  *** 2026-08-10m 의 정정 ***
     *
     * 배경 - 2026-08-10m 에서 무엇을 잘못 고쳤나:
     *   실기에서 각속도가 63 -> 70 -> 85deg/s 로 부풀던 문제를 잡으려고
     *   "명령한 회전량이 창의 90% 이상일 때만 포화로 인정" 이라는 조건을 걸었다.
     *   그런데 --no-encoder 에서 clamp_rotation_sec_rate() 의 안전계수는 0.7 이라
     *   명령 회전량은 '항상' 창의 70% 다. 즉 그 조건은 영원히 거짓이고,
     *   포화 보정 경로가 통째로 죽어버렸다.
     *
     *   포화 보정은 각속도 추정치를 '올릴 수 있는 유일한 길'이었다(스윕 학습은
     *   |오차|>=40도 에서만 돈다). 그래서 추정치가 한 번 낮게 내려가면
     *       sec = 창*0.7 / 추정각속도   -> 추정이 낮을수록 길어짐
     *       실제 회전 = 진짜각속도 * sec  -> 창을 넘김
     *   이 되어 스캔매칭이 창 밖으로 나가고, 그 뒤로는 품질 가드 때문에 학습도
     *   막혀서 스스로 못 돌아온다. A->B 통로에서 '맵이 한 번 깨지고 정체' 가 이것이다.
     *
     * 제대로 된 판별 기준:
     *   로봇은 명령한 방향의 반대로 돌 수 없다. 로그의 가짜 포화 스텝들은
     *       [slot] 정렬 중: 각도오차 +106.58deg   (시계로 돌라고 명령)
     *       [slot] 정렬 중: 각도오차 +108.88deg   (반시계로 2.3도 '관측')
     *   처럼 관측 부호가 명령과 반대였다. 그건 회전이 아니라 스캔매칭 점프이거나
     *   벽에 밀린 것이다. 창과 명령량을 비교하는 대신 이 부호만 보면 된다.
     *   부호가 맞는 관측은 (포화든 아니든) 실제 회전이므로 정상적으로 학습한다.
     * ============================================================ */
    if (commanded_dir != 0.0 && observed_rad_signed * commanded_dir < 0.0) {
        if (observed_rad > 0.25 * window_rad) {
            printf("[nav] 회전 관측 %.1fdeg 가 명령과 반대 방향입니다 "
                   "- 회전이 아니라 자세추정 점프로 보고 학습에서 뺍니다\n",
                   observed_rad * 180.0 / ANGLE_PI);
            fflush(stdout);
        }
        return;
    }

    if (corr_rad >= 0.80 * window_rad) {
        /* 포화 의심 - 관측값은 창에서 잘렸을 뿐 실제 회전량은 그 이상이다.
         * "적어도 창만큼은 돌았다"는 하한을 근거로 각속도를 올린다(맹목적 배수가
         * 아니라 관측 하한 기반이라, 상한과 맞물려 발산하지 않고 수렴함). */
        /* '적어도 이만큼은 돌았다'는 하한. 창에서 잘렸으므로 실제 회전량은
         * 관측값 이상이고, 최소한 창만큼은 보정이 필요했다 (2026-08-10q). */
        double lb_rad = (observed_rad > window_rad) ? observed_rad : window_rad;
        double lower_bound = lb_rad / commanded_sec;
        double bumped = lower_bound * 1.2;
        /* 한 번에 왕창 올리지 않는다 (2026-08-10).
         * 예전에는 짧은 스텝(0.07s)에서 창(25도)이 포화하면 곧바로
         * 25/0.07*1.2 = 428deg/s 로 뛰었다(상한 236 에서 잘림). 실기 로그의
         * '149 deg/s' 가 그 흔적이다. 각속도가 과대추정되면 (a) 최소 회전 스텝이
         * 12.4도가 되어 슬롯 정렬 허용오차(2.5도) 안으로 못 들어가고,
         * (b) 명령 적분 예측이 과하게 돌아 자세추정을 끌고 간다.
         * 한 번에 30%까지만 올린다 - 정말 빠르면 몇 번 더 포화하며 수렴한다. */
        double step_cap = e->rate * 1.30;
        if (bumped > step_cap) bumped = step_cap;
        if (bumped > e->rate) e->rate = bumped;
        rot_rate__clip(e);
        e->measured = true;
        e->n_update++;
        printf("[nav] 회전 포화(스캔매칭 보정 %.1fdeg >= 창의 80%%, 관측 %.1fdeg) "
               "-> 각속도 추정 %.0fdeg/s 로 상향\n",
               corr_rad * 180.0 / ANGLE_PI, observed_rad * 180.0 / ANGLE_PI,
               e->rate * 180.0 / ANGLE_PI);
        fflush(stdout);
        return;
    }
    /* 걸려서 거의 못 돈 스텝은 학습에서 제외한다.
     * 예전 기준(탐색창의 5% = 0.6도)은 너무 헐거웠음. 실기 로그에서 벽에 물린 채
     * 1~2도씩만 돈 스텝이 계속 학습돼 추정 각속도가 하한까지 무너졌다.
     * '이번 스텝에 기대한 회전량의 35%도 못 돌았다'면 그건 각속도가 느린 게 아니라
     * 물리적으로 막힌 것이므로 추정에 반영하지 않는다. */
    double expected = e->rate * commanded_sec;
    if (observed_rad < 0.02 || observed_rad < 0.35 * expected) return;

    double meas = observed_rad / commanded_sec;
    /* 물리 검산 (2026-08-10m): 스윕 학습 쪽에만 있던 '모델의 1.6배까지' 상한을
     * 일반 스텝에도 똑같이 건다. 스키드 스티어는 회전할 때 타이어를 옆으로
     * 문질러야 해서 모델을 크게 넘길 수 없다 - 넘겼다면 그건 자세추정 점프다. */
    if (e->rate_model > 1e-6 && meas > 1.6 * e->rate_model) return;
    e->rate = e->measured ? (0.7 * e->rate + 0.3 * meas) : meas;
    rot_rate__clip(e);
    e->measured = true;
    e->n_update++;
    if (e->n_update % 10 == 0) {
        printf("[nav] 회전 각속도 실측 추정: %.0f deg/s (%d회 갱신)\n",
               e->rate * 180.0 / ANGLE_PI, e->n_update);
        fflush(stdout);
    }
}


/* ============================================================
 * 회전 직후 스캔매칭이 무너졌을 때의 자가회복 (2026-08-10n)
 *
 * 호출부들은 'bad_match_streak==0 && 품질>0.60' 일 때만 rot_rate_update() 를
 * 부른다(틀린 자세로 각속도를 배우면 되먹임이 생기므로 옳은 가드다).
 * 그런데 그 가드 때문에 '매칭이 깨진 상태' 에서는 학습이 아예 멈춘다.
 * 매칭이 깨지는 가장 흔한 원인이 바로 '한 스텝에 탐색창보다 크게 돌았다'
 * = '각속도를 실제보다 낮게 잡고 있다' 인데, 정작 그때 추정을 못 고치므로
 * 한 번 깨지면 계속 깨진 채로 간다.
 *
 * 그래서 이 경우에만 쓰는 별도 경로를 둔다. 각속도를 올리면 다음 회전 스텝이
 * 짧아지므로(sec = 창*0.7/각속도), 원인이 무엇이든 스캔매칭에는 항상 안전한
 * 방향이다. 과하게 올라가도 정상 스텝 몇 번이면 실측으로 되돌아온다.
 * ============================================================ */
void rot_rate_on_bad_match(RotRateEst *e, double commanded_sec,
                                          double window_rad) {
    if (commanded_sec < 0.02 || window_rad <= 0.0) return;
    double lower = window_rad / commanded_sec;   /* '적어도 창만큼은 돌았다'는 하한 */
    double bumped = 1.25 * e->rate;
    if (bumped > 1.2 * lower) bumped = 1.2 * lower;
    if (bumped <= e->rate) return;
    e->rate = bumped;
    rot_rate__clip(e);
    e->measured = true;
    printf("[nav] 회전 스텝 뒤 스캔매칭이 깨졌습니다 - 한 스텝을 너무 크게 돈 것으로 보고\n"
           "      각속도 추정을 %.0fdeg/s 로 올려 다음 스텝을 짧게 만듭니다\n",
           e->rate * 180.0 / ANGLE_PI);
    fflush(stdout);
}


/* ============================================================
 * 명령 기반 추측항법 (2026-08-10 신규) *** 2번 문제의 근본 원인 ***
 *
 * 사용자 신고: "가로 통로를 지나다가 맵이 저렇게 회전하고, 그 뒤로 이상해진다."
 *
 * 원인을 코드에서 찾으면 이렇다. --no-encoder 이므로
 *     robot_runner__get_odom_delta() 는 항상 (0, 0, 0) 을 돌려준다.
 * 그래서 스캔매칭에 넘기는 예측자세가 언제나 '직전 자세 그대로', 즉
 * "너는 방금 하나도 안 움직였다" 다. 그런데 실제로는 한 회전 스텝에 최대
 * 17.5도(로그의 '회전 스텝 309ms, 예상 17.5deg')를 돌았다.
 *
 * 여기서 치명적인 건 단순히 '예측이 없다'가 아니다. scan_match_correct_pose()
 * 에는 오도메트리 벌점이 걸려 있다:
 *     penalty = 6.3 * n * dtheta^2
 * dtheta=17.5도(0.305rad) 면 벌점이 0.588n 이고, 점수 만점이 12n 이므로
 * 전체의 4.9% 다. 즉 '진짜로 17.5도 돈 자세'는 '안 돌았다는 자세'를
 * 4.9% 차이로 이겨야만 채택된다. 매칭 품질 차이가 그 정도인 건 흔해서,
 * 매 회전 스텝마다 추정 각도가 실제보다 조금씩 덜 돈 것으로 나왔다.
 *   -> 제어기는 "아직 덜 돌았다"며 같은 방향으로 계속 회전을 명령
 *   -> 몸통은 목표를 지나쳐 계속 돎 (로그: theta 271deg -> 431deg, 11스텝에 160도)
 *   -> 누적 오차가 임계를 넘는 순간 매칭이 엉뚱한 봉우리로 '툭' 넘어감
 *   -> GUI 에서 맵이 통째로 회전한 것처럼 보임
 * 로그의 '[nav] 회전 포화(관측 20.5deg >= 창의 80%)' 가 바로 이 순간의 증거다.
 * 벌점은 원래 '순간이동 방지용 고무줄'인데, 예측이 0 이면 정상적인 회전까지
 * 방해하는 브레이크가 되어버린 것이다.
 *
 * 고침: 엔코더가 없어도 '무엇을 명령했는지'는 우리가 안다. 명령한 좌우 속도와
 * 시간으로 이동량을 추정해 예측자세에 반영한다. 각속도는 이미 주행 중 실측
 * 추정치(RotRateEst)가 있으므로 그걸 쓰고, 선속도는 lin_scale 자가보정을 쓴다.
 * 그러면
 *   - 예측자세가 실제 근처에 놓이므로 벌점이 정상 동작(순간이동만 억제)한다
 *   - 탐색창이 '모델 오차'만 덮으면 되므로 창 포화가 사라진다
 *   - 위 slam.h 의 운동 타당성 게이트가 비로소 안전하게 성립한다
 *     (예측이 0 이면 게이트가 정상 이동까지 막아버리므로, 이 둘은 반드시 한 쌍이다)
 * ============================================================ */
/* 적분기가 모아둔 '부호 있는 PWM x 초'를 실제 이동량으로 환산해 꺼낸다.
 * heading 은 '움직이기 직전'의 방향각(rad) - 스캔매칭 직전이라면 slam->theta 다.
 * 1회용: 꺼내면 적분기가 0으로 비워진다. */
void robot_runner__cmd_odom_take(const RobotConfig *cfg,
                                                const RotRateEst *rot_est,
                                                double heading,
                                                double *out_dx, double *out_dy,
                                                double *out_dth) {
    *out_dx = 0.0; *out_dy = 0.0; *out_dth = 0.0;
    if (cfg == NULL) { fpga_cmd_track_reset(); return; }

    double lps = 0.0, rps = 0.0;          /* PWM x 초 */
    fpga_cmd_track_take(&lps, &rps);

    double cap = (double)cfg->pwm_safety_cap;
    if (cap < 1.0) cap = 255.0;
    double scale = *robot_runner__lin_scale();

    /* 바퀴가 실제로 굴러간 거리(m). linear_speed() 와 완전히 같은 환산식이다:
     *     v = (pwm / pwm_safety_cap) * ROBOT_MAX_WHEEL_MPS * 배율 */
    double dl = (lps / cap) * ROBOT_MAX_WHEEL_MPS * scale;
    double dr = (rps / cap) * ROBOT_MAX_WHEEL_MPS * scale;

    double ds  = 0.5 * (dl + dr);                       /* 전방 이동량 */
    double dth = (dr - dl) / cfg->wheel_separation;     /* 방향각 변화 */

    /* 각속도는 모델보다 실측 추정치가 훨씬 정확하다(로그: 모델 50deg/s vs
     * 실측 88~105deg/s). 모델을 실측 비율로 스케일한다. */
    double model_rate = robot_runner__rotation_rate(cfg, cfg->sg_step_speed);
    if (rot_est != NULL && model_rate > 1e-6) {
        double k = rot_rate_get(rot_est) / model_rate;
        /* 0.2~5.0 -> 0.5~2.0 (2026-08-10). 이 배율은 이제 '예측'에 직접 쓰이므로,
         * 추정기가 한 번 오염되면 예측이 통째로 틀어지고 그 틀린 예측이 다시
         * 매칭을 오염시키는 양의 되먹임이 생긴다. 모델 대비 2배 이상은
         * 물리적으로 나올 수 없으니 거기서 자른다. */
        if (k < 0.5) k = 0.5;
        if (k > 2.0) k = 2.0;
        dth *= k;
    }

    /* 물리적으로 불가능한 값 방어 (한 스텝이 3초를 넘을 수 없게 위에서 막았지만,
     * 혹시 오래 멈춰 있다 온 경우 예측이 폭주하지 않게 한 번 더 자른다) */
    if (ds >  0.60) ds =  0.60;
    if (ds < -0.60) ds = -0.60;
    if (dth >  3.2) dth =  3.2;
    if (dth < -3.2) dth = -3.2;

    double th_mid = heading + 0.5 * dth;                /* 중점 방향각 적분 */
    *out_dx  = ds * cos(th_mid);
    *out_dy  = ds * sin(th_mid);
    *out_dth = dth;
}
