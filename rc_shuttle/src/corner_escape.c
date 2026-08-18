#include "corner_escape.h"
#include "robot_runner.h"


/* ------------------------------------------------------------
 * (1) 차체를 차체 x축 방향으로 밀 때 실제로 갈 수 있는 거리(m).
 *
 * 부채꼴 최소거리와 무엇이 다른가:
 *   부채꼴은 '±25도 안에 있는 점의 최소거리'라, 구석에서는 옆벽 점이 그 안에
 *   들어와 정면 여유를 실제보다 훨씬 작게 만든다. 직진할 때 옆벽은 스치지도
 *   않는데도 그렇다.
 *   이 함수는 차체 반폭(+여유) 띠 안에 있는 점만 본다. 이것이 직진 스윕의
 *   정확한 판정이다.
 * 반환값은 '차체 앞끝에서 장애물까지 남은 거리'이며, 이미 파고들었으면 음수.
 * ------------------------------------------------------------ */
double robot_runner__axis_free_dist(const LidarScan *scan, int dir,
                                                   double half_len, double half_w,
                                                   double side_margin) {
    double best = 9.9;
    double band = half_w + side_margin;
    for (int i = 0; i < scan->count; i++) {
        double d = scan->readings[i].dist_mm / 1000.0;
        if (d <= 0.03) continue;
        double a = scan->readings[i].angle_deg * ANGLE_PI / 180.0;
        double px = d * cos(a), py = d * sin(a);
        if (fabs(py) > band) continue;              /* 옆으로 비켜간다 */
        double along = (dir > 0) ? px : -px;
        if (along <= 0.0) continue;                 /* 진행방향 반대편 점 */
        double room = along - half_len;
        if (room < best) best = room;
    }
    return best;
}


/* ------------------------------------------------------------
 * (2) 'shift_m 만큼 축방향 이동 후 dtheta_rad 만큼 제자리 회전' 의 최악 여유(m).
 *
 * robot_runner__rotation_is_safe() 와 같은 계산인데, 스캔을 복사하지 않고
 * 각 점을 그 자리에서 -shift_m 평행이동해서 본다(로봇이 +shift 가면 장애물은
 * 차체기준으로 -shift 만큼 온다).
 * ------------------------------------------------------------ */
double robot_runner__rot_gap_shifted(const LidarScan *scan, double shift_m,
                                                    double dtheta_rad,
                                                    double half_len, double half_w,
                                                    double *out_dir_deg) {
    const int NSTEP = 8;
    double dt = dtheta_rad / NSTEP;
    double worst = 1e9, worst_dir = 0.0;
    for (int i = 0; i < scan->count; i++) {
        double d = scan->readings[i].dist_mm / 1000.0;
        if (d <= 0.03) continue;
        double a0 = scan->readings[i].angle_deg * ANGLE_PI / 180.0;
        double px = d * cos(a0) - shift_m;
        double py = d * sin(a0);
        double dd = hypot(px, py);
        double a  = atan2(py, px);
        double need = 0.0;
        for (int k = 0; k <= NSTEP; k++) {
            double e = robot_runner__body_reach(a - dt * k, half_len, half_w);
            if (e > need) need = e;
        }
        double gap = dd - need;
        if (gap < worst) { worst = gap; worst_dir = a * 180.0 / ANGLE_PI; }
    }
    if (worst > 1e8) { worst = 9.9; worst_dir = 0.0; }
    if (out_dir_deg) *out_dir_deg = worst_dir;
    return worst;
}


/* ------------------------------------------------------------
 * (3) 회전 여유를 만드는 '가장 작은' 축방향 이동(m). +면 전진, -면 후진.
 *
 * 왜 '가장 큰 여유'가 아니라 '가장 작은 이동'인가:
 *   대기점에서 크게 벗어나면 이어지는 진입 게이트(좌우오차 30mm 초과)에서
 *   slot_lateral_align 이 걸리고, 그건 90도 회전 두 번(약 5.6초)짜리 기동이다.
 *   필요한 여유만 만들고 그 자리에 머무는 편이 언제나 싸다.
 * need_gap 를 만족하는 후보가 아예 없으면, 가능한 범위에서 여유가 가장 커지는
 * 이동을 돌려준다(그것도 도움이 되기는 하므로).
 * ------------------------------------------------------------ */
double robot_runner__best_axis_shift(const LidarScan *scan, double dtheta_rad,
                                                    double half_len, double half_w,
                                                    double back_limit, double fwd_limit,
                                                    double need_gap, double *out_gap) {
    const double STEP = 0.005;
    if (back_limit < 0.0) back_limit = 0.0;
    if (fwd_limit  < 0.0) fwd_limit  = 0.0;

    double base = robot_runner__rot_gap_shifted(scan, 0.0, dtheta_rad, half_len, half_w, NULL);
    double best_s = 0.0, best_gap = base;

    double lim = (back_limit > fwd_limit) ? back_limit : fwd_limit;
    for (double mag = STEP; mag <= lim + 1e-9; mag += STEP) {
        /* 같은 크기면 후진을 먼저 본다 - 슬롯/통로에서 뒤로 빠지는 쪽이
         * 대체로 안전하고, 앞은 벽을 향하고 있을 확률이 높다. */
        for (int k = 0; k < 2; k++) {
            double s = (k == 0) ? -mag : +mag;
            if (s < 0.0 && mag > back_limit) continue;
            if (s > 0.0 && mag > fwd_limit)  continue;
            double g = robot_runner__rot_gap_shifted(scan, s, dtheta_rad,
                                                      half_len, half_w, NULL);
            if (g > best_gap) { best_gap = g; best_s = s; }
            if (g >= need_gap) {                       /* 최소 이동으로 충족 */
                if (out_gap) *out_gap = g;
                return s;
            }
        }
    }
    if (out_gap) *out_gap = best_gap;
    return best_s;
}


/* ------------------------------------------------------------
 * (4) 호(arc) 이동 - 전/후진하면서 동시에 돈다.
 *
 * 제자리 회전은 차체가 반대각선 143mm 짜리 원을 쓸고 지나가지만, 호는 그
 * 회전량을 이동으로 나눠 가지므로 훨씬 좁은 곳에서도 각도를 바꿀 수 있다.
 * 사람이 좁은 주차장에서 하는 '빼면서 꺾기'가 정확히 이것이다.
 *
 * 부호 검산 (미분구동, w = (v_r - v_l)/wheel_sep, +w = 반시계):
 *   전진 반시계 : l=inner,  r=outer   -> v_r - v_l = outer-inner > 0  (w>0) OK
 *   후진 반시계 : l=-outer, r=-inner  -> v_r - v_l = outer-inner > 0  (w>0) OK
 *   전진 시계   : l=outer,  r=inner   -> v_r - v_l < 0                (w<0) OK
 *   후진 시계   : l=-inner, r=-outer  -> v_r - v_l < 0                (w<0) OK
 * (crab_pass 는 '옆으로 밀리는 방향'을 고정하려고 v 와 w 를 같이 뒤집는다.
 *  여기서는 반대로 '회전 방향'을 고정해야 하므로 부호 규칙이 다르다.)
 *
 * 안쪽 바퀴를 0 으로 두면 speed_to_pwm() 이 PWM 0(완전 정지)을 내보내고,
 * 스키드 스티어에서는 그 바퀴 마찰 때문에 차가 통째로 멈춘다(2026-08-09d에
 * 실기에서 확인됨). 그래서 안쪽도 반드시 min_arc_speed 이상으로 굴린다.
 * ------------------------------------------------------------ */
void robot_runner__arc_pass(FPGALink *fpga, const RobotConfig *cfg,
                                           bool forward, bool ccw, double sec) {
    int outer = cfg->controller_max_speed;
    int inner = (cfg->min_arc_speed > 1) ? cfg->min_arc_speed : 1;
    if (outer <= inner) outer = inner + 1;
    int l, r;
    if (ccw) {
        if (forward) { l =  inner; r =  outer; }
        else         { l = -outer; r = -inner; }
    } else {
        if (forward) { l =  outer; r =  inner; }
        else         { l = -inner; r = -outer; }
    }
    fpga_link_set_speed(fpga, l, r);
    robot_runner__sleep_sec(sec);
    fpga_link_set_speed(fpga, 0, 0);
    robot_runner__sleep_sec(cfg->sg_settle_sec);
}


/* 구석 탈출용 '크게 한 번' 회전. 실제로 명령한 회전량(rad, 부호 포함)을 돌려준다.
 * 눈을 감고 도는 구간이므로 호출부는 반드시 뒤에 재위치추정을 해야 한다. */
double robot_runner__corner_spin(FPGALink *fpga, const RobotConfig *cfg,
                                                double dtheta_rad) {
    RotRateEst *est = rot_rate_shared(cfg, cfg->sg_step_speed);
    double rate = rot_rate_get(est);
    /* 0.85 배만 명령한다 (sweep_rotate 의 SWEEP_FRACTION 과 같은 취지).
     * 각속도 추정이 실제보다 느리게 잡혀 있으면 그대로 지나치는데, 좁은 곳에서
     * 지나치면 반대쪽 벽을 새로 만난다. 모자라면 다음 라운드에 한 번 더 돌면 된다. */
    double sec = 0.85 * fabs(dtheta_rad) / rate;
    if (sec < cfg->sg_move_sec_min) sec = cfg->sg_move_sec_min;
    if (sec > 1.00) sec = 1.00;                 /* 눈 감고 도는 시간 상한 */
    int w = cfg->sg_step_speed;
    int l = (dtheta_rad > 0.0) ? -w : w;        /* +각도 = 반시계 = 왼바퀴 후진 */
    fpga_link_set_speed(fpga, l, -l);
    robot_runner__sleep_sec(sec);
    fpga_link_set_speed(fpga, 0, 0);
    robot_runner__sleep_sec(cfg->sg_settle_sec);
    return (dtheta_rad > 0.0 ? +1.0 : -1.0) * rate * sec;
}


/* ------------------------------------------------------------
 * (5) 구석 탈출 본체
 *
 * 사용자 요구:
 *   "구석의 반대 방향으로 확 세게 크게 돌아서 탈출하거나,
 *    애초에 구석에 몸을 박지 않도록 조심히 이동하면 좋겠다."
 *
 * 한 라운드에서 아래 순서로 시도한다. 앞 단계가 되면 뒷 단계는 안 한다.
 *   A) 축방향으로 조금 움직여서 해결되나? (가장 안전하고 자세도 안 잃는다)
 *   B) 좌우 두 회전방향 중 여유가 큰 쪽으로 크게 돈다.  <- 요구사항 그 자체
 *      '구석 반대쪽'을 사람이 지정하는 대신 라이다로 직접 고른다: 같은 크기로
 *      좌/우를 돌린다고 가정하고 최악 여유를 계산해, 큰 쪽을 쓴다.
 *   C) 제자리 회전이 아예 안 되면 호(arc)로 빼면서 꺾는다.
 *      막힌 방향이 차체기준 phi 일 때, 목표는 그 방향으로 뻗은 길이를 줄이는 것
 *      = phi 를 ±90도(차체 옆구리, 반폭 65mm)로 보내는 것이므로
 *          dtheta = (phi > 0) ? (phi - 90도) : (phi + 90도)
 *      가 되고, 그 부호가 회전방향이다. 진행방향은 앞/뒤 중 실제 여유가 넓은 쪽.
 *
 * 반환 true = 요구 여유(need_gap_m)를 확보함.
 * out_moved 에 이번 탈출에서 대략 얼마나 움직였는지(m) 를 담는다 - 호출부가
 * 재위치추정 범위를 정하는 데 쓴다.
 * ------------------------------------------------------------ */
bool robot_runner__corner_escape(FPGALink *fpga, LidarThread *lidar,
                                                const RobotConfig *cfg,
                                                double want_turn_rad, double need_gap_m,
                                                const char *tag, double *out_moved) {
    const double HL = 0.5 * cfg->body_length;
    const double HW = 0.5 * cfg->body_width;
    const double NEED = (need_gap_m > 0.0) ? need_gap_m : CORNER_ROT_MARGIN_M;

    /* 남은 회전량. 눈 감고 도는 구간이라 자세추정을 못 쓰므로, 명령한 만큼을
     * 빼면서 스스로 관리한다. 호출부가 0을 주면 90도 기준으로 여유를 본다
     * (슬롯 진입은 항상 90도 회전이므로). */
    double remain = want_turn_rad;
    if (fabs(remain) < 1e-6) remain = ANGLE_PI / 2.0;

    double moved_total = 0.0;
    double best_gap = -1e9;
    int    no_gain = 0;
    int    forced_used = 0;
    bool   announced = false;

    for (int round = 0; round < CORNER_ROUNDS_MAX && !g_stop_requested; round++) {
        fpga_link_set_speed(fpga, 0, 0);
        robot_runner__sleep_sec(cfg->sg_settle_sec);

        LidarScan scan;
        if (!wait_for_fresh_scan(lidar, &scan, 2.0)) {
            printf("[구석] %s: 스캔을 못 받아 탈출을 중단합니다\n", tag);
            fflush(stdout);
            break;
        }
        apply_lidar_frame_fix(&scan, cfg->lidar_mirror, cfg->lidar_yaw_offset_deg,
                              cfg->lidar_offset_forward_m);

        double gdir = 0.0;
        double gap0 = robot_runner__rot_gap_shifted(&scan, 0.0, remain, HL, HW, &gdir);
        double minc = 0.0;
        double bdir = robot_runner__min_clearance_dir(&scan, &minc);
        double front = robot_runner__axis_free_dist(&scan, +1, HL, HW, CORNER_SIDE_MARGIN_M);
        double rear  = robot_runner__axis_free_dist(&scan, -1, HL, HW, CORNER_SIDE_MARGIN_M);

        /* ---- 성공 판정 ----
         * (a) 남은 회전을 할 여유가 생겼거나
         * (b) 한 번이라도 움직인 뒤 어느 쪽으로든 넉넉히 직진할 수 있으면
         *     (그 다음은 호출부의 재위치추정 + 재계획이 알아서 한다) */
        bool ok = (gap0 >= NEED)
                  || (round > 0 && (front >= CORNER_FREE_RUN_M || rear >= CORNER_FREE_RUN_M));
        if (ok) {
            if (round > 0) {
                printf("[구석] %s 탈출 완료: 남은회전 %+.0fdeg 여유 %+.0fmm, "
                       "직진 가능 앞 %.0f / 뒤 %.0fmm (%d라운드)\n",
                       tag, remain * 180.0 / ANGLE_PI, gap0 * 1000.0,
                       front * 1000.0, rear * 1000.0, round);
                fflush(stdout);
            }
            if (out_moved) *out_moved = moved_total;
            return true;
        }

        if (!announced) {
            announced = true;
            printf("[구석] %s: 구석에 물렸습니다. 사방최소 %.0fmm(%+.0fdeg 방향), "
                   "남은회전 %+.0fdeg 여유 %+.0fmm(필요 %.0fmm)\n"
                   "       직진 가능거리 앞 %.0fmm / 뒤 %.0fmm. 빼내겠습니다.\n",
                   tag, minc * 1000.0, bdir,
                   remain * 180.0 / ANGLE_PI, gap0 * 1000.0, NEED * 1000.0,
                   front * 1000.0, rear * 1000.0);
            fflush(stdout);
        }

        /* 진행 판정 - 두 라운드 연속 나아지지 않으면 그만한다(무한 반복 금지) */
        if (best_gap > -1e8) {
            if (gap0 > best_gap + 0.003) no_gain = 0;
            else                         no_gain++;
        }
        if (gap0 > best_gap) best_gap = gap0;
        if (no_gain >= 2) {
            printf("[구석] %s: 두 번 더 움직여도 여유가 %.0fmm 에서 안 늘어납니다. "
                   "여기서 멈춥니다.\n", tag, best_gap * 1000.0);
            fflush(stdout);
            break;
        }

        /* ---- A) 축방향 미세 이동으로 해결되나 (가장 안전하고 자세도 안 잃는다) ---- */
        double back_lim = rear  - CORNER_WALL_MARGIN_M;
        double fwd_lim  = front - CORNER_WALL_MARGIN_M;
        if (back_lim > CORNER_SHIFT_MAX_M) back_lim = CORNER_SHIFT_MAX_M;
        if (fwd_lim  > CORNER_SHIFT_MAX_M) fwd_lim  = CORNER_SHIFT_MAX_M;
        double sgap = gap0;
        double s = robot_runner__best_axis_shift(&scan, remain, HL, HW,
                                                  back_lim, fwd_lim, NEED, &sgap);
        if (fabs(s) >= 0.005 && sgap > gap0 + 0.003) {
            int sp = robot_runner__escape_speed(cfg);
            double sec = robot_runner__move_sec_for_safe(cfg, sp, fabs(s), 1.0);
            printf("[구석] %s %d: %s %.0fmm - 회전 여유 %+.0f -> %+.0fmm\n",
                   tag, round + 1, (s > 0.0) ? "전진" : "후진", fabs(s) * 1000.0,
                   gap0 * 1000.0, sgap * 1000.0);
            fflush(stdout);
            int cmd = (s > 0.0) ? sp : -sp;
            fpga_link_set_speed(fpga, cmd, cmd);
            robot_runner__sleep_sec(sec);
            fpga_link_set_speed(fpga, 0, 0);
            moved_total += fabs(s);
            continue;
        }

        /* ---- 어느 쪽으로 도는 게 덜 막혔나 ----
         * 좌/우로 같은 각도(30도)를 돈다고 보고 최악 여유를 비교한다.
         * 이것이 사용자가 말한 '구석의 반대 방향'을 라이다로 직접 정하는 방법이다.
         *
         * 실기 자세(1.320, 0.710, -43deg)에서 오프라인으로 검산한 값:
         *     반시계 -11.0mm / 시계 -6.6mm   -> 시계가 4.4mm 덜 막힘
         * 그리고 실제로 시계로 계속 돌리면 -58도에서 앞이 뚫리고(+1.0mm),
         * -73도에서 +79.8mm, -88도에서 +225.9mm 로 완전히 풀린다.
         * 반시계로 돌리면 어느 각도에서도 -11mm 에서 벗어나지 못한다.
         * 즉 이 비교 한 줄이 '나오는 길'과 '더 박히는 길'을 정확히 갈라준다. */
        double probe = 30.0 * ANGLE_PI / 180.0;
        double g_ccw = robot_runner__rot_gap_shifted(&scan, 0.0, +probe, HL, HW, NULL);
        double g_cw  = robot_runner__rot_gap_shifted(&scan, 0.0, -probe, HL, HW, NULL);
        double sign;
        if      (g_ccw > g_cw + 0.002) sign = +1.0;
        else if (g_cw  > g_ccw + 0.002) sign = -1.0;
        else    sign = (remain >= 0.0) ? +1.0 : -1.0;   /* 비슷하면 가려던 쪽 유지 */

        /* ---- B) 안전하게 돌 수 있는 만큼 크게 돈다 ---- */
        double full = CORNER_TURN_MAX_DEG * ANGLE_PI / 180.0;
        double frac = robot_runner__max_safe_rotation_frac(&scan, sign * full, HL, HW,
                                                            CORNER_ROT_MARGIN_M);
        double deg = CORNER_TURN_MAX_DEG * frac;
        if (deg >= CORNER_TURN_MIN_DEG) {
            printf("[구석] %s %d: %s으로 %.0fdeg 크게 돕니다 "
                   "(30도 기준 여유 반시계 %+.0f / 시계 %+.0fmm)\n",
                   tag, round + 1, (sign > 0.0) ? "반시계" : "시계", deg,
                   g_ccw * 1000.0, g_cw * 1000.0);
            fflush(stdout);
            double done = robot_runner__corner_spin(fpga, cfg,
                                                     sign * deg * ANGLE_PI / 180.0);
            remain = normalize_angle(remain - done);
            moved_total += 0.030;   /* 회전중심 오차만큼은 움직인다 - 재위치추정용 여유 */
            continue;
        }

        /* ---- C) 강제 회전 (2026-08-10b) ***사용자 요구: 확 세게 크게 돌기***
         *
         * 여기까지 왔다는 건 사방 여유가 전부 0 이하라 '안전한 회전량'이 계산되지
         * 않는다는 뜻이다(차체가 이미 벽에 닿아 있음). 그래도 제자리 회전은 차체
         * 중심을 옮기지 않으므로, 직진처럼 벽으로 더 밀어붙이지는 않는다.
         * 위에서 고른 '덜 막힌 쪽'으로 한 번에 크게 돌린다. 폼보드 벽과 스키드
         * 스티어 타이어는 이 정도로는 미끄러져 빠지는 경우가 대부분이다.
         * 무한 반복을 막기 위해 횟수를 제한하고, 매번 멈춰서 다시 관측한다. */
        if (forced_used < CORNER_FORCED_MAX) {
            forced_used++;
            double fdeg = CORNER_FORCED_DEG;
            if (fdeg > fabs(remain) * 180.0 / ANGLE_PI + 20.0)
                fdeg = fabs(remain) * 180.0 / ANGLE_PI + 20.0;
            if (fdeg < CORNER_TURN_MIN_DEG) fdeg = CORNER_TURN_MIN_DEG;
            printf("[구석] %s %d: 안전한 회전량이 안 나옵니다(사방 최소거리 %.0fmm). "
                   "덜 막힌 %s으로 %.0fdeg 강하게 돌립니다 (%d/%d)\n"
                   "       (제자리 회전은 중심을 안 옮기므로 벽으로 더 밀지 않습니다. "
                   "30도 기준 여유 반시계 %+.0f / 시계 %+.0fmm)\n",
                   tag, round + 1, minc * 1000.0,
                   (sign > 0.0) ? "반시계" : "시계", fdeg, forced_used, CORNER_FORCED_MAX,
                   g_ccw * 1000.0, g_cw * 1000.0);
            fflush(stdout);
            double done = robot_runner__corner_spin(fpga, cfg,
                                                     sign * fdeg * ANGLE_PI / 180.0);
            remain = normalize_angle(remain - done);
            moved_total += 0.040;
            no_gain = 0;            /* 강제 회전은 한 번 더 기회를 준다 */
            continue;
        }

        /* ---- D) 호(arc)로 빼면서 꺾기 ----
         * 막힌 방향 phi 로 뻗은 차체 길이를 줄이려면 phi 를 +-90도(반폭 65mm)로
         * 보내야 하므로  dtheta = (phi>0) ? phi-90 : phi+90  이고 그 부호가 회전방향. */
        double phi = bdir;
        double dth_deg = normalize_angle_deg((phi > 0.0) ? (phi - 90.0) : (phi + 90.0));
        bool ccw = (dth_deg > 0.0);
        bool forward = (front > rear);
        double room = forward ? front : rear;
        if (room < 0.008) { forward = !forward; room = forward ? front : rear; }
        if (room < 0.008) {
            printf("[구석] %s: 앞(%.0fmm)/뒤(%.0fmm) 모두 여유가 없어 호(arc)로도 "
                   "못 뺍니다.\n"
                   "       사방최소 %.0fmm(%+.0fdeg). 차체를 손으로 살짝만 밀어 주세요.\n",
                   tag, front * 1000.0, rear * 1000.0, minc * 1000.0, bdir);
            fflush(stdout);
            break;
        }
        double arc_m = room * 0.6;
        if (arc_m > 0.035) arc_m = 0.035;
        double arc_sec = robot_runner__move_sec_for_safe(cfg, cfg->controller_max_speed,
                                                          arc_m, 0.6);
        printf("[구석] %s %d: %s하며 %s으로 꺾습니다 "
               "(막힌 쪽 %+.0fdeg, 여유 %.0fmm, %.2fs)\n",
               tag, round + 1, forward ? "전진" : "후진",
               ccw ? "반시계" : "시계", phi, room * 1000.0, arc_sec);
        fflush(stdout);
        robot_runner__arc_pass(fpga, cfg, forward, ccw, arc_sec);
        moved_total += arc_m;
    }

    if (out_moved) *out_moved = moved_total;
    return false;
}


/* ------------------------------------------------------------
 * (6) 구석 사전 회피 - '박기 전에' 벽에서 떨어뜨린다.
 *
 * 대기점에 도착한 직후처럼 "이제 곧 크게 돌 것"이 확정된 시점에서 부른다.
 * 지금 자리에서 turn_rad 를 돌 수 있으면 아무것도 안 하고, 못 돌면
 * '가장 작은 축방향 이동'으로만 여유를 만든다. 절대 벽 쪽으로는 안 민다
 * (best_axis_shift 가 벽 쪽 이동은 여유가 줄어들어 뽑지 않는다).
 *
 * 예전 ensure_rotation_room 과의 차이:
 *   예전: 방향을 sin(heading) 부호로 '가정'했다 -> heading 0도에서 벽으로 전진.
 *   지금: 두 방향을 다 예측해 보고 실제로 여유가 늘어나는 쪽만 고른다.
 * ------------------------------------------------------------ */
bool robot_runner__corner_backoff(FPGALink *fpga, LidarThread *lidar,
                                                 OccupancyGridSLAM *nav_slam,
                                                 const RobotConfig *cfg,
                                                 double turn_rad, double need_gap_m,
                                                 double max_total_m,
                                                 double *prev_rx, double *prev_ry,
                                                 double *prev_rth) {
    const double HL = 0.5 * cfg->body_length;
    const double HW = 0.5 * cfg->body_width;
    const double NEED = (need_gap_m > 0.0) ? need_gap_m : 0.015;
    double moved_total = 0.0;

    for (int i = 0; i < 6 && !g_stop_requested; i++) {
        fpga_link_set_speed(fpga, 0, 0);
        robot_runner__sleep_sec(cfg->sg_settle_sec);

        LidarScan scan;
        if (!robot_runner__take_scan(lidar, cfg, &scan, 2.0)) return false;

        double x, y, th, odx, ody, odth;
        robot_runner__predict_delta(fpga, cfg, nav_slam->theta, prev_rx, prev_ry, prev_rth,
                                     &odx, &ody, &odth);
        slam_localize_step(nav_slam, &scan, odx, ody, odth, &x, &y, &th);
        robot_runner__emit_state("room", i, x, y, th, &scan);

        double gdir = 0.0;
        double gap = robot_runner__rot_gap_shifted(&scan, 0.0, turn_rad, HL, HW, &gdir);
        if (gap >= NEED) {
            if (i > 0) {
                printf("[slot] 돌 자리 확보 완료: %.0fmm 움직여 여유 %+.0fmm "
                       "(필요 %.0fmm, %+.0fdeg 방향)\n",
                       moved_total * 1000.0, gap * 1000.0, NEED * 1000.0, gdir);
                fflush(stdout);
            }
            return true;
        }
        if (moved_total >= max_total_m) {
            printf("[slot] 돌 자리 확보 실패: %.0fmm 움직였는데도 %+.0fdeg 방향 여유가 "
                   "%.0fmm 모자랍니다\n", moved_total * 1000.0, gdir, (NEED - gap) * 1000.0);
            fflush(stdout);
            return false;
        }

        double front = robot_runner__axis_free_dist(&scan, +1, HL, HW, CORNER_SIDE_MARGIN_M);
        double rear  = robot_runner__axis_free_dist(&scan, -1, HL, HW, CORNER_SIDE_MARGIN_M);
        double back_lim = rear  - CORNER_WALL_MARGIN_M;
        double fwd_lim  = front - CORNER_WALL_MARGIN_M;
        double left = max_total_m - moved_total;
        if (back_lim > left) back_lim = left;
        if (fwd_lim  > left) fwd_lim  = left;
        if (back_lim > CORNER_SHIFT_MAX_M) back_lim = CORNER_SHIFT_MAX_M;
        if (fwd_lim  > CORNER_SHIFT_MAX_M) fwd_lim  = CORNER_SHIFT_MAX_M;

        double sgap = gap;
        double s = robot_runner__best_axis_shift(&scan, turn_rad, HL, HW,
                                                  back_lim, fwd_lim, NEED, &sgap);
        if (fabs(s) < 0.005 || sgap <= gap + 0.002) {
            printf("[slot] 돌 자리 확보: 앞으로 %.0fmm / 뒤로 %.0fmm 가 한계라 "
                   "이 자리에서는 더 못 벌립니다 (여유 %+.0fmm, 필요 %.0fmm)\n",
                   front * 1000.0, rear * 1000.0, gap * 1000.0, NEED * 1000.0);
            fflush(stdout);
            return false;
        }

        int sp = (cfg->dock_step_speed > 0) ? cfg->dock_step_speed : cfg->sg_step_speed;
        double sec = robot_runner__move_sec_for_safe(cfg, sp, fabs(s), 1.0);
        printf("[slot] 돌 자리 부족(%+.0fdeg 방향 여유 %+.0fmm < 필요 %.0fmm) "
               "-> 벽 반대쪽으로 %s %.0fmm (%.2fs, 예상 여유 %+.0fmm)\n",
               gdir, gap * 1000.0, NEED * 1000.0,
               (s > 0.0) ? "전진" : "후진", fabs(s) * 1000.0, sec, sgap * 1000.0);
        fflush(stdout);
        int cmd = (s > 0.0) ? sp : -sp;
        fpga_link_set_speed(fpga, cmd, cmd);
        robot_runner__sleep_sec(sec);
        fpga_link_set_speed(fpga, 0, 0);
        moved_total += fabs(s);
    }
    return false;
}
