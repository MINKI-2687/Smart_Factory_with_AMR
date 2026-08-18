#include "robot_escape.h"
#include "robot_runner.h"


/* 직선으로 dir 방향 room 만큼 민다. 실제로 움직였으면 true.
 * 느린 속도로 먼저 시도하고, 정지마찰 때문에 안 움직였으면 원래 속도로 재시도한다. */
bool robot_runner__escape_push(FPGALink *fpga, LidarThread *lidar,
                                              const RobotConfig *cfg, int dir, double room,
                                              double before_dist, double *out_after_dist) {
    for (int pass = 0; pass < 2; pass++) {
        int speed = (pass == 0) ? robot_runner__escape_speed(cfg) : cfg->sg_step_speed;
        double sec = robot_runner__move_sec_for_safe(cfg, speed, room, 1.2);
        if (sec <= 0.0) return false;

        int sp = dir * speed;
        fpga_link_set_speed(fpga, sp, sp);      /* 조향 없이 곧게 */
        robot_runner__sleep_sec(sec);
        fpga_link_set_speed(fpga, 0, 0);
        robot_runner__sleep_sec(cfg->sg_settle_sec);

        LidarScan after;
        if (!robot_runner__take_scan(lidar, cfg, &after, 2.0)) return true;   /* 확인 못 함 */
        /* 전후 측정을 escape_to_open 과 같은 척도(직사각형 스윕)로 통일 (2026-08-10b).
         * 부채꼴은 구석에서 옆벽을 물고 들어와 '움직였는데 안 움직였다'고 읽는다. */
        double a = robot_runner__axis_free_dist(&after, dir, 0.5 * cfg->body_length,
                                                 0.5 * cfg->body_width,
                                                 CORNER_SIDE_MARGIN_M)
                   + 0.5 * cfg->body_length;
        if (out_after_dist) *out_after_dist = a;

        /* 선속도 자가보정. 다만 '진짜 벽'을 보고 있을 때만 - 예전에는 검증 없이
         * 받아서, 부채꼴 안의 최근접 물체가 바뀌면 엉뚱한 값을 학습했다. */
        if (before_dist < 1.2 && a < 1.2 && a > 0.02)
            robot_runner__lin_scale_update(room, before_dist - a);

        if (fabs(before_dist - a) > 0.004) return true;   /* 움직였다 */
        if (pass == 0) {
            printf("[main]   (느린 속도 %d 로는 안 움직여서 %d 로 다시 밉니다)\n",
                   robot_runner__escape_speed(cfg), cfg->sg_step_speed);
            fflush(stdout);
        }
    }
    return false;
}


bool robot_runner__escape_to_open(FPGALink *fpga, LidarThread *lidar,
                                                 const RobotConfig *cfg, double max_move_m,
                                                 double need_clearance_m, bool allow_crab) {
    const double HALF_LEN = 0.5 * cfg->body_length;   /* 차체 절반 길이 */
    const double HALF_W   = 0.5 * cfg->body_width;
    /* 0.04 -> 0.015 (2026-08-10b) *** '탈출 실패: 앞뒤 모두 막힘' 의 직접 원인 ***
     * 예전 조합(MARGIN 40mm + room<=10mm 이면 포기)은 진행방향 여유가 177.5mm
     * 미만이면 아예 시도조차 하지 않았다. 실기 로그의
     *     탈출 실패: 앞(155mm) 뒤(149mm) 모두 막혔습니다
     * 는 앞으로 27.5mm 가 실제로 비어 있는데도 포기한 것이다. 좁은 곳에서
     * 27mm 는 결코 작은 값이 아니다 - 회전 여유가 그만큼 늘어난다. */
    const double MARGIN   = 0.015;    /* 벽 앞에 남길 여유 */
    const double MAX_MOVE = (max_move_m > 0.0) ? max_move_m : ESCAPE_STEP_M;
    const double NEED_M   = (need_clearance_m > 0.0) ? need_clearance_m
                                                     : (cfg->robot_radius + 0.01);
    (void)allow_crab;   /* 게걸음은 아래 설명대로 더 이상 쓰지 않는다 */

    /* *** 방향 고정 *** - 이 변수가 루프 밖에 있는 것이 이번 수정의 핵심이다. */
    int  dir_lock  = 0;        /* 0=미정, +1=전진, -1=후진 */
    bool flipped   = false;    /* 막혀서 한 번 뒤집었는가 (두 번은 안 뒤집는다) */
    double best_minc = -1.0;
    int    no_gain = 0;

    for (int attempt = 0; attempt < ESCAPE_MAX_TRY && !g_stop_requested; attempt++) {
        LidarScan scan;
        if (!robot_runner__take_scan(lidar, cfg, &scan, 2.0)) return false;

        /* 부채꼴(+-25도) 최소거리 -> 직사각형 스윕으로 교체 (2026-08-10b).
         * 구석에서는 옆벽 점이 부채꼴 안에 들어와 정면 여유를 실제보다 훨씬
         * 작게 만든다(그 오판이 '앞뒤 모두 막힘' 오진의 절반이었다).
         * axis_free_dist 는 '차체 앞끝에서 남은 거리'를 바로 돌려주므로,
         * 아래 계산과 맞추기 위해 HALF_LEN 을 다시 더해 '벽까지 거리'로 쓴다. */
        double front = robot_runner__axis_free_dist(&scan, +1, HALF_LEN, HALF_W,
                                                     CORNER_SIDE_MARGIN_M) + HALF_LEN;
        double rear  = robot_runner__axis_free_dist(&scan, -1, HALF_LEN, HALF_W,
                                                     CORNER_SIDE_MARGIN_M) + HALF_LEN;
        double minc  = 0.0;
        double block_dir = robot_runner__min_clearance_dir(&scan, &minc);

        /* 이미 충분히 열렸으면 끝 */
        if (minc >= NEED_M) {
            if (attempt > 0) {
                printf("[main] 탈출 완료: 사방 최소 %.0fmm >= 필요 %.0fmm (%d회 이동)\n",
                       minc * 1000.0, NEED_M * 1000.0, attempt);
                fflush(stdout);
            }
            return true;
        }

        /* 진행 판정. 예전에는 '안 좋아지면 방향을 뒤집는다'였는데, 그것이
         * 바로 무한 왕복의 원인이었다. 이제는 뒤집지 않고, 좋아지지 않는 상태가
         * 이어지면 그냥 포기한다(호출부가 '천천히 회전'으로 넘어간다). */
        if (best_minc > 0.0) {
            if (minc > best_minc + 0.003) { no_gain = 0; }
            else                          { no_gain++; }
        }
        if (minc > best_minc) best_minc = minc;
        if (no_gain >= 2) {
            printf("[main] 탈출 중단: 두 번 더 밀어도 사방 최소가 %.0fmm 에서 안 늘어납니다.\n"
                   "       앞뒤 왕복 대신 여기서 멈추고, 돌 수 있는 만큼만 돌겠습니다.\n",
                   best_minc * 1000.0);
            fflush(stdout);
            return false;
        }

        /* ---- 어느 쪽으로 뺄지 (처음 한 번만 결정) ----
         * 게걸음(K턴)은 삭제했다. 한 왕복에 12~17mm 밖에 못 벌면서 0.7초를 쓰고,
         * 무엇보다 '앞으로 갔다 뒤로 오는' 동작이라 화면에서 왕복으로 보인다.
         * 경로추종에 필요한 건 옆으로 평행이동이 아니라 '돌 자리를 조금 버는 것'이므로
         * 한 방향 직선 후퇴로 충분하다. */
        if (dir_lock == 0) {
            dir_lock = (front >= rear) ? +1 : -1;
            printf("[main] 끼임 탈출 시작: 막힌 쪽 %+.0fdeg(%.0fmm), 정면 %.2fm / 후면 %.2fm\n"
                   "       -> %s 한 방향으로만 %.0fmm 씩 천천히 뺍니다 "
                   "(필요 여유 %.0fmm, 최대 %d회)\n",
                   block_dir, minc * 1000.0, front, rear,
                   dir_lock > 0 ? "전진" : "후진", MAX_MOVE * 1000.0,
                   NEED_M * 1000.0, ESCAPE_MAX_TRY);
            fflush(stdout);
        }

        double clearance = (dir_lock > 0) ? front : rear;
        double room = clearance - HALF_LEN - MARGIN;
        if (room > MAX_MOVE) room = MAX_MOVE;

        if (room <= 0.005) {
            /* 고정한 방향이 막혔다. 아직 안 뒤집었으면 딱 한 번만 뒤집는다. */
            if (!flipped) {
                flipped = true;
                dir_lock = -dir_lock;
                printf("[main] %s 쪽이 %.0fmm 밖에 안 남아 이번 한 번만 방향을 %s으로 바꿉니다\n",
                       dir_lock > 0 ? "후진" : "전진", clearance * 1000.0,
                       dir_lock > 0 ? "전진" : "후진");
                fflush(stdout);
                clearance = (dir_lock > 0) ? front : rear;
                room = clearance - HALF_LEN - MARGIN;
                if (room > MAX_MOVE) room = MAX_MOVE;
            }
            if (room <= 0.005) {
                /* PATCH (2026-08-10b): 여기서 \"손으로 빼주세요\" 로 끝내지 않는다.
                 * 앞뒤가 다 막혔다는 건 차체가 비스듬히 구석에 물렸다는 뜻이고,
                 * 그때 사람이 하는 일은 '핸들을 끝까지 꺾어서 크게 도는 것'이다.
                 * corner_escape 가 좌/우 중 덜 막힌 쪽을 라이다로 직접 골라
                 * 한 번에 최대 60도까지 돌리고, 그것도 안 되면 호(arc)로 뺀다. */
                printf("[main] 앞(%.0fmm)/뒤(%.0fmm) 모두 막혔습니다 - 직선 탈출을 접고\n"
                       "       구석 반대쪽으로 크게 돌아서 빠져나오겠습니다.\n",
                       front * 1000.0, rear * 1000.0);
                fflush(stdout);
                double moved_c = 0.0;
                if (robot_runner__corner_escape(fpga, lidar, cfg, 0.0,
                                                 CORNER_ROT_MARGIN_M,
                                                 "직선탈출 실패", &moved_c))
                    return true;
                printf("[main] 탈출 실패: 사방 최소 %.0fmm(%+.0fdeg 방향)에서 "
                       "회전으로도 못 빠져나왔습니다. 손으로 조금 빼주세요.\n",
                       minc * 1000.0, block_dir);
                fflush(stdout);
                return false;
            }
        }

        printf("[main] 탈출 %d/%d: %s으로 %.0fmm (사방 최소 %.0fmm -> 목표 %.0fmm)\n",
               attempt + 1, ESCAPE_MAX_TRY, dir_lock > 0 ? "전진" : "후진",
               room * 1000.0, minc * 1000.0, NEED_M * 1000.0);
        fflush(stdout);

        double after = clearance;
        if (!robot_runner__escape_push(fpga, lidar, cfg, dir_lock, room,
                                        clearance, &after)) {
            printf("[main] *** 밀라고 명령했는데 차체가 전혀 안 움직였습니다 ***\n"
                   "       %s 여유 %.3f -> %.3f (4mm 미만 변화)\n"
                   "       바퀴가 정지마찰을 못 이기거나 이미 벽에 물려 있습니다. 확인할 것:\n"
                   "       (1) fpga_link.h 의 PWM_DEADZONE(%d)이 실측값인지\n"
                   "       (2) --sg-move-min(%.2fs)이 너무 짧지 않은지\n",
                   dir_lock > 0 ? "정면" : "후면", clearance, after,
                   PWM_DEADZONE, cfg->sg_move_sec_min);
            fflush(stdout);
            return false;
        }
    }

    printf("[main] 탈출 종료: %d회 밀었지만 필요한 여유(%.0fmm)를 다 못 만들었습니다.\n"
           "       왕복하지 않고 여기서 멈춥니다 - 돌 수 있는 만큼만 돌겠습니다.\n",
           ESCAPE_MAX_TRY, NEED_M * 1000.0);
    fflush(stdout);
    return false;
}
