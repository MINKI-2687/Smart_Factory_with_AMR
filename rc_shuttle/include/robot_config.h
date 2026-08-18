#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif

#include <stdio.h>
#include <pthread.h>
#include <signal.h>
#include <time.h>
#include <string.h>
#include "fpga_serial.h"
#include "rplidar_reader.h"
#include "lidar_thread.h"
#include "dynamic_navigator.h"
#include "slam.h"
#include "frontier_exploration.h"
#include "obstacle_list.h"
#include "lidar_clustering.h"
#include "map_io.h"
#include "trigger.h"


typedef enum {
    MAP_SOURCE_SLAM,    /* 프론티어탐색으로 직접 매핑 */
    MAP_SOURCE_FIXED,   /* known_obstacles로 지도 구성 */
    MAP_SOURCE_LOAD,    /* 파일에서 이전에 저장해둔 지도 불러오기 (매핑 생략) */
} MapSource;

/* 셔틀 왕복의 한 지점 (주차위치 + 각도). 이 프로젝트는 항상 각도까지 맞춰서
 * 주차해야 하므로 theta_deg는 선택이 아니라 항상 값을 넣어야 함. */
typedef struct {
    double x, y;
    double theta_deg;
} ShuttlePoint;

typedef struct {
    double wheel_diameter, wheel_separation;
    int ticks_per_rev;
    int controller_max_speed, pwm_safety_cap;

    double resolution, robot_radius;
    int control_hz;

    /* 차체 실제 치수(m). robot_radius 는 A* 팽창용 '원 근사 반경'인데, 제자리 회전
     * 가능 여부는 원이 아니라 직사각형으로 따져야 훨씬 덜 보수적이다
     * (robot_runner__rotation_is_safe 주석 참고, 2026-08-09f). */
    double body_length, body_width;

    /* ---- 제자리 회전 전용 스캔매칭 탐색창 (2026-08-09h) ----
     * 회전 스텝 한 번에 돌 수 있는 각도는 결국 스캔매칭 각도 탐색창이 정한다
     * (robot_runner__clamp_rotation_sec_rate: 창의 70%). 기본창이 ±12도라
     * 한 스텝 8.4도이고, 90도 돌려면 11스텝 x 약 0.4초 = 4.4초가 걸린다.
     *
     * 그런데 '제자리' 회전에서는 위치가 거의 안 변한다(실측: 48도 회전에 21mm).
     * 그래서 위치창을 ±150mm -> ±60mm 로 좁히는 대신 각도창을 ±12 -> ±25도로
     * 넓히면, 탐색 후보 수는 오히려 줄고(11x11x9=1089 -> 5x5x26=650)
     * 한 스텝 회전량은 8.4 -> 17.5도로 두 배가 된다. 즉 회전 시간이 절반이 된다.
     * 계산량이 줄어드는 이유: 후보 수가 n_xy^2 * n_th 이라 위치창 축소가 제곱으로 효과. */
    /* 곡선 주행에서 '안쪽 바퀴'가 가져야 할 최소 속도값 (2026-08-09j).
     *
     * speed_to_pwm() 은 speed s(1~80)를 PWM 170 + s/80*85 로 옮긴다. 즉 s=3 이면
     * PWM 173 으로, 데드존 170 을 겨우 3 넘긴다. 무부하에서는 돌지 몰라도 차체
     * 무게와 스크럽 저항이 걸리면 그냥 끌린다.
     * 실기 로그(가로 통로):
     *   step=93~96 pose=(0.700,0.745) cmd=(31,3)->(34,3)  <- 4스텝 동안 5mm 이동
     *   [main] 끼임 감지 - 탈출 시도 3/6  -> 게걸음 탈출 1/4, 2/4, 3/4 ...
     * 즉 "끼인 것"이 아니라 안쪽 바퀴가 안 돌아 제자리에서 문지르고 있었던 것이고,
     * 그걸 끼임으로 오판해 게걸음까지 갔다.
     * s=14 면 PWM 185 로 데드존을 15 넘어 확실히 굴러간다. 그때 선회반경은
     * (b/2)(vo+vi)/(vo-vi) = 0.08*(206+185)/(206-185) = 1.49m 이고,
     * 통로 0.5m 를 지나는 동안 옆으로 82mm 를 옮길 수 있어 경로복귀에 충분하다. */
    int    min_arc_speed;

    /* ---- 구간별 목표 속도 (2026-08-09m) ----
     * 이 하드웨어는 PWM 데드존 170~255 사이에서만 속도를 만들 수 있어서, 실제로
     * 쓸 수 있는 범위가 대략 0.19~0.33 m/s 뿐이다(=약 1.7배). 그래서 "많이 느리게"는
     * 불가능하고, 대신 구간별로 적당히 나눠 쓴다.
     *   cruise_speed    : 통로 연속주행. 낮출수록 경로에서 덜 벗어나지만 선회반경이
     *                     커진다(34->26 이면 반경 1.47m -> 2.36m). 통로 1.05m 를
     *                     지나며 만들 수 있는 횡보정이 360mm -> 230mm 이라 여전히 충분.
     *   dock_step_speed : 마지막 대기점 접근. 한 스텝 보폭이 크면 목표를 넘나들며
     *                     전진/후진을 반복하므로 여기만 더 낮춘다. */
    int    cruise_speed;
    int    dock_step_speed;

    /* 게걸음(crab) 허용 여부 (2026-08-09m).
     * 게걸음은 '제자리에서 돌아야 하는데 옆 여유가 모자란' 상황을 위한 마지막 수단인데,
     * 한 번에 12~17mm 벌면서 왕복 0.7초씩 쓰고 보기에도 안 좋다. 기본으로 끈다.
     * 대신 '살짝 물러나서 -> 여유 생기면 회전 -> 다시 전진'(직선 탈출)으로 처리한다. */
    bool   allow_crab;

    /* 좌우 보정을 할 때 슬롯 축에서 얼마나 기울일지(도) (2026-08-09m).
     * 90도면 순수 좌우 이동이지만 회전이 두 번(왕복 약 5.6초) 든다.
     * 55도면 이동거리가 1/sin55 = 1.22배로 늘어나는 대신 회전이 55도씩으로 줄어든다. */
    double slot_trim_tilt_deg;

    /* 출발 트리거를 받은 뒤 실제로 움직이기까지 기다릴 시간(초) (2026-08-09n).
     * 시연에서 사람이 짐을 올리고 손을 빼는 시간이 필요하고, 트리거 순간에 바로
     * 튀어나가면 위험하다. 기다리는 동안 스캔은 계속 들어오므로 자세추정도 안정된다. */
    double trigger_delay_sec;
    double rot_win_m, rot_step_m;       /* 회전 중 위치 탐색창/간격 (m) */
    double rot_win_deg, rot_step_deg;   /* 회전 중 각도 탐색창/간격 (도) */

    double grid_size_x, grid_size_y;
    Obstacle known_obstacles[16];
    int n_known_obstacles;

    double start_x, start_y;
    double goal_x, goal_y;
    bool has_goal_theta;
    double goal_theta_deg;

    int slam_mapping_max_steps;
    double slam_mapping_goal_tolerance;
    double slam_occ_threshold;

    /* FPGA로 바퀴명령을 내보내는 주기(Hz). 제어루프(control_hz=20)와 별개로 더 빠르게
     * 보내는 이유: fpga_serial.h의 듀티 사이클링(느린 속도를 펄스로 흉내내는 것)이
     * 이 주기로 동작하는데, 20Hz면 펄스 간격이 최대 0.05초 단위로 뚝뚝 끊겨서 로봇이
     * 덜컹거림. 100Hz면 모터의 기계적 관성이 펄스를 부드럽게 뭉개줌. 명령 13바이트를
     * 115200bps로 보내면 1.1ms이므로 100Hz(10ms 주기)는 대역폭상 충분함. */
    int fpga_send_hz;

    /* ---- Stop-and-go 주행 ----
     * PATCH (2026-08-05 v3): 이 하드웨어는 (1) 정지마찰 때문에 연속 저속이 불가능하고,
     * (2) 엔코더가 없어 위치추정이 100% 라이다 스캔매칭인데, 움직이면서 찍은 스캔은
     * 라이다가 한 바퀴 도는 동안(~100ms) 로봇이 이동/회전해버려서 찌그러짐(motion skew)
     * - 찌그러진 스캔으로 매칭하면 pose가 매번 조금씩 다른 곳으로 튀고, 그걸 쫓아
     * 조향하니 좌우로 출렁임. 두 문제 모두 "확실히 조금 움직이고 -> 완전히 멈추고 ->
     * 멈춘 상태에서 깨끗한 스캔으로 위치를 다시 잡고 -> 다시 움직임"으로 해결됨.
     * 속도는 항상 sg_step_speed로 고정(하드웨어가 좋아하는 큰 값)하고 "얼마나 움직일지"는
     * 이동 시간으로 조절함 - 진폭변조가 아니라 시간변조. 모터가 못 하는 미세조절을
     * 소프트웨어 타이밍으로 대신하는 것. */
    /* 라이다 0도가 차체 정면을 기준으로 몇 도 틀어져 장착됐는지(도).
     * 라이다가 뒤를 보고 있으면 180. apply_lidar_mount_offset() 주석 참고. */
    double lidar_yaw_offset_deg;

    /* RPLIDAR의 시계방향 각도를 반시계 규약으로 뒤집을지. apply_lidar_frame_fix() 주석
     * 참고. RPLIDAR 스펙상 true가 맞지만, 세트장이 좌우대칭이면 틀려도 점수로는
     * 안 드러나므로 눈으로 확인 후 설정할 것. */
    bool lidar_mirror;

    /* 라이다가 차체 회전중심보다 앞으로 나간 거리(m). 사진 기준 실측해서 넣을 것.
     * apply_lidar_frame_fix() 주석 참고 - 안 넣으면 회전할 때마다 위치추정이 이 값의
     * 2배만큼 출렁이고, 주차 깊이도 이 값만큼 모자라게 멈춤. */
    double lidar_offset_forward_m;

    /* 엔코더 틱 부호 보정. 직진(양쪽 PWM +) 시 좌우 틱이 같은 부호로 증가해야 정상.
     * 모터가 마주보게 달리면 한쪽이 반대로 나오므로 그 쪽을 true로. */
    bool encoder_invert_left, encoder_invert_right;


    /* 회전 스텝 클램프 계수 (탐색창 대비). test_encoder_ab.c 실험상 0.6~1.5 구간에서
     * 결과 차이가 오차범위 안이라, 엔코더가 있으면 2.0으로 둠(위 주석 참고). */
    double rot_clamp_frac;

    /* ---- 주차 슬롯 ----
     * point_a/point_b는 '슬롯 안쪽의 최종 주차 자세'로 해석하고, A*는 슬롯 입구
     * 바깥의 대기점(같은 x, y=slot_staging_y)까지만 담당함. 이유는
     * slot_drive() 주석 참고 (A*의 원 근사로는 17cm 슬롯을 못 지나감). */
    /* ============================================================
     * 자세를 믿고 움직여도 되는 최소 스캔-지도 일치도 (2026-08-10f 신규)
     *
     * 사용자 지적 그대로다: "딱 봐도 어긋나 있는데 왜 그냥 그대로 쓰면서 이동하는가."
     * 실제로 슬롯 구간에는 품질 감시가 사실상 없었다. check_lost() 는 슬롯 정렬
     * '시작 전' 딱 한 번 불리고 그마저 반환값을 버렸다. 그 뒤의 slot_align /
     * slot_lateral_align / slot_drive 는 일치도가 0.50 이든 0.95 든 똑같이 돌았다.
     * 실기 로그가 그 결과다 - 일치도 0.55~0.60 으로 수십 스텝을 계속 회전.
     *
     * 이제 이 값 아래로 연속해서 떨어지면 '바퀴를 멈추고 자세부터 고친다'.
     * 못 고치면 진행하지 않고 실패를 반환한다 (틀린 자세로 슬롯에 밀어넣는 것보다
     * 멈추는 편이 항상 낫다).
     *
     * 근거: 같은 세트장에서 자세가 맞으면 전역탐색이 99% 를 낸 기록이 있다
     * (실기 로그: "전역 재위치추정 완료 ... 일치도 62% -> 99%"). 즉 지도/미러/
     * 라이다 장착은 충분히 맞고, 0.70 은 여유 있게 도달 가능한 값이다.
     * 너무 높게 잡으면 지도에 없는 물체(사람 손, 짐)가 조금만 보여도 멈추므로
     * 0.65~0.80 사이에서 --min-match-quality 로 조정한다. 0 이면 감시를 끈다.
     * ============================================================ */
    double min_match_quality;

    bool slot_parking;
    double slot_staging_y;   /* 슬롯 입구 바깥 대기점의 y */
    double slot_align_tol_deg;  /* 진입 전 허용 각도오차(도) - slot_align() 참고 */
    /* 한 지점에서 주차를 몇 번까지 다시 시도할지 (2026-08-09a).
     * 예전에는 정렬/좌우정렬/게이트/진입 중 어디서 실패하든 셔틀 전체를 즉시 중단했음.
     * 사람이라면 빼서 다시 넣지 시동을 끄지 않는다. 이제 실패하면
     * '슬롯 밖으로 빼기 -> 접촉 풀기 -> 대기점으로 A* 재접근 -> 재정렬 -> 재진입'을
     * 이 횟수만큼 반복하고, 그래도 안 되면 그때 멈춘다. run_shuttle() 참고. */
    int slot_max_tries;

    bool stop_and_go;
    /* true면 전진 위주 구간에서 멈추지 않고 연속 주행(회전할 때만 정지-관측).
     * 위 while 루프의 하이브리드 주행 주석 참고. */
    bool hybrid_cruise;
    bool pure_pursuit;        /* A* 경로 추종에 pure pursuit 사용 */
    double lookahead_m;       /* 전방주시 거리 */
    double sg_settle_sec;     /* 정지 후 흔들림이 가라앉기를 기다리는 시간 */
    double rot_breakaway_sec; /* 제자리 회전이 실제로 시작되는 최소 펄스(s) */
    double sg_move_sec_min;   /* 한 번 이동의 최소 시간(정지마찰 돌파 하한) */
    double sg_move_sec_max;   /* 한 번 이동의 최대 시간 */
    int sg_step_speed;        /* 이동 중 사용할 고정 속도(컨트롤러 단위) */
    int slot_step_speed;      /* 슬롯 진입/후진 전용 속도. 벽에 닿으면서 들어가는
                               * 구간이라 일반 주행보다 느려야 맵이 안 밀림 */
    int slot_touch_speed;     /* 안쪽 벽에 밀착시키는 마지막 한 번의 속도 */
    double slot_move_sec;     /* 슬롯 안에서 한 스텝에 미는 시간(초) */
    /* 이 y 보다 위(=깔때기 위 열린 구간)에서는 한 스텝을 길게 밀어 스텝 수를 줄인다.
     * <=0 이면 slot_staging_y - 0.22 로 자동 계산 (2026-08-09h). */
    double slot_fast_above_y;
    double slot_fast_gain;    /* 그 구간에서 slot_move_sec 에 곱할 배수 */
    /* 열린 구간(깔때기 위) 전용 상한/속도 (2026-08-10j).
     *   slot_open_move_sec : 한 스텝 최대 밀기 시간. 0 이면 sg_move_sec_max.
     *                        '깔때기 입구를 넘지 않는다'는 상한이 따로 걸리므로
     *                        크게 줘도 안전하다(기본 0.70 = 220mm 를 한 스텝에).
     *   slot_open_speed    : 그 구간의 속도. 0 이면 sg_step_speed(일반주행 34). */
    double slot_open_move_sec;
    int    slot_open_speed;
    double slot_settle_sec;   /* 슬롯 안에서 스텝 사이 정지 시간(초) */

    /* ---- 재진입 반대편 기울임 (2026-08-09f) ----
     * 요청: "벽에 걸려서 각도를 재조정할 때, 축(-90도)으로만 되돌리지 말고
     *        걸린 벽 반대쪽으로 좀 더 틀어서 넣어라. 걸렸다는 건 그 벽에 가깝다는
     *        뜻이라, 각도만 바로잡아 넣으면 같은 벽에 두 번은 더 걸린다."
     * 맞는 지적이다. 차동구동은 옆으로 못 가므로 좌우 위치를 바꾸는 유일한 방법이
     * '비스듬히 전진'이다. 걸린 벽 반대쪽으로 b 도 기울여 d 만큼 전진하면
     * 옆으로 d*sin(b) 만큼 옮겨간다 - b=8도, d=80mm 면 11mm.
     * 슬롯 편측 여유가 (210-130)/2 = 40mm 이므로 10mm대 보정이면 충분하다.
     * 안전한 상한: yaw b 에서 필요한 가로폭이 130*cos(b)+255*sin(b) 이고 이 값이
     * 210mm 를 넘는 임계각이 20.2도이므로, 8도(164mm)는 여유가 크다. */
    double slot_escape_bias_deg;  /* 재진입 시 걸린 벽 반대로 기울일 각도 */
    double slot_bias_span_m;      /* 그 기울임을 유지할 진입 깊이(m) */
    /* ============================================================
     * 큰 기울임이 끝난 뒤에도 남겨 둘 '잔여 기울임'(도) (2026-08-10g 신규)
     *
     * 사용자 요청 그대로다: "슬롯 진입 중 벽에 걸려서 각도 정렬을 하고 다시
     * 들어가는데, 또 같은 방향 벽에 걸리는 경우가 가끔 있다. 올바른 각도(-90도)에서
     * 2도쯤 더 틀어서 들어가게 해달라."
     *
     * 정확한 지적이다. 지금 코드는 걸린 순간 빈 쪽으로 8도를 틀지만,
     * slot_bias_span_m(80mm)만 들어가면 기울임을 0으로 되돌려 슬롯 축에 정확히
     * 나란하게 만든다. 그런데 '걸렸다'는 사실 자체가 차체가 그 벽 쪽으로
     * 치우쳐 있다는 관측이므로, 축으로 완전히 되돌리면 남은 치우침이 그대로 남아
     * 같은 벽을 또 긁는다.
     *
     * 그래서 큰 기울임이 끝난 뒤에도 '걸렸던 벽의 반대쪽'으로 이만큼은 남겨 둔다.
     * 2도면 남은 진입깊이 200mm 동안 옆으로 7mm 를 더 벌어준다
     * (200 x sin2도 = 7.0mm). 슬롯 편측 여유가 40mm 이므로 안전하고,
     * 착석 판정의 자세 조건(6도)에도 여유 있게 들어간다.
     * 0 으로 주면 예전처럼 축으로 완전히 되돌린다. */
    double slot_residual_bias_deg;

    /* ---- 안쪽 벽 밀착 (2026-08-09f) ----
     * 요청: "머리가 박을 때까지 완전히 딱 붙여라. 벽면은 튼튼하다."
     * true 면 목표 y 도달을 종료조건으로 쓰지 않고, '더 이상 안 들어감(정체)'
     * 즉 실제 접촉만을 종료조건으로 쓴다. 목표 y 는 감속 기준점과 안전한계로만 쓴다. */
    bool   slot_press_to_wall;
    double slot_press_over_m;     /* 목표 y 보다 이만큼 더 들어가면 안전상 정지 */
    double slot_press_sec;        /* 접촉 후 확실히 앉히는 마지막 밀기 시간(초) */
    double slot_seat_depth_tol_m; /* 착석 인정 깊이오차 - 목표 y 로부터 이 안이어야 함 */

    /* 슬롯 대기점 접근을 끝낼 상자 (2026-08-09g).
     * 좌우(슬롯 축에 수직) / 전후(슬롯 축 방향) 허용오차.
     * 이 안에 들어오면 도킹 미세조정을 생략하고 슬롯 정렬 루틴으로 넘긴다. */
    double slot_approach_x_tol;
    double slot_approach_y_tol;

    /* ---- 슬롯 안 라이다 실측 재정박 (2026-08-11a) ----
     * robot_runner__slot_reanchor() 주석 참고. 슬롯 평행구간 안에서는 스캔매칭이
     * 원리적으로 옆으로 미끄러질 수 있으므로(likelihood field 가 벽 속을 만점으로
     * 주기 때문), 좌우/각도는 라이다 실측으로 못 박고 깊이만 1차원으로 다시 맞춘다.
     * false 로 두면 예전(2026-08-10n) 동작 그대로. --no-slot-lidar-fix */
    bool   slot_lidar_fix;
    /* 라이다가 '슬롯 평행구간 안 + 좌우 중앙 + 안쪽 벽 접촉'을 확인해 주면
     * 이 각도까지는 착석으로 인정한다 (사용자 요청: "각도가 조금 틀어진 정도는
     * 진입 시엔 괜찮다. 탈출할 때 알아서 나가면 된다").
     * 근거: yaw e 에서 필요한 가로폭이 130*cos(e)+255*sin(e) 이고 210mm 를 넘는
     * 임계각이 20.2도다. 12도면 가로폭 180mm 로 30mm 여유가 남는다. */
    double slot_seat_yaw_deg;

    /* 엔코더가 아직 물리적으로 없거나 신뢰할 수 없을 때 false로 설정. false면
     * fpga_link_get_pose()의 값을 오도메트리로 아예 쓰지 않고(엔코더 배선이 없어서
     * 실제로는 항상 (0,0,0)만 돌아오거나, 뜬 핀 노이즈로 유령 틱이 들어와 조용히
     * 지도를 오염시킬 수 있음), 매 스텝 오도메트리 변화량을 명시적으로 (0,0,0)으로
     * 고정한 뒤 라이다 스캔매칭(scan_match_correct_pose)만으로 위치를 추정함.
     * 안전성 근거: 이 로봇의 최대 바퀴속도(MAX_WHEEL_MPS ~0.165m/s)와 제어주기
     * (control_hz=20Hz) 기준 스텝당 최대 이동량은 위치 ~8mm, 회전 ~6deg 수준으로,
     * slam_init(_from_grid)에 쓰이는 기본 탐색창(위치 ±0.15m, 각도 ±12deg)에
     * 충분히 여유있게 들어옴 - 즉 오도메트리 예측 없이(항상 "제자리" 예측) 매 스텝
     * 스캔매칭만으로도 정상 추적 가능. 단, 실제 제어루프가 계산부하 등으로 20Hz보다
     * 느려지면(=스텝당 실제 이동량이 커짐) 이 가정이 깨질 수 있으니, 하드웨어에서
     * 로그의 실제 스텝 주기를 반드시 확인할 것. */
    bool has_encoder;

    /* 시작 각도(도). auto_localize_start=true면 이 값은 무시되고 라이다로 자동
     * 탐색함. 자동탐색이 방 대칭 때문에 엉뚱한 방향을 고르면, --start-theta로
     * 직접 지정하고 --no-auto-localize를 주면 됨. */
    double start_theta_deg;
    bool auto_localize_start;
} RobotConfig;
void robot_config_defaults(RobotConfig *cfg);

extern volatile sig_atomic_t g_stop_requested;
void robot_handle_sigint(int sig);

#endif /* ROBOT_CONFIG_H */
