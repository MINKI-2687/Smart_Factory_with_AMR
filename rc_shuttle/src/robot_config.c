#include "robot_config.h"
#include "robot_runner.h"


void robot_config_defaults(RobotConfig *cfg) {
    cfg->wheel_diameter = 0.066;
    cfg->wheel_separation = 0.160;
    cfg->ticks_per_rev = 20;
    cfg->controller_max_speed = 80;
    cfg->pwm_safety_cap = 255;

    cfg->resolution = 0.05;
    cfg->robot_radius = 0.10;
    cfg->control_hz = 20;
    cfg->body_length = 0.255;
    cfg->body_width  = 0.130;
    cfg->min_arc_speed = 14;
    cfg->cruise_speed = 26;
    cfg->dock_step_speed = 22;
    cfg->allow_crab = false;
    cfg->slot_trim_tilt_deg = 55.0;
    cfg->trigger_delay_sec = 7.0;
    /* 0.06 -> 0.10 (2026-08-09n).
     * 회전 중에는 위치가 거의 안 변한다는 전제였지만, 회전중심 오차(실측 25mm)와
     * 연속 스윕 때문에 실제로는 한 스텝에 50mm 가까이 밀릴 수 있다. 창이 60mm 면
     * 그 순간 봉우리가 창 밖으로 나가고, 한 번 나가면 다시는 못 돌아온다.
     * 간격을 0.03 -> 0.04 로 키워 후보 수는 6x6x26=936 으로 유지(기본 1089보다 적음). */
    cfg->rot_win_m    = 0.10;
    cfg->rot_step_m   = 0.04;
    cfg->rot_win_deg  = 25.0;
    cfg->rot_step_deg = 2.0;

    cfg->grid_size_x = 3.0;
    cfg->grid_size_y = 3.0;
    cfg->known_obstacles[0].x = 1.5; cfg->known_obstacles[0].y = 1.5; cfg->known_obstacles[0].radius = 0.2;
    cfg->n_known_obstacles = 1;

    cfg->start_x = 0.2; cfg->start_y = 0.2;
    cfg->goal_x = 2.5; cfg->goal_y = 2.5;
    cfg->has_goal_theta = true;
    cfg->goal_theta_deg = 0.0;

    cfg->slam_mapping_max_steps = 20 * 60;
    cfg->has_encoder = true;   /* 하드웨어에 엔코더가 없으면 호출부에서 false로 바꿀 것 */
    cfg->start_theta_deg = 0.0;
    cfg->auto_localize_start = true;
    cfg->slam_mapping_goal_tolerance = 0.15;
    cfg->fpga_send_hz = 100;
    cfg->lidar_yaw_offset_deg = 0.0;
    cfg->lidar_mirror = false;
    cfg->lidar_offset_forward_m = 0.0;
    cfg->encoder_invert_left = false;
    cfg->encoder_invert_right = false;
    cfg->rot_clamp_frac = 0.0;   /* 0이면 has_encoder에 따라 자동(0.6 / 1.0) */
    cfg->min_match_quality = 0.70;
    cfg->slot_parking = false;
    /* 대기점 y. PATCH(2026-08-06): 0.57이었는데, 그 높이에서는 제자리 회전 시 차체
     * 뒤쪽이 y=0.422까지 내려가 슬롯 입구 벽(0.45)을 침범해서 180도 회전이 물리적으로
     * 불가능했음(시뮬레이션에서 B로 가는 구간이 벽을 긁으며 영원히 멈춰있는 것으로
     * 발견됨). 차체 반대각선이 14.8cm이므로 회전 가능한 중심 y는 0.598~0.702이고,
     * 그 정중앙인 0.65로 옮김. */
    /* 0.65 -> 0.60 (2026-08-08). 자세추정값은 '라이다 위치'이고 라이다가 차체중심보다
     * 6cm 앞에 있으므로, 슬롯을 보고 설 때 차 뒤끝은 pose_y + 0.06 + 0.1275 임.
     * 0.65면 뒤끝이 y=0.837 -> 천장벽(0.85)까지 13mm밖에 안 남아서 후진 중 벽에 닿음
     * (실기 확인). 제자리 회전까지 하려면 차체중심 반경 0.16m가 비어야 하므로
     * 가능한 범위는 0.550~0.630이고, 그 중간인 0.600으로 잡음. */
    cfg->slot_staging_y = 0.60;
    /* 1.0 -> 2.5 (2026-08-09a). --help 안내문에는 이미 "1.0은 너무 작아 제자리 회전을
     * 반복함, 2.5 권장"이라고 적혀 있었는데 기본값만 1.0으로 남아 있었음.
     * 이 하드웨어의 최소 회전 스텝이 약 3.5도라 1도 안으로는 물리적으로 못 들어가고,
     * 스캔매칭 각도 노이즈도 1~2도라 노이즈를 쫓아 계속 돌게 됨
     * (사용자 신고: "각도가 맞는데 계속 틀어버린다"). 진입 깊이 40cm 기준 2.5도는
     * 좌우 약 17mm 밀림이고, 그 정도는 V자 깔때기가 잡아준다. */
    cfg->slot_align_tol_deg = 2.5;
    cfg->slot_max_tries = 3;
    cfg->slot_lidar_fix = true;
    cfg->slot_seat_yaw_deg = 12.0;
    cfg->stop_and_go = true;
    cfg->hybrid_cruise = true;
    cfg->pure_pursuit = true;
    cfg->lookahead_m = 0.22;
    /* 0.45 -> 0.30 (2026-08-08). 이 대기의 목적은 (a) 라이다가 정지 상태에서 한 바퀴
     * (102ms)를 새로 돌게 하고 (b) 차체 진동이 잦아들기를 기다리는 것. 0.30이면 라이다
     * 한 바퀴의 약 3배라 충분함. 스텝 수가 많은 회전에서 총 시간의 대부분이 이 대기라
     * 여기를 줄이는 게 속도에 가장 효과적임(측정: 6.0s -> 4.5s, 오차 변화 없음). */
    cfg->sg_settle_sec = 0.30;
    cfg->sg_move_sec_min = 0.07;
    cfg->sg_move_sec_max = 0.45;
    /* 제자리 회전의 '정지마찰 돌파' 하한 (2026-08-10p).
     * 스키드 스티어는 회전할 때 타이어 네 개를 옆으로 문질러야 해서 직진보다
     * 훨씬 큰 토크가 필요하다. 실기 측정:
     *     70ms 펄스 -> '탁' 소리만 나고 자세 변화 0.0deg (사용자 신고)
     *     88ms 펄스 -> 7.4deg 회전 (같은 차, 같은 바닥)
     * 즉 돌파점이 70~88ms 사이다. 이보다 짧은 회전 펄스는 명령해봐야 소용이
     * 없으므로, 그런 스텝은 아예 만들지 않고 '자리를 만드는' 쪽으로 보낸다. */
    cfg->rot_breakaway_sec = 0.09;
    cfg->sg_step_speed = 34;
    /* 맵 보강 후 상향(2026-08-08). 22/0.105s는 V자 경사벽에서 미끄러지지 못하고
     * 걸리는 경우가 있었음. 28이면 약 58mm/s로, 예전 기본값(34=70mm/s)보다는
     * 여전히 느려서 충돌 에너지는 낮게 유지됨. */
    cfg->slot_step_speed = 28;
    cfg->slot_touch_speed = 18;
    /* 0.16 -> 0.22 (2026-08-10i). 위 '남은 거리보다 더 밀지 않는다' 블록이
     * 벽 근처에서 자동으로 줄여주므로, 열린 구간에서만 크게 물게 된다. */
    cfg->slot_move_sec = 0.22;
    cfg->slot_fast_above_y = 0.0;   /* 0 = 자동 (slot_staging_y - 0.22) */
    cfg->slot_fast_gain = 2.5;
    /* 열린 구간(깔때기 위 220mm)을 한 스텝에 통째로 (2026-08-10j).
     * 0.70s x 360mm/s = 252mm 이고, 실제로는 '깔때기 입구를 넘지 않는다'는
     * 상한이 먼저 걸려 정확히 입구에서 멈춘다. 그 구간이 2스텝 -> 1스텝이 되어
     * 고정비용(정지+스캔+계산 0.33s)을 한 번만 낸다. */
    cfg->slot_open_move_sec = 0.70;
    cfg->slot_open_speed = 0;       /* 0 = sg_step_speed(34) */
    /* 0.30 -> 0.15 (2026-08-10i) -> 0.12 (2026-08-10j).
     * 이 대기의 목적은 '정지 명령 뒤 차체 진동이 가라앉는 것'이고, 뒤이은
     * wait_for_fresh_scan() 이 스캔 2바퀴를 더 기다리므로 실제 보장은
     * "정지 후 slot_settle_sec 이 지난 뒤에 시작된 스캔"이다. 주행 구간에서
     * 이미 --sg-settle 0.12 로 문제없이 쓰고 있으므로 같은 값으로 맞춘다.
     * (한 스텝 고정비용 0.51s -> 0.33s) */
    cfg->slot_settle_sec = 0.12;
    cfg->slot_escape_bias_deg = 8.0;
    cfg->slot_bias_span_m = 0.08;
    cfg->slot_residual_bias_deg = 2.0;
    cfg->slot_press_to_wall = true;
    cfg->slot_press_over_m = 0.04;
    cfg->slot_press_sec = 0.35;
    cfg->slot_seat_depth_tol_m = 0.030;
    cfg->slot_approach_x_tol = 0.030;
    /* 0.100 -> 0.040 (2026-08-09h 회귀 수정).
     * 100mm 를 허용하면 y=0.5475 에서 접근이 끝날 수 있는데, 그 자리는 통로
     * 아래벽(y=0.45)까지 97mm 뿐이다. 이어지는 slot_align 은 90도 제자리 회전이라
     * 반길이 127.5mm + 여유 15mm = 142.5mm 가 필요하므로 돌지 못하고, 끼임 탈출을
     * 반복하게 된다(실기 로그에서 그대로 관측됨).
     * 통로 y 0.45~0.845 에서 90도 회전이 가능한 띠는 y in [0.5925, 0.7025]
     * = 대기점 0.6475 +-55mm 이므로 40mm 로 잡는다. 전후오차는 경로추종 특성상
     * 원래 10mm 수준이라 이 값으로 조여도 접근 시간이 늘지 않는다. */
    cfg->slot_approach_y_tol = 0.040;
    cfg->slam_occ_threshold = 0.6;
}

volatile sig_atomic_t g_stop_requested = 0;
void robot_handle_sigint(int sig) { (void)sig; g_stop_requested = 1; }
