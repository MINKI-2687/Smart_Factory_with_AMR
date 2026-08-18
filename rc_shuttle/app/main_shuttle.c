/* main_shuttle.c
 * 스마트팩토리 물류운송용 최종 실행파일. 지도를 한 번 획득(SLAM매핑/고정맵/파일불러오기)한
 * 뒤, A지점<->B지점을 무한 왕복. 각 지점에서 트리거(지금은 Enter키, 나중엔 감압센서)를
 * 기다렸다가 출발. Ctrl+C로 종료.
 *
 * 빌드: gcc -std=c11 -O2 -o main_shuttle main_shuttle.c -lm -lpthread  (또는 make main_shuttle)
 *
 * 사용 예:
 *   # 처음 한 번: SLAM으로 매핑하면서 지도를 파일로 저장
 *   ./main_shuttle --map-source slam --save-map factory.map \
 *          --fpga-device /dev/ttyUSB1 --lidar-device /dev/ttyUSB0 \
 *          --start 0.2 0.2 \
 *          --point-a 0.5 0.5 0 --point-b 2.5 2.5 180
 *
 *   # 이후: 저장해둔 지도를 불러와서 매핑 생략하고 바로 셔틀 시작
 *   ./main_shuttle --map-source load --load-map factory.map \
 *          --fpga-device /dev/ttyUSB1 --lidar-device /dev/ttyUSB0 \
 *          --start 0.5 0.5 \
 *          --point-a 0.5 0.5 0 --point-b 2.5 2.5 180
 *
 * 주의: --start(고정맵/불러오기 모드) 또는 매핑 시작위치(SLAM 모드)는 로봇의 실제
 * 물리적 현재 위치와 반드시 일치해야 합니다. --load-map을 쓸 땐 특히, 로봇을
 * 지도상의 정확한 좌표에 직접 놓아준 뒤 실행해야 합니다(자동으로 알아낼 방법이 없음).
 */
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "serial_port.h"
#include "robot_runner.h"

static void print_usage(const char *prog) {
    fprintf(stderr,
        "사용법: %s [옵션들]\n"
        "\n"
        "필수:\n"
        "  --fpga-device <경로>       예: /dev/ttyUSB1\n"
        "  --lidar-device <경로>      예: /dev/ttyUSB0\n"
        "  --point-a <X> <Y> <도>     A지점 좌표+주차각도\n"
        "  --point-b <X> <Y> <도>     B지점 좌표+주차각도\n"
        "\n"
        "지도 관련 (--map-source에 따라 달라짐):\n"
        "  --map-source slam|fixed|load   기본값: fixed\n"
        "  --save-map <파일경로>      slam/fixed로 새로 만든 지도를 저장 (선택)\n"
        "  --load-map <파일경로>      map-source=load일 때 필수, 이전에 저장한 지도 불러오기\n"
        "  --obstacle <X> <Y> <R>     (fixed 모드 전용) 알고 있는 장애물, 여러 번 반복 가능\n"
        "  --max-mapping-seconds <초> (slam 모드 전용) 매핑 최대시간, 기본값: 300초\n"
        "\n"
        "선택:\n"
        "  --fpga-baud <숫자>        기본값: 115200\n"
        "  --lidar-baud <숫자>       기본값: 460800\n"
        "  --grid-size <W> <H>       방 크기(m), 기본값: 3.0 3.0\n"
        "  --robot-radius <m>        기본값: 0.10\n"
        "  --start <X> <Y>           매핑/주행 시작 시 로봇의 실제 위치(m), 기본값: 0.2 0.2\n"
        "                            (반드시 로봇의 실제 물리적 시작위치와 일치해야 함)\n"
        "  --occ-threshold <0~1>     점유확률 임계값, 기본값: 0.6\n"
        "  --no-encoder              엔코더 미장착. 오도메트리를 아예 안 쓰고 라이다\n"
        "                            스캔만으로 위치추정 (기본값: 엔코더 있음)\n"
        "  --ticks-per-rev <숫자>    바퀴 1회전당 엔코더 틱수. 기본 20인데 이건 '바퀴축\n"
        "                            20슬롯' 가정값이라, 감속기 달린 모터축 엔코더면\n"
        "                            1000 이상인 경우가 많음. 이 값이 틀리면 오도메트리\n"
        "                            전체가 무의미해지고 자세추정이 발산함.\n"
        "                            ./encoder_cal <장치> <보드레이트> 로 실측할 것.\n"
        "  --start-theta <도>        시작 각도를 직접 지정 (자동탐색을 끔)\n"
        "  --no-auto-localize        시작자세 자동탐색을 끄고 0도(또는 --start-theta)로 시작\n"
        "  --no-pure-pursuit         경로추종을 예전 방식(웨이포인트 하나씩 찍기)으로\n"
        "  --lookahead <m>           pure pursuit 전방주시 거리, 기본 0.22\n"
        "  --no-hybrid-cruise        전진 구간도 매번 멈춤(예전 stop-and-go 전체 적용)\n"
        "  --slot-parking            주차 슬롯 모드. point-a/b를 슬롯 안 최종 주차자세로\n"
        "                            보고, A*는 입구 바깥 대기점까지만. 나올 땐 후진\n"
        "  --slot-staging <y>        슬롯 입구 바깥 대기점의 y (기본 0.57)\n"
        "  --lidar-mirror            라이다 각도를 좌우반전(시계->반시계 규약 변환).\n"
        "                            GUI에서 로봇이 좌우 뒤집힌 위치로 보이면 이걸 켤 것\n"
        "  --lidar-yaw <도>          라이다 0도가 차체 정면 기준 몇 도 틀어져 장착됐는지.\n"
        "                            live_view.py의 방향선이 차체 뒤를 가리키면 180. 기본 0\n"
        "  --trigger-gpio-lines <목록> Enter 대신 GPIO 입력을 출발 트리거로 사용.\n"
        "                            쉼표로 여러 개(예: 17,27,22,23). OR 판정이라\n"
        "                            하나라도 1이면 '박스 있음'으로 봄.\n"
        "                            A지점=박스 올라오면 출발,\n"
        "                            B지점=박스 내려가면 출발.\n"
        "                            (--trigger-gpio 도 같은 옵션의 별칭)\n"
        "  --trigger-gpio-chip <경로> 기본 /dev/gpiochip0. 라즈베리파이 5는 /dev/gpiochip4\n"
        "                            (`gpiodetect`로 확인)\n"
        "  --trigger-active-low      센서가 '감지됨'일 때 0을 내는 경우\n"
        "  --trigger-pull-up         내부 풀업 사용(오픈드레인 출력 센서)\n"
        "  --trigger-debounce <초>   기본 0.05. 조도센서가 튀면 0.1~0.2로 올릴 것\n"
        "  --trigger-allow-enter     GPIO 모드에서도 Enter로 수동 통과 허용(디버그용)\n"
        "  --encoder-invert-left     좌측 엔코더 틱 부호 반전(직진 시 좌우 부호가\n"
        "                            반대로 나올 때). --encoder-invert-right 도 있음\n"
        "  --rot-clamp-frac <배>     회전 스텝 클램프 계수(탐색창 대비). 한 스텝에\n"
        "                            '탐색창 x 이 값'보다 크게 돌지 않도록 이동시간을\n"
        "                            제한함. 기본은 엔코더 없으면 0.7, 있으면 2.0.\n"
        "                            (회전 각속도는 모델값이 아니라 주행 중 스캔매칭\n"
        "                            관측으로 계속 실측 보정되므로 보통 손댈 필요 없음)\n"
        "  --lidar-offset <m>        라이다가 차체 회전중심보다 앞으로 나간 거리(m).\n"
        "                            실측해서 넣을 것. 안 넣으면 회전마다 위치추정이\n"
        "                            이 값의 2배만큼 출렁임. 기본 0\n"
        "  --slot-residual-bias <도> 슬롯 진입 중 벽에 걸려 각도를 복구한 뒤, 슬롯 축으로\n"
        "                            완전히 되돌리지 않고 걸렸던 벽 반대로 남겨 둘 각도.\n"
        "                            기본 2.0 (남은 200mm 동안 옆으로 7mm 더 벌어짐).\n"
        "                            같은 벽에 또 걸리면 3~4 로 올리세요. 0 이면 끔.\n"
        "  --min-match-quality <0~1> 이 일치도 아래로 3스텝 연속 떨어지면 바퀴를 멈추고\n"
        "                            자세부터 다시 잡습니다. 못 고치면 슬롯에 진입하지\n"
        "                            않고 이번 시도를 접습니다. 기본 0.70, 0 이면 끔.\n"
        "                            (자세가 맞으면 이 세트장에서 99%%까지 나옵니다.\n"
        "                             자꾸 멈추면 0.60 으로, 헛주차가 보이면 0.80 으로)\n"
        "  --slot-align-tol <도>     슬롯 진입 전 허용 각도오차, 기본 2.5.\n"
        "                            (2026-08-09a: 기본값을 1.0에서 2.5로 올림.\n"
        "                             한 스텝 최소 회전이 약 3.5도이고 스캔매칭 각도\n"
        "                             노이즈가 1~2도라, 1.0으로는 목표를 사이에 두고\n"
        "                             제자리 회전만 반복하다 끝났음)\n"
        "  --slot-seat-yaw <도>      슬롯 안에서 착석으로 인정할 각도오차 상한, 기본 12.\n"
        "                            좌우 벽이 둘 다 보일 때만 적용됨(라이다 실측 각도).\n"
        "                            '진입할 때 각도가 조금 틀어진 건 두고, 나갈 때\n"
        "                             통로에서 푼다'는 방침. 임계각(210mm 슬롯에\n"
        "                             130x255mm 차체가 안 들어가는 각)이 20.2도라\n"
        "                             12도면 가로폭 180mm 로 30mm 여유가 남음.\n"
        "  --no-slot-lidar-fix       슬롯 안 라이다 실측 재정박을 끔(예전 동작).\n"
        "                            켜져 있으면 슬롯 평행구간 안에서 좌우/각도를\n"
        "                            라이다로 못 박고 깊이만 1차원으로 다시 맞춤.\n"
        "                            스캔매칭이 옆으로 미끄러져 '다 넣어놓고 도로\n"
        "                             빼는' 사고를 막는 장치이므로 끄지 않길 권장.\n"
        "  --slot-max-tries <n>      한 지점에서 주차를 다시 시도할 횟수, 기본 3.\n"
        "                            실패하면 '슬롯 밖으로 빼기 -> 접촉 풀기 ->\n"
        "                            대기점으로 A* 재접근 -> 재정렬 -> 재진입'을\n"
        "                            이 횟수만큼 반복하고, 그래도 안 되면 중단함.\n"
        "                            1로 주면 예전처럼 한 번 실패에 바로 중단\n"
        "  --slot-speed <0~80>       슬롯 진입/후진 속도, 기본 28 (일반주행 34보다 느리게)\n"
        "  --slot-touch-speed <0~80> 안쪽 벽 밀착 속도, 기본 18\n"
        "  --slot-move-sec <초>      슬롯 안에서 한 스텝에 미는 시간, 기본 0.22.\n"
        "                            벽 근처에서는 남은 거리에 맞춰 자동으로 짧아지므로\n"
        "                            0.28~0.30 까지 올려도 안전합니다(더 빠른 진입).\n"
        "                            경사벽에 걸리면 늘리고, 너무 세게 박으면 줄일 것\n"
        "  --slot-settle-sec <초>    슬롯 안 스텝 사이 정지 시간, 기본 0.12.\n"
        "                            더 답답하면 0.10, 흔들리면 0.20 으로\n"
        "  --slot-open-move-sec <초> 깔때기 위 '열린 구간'에서 한 스텝 최대 밀기 시간,\n"
        "                            기본 0.70. 이 구간(대기점~깔때기 입구 약 220mm)은\n"
        "                            좌우가 넓어 중간에 다시 볼 이유가 없어서 한 번에\n"
        "                            통째로 밉니다. '깔때기 입구를 넘지 않는다'는 상한이\n"
        "                            따로 걸리므로 크게 줘도 안전합니다. 0 이면 예전처럼\n"
        "                            --sg-move-sec-max(0.45) 로 잘립니다\n"
        "  --slot-open-speed <0~80>  그 열린 구간의 속도. 기본 0 = --sg-step-speed(34).\n"
        "                            깔때기 아래로는 항상 --slot-speed(28)를 씁니다\n"
        "  --slot-bias-deg <도>      벽에 걸려 재진입할 때, 슬롯 축이 아니라 '걸린 벽\n"
        "                            반대쪽'으로 이만큼 더 틀어서 넣음. 기본 8.\n"
        "                            걸렸다 = 그 벽에 붙어 있다는 뜻이라, 각도만\n"
        "                            바로잡아 넣으면 같은 벽에 또 걸림. 비스듬히\n"
        "                            들어가야 좌우 위치가 바뀜(8도로 80mm 들어가면\n"
        "                            옆으로 11mm). 0으로 주면 예전처럼 축으로만 복귀.\n"
        "                            상한 15(임계각 20.2도보다 안쪽)\n"
        "  --rot-breakaway <s>       제자리 회전이 실제로 시작되는 최소 펄스, 기본 0.09.\n"
        "                            이보다 짧은 회전 명령은 '탁' 소리만 나고 안 돕니다.\n"
        "                            모터/바닥/배터리가 바뀌면 여기를 조정하세요.\n"
        "  --slot-bias-span <m>      그 기울임을 유지할 진입 깊이, 기본 0.08.\n"
        "                            이만큼 더 들어가면 슬롯 축으로 되돌림\n"
        "  --no-slot-press           안쪽 벽에 밀착시키지 않고 목표 y 에서 멈춤.\n"
        "                            기본은 밀착(머리가 벽에 닿을 때까지 진입)\n"
        "  --slot-press-sec <초>     마지막 밀착 밀기 시간, 기본 0.35\n"
        "  --rot-window <각도창deg> <각도간격deg> <위치창m> <위치간격m>\n"
        "                            제자리 회전 중 스캔매칭 탐색창, 기본 25 2.0 0.06 0.03.\n"
        "                            한 스텝 회전량 = 각도창 x rot-clamp-frac 이므로\n"
        "                            각도창을 넓히면 회전이 빨라짐. 회전 중엔 위치가\n"
        "                            거의 안 변해 위치창을 좁혀도 되고, 후보 수가\n"
        "                            n_xy^2 x n_th 라 총 계산량은 오히려 줄어듦\n"
        "  --slot-fast <y> <배수>    이 y 보다 위(깔때기 위 열린 구간)에서 한 스텝을\n"
        "                            몇 배로 길게 밀지. 기본 자동(대기점-0.22) 2.5.\n"
        "                            배수 1 이면 예전처럼 전 구간 같은 속도\n"
        "  --trigger-delay <초>      출발 트리거를 받은 뒤 실제로 움직이기까지 대기,\n"
        "                            기본 7.0. 0 이면 즉시 출발\n"
        "  --slam-sigma <거친> <정밀> 스캔매칭 점수판 시그마(m), 기본 0.06 0.04.\n"
        "                            거친 값이 클수록 벽 오차에 관대하지만 봉우리가\n"
        "                            뭉툭해져 위치가 덜 정밀해짐. 세트장 벽이 잘\n"
        "                            맞으면 둘 다 낮추고, 들쭉날쭉하면 높이세요\n"
        "  --cruise-speed <값>       통로 연속주행 목표 속도, 기본 26 (sg-step-speed 는 34).\n"
        "                            낮출수록 경로 이탈이 줄지만 선회반경이 커짐\n"
        "                            (26 -> 반경 약 2.4m, 34 -> 약 1.5m).\n"
        "                            데드존 때문에 실제로 쓸 수 있는 범위는 약 18~80\n"
        "  --dock-speed <값>         마지막 대기점 접근 목표 속도, 기본 22.\n"
        "                            보폭이 크면 목표를 넘나들며 전진/후진을 반복함\n"
        "  --allow-crab              게걸음(옆으로 밀어내기)을 허용. 기본은 꺼져 있고,\n"
        "                            대신 앞/뒤로 물러나 여유를 만든 뒤 회전함\n"
        "  --slot-trim-tilt <도>     좌우 보정 시 슬롯 축에서 기울일 각도, 기본 55.\n"
        "                            90 이면 순수 좌우 이동(회전 왕복 180도),\n"
        "                            55 면 이동거리 1.22배 대신 회전 왕복 110도.\n"
        "                            작을수록 회전은 줄지만 세로로 더 밀림\n"
        "  --slot-seat-tol <m>       착석 인정 깊이오차, 기본 0.030. 목표 y 에서 이\n"
        "                            안으로 들어온 상태에서 벽에 닿아야 주차 완료로\n"
        "                            인정. 접촉만으로는 완료가 되지 않음\n"
        "  --slot-approach-tol <좌우> <전후>\n"
        "                            대기점 접근을 끝낼 상자(m), 기본 0.030 0.100.\n"
        "                            이 안에 들어오면 도킹 미세조정을 생략하고 슬롯\n"
        "                            정렬 루틴으로 넘김 (슬롯 앞 찔끔찔끔 방지).\n"
        "                            0 0 으로 주면 예전처럼 도킹까지 다 함\n"
        "  --body-size <길이> <폭>   차체 실측 치수(m), 기본 0.255 0.130.\n"
        "                            제자리 회전 가능 판정에 씀 - 원(robot-radius)이\n"
        "                            아니라 직사각형으로 따져야 통로에서 과잉판정이\n"
        "                            안 남\n"
        "  --no-stop-and-go          연속주행으로 되돌림 (기본은 멈춤-관측-이동 반복)\n"
        "  --sg-step-speed <0~80>    이동 중 고정속도, 기본 34\n"
        "  --sg-settle <초>          정지 후 안정화 대기, 기본 0.30\n"
        "  --sg-move-min <초>        한 번 이동 최소시간, 기본 0.10\n"
        "  -h, --help                도움말 출력\n"
        "\n"
        "트리거(현재는 Enter키 테스트용, 나중에 감압센서로 교체 예정 - trigger.h 참고):\n"
        "  A지점 도착 후: 짐이 얹히는 트리거를 기다림\n"
        "  B지점 도착 후: 짐이 내려가는 트리거를 기다림\n",
        prog);
}

int main(int argc, char **argv) {
    RobotConfig cfg;
    robot_config_defaults(&cfg);

    /* 출발 트리거 설정 - 기본은 터미널 Enter, --trigger-gpio로 외부센서 전환 */
    TriggerConfig trigger_cfg;
    trigger_config_default(&trigger_cfg);
    cfg.n_known_obstacles = 0;
    cfg.slam_mapping_max_steps = cfg.control_hz * 300;

    const char *map_source_str = "fixed";
    const char *fpga_device = NULL;
    int fpga_baud = 115200;
    const char *lidar_device = NULL;
    int lidar_baud = 460800;
    const char *save_map_path = NULL;
    const char *load_map_path = NULL;

    ShuttlePoint point_a = {0}, point_b = {0};
    bool point_a_set = false, point_b_set = false;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
            print_usage(argv[0]);
            return 0;
        } else if (!strcmp(argv[i], "--map-source") && i + 1 < argc) {
            map_source_str = argv[++i];
        } else if (!strcmp(argv[i], "--fpga-device") && i + 1 < argc) {
            fpga_device = argv[++i];
        } else if (!strcmp(argv[i], "--fpga-baud") && i + 1 < argc) {
            fpga_baud = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--lidar-device") && i + 1 < argc) {
            lidar_device = argv[++i];
        } else if (!strcmp(argv[i], "--lidar-baud") && i + 1 < argc) {
            lidar_baud = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--grid-size") && i + 2 < argc) {
            cfg.grid_size_x = atof(argv[++i]);
            cfg.grid_size_y = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--robot-radius") && i + 1 < argc) {
            cfg.robot_radius = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--start") && i + 2 < argc) {
            cfg.start_x = atof(argv[++i]);
            cfg.start_y = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--point-a") && i + 3 < argc) {
            point_a.x = atof(argv[++i]);
            point_a.y = atof(argv[++i]);
            point_a.theta_deg = atof(argv[++i]);
            point_a_set = true;
        } else if (!strcmp(argv[i], "--point-b") && i + 3 < argc) {
            point_b.x = atof(argv[++i]);
            point_b.y = atof(argv[++i]);
            point_b.theta_deg = atof(argv[++i]);
            point_b_set = true;
        } else if (!strcmp(argv[i], "--obstacle") && i + 3 < argc) {
            if (cfg.n_known_obstacles < 16) {
                cfg.known_obstacles[cfg.n_known_obstacles].x = atof(argv[++i]);
                cfg.known_obstacles[cfg.n_known_obstacles].y = atof(argv[++i]);
                cfg.known_obstacles[cfg.n_known_obstacles].radius = atof(argv[++i]);
                cfg.n_known_obstacles++;
            } else {
                fprintf(stderr, "[main_shuttle] 경고: 장애물은 최대 16개까지만 등록됩니다\n");
                i += 3;
            }
        } else if (!strcmp(argv[i], "--max-mapping-seconds") && i + 1 < argc) {
            cfg.slam_mapping_max_steps = (int)(atof(argv[++i]) * cfg.control_hz);
        } else if (!strcmp(argv[i], "--occ-threshold") && i + 1 < argc) {
            cfg.slam_occ_threshold = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--ticks-per-rev") && i + 1 < argc) {
            cfg.ticks_per_rev = atoi(argv[++i]);
            if (cfg.ticks_per_rev < 1) {
                fprintf(stderr, "[인자] --ticks-per-rev 는 1 이상이어야 합니다\n");
                return 1;
            }
        } else if (!strcmp(argv[i], "--no-encoder")) {
            cfg.has_encoder = false;
        } else if (!strcmp(argv[i], "--start-theta") && i + 1 < argc) {
            cfg.start_theta_deg = atof(argv[++i]);
            cfg.auto_localize_start = false;
        } else if (!strcmp(argv[i], "--no-auto-localize")) {
            cfg.auto_localize_start = false;
        } else if (!strcmp(argv[i], "--no-pure-pursuit")) {
            cfg.pure_pursuit = false;
        } else if (!strcmp(argv[i], "--lookahead") && i + 1 < argc) {
            cfg.lookahead_m = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--no-hybrid-cruise")) {
            cfg.hybrid_cruise = false;
        } else if (!strcmp(argv[i], "--slot-parking")) {
            cfg.slot_parking = true;
        } else if ((!strcmp(argv[i], "--slot-staging") ||
                    !strcmp(argv[i], "--slot-staging-y")) && i + 1 < argc) {
            cfg.slot_staging_y = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--lidar-mirror")) {
            cfg.lidar_mirror = true;
        } else if (!strcmp(argv[i], "--lidar-yaw") && i + 1 < argc) {
            cfg.lidar_yaw_offset_deg = atof(argv[++i]);
        } else if ((!strcmp(argv[i], "--trigger-gpio") ||
                    !strcmp(argv[i], "--trigger-gpio-lines")) && i + 1 < argc) {
            /* "17" 또는 "17,27,22,23" 둘 다 받음. 여러 개면 OR 판정. */
            trigger_cfg.type = TRIGGER_GPIO;
            trigger_cfg.gpio_line_count = 0;
            for (char *tok = strtok(argv[++i], ","); tok != NULL; tok = strtok(NULL, ",")) {
                if (trigger_cfg.gpio_line_count >= TRIGGER_MAX_GPIO_LINES) {
                    fprintf(stderr, "[인자] GPIO 라인은 최대 %d개까지입니다\n",
                            TRIGGER_MAX_GPIO_LINES);
                    return 1;
                }
                trigger_cfg.gpio_lines[trigger_cfg.gpio_line_count++] = atoi(tok);
            }
            if (trigger_cfg.gpio_line_count == 0) {
                fprintf(stderr, "[인자] --trigger-gpio-lines 값이 비어 있습니다\n");
                return 1;
            }
        } else if (!strcmp(argv[i], "--trigger-gpio-chip") && i + 1 < argc) {
            trigger_cfg.gpio_chip = argv[++i];
        } else if (!strcmp(argv[i], "--trigger-active-low")) {
            trigger_cfg.gpio_active_low = true;
        } else if (!strcmp(argv[i], "--trigger-pull-up")) {
            trigger_cfg.gpio_pull_up = true;
        } else if (!strcmp(argv[i], "--trigger-allow-enter")) {
            trigger_cfg.allow_enter_override = true;
        } else if (!strcmp(argv[i], "--trigger-debounce") && i + 1 < argc) {
            trigger_cfg.gpio_debounce_sec = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--encoder-invert-left")) {
            cfg.encoder_invert_left = true;
        } else if (!strcmp(argv[i], "--encoder-invert-right")) {
            cfg.encoder_invert_right = true;
        } else if (!strcmp(argv[i], "--rot-clamp-frac") && i + 1 < argc) {
            cfg.rot_clamp_frac = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--lidar-offset") && i + 1 < argc) {
            cfg.lidar_offset_forward_m = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--slot-residual-bias") && i + 1 < argc) {
            cfg.slot_residual_bias_deg = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--min-match-quality") && i + 1 < argc) {
            cfg.min_match_quality = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--slot-align-tol") && i + 1 < argc) {
            cfg.slot_align_tol_deg = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--slot-seat-yaw") && i + 1 < argc) {
            cfg.slot_seat_yaw_deg = atof(argv[++i]);
            if (cfg.slot_seat_yaw_deg < 2.0)  cfg.slot_seat_yaw_deg = 2.0;
            if (cfg.slot_seat_yaw_deg > 18.0) cfg.slot_seat_yaw_deg = 18.0;
        } else if (!strcmp(argv[i], "--no-slot-lidar-fix")) {
            cfg.slot_lidar_fix = false;
        } else if (!strcmp(argv[i], "--slot-max-tries") && i + 1 < argc) {
            cfg.slot_max_tries = atoi(argv[++i]);
            if (cfg.slot_max_tries < 1) cfg.slot_max_tries = 1;
        } else if (!strcmp(argv[i], "--slot-speed") && i + 1 < argc) {
            cfg.slot_step_speed = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--slot-move-sec") && i + 1 < argc) {
            cfg.slot_move_sec = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--slot-settle-sec") && i + 1 < argc) {
            cfg.slot_settle_sec = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--slot-bias-deg") && i + 1 < argc) {
            cfg.slot_escape_bias_deg = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--rot-breakaway") && i + 1 < argc) {
            cfg.rot_breakaway_sec = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--slot-bias-span") && i + 1 < argc) {
            cfg.slot_bias_span_m = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--no-slot-press")) {
            cfg.slot_press_to_wall = false;
        } else if (!strcmp(argv[i], "--slot-press-sec") && i + 1 < argc) {
            cfg.slot_press_sec = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--rot-window") && i + 4 < argc) {
            cfg.rot_win_deg  = atof(argv[++i]);
            cfg.rot_step_deg = atof(argv[++i]);
            cfg.rot_win_m    = atof(argv[++i]);
            cfg.rot_step_m   = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--slot-fast") && i + 2 < argc) {
            cfg.slot_fast_above_y = atof(argv[++i]);
            cfg.slot_fast_gain    = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--slot-open-move-sec") && i + 1 < argc) {
            cfg.slot_open_move_sec = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--slot-open-speed") && i + 1 < argc) {
            cfg.slot_open_speed = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--trigger-delay") && i + 1 < argc) {
            cfg.trigger_delay_sec = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--slam-sigma") && i + 2 < argc) {
            g_slam_sigma_coarse = atof(argv[++i]);
            g_slam_sigma_fine   = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--cruise-speed") && i + 1 < argc) {
            cfg.cruise_speed = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--dock-speed") && i + 1 < argc) {
            cfg.dock_step_speed = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--allow-crab")) {
            cfg.allow_crab = true;
        } else if (!strcmp(argv[i], "--slot-trim-tilt") && i + 1 < argc) {
            cfg.slot_trim_tilt_deg = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--slot-seat-tol") && i + 1 < argc) {
            cfg.slot_seat_depth_tol_m = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--slot-approach-tol") && i + 2 < argc) {
            cfg.slot_approach_x_tol = atof(argv[++i]);
            cfg.slot_approach_y_tol = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--body-size") && i + 2 < argc) {
            cfg.body_length = atof(argv[++i]);
            cfg.body_width  = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--slot-touch-speed") && i + 1 < argc) {
            cfg.slot_touch_speed = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--no-stop-and-go")) {
            cfg.stop_and_go = false;
        } else if (!strcmp(argv[i], "--sg-step-speed") && i + 1 < argc) {
            cfg.sg_step_speed = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--sg-settle") && i + 1 < argc) {
            cfg.sg_settle_sec = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--sg-move-min") && i + 1 < argc) {
            cfg.sg_move_sec_min = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--save-map") && i + 1 < argc) {
            save_map_path = argv[++i];
        } else if (!strcmp(argv[i], "--load-map") && i + 1 < argc) {
            load_map_path = argv[++i];
        } else {
            fprintf(stderr, "[main_shuttle] 알 수 없는 옵션: %s\n\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
    }

    MapSource map_source;
    if (!strcmp(map_source_str, "slam")) map_source = MAP_SOURCE_SLAM;
    else if (!strcmp(map_source_str, "fixed")) map_source = MAP_SOURCE_FIXED;
    else if (!strcmp(map_source_str, "load")) map_source = MAP_SOURCE_LOAD;
    else {
        fprintf(stderr, "[main_shuttle] --map-source는 slam, fixed, load 중 하나여야 합니다\n");
        return 1;
    }

    if (!fpga_device || !lidar_device) {
        fprintf(stderr, "[main_shuttle] --fpga-device / --lidar-device는 필수입니다\n\n");
        print_usage(argv[0]);
        return 1;
    }
    if (!point_a_set || !point_b_set) {
        fprintf(stderr, "[main_shuttle] --point-a와 --point-b는 둘 다 필수입니다 (각각 X Y 각도)\n\n");
        print_usage(argv[0]);
        return 1;
    }
    if (map_source == MAP_SOURCE_LOAD && !load_map_path) {
        fprintf(stderr, "[main_shuttle] --map-source load일 땐 --load-map <파일경로>가 필수입니다\n");
        return 1;
    }
    if (map_source == MAP_SOURCE_FIXED && cfg.n_known_obstacles == 0) {
        fprintf(stderr, "[main_shuttle] 경고: --map-source fixed인데 --obstacle이 하나도 없습니다. "
                        "빈 방(벽만 있음)으로 진행합니다.\n");
    }

    printf("=== 설정 ===\n");
    if (load_map_path) {
        printf("map_source   = %s (load-map=%s)\n", map_source_str, load_map_path);
    } else {
        printf("map_source   = %s\n", map_source_str);
    }
    printf("fpga         = %s @ %d baud\n", fpga_device, fpga_baud);
    printf("lidar        = %s @ %d baud\n", lidar_device, lidar_baud);
    printf("grid_size    = %.2f x %.2f m\n", cfg.grid_size_x, cfg.grid_size_y);
    printf("start(초기)  = (%.2f, %.2f)\n", cfg.start_x, cfg.start_y);
    printf("point A      = (%.2f, %.2f), %.1f deg  [트리거: 짐 얹히면 출발]\n",
           point_a.x, point_a.y, point_a.theta_deg);
    printf("point B      = (%.2f, %.2f), %.1f deg  [트리거: 짐 내려가면 출발]\n",
           point_b.x, point_b.y, point_b.theta_deg);
    if (save_map_path) printf("save_map     = %s\n", save_map_path);
    if (cfg.slot_parking)
        printf("parking      = 슬롯 모드 (대기점 y=%.2f -> 직진 진입, 후진 이탈)\n",
               cfg.slot_staging_y);
    /* BUGFIX (2026-08-08): --lidar-offset 은 --lidar-yaw 가 맞아야만 의미가 있음.
     * 평행이동을 "차체 정면 방향"으로 해야 하는데, 스캔 각도가 아직 차체 기준이
     * 아니면 엉뚱한 방향으로 6cm를 밀게 됨. 그러면 회전할 때마다 지렛대 오차가
     * 같이 돌아서 자세추정이 빙글빙글 돌다가 발산함(실기에서 확인).
     * 각도 보정 없이 오프셋만 켜는 건 거의 항상 잘못된 조합이므로 막아줌. */
    if (cfg.lidar_offset_forward_m != 0.0 && cfg.lidar_yaw_offset_deg == 0.0) {
        fprintf(stderr,
            "[경고] --lidar-offset 을 썼는데 --lidar-yaw 가 0입니다.\n"
            "       오프셋 평행이동은 스캔 각도가 '차체 정면 기준'일 때만 맞습니다.\n"
            "       라이다 0도가 차체 정면이 아니면 6cm를 엉뚱한 방향으로 밀게 되고,\n"
            "       회전할 때마다 자세추정이 돌다가 발산합니다.\n"
            "       --lidar-yaw 를 먼저 실측해서 넣거나, --lidar-offset 을 빼세요.\n");
    }
    printf("lidar_frame  = 좌우반전 %s, 장착각도 %.1f deg, 전방오프셋 %.3f m\n",
           cfg.lidar_mirror ? "적용" : "안함", cfg.lidar_yaw_offset_deg,
           cfg.lidar_offset_forward_m);
    if (trigger_cfg.type == TRIGGER_GPIO) {
        printf("trigger      = GPIO %s, 센서 %d개 OR 판정", trigger_cfg.gpio_chip,
               trigger_cfg.gpio_line_count);
        for (int k = 0; k < trigger_cfg.gpio_line_count; k++)
            printf("%s%d", k ? "," : " [line ", trigger_cfg.gpio_lines[k]);
        printf("]%s%s, 디바운스 %.0fms\n",
               trigger_cfg.gpio_active_low ? ", active-low" : "",
               trigger_cfg.gpio_pull_up ? ", pull-up" : "",
               trigger_cfg.gpio_debounce_sec * 1000.0);
        printf("               A지점=박스 올라오면 출발 / B지점=박스 내려가면 출발, %s\n",
               trigger_cfg.allow_enter_override
                 ? "Enter 수동 오버라이드 켜짐(디버그)" : "Enter 오버라이드 꺼짐");
    } else {
        printf("trigger      = 터미널 Enter (테스트용)\n");
    }
    if (cfg.slot_parking) {
        printf("slot_speed   = 진입/후진 %d, 밀착 %d (일반주행 %d), "
               "스텝 %.2fs 이동 / %.2fs 정지\n",
               cfg.slot_step_speed, cfg.slot_touch_speed, cfg.sg_step_speed,
               cfg.slot_move_sec, cfg.slot_settle_sec);
        printf("slot_retry   = 진입 정렬 허용오차 %.1fdeg, 실패 시 재시도 %d회\n",
               cfg.slot_align_tol_deg, cfg.slot_max_tries);
        printf("slot_lidar   = 슬롯 안 라이다 실측 재정박 %s, 착석 허용 각도 %.1fdeg\n",
               cfg.slot_lidar_fix ? "켜짐" : "꺼짐(--no-slot-lidar-fix)",
               cfg.slot_seat_yaw_deg);
        printf("slot_bias    = 재진입 시 걸린 벽 반대로 %.1fdeg, %.0fmm 들어갈 때까지 유지 "
               "(옆으로 약 %.0fmm 이동)\n",
               cfg.slot_escape_bias_deg, cfg.slot_bias_span_m * 1000.0,
               cfg.slot_bias_span_m * sin(cfg.slot_escape_bias_deg * 3.14159265358979 / 180.0)
                 * 1000.0);
        printf("slot_press   = %s\n",
               cfg.slot_press_to_wall
                 ? "안쪽 벽에 닿을 때까지 진입 후 밀착"
                 : "목표 y 에서 정지 (--no-slot-press)");
        printf("slot_seat    = 깊이오차 %.0fmm 이내 + 벽 접촉 + 자세 6deg 이내 를 모두 "
               "만족해야 주차 완료\n", cfg.slot_seat_depth_tol_m * 1000.0);
        printf("slot_approach= 대기점 상자 좌우 %.0fmm / 전후 %.0fmm 안에 들면 접근 종료 "
               "-> 슬롯 정렬로 인계\n",
               cfg.slot_approach_x_tol * 1000.0, cfg.slot_approach_y_tol * 1000.0);
        printf("body_size    = %.3f x %.3f m (제자리 회전 판정용)\n",
               cfg.body_length, cfg.body_width);
        printf("trigger      = 출발신호 후 %.1f초 대기\n", cfg.trigger_delay_sec);
        printf("slam_sigma   = 거친 %.0fmm / 정밀 %.0fmm\n",
               g_slam_sigma_coarse * 1000.0, g_slam_sigma_fine * 1000.0);
        printf("speeds       = 통로 %d / 도킹 %d / 회전·슬롯 %d (안쪽바퀴 하한 %d)\n",
               cfg.cruise_speed, cfg.dock_step_speed, cfg.sg_step_speed, cfg.min_arc_speed);
        printf("crab         = %s\n",
               cfg.allow_crab ? "허용 (--allow-crab)"
                              : "사용 안 함 - 앞/뒤로 물러나 여유를 만든 뒤 회전");
        printf("trim_tilt    = 좌우 보정 시 슬롯 축에서 %.0fdeg 기울임 "
               "(이동 %.2f배, 회전 왕복 %.0fdeg)\n",
               cfg.slot_trim_tilt_deg,
               1.0 / sin(cfg.slot_trim_tilt_deg * 3.14159265358979 / 180.0),
               2.0 * cfg.slot_trim_tilt_deg);
        printf("rot_window   = 회전 중 각도 +-%.0fdeg/%.1fdeg, 위치 +-%.0fmm/%.0fmm "
               "-> 한 스텝 최대 %.1fdeg\n",
               cfg.rot_win_deg, cfg.rot_step_deg, cfg.rot_win_m * 1000.0,
               cfg.rot_step_m * 1000.0,
               cfg.rot_win_deg * (cfg.rot_clamp_frac > 0.0 ? cfg.rot_clamp_frac
                                                           : (cfg.has_encoder ? 2.0 : 0.7)));
        printf("slot_fast    = y > %.3f 구간은 한 스텝 %.1f배로 밀기 "
               "(속도 %d, 최대 %.2fs, 깔때기 입구에서 자동 정지)\n",
               cfg.slot_fast_above_y > 0.0 ? cfg.slot_fast_above_y
                                           : (cfg.slot_staging_y - 0.22),
               cfg.slot_fast_gain,
               cfg.slot_open_speed > 0 ? cfg.slot_open_speed : cfg.sg_step_speed,
               cfg.slot_open_move_sec > 0.0 ? cfg.slot_open_move_sec
                                            : cfg.sg_move_sec_max);
        /* 기본값 1.0을 그대로 쓰던 예전 스크립트를 그대로 돌리는 경우를 위한 경고 */
        if (cfg.slot_align_tol_deg < 2.0) {
            fprintf(stderr,
                "[경고] --slot-align-tol 이 %.1fdeg 입니다. 이 하드웨어의 최소 회전\n"
                "       스텝이 약 3.5도라 그 안으로는 물리적으로 못 들어갑니다.\n"
                "       제자리 회전만 반복하다 시간을 다 쓰므로 2.5 이상을 권장합니다.\n",
                cfg.slot_align_tol_deg);
        }
    }
    printf("cruise       = %s, pure_pursuit=%s(lookahead %.2fm)\n",
           cfg.hybrid_cruise ? "하이브리드(직진 연속/회전만 정지)" : "전구간 stop-and-go",
           cfg.pure_pursuit ? "켬" : "끔", cfg.lookahead_m);
    printf("drive_mode   = %s\n", cfg.stop_and_go ? "stop-and-go (멈춤-관측-이동 반복)" : "연속주행");
    if (cfg.stop_and_go)
        printf("  stop-and-go: 이동속도=%d, 안정화=%.2fs, 이동시간=%.2f~%.2fs\n",
               cfg.sg_step_speed, cfg.sg_settle_sec, cfg.sg_move_sec_min, cfg.sg_move_sec_max);
    printf("start_theta  = %s\n", cfg.auto_localize_start ? "자동탐색(라이다 360도)" : "수동지정");
    if (cfg.has_encoder && (cfg.encoder_invert_left || cfg.encoder_invert_right))
        printf("encoder_sign = 좌 %s, 우 %s\n",
               cfg.encoder_invert_left ? "반전" : "정상",
               cfg.encoder_invert_right ? "반전" : "정상");
    if (cfg.has_encoder) {
        printf("ticks_per_rev= %d  (틀리면 오도메트리 전체가 무의미해짐 - "
               "./encoder_cal 로 실측 권장)\n", cfg.ticks_per_rev);
        if (cfg.ticks_per_rev == 20)
            printf("               ^ 기본값 그대로입니다. 감속기 달린 모터축 엔코더면 "
                   "보통 1000 이상입니다.\n");
    }
    printf("encoder      = %s%s\n", cfg.has_encoder ? "있음" : "없음 (--no-encoder)",
           cfg.has_encoder ? "" : " - 오도메트리 미사용, 라이다 스캔매칭만으로 위치추정");
    printf("============\n\n");

    printf("[main_shuttle] FPGA 포트 여는 중: %s @ %d baud\n", fpga_device, fpga_baud);
    int fpga_fd = serial_port_open(fpga_device, fpga_baud);
    if (fpga_fd < 0) {
        fprintf(stderr, "[main_shuttle] FPGA 포트 열기 실패, 종료\n");
        return 1;
    }

    printf("[main_shuttle] 라이다 포트 여는 중: %s @ %d baud\n", lidar_device, lidar_baud);
    int lidar_fd = serial_port_open(lidar_device, lidar_baud);
    if (lidar_fd < 0) {
        fprintf(stderr, "[main_shuttle] 라이다 포트 열기 실패, 종료\n");
        serial_port_close(fpga_fd);
        return 1;
    }

    int result = run_shuttle(fpga_fd, lidar_fd, &cfg, map_source, load_map_path, save_map_path,
                              point_a, point_b, &trigger_cfg);

    serial_port_close(fpga_fd);
    serial_port_close(lidar_fd);
    return result;
}
