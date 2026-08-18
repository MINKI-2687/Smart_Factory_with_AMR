#include "robot_scan.h"
#include "robot_runner.h"




/* ------------------------------------------------------------
 * 라이다 좌표계 보정 (거울상 + 장착 각도).
 *
 * PATCH (2026-08-05 v7): RPLIDAR는 위에서 봤을 때 각도가 *시계방향*으로 증가하는데,
 * 이 코드는 그 각도를 x+d*cos(theta+a), y+d*sin(theta+a) 라는 *반시계* 수학 규약에
 * 그대로 넣고 있었음. 즉 스캔 전체가 좌우로 뒤집힌 채로 처리되고 있었음.
 *
 * 이걸 점수로는 찾을 수 없었던 이유: 우리 세트장 지도가 좌우 완전 대칭이라(가운데
 * 장애물이 방 정중앙 x=0.75에 있음), 뒤집힌 스캔이 "뒤집힌 위치"에 똑같이 97%로
 * 들어맞음. 원본이든 거울상이든 점수가 같아서 알고리즘은 구분 불가. 실기에서 로봇이
 * 왼쪽 구석에 있는데 GUI가 오른쪽 구석으로 표시하는 걸 사람이 보고서야 판명됨.
 *
 * mirror=true면 각도 부호를 뒤집어 시계->반시계로 바꾸고, 그 다음 장착 각도를 더함
 * (차체 기준으로 돌려놓기). 순서 중요: 거울상 보정이 먼저.
 * ------------------------------------------------------------ */
/* PATCH (2026-08-07): 라이다 장착 위치 보정 추가.
 *
 * 라이다가 차체 회전중심이 아니라 앞쪽으로 d 만큼 나가 있으면, 스캔매칭이 추정하는
 * 자세는 "차체 중심"이 아니라 "라이다 위치"임. 그런데 주차 목표좌표(park_y =
 * 벽 + 차체길이/2), robot_radius, 슬롯 중심선은 전부 차체 중심 기준이라 좌표계가
 * d 만큼 어긋남. 특히 제자리 회전을 하면 라이다가 반지름 d 짜리 원을 그리므로
 * 추정 위치가 회전할 때마다 최대 2d 만큼 출렁임 - 17cm 슬롯(좌우 여유 ±9.5mm)
 * 에서는 치명적임.
 *
 * 고치는 방법: 스캔 자체를 차체중심 좌표계로 옮겨버림(각 점에 +d 평행이동).
 * 이러면 SLAM이 내놓는 자세가 곧 차체중심 자세가 되어, 바깥 코드는 한 줄도
 * 안 고쳐도 됨. 극좌표라 각도/거리를 다시 계산해야 함.
 * *** 전제조건 (2026-08-08 실기에서 물림) ***
 * 이 평행이동은 각도가 이미 "차체 정면 기준"으로 보정된 뒤에만 맞음. 즉
 * offset_forward_m 을 쓰려면 offset_deg(--lidar-yaw)가 먼저 맞아야 함.
 * 라이다 0도가 차체 정면이 아닌 상태에서 offset만 켜면 6cm를 엉뚱한 방향으로
 * 밀게 되고, 그 오차가 로봇이 회전할 때마다 같이 회전해서 자세추정이 빙글빙글
 * 돌다가 지도 밖으로 발산함(실기: pose y가 0.63 -> 4.57로 튐).
 * --lidar-yaw 를 모르면 --lidar-offset 은 0으로 두는 게 안전함. */
void apply_lidar_frame_fix(LidarScan *scan, bool mirror, double offset_deg,
                                          double offset_forward_m) {
    if (!mirror && offset_deg == 0.0 && offset_forward_m == 0.0) return;
    double d_mm = offset_forward_m * 1000.0;
    for (int i = 0; i < scan->count; i++) {
        double a = scan->readings[i].angle_deg;
        if (mirror) a = -a;
        a += offset_deg;
        while (a >= 180.0) a -= 360.0;
        while (a < -180.0) a += 360.0;

        if (d_mm != 0.0) {
            double rad = a * ANGLE_PI / 180.0;
            double px = scan->readings[i].dist_mm * cos(rad) + d_mm;
            double py = scan->readings[i].dist_mm * sin(rad);
            scan->readings[i].dist_mm = sqrt(px * px + py * py);
            a = atan2(py, px) * 180.0 / ANGLE_PI;
        }
        scan->readings[i].angle_deg = a;
    }
}



/* ============================================================
 * 라이다가 실제로 유효한 스캔을 내놓을 때까지 대기.
 * PATCH (2026-08-05): 실기 로그에서 처음 21스텝(약 1초) 동안 scan.count==0
 * (lidar_scans=0)인 채로 제어루프가 이미 돌고 있는 것이 확인됨 - 라이다 모터가
 * 회전수를 올리고 첫 바퀴를 다 돌기 전까지는 당연히 스캔이 비어있음. 그 동안
 * 엔코더도 없으니(--no-encoder) 위치추정 근거가 아예 0인 상태로 바퀴명령이
 * 나가고 있었음. 스캔이 들어오기 시작할 때쯤엔 로봇이 이미 시작위치에서
 * 벗어나 있어서, "start 좌표에 서 있다"는 전제 자체가 깨진 채로 스캔매칭이
 * 시작됨 -> 초기부터 pose가 크게 틀어짐. 그래서 움직이기 전에 반드시 기다림.
 * 반환: 유효 스캔을 얻으면 true, 시간초과/중단이면 false. */
bool wait_for_lidar_ready(LidarThread *lidar, int min_points,
                                          double timeout_sec, LidarScan *out_scan) {
    const double poll_sec = 0.05;
    int max_polls = (int)(timeout_sec / poll_sec);
    for (int i = 0; i < max_polls; i++) {
        if (g_stop_requested) return false;
        lidar_thread_get_latest(lidar, out_scan);
        if (out_scan->count >= min_points) {
            printf("[lidar] 준비 완료: %d개 점 (대기 %.2f초)\n", out_scan->count, i * poll_sec);
            fflush(stdout);
            return true;
        }
        struct timespec ts = {0, (long)(poll_sec * 1e9)};
        nanosleep(&ts, NULL);
    }
    fprintf(stderr, "[lidar] 경고: %.1f초 안에 유효한 스캔(%d점 이상)을 못 받음\n",
            timeout_sec, min_points);
    return false;
}



/* 지금 이후에 "새로 시작해서 완성된" 스캔을 기다림. 로봇이 멈춘 직후에 호출하면
 * 정지 상태에서만 수집된(=찌그러지지 않은) 스캔을 얻을 수 있음.
 * lidar_thread_get_latest()가 주는 최신 스캔은 이동 중에 수집됐을 수 있어서, 정지
 * 직후에 그냥 가져다 쓰면 motion skew가 그대로 남음. scan_count가 2 늘어날 때까지
 * 기다리는 이유: 1 늘어난 시점의 스캔은 "정지 전에 시작해서 정지 후에 끝난" 걸칠 수
 * 있으므로, 확실히 정지 후에 시작된 스캔을 받으려면 하나 더 기다려야 함. */
bool wait_for_fresh_scan(LidarThread *lidar, LidarScan *out, double timeout_sec) {
    pthread_mutex_lock(&lidar->lock);
    long start_count = lidar->scan_count;
    pthread_mutex_unlock(&lidar->lock);

    const double poll_sec = 0.005;
    int max_polls = (int)(timeout_sec / poll_sec);
    for (int i = 0; i < max_polls; i++) {
        if (g_stop_requested) return false;
        pthread_mutex_lock(&lidar->lock);
        long now = lidar->scan_count;
        if (now >= start_count + 2) {
            *out = lidar->latest_scan;
            pthread_mutex_unlock(&lidar->lock);
            return out->count > 0;
        }
        pthread_mutex_unlock(&lidar->lock);
        struct timespec ts = {0, (long)(poll_sec * 1e9)};
        nanosleep(&ts, NULL);
    }
    lidar_thread_get_latest(lidar, out);   /* 시간초과 - 있는 거라도 씀 */
    return out->count > 0;
}



void robot_runner__sleep_sec(double sec) {
    if (sec <= 0) return;
    struct timespec ts;
    ts.tv_sec = (time_t)sec;
    ts.tv_nsec = (long)((sec - ts.tv_sec) * 1e9);
    nanosleep(&ts, NULL);
}

/* 스캔 한 장을 '바로 쓸 수 있는 상태'로 받아온다 (2026-08-11 리팩토링에서 신설).
 *
 * 원래는 아래 네 줄이 여러 함수에 그대로 복사돼 있었다.
 *     LidarScan scan;
 *     if (!wait_for_fresh_scan(lidar, &scan, 2.0)) ...;
 *     apply_lidar_frame_fix(&scan, cfg->lidar_mirror, cfg->lidar_yaw_offset_deg,
 *                           cfg->lidar_offset_forward_m);
 *
 * 문제는 '획득'과 '프레임 보정'이 분리돼 있다는 점이다. 새 호출부에서 보정을
 * 한 번만 빠뜨려도 그 지점만 좌우반전/6cm 오프셋이 안 먹은 스캔으로 판단하게 되는데,
 * 이런 버그는 로그에 안 찍히고 실기에서 '가끔 이상하다'로만 나타난다.
 * 둘을 한 함수로 묶어 그 가능성 자체를 없앤다. 실패 시 동작(continue/return)은
 * 호출부마다 다르므로 bool 만 돌려주고 분기는 호출부에 남긴다. */
bool robot_runner__take_scan(LidarThread *lidar, const RobotConfig *cfg,
                             LidarScan *out, double timeout_sec) {
    if (!wait_for_fresh_scan(lidar, out, timeout_sec)) return false;
    apply_lidar_frame_fix(out, cfg->lidar_mirror, cfg->lidar_yaw_offset_deg,
                          cfg->lidar_offset_forward_m);
    return true;
}
