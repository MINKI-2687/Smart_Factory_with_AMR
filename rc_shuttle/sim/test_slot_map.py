"""
test_slot_map.py
main_shuttle.c의 슬롯 주차 셔틀(A* 이동 + slot_drive 진입/후진)을 그대로 파이썬에서
재현한 시뮬레이션. 판단 로직은 전부 진짜 C 코드(libnav.so, libslam.so)를 그대로
호출해서 계산함 - 파이썬은 물리엔진(SimRobot, run_scenario.py)과 화면만 담당.

C 쪽과 다른 점(=단순화한 부분, 정직하게 밝힘):
  - 실제 fpga_serial.h의 "정지마찰 킥 + 펄스 구동"은 흉내내지 않음. 대신
    run_scenario.SimRobot의 모터 노이즈(±5%) + PWM 데드존 모델만 사용함.
    즉 이 시뮬레이션은 "위치추정 + 경로계획 + 슬롯진입 로직"이 맞는지 검증하는
    용도지, 실기의 정지마찰/펄싱 문제까지는 재현하지 않음.
  - slot_drive의 판정/조향 계수(K_LATERAL, K_HEADING, MAX_DIFF)는
    robot_runner.h의 값과 반드시 맞춰서 유지할 것 - 여기서 값을 바꾸면 실기와
    다른 걸 검증하게 됨.

사용법: python3 test_slot_map.py
  (먼저 python3 make_slot_map.py 로 set_map.txt를 만들어둘 것)
"""
import math
import random

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button

from slam_cwrap import CSlam
from nav_cwrap import CDynamicNavigator
from run_scenario import (SimRobot, LIDAR_NOISE_STD_M, LIDAR_ANGULAR_RESOLUTION_DEG,
                          _speed_to_pwm, _pwm_to_wheel_mps)

MAP_PATH = "set_map.txt"

# robot_runner.h의 slot_drive()와 반드시 동일하게 유지
K_LATERAL = 60.0
K_HEADING = 25.0
MAX_DIFF = 10.0
SG_STEP_SPEED = 34
SLOT_MAX_STEPS = 120
ALIGN_TOL_DEG = 1.0        # robot_runner.h의 cfg->slot_align_tol_deg와 동일하게 유지
USE_PURE_PURSUIT = True    # robot_runner.h의 cfg->pure_pursuit
LOOKAHEAD_M = 0.22         # robot_runner.h의 cfg->lookahead_m
HYBRID_CRUISE = True       # robot_runner.h의 cfg->hybrid_cruise

# --- 시간 모델 (robot_runner.h의 실제 대기시간과 맞춤) ---
# 지금까지 이 시뮬레이션은 모든 스텝을 균일한 50ms로 취급해서, "멈춰서 관측하는 시간"이
# 아예 없었음. 그래서 하이브리드 주행(직진은 안 멈춤)의 이득이 측정 자체가 불가능했음.
# 아래 값들로 시뮬레이션 시계를 따로 굴려서 실제 소요시간을 비교할 수 있게 함.
T_CONTROL = 0.05           # 제어주기 (연속 주행 시 한 스텝)
T_SETTLE = 0.45            # cfg->sg_settle_sec - 정지 후 흔들림 가라앉히기
T_SCAN_WAIT = 0.20         # 정지 상태의 깨끗한 스캔 대기(라이다 2회전)
T_MATCH = 0.088            # 스캔매칭 계산 (실측)
LIDAR_REV_SEC = 1.0 / 9.82 # 라이다 1회전 - 이동 중 스캔 왜곡 계산에 씀

# --- 지도에 없는 돌발 장애물 (x, y, 반지름) ---
# 물리 세계(레이캐스팅/충돌)에만 존재하고, SLAM/A*에 주는 GRID에는 없음.
# 실기에서 사람이 지나가거나 물건이 놓인 상황에 해당. dynnav가 라이다로 감지해서
# 스스로 재계획(replan)하는지 검증하는 용도.
EXTRA_OBSTACLES = []
SLOT_STOP_TOL = 0.015      # robot_runner.h의 SLOT_STOP_TOL_M
SLOT_STALL_EPS = 0.004     # robot_runner.h의 SLOT_STALL_EPS_M
SLOT_STALL_STEPS = 3       # robot_runner.h의 SLOT_STALL_STEPS
SLOT_SEATED_MARGIN = 0.05  # robot_runner.h의 SLOT_SEATED_MARGIN_M

ROBOT_RADIUS = 0.10          # A* 팽창 반경 (실기와 동일 - 대기점까지만 A* 사용)
ROBOT_LENGTH = 0.255         # 시각화용 (차체 사각형 그리기)
ROBOT_WIDTH = 0.151


def load_map(path):
    with open(path) as f:
        header = f.readline().split()
        rows, cols, res = int(header[0]), int(header[1]), float(header[2])
        grid = []
        for _ in range(rows):
            line = f.readline().strip()
            grid.append([1 if ch == '1' else 0 for ch in line])
    return grid, rows, cols, res


GRID, ROWS, COLS, RES = load_map(MAP_PATH)

# make_slot_map.py와 동일한 상수 (슬롯 중심/대기점/주차자세) - 지도를 다시 만들면
# 이 값들도 같이 바뀌어야 하므로, 가능하면 make_slot_map.py를 import해서 쓰는 게 더
# 안전하지만(값이 하나의 소스에서만 나옴), 그 스크립트가 실행 시 파일을 바로 써버려서
# 여기서는 상수만 다시 계산함. 값 정의가 바뀌면 두 파일 다 확인할 것.
WALL = 0.05
IN_X0, IN_X1 = WALL, COLS * RES - WALL
IN_Y0 = WALL
MOUTH_W = 0.35
# 지도에서 슬롯 실제 폭을 자동 측정 (make_slot_map.py에서 폭을 바꿔도 여기가 따라옴)
def _measure_slot_width():
    r = int((IN_Y0 + 0.02) / RES)          # 평행구간 높이의 한 줄
    c0 = int((IN_X0 + MOUTH_W / 2) / RES)  # 왼쪽 슬롯 중심
    left = c0
    while left > 0 and not GRID[r][left]:
        left -= 1
    right = c0
    while right < COLS - 1 and not GRID[r][right]:
        right += 1
    return (right - left - 1) * RES
SLOT_W = _measure_slot_width()
PARK_Y = IN_Y0 + ROBOT_LENGTH / 2.0

POINT_A = (IN_X0 + MOUTH_W / 2.0, PARK_Y, -90.0)
POINT_B = (IN_X1 - MOUTH_W / 2.0, PARK_Y, -90.0)
STAGING_Y = 0.65   # robot_runner.h의 cfg->slot_staging_y와 동일하게 유지
                   # (0.57에서는 제자리 회전 시 차체가 슬롯 입구 벽을 침범해 불가능)

# 로봇을 반대편(B) 대기점에서 시작시켜서, 첫 leg가 실제 셔틀 운행과 같은 '진짜
# 이동'이 되게 함(A 대기점에서 시작하면 목적지와 시작점이 같아져서 도킹 로직이
# "이미 도착했지만 각도만 안 맞음" 특이 케이스로 빠짐 - 실사용에서는 거의 안 생기는
# 상황이라 여기서 재현할 필요 없음).
ROBOT_START = (POINT_B[0], STAGING_Y, math.radians(-90.0))


def normalize_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def grid_raycast(rx, ry, angle, max_range=2.0):
    """PATCH: 1cm 해상도에서 res*0.5(=5mm) 간격으로 훑으면 파이썬 순수루프가 스텝당
    ~0.1초까지 느려져서(레이 501개 x 최대 400스텝) 헤드리스 배치 테스트가 사실상
    불가능해짐. res(=1cm) 간격으로 두 배 성기게 해도 위치추정 노이즈(수mm) 대비
    무시할 만한 오차라 정확도에 영향 없음. max_range도 방 대각선(~1.7m)보다 넉넉한
    2.0m로 줄여서 벽에 안 맞는 예외적인 광선의 낭비를 줄임."""
    step = RES
    dist = step
    dx, dy = math.cos(angle), math.sin(angle)
    hit = max_range
    while dist < max_range:
        px, py = rx + dx * dist, ry + dy * dist
        col, row = int(px / RES), int(py / RES)
        if row < 0 or row >= ROWS or col < 0 or col >= COLS:
            hit = dist
            break
        if GRID[row][col]:
            hit = dist
            break
        dist += step

    # 지도에 없는 원형 장애물도 광선을 막음 (광선-원 교점)
    for (cx, cy, r) in EXTRA_OBSTACLES:
        ox, oy = cx - rx, cy - ry
        proj = ox * dx + oy * dy
        if proj < 0:
            continue
        perp2 = (ox * ox + oy * oy) - proj * proj
        if perp2 > r * r:
            continue
        t = proj - math.sqrt(r * r - perp2)
        if 0 < t < hit:
            hit = t
    return hit


def lidar_scan(rx, ry, rtheta, rng, mirror=False, v=0.0, w=0.0):
    """라이다 한 바퀴(약 102ms) 도는 동안 로봇이 v[m/s], w[rad/s]로 움직이고 있으면
    각 광선이 서로 다른 자세에서 측정됨 - 이게 motion skew. 그런데 소프트웨어는 그
    스캔 전체를 '한 순간에 찍힌 것'으로 취급하므로 스캔이 찌그러져 보이고 위치추정이
    나빠짐. 연속 주행(하이브리드)의 대가가 바로 이것이므로, 이걸 모델에 넣지 않으면
    '연속 주행은 공짜로 빠르다'는 잘못된 결론이 나옴.

    mirror=True면 로봇 좌표계 각도가 뒤집힘 - 실기 --lidar-mirror 검증용."""
    scan = []
    a = -180.0
    n_rays = int(360.0 / LIDAR_ANGULAR_RESOLUTION_DEG)
    k = 0
    while a < 180.0:
        # 이 광선이 측정되는 시점(회전 시작 후 f 비율)에서의 로봇 자세
        f = (k / n_rays) * LIDAR_REV_SEC
        th_k = rtheta + w * f
        x_k = rx + v * math.cos(rtheta) * f
        y_k = ry + v * math.sin(rtheta) * f
        wa = th_k + math.radians(a)
        d = grid_raycast(x_k, y_k, wa)
        if d < 2.0 - 1e-6:
            noisy = max(0.01, d + rng.gauss(0, LIDAR_NOISE_STD_M))
            reported_a = -a if mirror else a
            scan.append((reported_a, noisy * 1000.0))
        a += LIDAR_ANGULAR_RESOLUTION_DEG
        k += 1
    return scan


class SlotShuttleSim:
    """robot_runner.h의 run_shuttle() + slot_drive()를 그대로 옮긴 상태기계."""

    PHASE_CRUISE = "cruise"     # A*로 대기점까지
    PHASE_ALIGN = "align"       # 진입 전 제자리 각도 정렬
    PHASE_ENTER = "enter"       # 슬롯 직진 진입
    PHASE_SEAT = "seat"         # 안쪽 벽에 밀착
    PHASE_WAIT = "wait"         # 트리거 대기 (여기선 자동 통과)
    PHASE_EXIT = "exit"         # 슬롯 후진 이탈

    def __init__(self, seed=0, auto_trigger=False):
        self.auto_trigger = auto_trigger   # True면 Enter 없이 자동 왕복(headless용)
        self.trigger_pending = False
        self.rng = random.Random(seed)
        x0, y0, th0 = ROBOT_START
        self.robot = SimRobot(x0, y0, th0, seed=seed)

        # 실기의 --map-source load와 동일하게, 알고 있는 격자로 바로 초기화.
        # (예전엔 빈 지도로 시작해 합성 스캔으로 채웠는데, 그 과정에서 초기 자세가
        #  12cm나 어긋나 경로 전체가 밀리는 문제가 있었음)
        self.slam = CSlam.from_grid(GRID, ROWS, COLS, RES, start_pose=(x0, y0, th0),
                                    search_window_m=0.15, search_window_theta_deg=12,
                                    search_step_m=0.03, search_step_theta_deg=3)
        self.slam.enable_likelihood_field()

        self.nav = CDynamicNavigator(GRID, ROWS, COLS, RES, ROBOT_RADIUS)
        # 실기(RobotConfig.pure_pursuit / lookahead_m)와 동일하게 설정
        self.nav.set_pure_pursuit(USE_PURE_PURSUIT, LOOKAHEAD_M)

        self.leg = 0             # 0: A로 감, 1: B로 감
        self.phase = self.PHASE_CRUISE
        self.seat_steps_left = 0
        self.wait_steps_left = 0
        self.slot_step = 0
        self.align_step = 0
        self.last_slot_y = 1e9
        self.stall_count = 0
        self.plan_ok = True
        self.done_all = False
        self.blocked = False   # 경로가 막혀 목적지 도달 실패
        self.collisions = 0      # 차체가 벽에 부딪혀 이동이 막힌 횟수
        self.sim_time = 0.0      # 시뮬레이션 경과시간(초) - 실제 소요시간 비교용
        self.cur_v = 0.0         # 현재 병진속도 (스캔 왜곡 계산용)
        self.cur_w = 0.0         # 현재 각속도
        self.prev_was_rotation = True   # 첫 스텝은 안전하게 정지 관측
        self.log = []            # 최근 로그 줄 (화면 표시용)
        self.park_errors = []    # [(leg_label, est_err_mm, true_err_mm, heading_err_deg), ...]

        self._start_cruise_leg()

    def _flood_fill_map(self):
        """로봇을 지도 여러 지점에 두고 합성(노이즈 없는) 스캔으로 update()를 반복해
        log_odds를 채움 - CSlam이 노출한 정식 API(update)만 사용. 실기의
        --map-source load(파일을 통째로 읽어 채움)와 최종 상태는 동등해야 하므로,
        점들을 충분히 넓게/여러 각도로 잡아 사각지대가 없게 함."""
        sample_pts = [(0.20, 0.20), (0.75, 0.20), (1.30, 0.20),
                      (0.20, 0.75), (0.75, 0.75), (1.30, 0.75),
                      (0.20, 0.50), (1.30, 0.50)]
        first = True
        for (sx, sy) in sample_pts:
            r, c = int(sy / RES), int(sx / RES)
            if not (0 <= r < ROWS and 0 <= c < COLS) or GRID[r][c]:
                continue
            scan = []
            a = -180.0
            while a < 180.0:
                d = grid_raycast(sx, sy, math.radians(a))
                if d < 2.0 - 1e-6:
                    scan.append((a, d * 1000.0))
                a += 0.72
            if first:
                self.slam.update(scan, 0, 0, 0)
                first = False
            else:
                px, py, pth = self.slam.get_pose()
                self.slam.update(scan, sx - px, sy - py, 0.0 - pth)
        # 지도 채우기용 이동은 진짜 로봇의 움직임이 아니므로, pose를 시작위치로 되돌림
        x0, y0, th0 = ROBOT_START
        dummy = []
        a = -180.0
        while a < 180.0:
            d = grid_raycast(x0, y0, th0 + math.radians(a))
            if d < 2.0 - 1e-6:
                dummy.append((a, d * 1000.0))
            a += 0.72
        px, py, pth = self.slam.get_pose()
        self.slam.update(dummy, x0 - px, y0 - py, th0 - pth)

    def _body_corners(self, cx, cy, th):
        """차체 사각형의 네 모서리 좌표. x,y는 두 바퀴 사이 중점이고, 여기서는 그것이
        차체의 기하학적 중심이라고 가정함 - 실제 RC카는 바퀴축이 차체 중앙에 없을 수
        있으므로, 실기 적용 전에 '바퀴축에서 앞범퍼까지'와 '뒷범퍼까지'를 실측해서
        아래 FRONT_LEN/REAR_LEN으로 나눠주는 게 정확함(지금은 절반씩으로 가정). """
        hw, hl = ROBOT_WIDTH / 2.0, ROBOT_LENGTH / 2.0
        ct, st = math.cos(th), math.sin(th)
        pts = []
        for dx, dy in ((hl, hw), (hl, -hw), (-hl, hw), (-hl, -hw)):
            pts.append((cx + dx * ct - dy * st, cy + dx * st + dy * ct))
        return pts

    def _body_collides(self, cx, cy, th):
        """차체 사각형이 벽(점유셀)과 겹치는지. 네 모서리 + 각 변을 잘게 나눈 점들을
        검사함. PATCH(2026-08-06): 이전에는 '중심점 하나'만 격자와 비교해서, 차체
        반폭(75.5mm)만큼 벽을 파고들어도 통과로 판정됐음 - 슬롯 허용오차가 ±9.5mm인데
        시뮬레이션은 82.6mm 오차를 '성공'으로 보고했음(실기라면 충돌). 반드시 차체
        외형으로 판정해야 이 테스트가 의미가 있음."""
        corners = self._body_corners(cx, cy, th)
        pts = list(corners)
        # 각 변을 5등분한 점도 같이 검사 (모서리만 보면 얇은 벽을 타고 넘어갈 수 있음)
        for i in range(4):
            for j in range(4):
                ax, ay = corners[i]
                bx, by = corners[(i + 1) % 4]
                t = (j + 1) / 5.0
                pts.append((ax + (bx - ax) * t, ay + (by - ay) * t))
        for (px, py) in pts:
            r, c = int(py / RES), int(px / RES)
            if not (0 <= r < ROWS and 0 <= c < COLS) or GRID[r][c]:
                return True
            for (ox, oy, orad) in EXTRA_OBSTACLES:
                if math.hypot(px - ox, py - oy) < orad:
                    return True
        return False

    def _robot_step(self, left, right):
        """SimRobot.step()을 감싸서 충돌을 처리함.

        PATCH (2026-08-06): 처음엔 '부딪히면 그 자리에 완전히 멈춤'으로 했는데, 그러면
        V자 입구의 핵심 이점 - 벽을 타고 미끄러지며 스스로 정렬되는 효과 - 가 통째로
        빠져서 실제보다 훨씬 비관적인 결과가 나옴(슬롯 진입이 무조건 실패). 실제 벽은
        수직항력만 주고 접선 방향으로는 미끄러지게 두므로, 여기서도 이동을 x성분과
        y성분으로 나눠서 '막히지 않는 쪽만' 살려주는 방식으로 근사함. 회전도 마찬가지로
        따로 판정해서, 위치가 막혀도 각도는 바뀔 수 있게 함(실제로 벽에 눌리면 차체가
        돌아가면서 정렬되는 것에 해당)."""
        # 이 명령이 만들어내는 실제 속도 - 스캔 왜곡 계산에 사용
        vl = _pwm_to_wheel_mps(_speed_to_pwm(left))
        vr = _pwm_to_wheel_mps(_speed_to_pwm(right))
        self.cur_v = (vl + vr) / 2.0
        self.cur_w = (vr - vl) / self.robot.wheel_sep

        px, py, pth = self.robot.x, self.robot.y, self.robot.theta
        ret = self.robot.step(left, right)
        nx, ny, nth = self.robot.x, self.robot.y, self.robot.theta

        if not self._body_collides(nx, ny, nth):
            return ret

        # 1) 벽을 타고 미끄러짐: x만 이동 / y만 이동 중 가능한 쪽 채택
        for cx, cy in ((nx, py), (px, ny)):
            if not self._body_collides(cx, cy, nth):
                self.robot.x, self.robot.y, self.robot.theta = cx, cy, nth
                self.collisions += 1
                return ret
        # 2) 위치는 막혔지만 회전만은 가능한 경우
        if not self._body_collides(px, py, nth):
            self.robot.x, self.robot.y, self.robot.theta = px, py, nth
            self.collisions += 1
            return ret
        # 3) 완전히 끼임
        self.robot.x, self.robot.y, self.robot.theta = px, py, pth
        self.collisions += 1
        return ret

    def move_sec_for(self, left, right):
        """robot_runner.h의 이동시간 계산과 동일 - 원래 명령 크기에 비례하되 하한/상한."""
        m = max(abs(left), abs(right))
        if m <= 0:
            return T_CONTROL
        sec = m * T_CONTROL / SG_STEP_SPEED
        return max(0.07, min(0.45, sec))

    def _current_target(self):
        return POINT_A if self.leg == 0 else POINT_B

    def _other_point(self):
        return POINT_B if self.leg == 0 else POINT_A

    def _start_cruise_leg(self):
        target = self._current_target()
        x, y, th = self.slam.get_pose()
        ok = self.nav.set_goal((x, y), (target[0], STAGING_Y), math.radians(target[2]))
        self.plan_ok = ok
        self._log(f"[shuttle] leg={self.leg} A* to staging: {'OK' if ok else 'FAILED'}")
        self.phase = self.PHASE_CRUISE

    def _log(self, s):
        self.log.append(s)
        if len(self.log) > 6:
            self.log.pop(0)

    def _get_localized_scan(self, moving=False):
        """moving=True면 현재 속도로 이동 중인 상태의 스캔(왜곡 포함)을 만듦.
        멈춰서 찍는 경우(stop-and-go)는 왜곡이 없으므로 v=w=0."""
        v = self.cur_v if moving else 0.0
        w = self.cur_w if moving else 0.0
        scan = lidar_scan(self.robot.x, self.robot.y, self.robot.theta, self.rng, v=v, w=w)
        x, y, th = self.slam.localize_only(scan, 0.0, 0.0, 0.0)
        return scan, x, y, th

    def step(self):
        """main_shuttle.c 제어루프 한 스텝에 해당. True를 반환하면 계속, False면 끝."""
        if self.done_all or not self.plan_ok:
            return False
        if self.phase == self.PHASE_CRUISE:
            return self._step_cruise()
        elif self.phase == self.PHASE_ALIGN:
            return self._step_align()
        elif self.phase == self.PHASE_ENTER:
            return self._step_slot(forward=True)
        elif self.phase == self.PHASE_SEAT:
            return self._step_seat()
        elif self.phase == self.PHASE_WAIT:
            return self._step_wait()
        elif self.phase == self.PHASE_EXIT:
            return self._step_slot(forward=False)
        return False

    def _step_cruise(self):
        """robot_runner.h의 하이브리드 주행과 동일:
        직전 명령이 제자리 회전(좌우 부호 반대)이었으면 멈춰서 깨끗한 스캔을 찍고,
        전진/완만한 곡선이었으면 안 멈추고 계속 굴러가며 최신 스캔을 그대로 씀.
        멈추는 경우에만 안정화+스캔대기 시간이 들고, 대신 스캔 왜곡이 없음."""
        need_stop = self.prev_was_rotation or not HYBRID_CRUISE
        if need_stop:
            self.cur_v = self.cur_w = 0.0            # 멈춘 상태 = 왜곡 없음
            self.sim_time += T_SETTLE + T_SCAN_WAIT
            scan, x, y, th = self._get_localized_scan(moving=False)
        else:
            scan, x, y, th = self._get_localized_scan(moving=True)
        self.sim_time += T_MATCH

        left, right, done = self.nav.update(x, y, th, scan)
        rotation_dominant = (left * right) < 0
        self.sim_time += (self.move_sec_for(left, right)
                          if (rotation_dominant or not HYBRID_CRUISE) else T_CONTROL)
        self.prev_was_rotation = rotation_dominant
        self._robot_step(left, right)
        self._log(f"[cruise] pose=({x:.3f},{y:.3f},{math.degrees(th):.0f}deg) "
                  f"wp={self.nav.get_current_wp_idx()}/{len(self.nav.get_waypoints())} cmd=({left},{right})")
        if done and self.nav.goal_unreachable():
            # 실기 navigate_one_leg와 동일 - 도달 못 했으면 슬롯 진입으로 넘어가지 않음
            self._log("[shuttle] *** BLOCKED: could not reach staging. Stopping. ***")
            self.blocked = True
            self.plan_ok = False
            return False
        if done:
            self._log("[shuttle] At staging -> aligning heading")
            self.slot_step = 0
            self.align_step = 0
            self.last_slot_y = 1e9
            self.stall_count = 0
            self.phase = self.PHASE_ALIGN
        return True

    def _step_align(self):
        """robot_runner.h의 slot_align()과 동일 - 제자리 회전으로 각도만 다듬음."""
        target = self._current_target()
        heading_rad = math.radians(target[2])
        self.sim_time += T_SETTLE + T_SCAN_WAIT + T_MATCH
        _, x, y, th = self._get_localized_scan(moving=False)
        err = normalize_angle(th - heading_rad)
        err_deg = math.degrees(err)
        if abs(err_deg) <= ALIGN_TOL_DEG or self.align_step >= 40:
            self._robot_step(0, 0)
            self._log(f"[slot] Aligned: heading err={err_deg:+.2f}deg (tol +-{ALIGN_TOL_DEG})")
            self.slot_step = 0
            self.phase = self.PHASE_ENTER
            return True
        w = SG_STEP_SPEED
        l = w if err > 0 else -w
        r = -l
        self._robot_step(l, r)
        self.align_step += 1
        return True

    def _step_slot(self, forward):
        label = "Slot ENTER" if forward else "Slot EXIT"
        # 진입이든 후진이든 '지금 있는 슬롯'이 기준선임. (C쪽 run_shuttle은 후진을
        # 다음 leg 시작 시점에 수행해서 from=반대편이 맞지만, 여기서는 주차 직후
        # 곧바로 후진하므로 현재 목표 슬롯이 기준. 이 부분이 반대편(1.275)을 향해
        # 후진하려다 실패해서, A 주차 후 왕복이 멈추는 원인이었음.)
        target = self._current_target()
        lane_x = target[0]
        heading_rad = math.radians(target[2])
        stop_y = target[1] if forward else STAGING_Y

        self.sim_time += T_SETTLE + T_SCAN_WAIT + T_MATCH + T_CONTROL
        scan, x, y, th = self._get_localized_scan(moving=False)

        # robot_runner.h와 동일: 목표 근접 OR 더 이상 안 들어감(벽 접촉)
        reached = (y <= stop_y + SLOT_STOP_TOL) if forward else (y >= stop_y - SLOT_STOP_TOL)
        if abs(y - self.last_slot_y) < SLOT_STALL_EPS:
            self.stall_count += 1
        else:
            self.stall_count = 0
        self.last_slot_y = y
        # 입구에서 끼인 것을 '주차 완료'로 오판하지 않도록, 목표 깊이 근처에서
        # 멈춘 경우에만 착석으로 인정 (robot_runner.h의 SLOT_SEATED_MARGIN_M과 동일)
        near_depth = (y <= stop_y + SLOT_SEATED_MARGIN) if forward else True
        stalled = self.stall_count >= SLOT_STALL_STEPS and self.slot_step > 3 and near_depth
        done = reached or (forward and stalled)
        if done or self.slot_step >= SLOT_MAX_STEPS:
            self._robot_step(0, 0)
            if not done:
                self._log(f"[slot] {label} step limit exceeded - aborted")
                self.done_all = True
                self.plan_ok = False
                return False
            self._log(f"[slot] {label} done: pose=({x:.3f},{y:.3f},{math.degrees(th):.1f}deg)")
            if forward:
                self.phase = self.PHASE_SEAT
                self.seat_steps_left = 9
            else:
                self.leg = 1 - self.leg
                self._start_cruise_leg()
            return True

        lateral_err = x - lane_x
        heading_err = normalize_angle(th - heading_rad)
        # PATCH: robot_runner.h와 동일한 부호오류를 찾아 여기도 같이 고침 - 아래 diff에
        # 마이너스가 붙어야 함(검산은 robot_runner.h의 slot_drive 주석 참고).
        diff = -(K_LATERAL * lateral_err + K_HEADING * heading_err)
        if not forward:
            diff = -diff
        diff = max(-MAX_DIFF, min(MAX_DIFF, diff))

        base = SG_STEP_SPEED if forward else -SG_STEP_SPEED
        left = int(round(base - diff))
        right = int(round(base + diff))
        self._robot_step(left, right)

        self._log(f"[slot] {label} step={self.slot_step} pose=({x:.3f},{y:.3f},{math.degrees(th):.1f}deg) "
                  f"lat={lateral_err*1000:+.1f}mm hdg={math.degrees(heading_err):+.1f}deg cmd=({left},{right})")
        self.slot_step += 1
        return True

    def _step_seat(self):
        self._robot_step(SG_STEP_SPEED, SG_STEP_SPEED)
        self.seat_steps_left -= 1
        if self.seat_steps_left <= 0:
            self._robot_step(0, 0)
            _, x, y, th = self._get_localized_scan()
            target = self._current_target()
            lat_mm = (self.robot.x - target[0]) * 1000.0        # 좌우(슬롯 폭 방향) - 충돌을 결정하는 값
            depth_mm = (self.robot.y - target[1]) * 1000.0       # 깊이(진입 방향)
            true_err_mm = math.hypot(self.robot.x - target[0], self.robot.y - target[1]) * 1000.0
            heading_err_deg = abs(math.degrees(normalize_angle(th - math.radians(target[2]))))
            label = "A" if self.leg == 0 else "B"
            margin_mm = (SLOT_W / 2.0 - ROBOT_WIDTH / 2.0) * 1000.0
            verdict = "OK" if abs(lat_mm) <= margin_mm else "*** WALL HIT ***"
            self._log(f"[shuttle] Parked {label}: lat={lat_mm:+.1f}mm (tol +-{margin_mm:.1f}) "
                      f"depth={depth_mm:+.1f}mm hdg={heading_err_deg:.1f}deg {verdict}")
            self.park_errors.append((label, lat_mm, depth_mm, heading_err_deg, true_err_mm, verdict))
            self.trigger_pending = True
            self.phase = self.PHASE_WAIT
        return True

    def _step_wait(self):
        """실기의 trigger_wait()에 해당 - 짐을 싣고 내리는 동안 대기.
        GUI에서는 Enter 키를 눌러야 다음 구간으로 출발함(실기의 Enter 트리거와 동일).
        headless 모드에서는 사람이 없으므로 자동 통과."""
        if self.auto_trigger:
            self.phase = self.PHASE_EXIT
            self.slot_step = 0
            self.last_slot_y = 1e9
            self.stall_count = 0
            return True
        if self.trigger_pending:
            return True          # Enter를 누를 때까지 여기서 대기
        self._log("[trigger] Received -> reversing out")
        self.phase = self.PHASE_EXIT
        self.slot_step = 0
        self.last_slot_y = 1e9
        self.stall_count = 0
        return True

    def fire_trigger(self):
        """GUI에서 Enter를 눌렀을 때 호출됨."""
        if self.phase == self.PHASE_WAIT:
            self.trigger_pending = False
            return True
        return False


def run_headless(n_cycles=6, seed=1, verbose=True):
    """GUI 없이 n_cycles(=A->B->A->... 왕복 횟수)만큼 돌려서 주차오차 통계만 뽑음.
    화면 없이 빠르게 여러 시드로 검증하고 싶을 때 사용."""
    sim = SlotShuttleSim(seed=seed, auto_trigger=True)
    max_steps = 20000
    steps = 0
    while len(sim.park_errors) < n_cycles and steps < max_steps:
        if not sim.step():
            if verbose:
                print(f"  [seed={seed}] aborted after {len(sim.park_errors)} cycle(s): {sim.log[-1] if sim.log else ''}")
            break
        steps += 1
    if verbose:
        for label, lat, depth, hdg, tot, verdict in sim.park_errors:
            print(f"  [seed={seed}] {label}: lat={lat:+6.1f}mm depth={depth:+6.1f}mm hdg={hdg:4.1f}deg  {verdict}")
        print(f"  [seed={seed}] blocked-by-wall count: {sim.collisions}")
    return sim.park_errors


# ------------------------------------------------------------
# matplotlib GUI
# ------------------------------------------------------------
def main():
    sim = SlotShuttleSim(seed=1, auto_trigger=False)

    fig, ax = plt.subplots(figsize=(11, 7))
    ax.set_aspect('equal')
    extent = [0, COLS * RES, 0, ROWS * RES]
    ax.imshow(GRID, origin='lower', extent=extent, cmap='Greys', alpha=0.85, zorder=1)
    ax.set_xlim(-0.05, COLS * RES + 0.05)
    ax.set_ylim(-0.05, ROWS * RES + 0.05)

    for (px, py, pdeg) in (POINT_A, POINT_B):
        ax.plot([px], [py], '*', color='gold', ms=16, zorder=5)

    true_dot, = ax.plot([], [], 'o', color='orange', ms=8, label='True (physics) pose', zorder=6)
    est_dot, = ax.plot([], [], 'o', color='lime', ms=6, label='Estimated pose', zorder=7)
    heading_ln, = ax.plot([], [], '-', color='lime', lw=2, zorder=7)
    # 차체 외형(폭 15.1 x 길이 25.5cm)을 실제로 그려서 슬롯에 들어가는지 눈으로 확인.
    # 점만 그리면 여유가 ±9.5mm뿐이라는 게 전혀 안 보임.
    body_ln, = ax.plot([], [], '-', color='orange', lw=1.8, zorder=6, label='Robot body')
    scan_sc = ax.scatter([], [], s=3, c='red', alpha=0.5, zorder=4, label='Lidar scan')
    ax.legend(loc='upper right', fontsize=8)
    title = ax.set_title("")
    info = ax.text(0.01, 0.99, "", transform=ax.transAxes, va='top', ha='left',
                   fontsize=8, family='monospace',
                   bbox=dict(boxstyle='round', fc='white', alpha=0.85), zorder=10)

    STEPS_PER_FRAME = 2
    state = {"running": True}

    def toggle(_event):
        state["running"] = not state["running"]
        btn.label.set_text("Pause" if state["running"] else "Resume")

    btn_ax = fig.add_axes((0.4, 0.01, 0.2, 0.05))
    btn = Button(btn_ax, "Pause")
    btn.on_clicked(toggle)

    def on_key(event):
        # 실기의 trigger_wait()와 동일하게, 슬롯에 주차한 상태에서 Enter를 누르면
        # 짐을 싣고/내린 것으로 보고 다음 지점으로 출발함
        if event.key == "enter":
            if sim.fire_trigger():
                state["running"] = True
                btn.label.set_text("Pause")

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(_frame):
        if state["running"]:
            for _ in range(STEPS_PER_FRAME):
                if not sim.step():
                    state["running"] = False
                    btn.label.set_text("Finished")
                    break

        x, y, th = sim.slam.get_pose()
        est_dot.set_data([x], [y])
        heading_ln.set_data([x, x + 0.12 * math.cos(th)], [y, y + 0.12 * math.sin(th)])
        true_dot.set_data([sim.robot.x], [sim.robot.y])
        corners = sim._body_corners(sim.robot.x, sim.robot.y, sim.robot.theta)
        order = [corners[0], corners[1], corners[3], corners[2], corners[0]]
        body_ln.set_data([p[0] for p in order], [p[1] for p in order])

        latest = lidar_scan(sim.robot.x, sim.robot.y, sim.robot.theta, sim.rng)
        pts = []
        for a, d_mm in latest:
            wa = th + math.radians(a)
            d = d_mm / 1000.0
            pts.append((x + d * math.cos(wa), y + d * math.sin(wa)))
        if pts:
            import numpy as np
            scan_sc.set_offsets(np.array(pts))

        lines = [f"phase={sim.phase}   leg={sim.leg}"]
        if sim.phase == sim.PHASE_WAIT and sim.trigger_pending:
            lines.append(">>> PARKED. Press ENTER to depart for the other slot <<<")
        lines += sim.log[-5:]
        if sim.park_errors:
            lines.append("--- Parking results ---")
            for label, lat, depth, hdg, tot, verdict in sim.park_errors[-4:]:
                lines.append(f"{label}: lat {lat:+6.1f}mm  depth {depth:+6.1f}mm  hdg {hdg:4.1f}deg  {verdict}")
        info.set_text("\n".join(lines))
        title.set_text(f"Slot Parking Shuttle Sim - phase={sim.phase}")
        return est_dot, heading_ln, true_dot, body_ln, scan_sc, title, info

    anim = animation.FuncAnimation(fig, update, interval=60, blit=False, cache_frame_data=False)
    plt.show()
    return anim


if __name__ == "__main__":
    import sys
    if "--headless" in sys.argv:
        n = 6
        for i, a in enumerate(sys.argv):
            if a == "--cycles" and i + 1 < len(sys.argv):
                n = int(sys.argv[i + 1])
        run_headless(n_cycles=n)
    else:
        main()
