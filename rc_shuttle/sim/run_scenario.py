"""
run_scenario.py
시나리오 하나를 실제로 실행하는 공용 로직. headless 테스트와 GUI 둘 다 이 함수를 그대로 씀.

판단 로직(매핑/프론티어탐색/A*계획/회피/도킹)은 100% C(session_cwrap.CMappingSession,
즉 mapping_session.h)가 함 - 여기엔 판단 로직이 전혀 없습니다. SLAM 시나리오든 고정맵
시나리오든 이제 CMappingSession 하나로 통일됩니다 (live_scenario_gui.py, robot_runner.h와
동일한 결정로직을 공유).

노이즈 모델(실제 부품 스펙 기준):
  - 라이다(SLAMTEC RPLidar C1): 거리오차 스펙 ±30mm -> 가우시안 표준편차 10mm로 반영
    (±30mm를 대략 3시그마로 보고 역산). 각도해상도는 실제 스펙인 0.72도로 반영
    (기존엔 5도 간격이었음 - 실제보다 훨씬 성긴 스캔으로 테스트되고 있었음).
  - 엔코더(HC-020K, 20틱/회전): 정규분포 노이즈가 아니라, 실제 하드웨어처럼 정수 틱 단위로
    "양자화"해서 시뮬레이션함 - 즉 오도메트리가 연속값이 아니라 20틱/회전 해상도로만 측정됨
  - 모터(TT모터) + PWM 데드존: 명령값(left/right)이 fpga_link.h의 speed_to_pwm과 동일한
    공식으로 PWM으로 변환된 뒤, PWM_DEADZONE(실측 100) 미만이면 물리적으로 0 rad/s(정지마찰
    못 이김 - 실측 확인된 사실), 그 이상이면 PWM에 비례한 속도로 움직임. 그 위에 좌우
    독립적으로 ±5% 가우시안 배율 노이즈(모터 개체차/배터리전압 편차 등)를 얹음.
    (기존엔 scale=0.001 선형배율만 있었고 데드존이 전혀 없어서, 도킹 최종오차가 실기보다
    낙관적으로 나오는 문제가 있었음 - fpga_link.h 패치와 반드시 값을 맞춰야 함)
"""
import math
import random

from nav_cwrap import make_grid_from_circles
from session_cwrap import CMappingSession, SessionPhase

LIDAR_NOISE_STD_M = 0.010          # RPLidar C1: ±30mm 스펙 -> 표준편차 10mm로 역산
LIDAR_ANGULAR_RESOLUTION_DEG = 0.72  # RPLidar C1 실제 스펙
MOTOR_SPEED_NOISE_STD = 0.05       # TT모터: 좌우 각각 독립적으로 ±5% 속도편차
ENCODER_TICKS_PER_REV = 20         # HC-020K
WHEEL_DIAMETER_M = 0.066
WHEEL_RADIUS_M = WHEEL_DIAMETER_M / 2.0

# ---- fpga_link.h의 (데드존 보정된) speed_to_pwm과 반드시 동일해야 하는 상수들 ----
CONTROLLER_MAX_SPEED = 80
PWM_SAFETY_CAP = 207
PWM_DEADZONE = 100                 # 실측: 이 미만이면 정지마찰 못 이겨서 안 돎
MAX_WHEEL_RAD_S = 5.0              # PWM_SAFETY_CAP에서의 대략적 최대 바퀴각속도 가정
MAX_WHEEL_MPS = MAX_WHEEL_RAD_S * WHEEL_RADIUS_M


def lidar_scan(rx, ry, rtheta, obstacles, max_range=4.0, rng=None):
    rng = rng or random
    scan = []
    angle_deg = 0.0
    while angle_deg < 360.0:
        world_angle = rtheta + math.radians(angle_deg)
        dx, dy = math.cos(world_angle), math.sin(world_angle)
        min_dist = max_range
        for ox, oy, r in obstacles:
            fx, fy = rx - ox, ry - oy
            b = 2 * (fx * dx + fy * dy)
            c = fx * fx + fy * fy - r * r
            disc = b * b - 4 * c
            if disc < 0:
                continue
            sd = math.sqrt(disc)
            t1, t2 = (-b - sd) / 2, (-b + sd) / 2
            if 0 < t1 < min_dist:
                min_dist = t1
            if 0 < t2 < min_dist:
                min_dist = t2
        if min_dist < max_range:
            noisy_dist = min_dist + rng.gauss(0, LIDAR_NOISE_STD_M)
            noisy_dist = max(0.01, noisy_dist)  # 음수 방지
            scan.append((angle_deg, noisy_dist * 1000.0))
        angle_deg += LIDAR_ANGULAR_RESOLUTION_DEG
    return scan


def _speed_to_pwm(speed):
    """fpga_link.h의 (데드존 보정된) speed_to_pwm과 동일 로직. 0이 아닌 명령이면
    항상 PWM_DEADZONE 이상을 보장 - 로직 자체는 여기서 이미 데드존을 넘도록 보정해줌."""
    abs_speed = abs(speed)
    if abs_speed == 0:
        return 0
    mag = PWM_DEADZONE + round(abs_speed / CONTROLLER_MAX_SPEED * (PWM_SAFETY_CAP - PWM_DEADZONE))
    mag = max(PWM_DEADZONE, min(PWM_SAFETY_CAP, mag))
    return mag if speed >= 0 else -mag


def _pwm_to_wheel_mps(pwm):
    """하드웨어의 물리적 사실: PWM_DEADZONE 미만이면 정지마찰을 못 이겨서 실제로는
    0 rad/s (실측 확인됨). _speed_to_pwm이 이미 데드존 이상을 보장해주므로, 이 함수가
    직접 호출될 땐 사실상 항상 그 이상이지만, 하드웨어 사실과 소프트웨어 매핑을
    개념적으로 분리해두기 위해 별도 함수로 둠 (나중에 실측 데드존 값이 바뀌면 여기만
    고치면 됨)."""
    mag = abs(pwm)
    if mag < PWM_DEADZONE:
        return 0.0
    frac = mag / PWM_SAFETY_CAP
    mps = frac * MAX_WHEEL_MPS
    return mps if pwm >= 0 else -mps


class SimRobot:
    """
    실제 부품 노이즈를 반영한 로봇 시뮬레이터.
    - self.x/y/theta: 참(진짜) 위치 - 시뮬레이션 물리엔진이 아는 정답, 모터노이즈의 영향을 받음
    - step()이 반환하는 (odom_dx, odom_dy, odom_dtheta): 로봇이 "측정했다고 믿는" 오도메트리
      - 엔코더 정수틱 양자화를 거친 값이라, 참값과 항상 미세하게 다름(이게 SLAM/위치추정이
        보정해야 할 실제 오차)
    """
    def __init__(self, x, y, theta=0.0, wheel_sep=0.160, seed=None):
        self.x, self.y, self.theta = x, y, theta
        self.wheel_sep = wheel_sep
        self.rng = random.Random(seed)
        self.meters_per_tick = math.pi * WHEEL_DIAMETER_M / ENCODER_TICKS_PER_REV
        self._left_tick_frac = 0.0   # 아직 정수틱으로 안 깎인 소수부(누적)
        self._right_tick_frac = 0.0
        self.believed_theta = theta  # 로봇이 "측정만으로" 누적해온 방향 - 참 theta와는 별개로 흘러감

    def step(self, left, right, dt=0.05):
        # TT모터 + PWM 데드존: 명령값 -> PWM(데드존 보정 매핑) -> 실제 물리속도(데드존
        # 미만이면 0) -> 좌우 각각 독립적으로 ±5% 배율노이즈(모터 개체차/배터리전압 편차)
        left_pwm = _speed_to_pwm(left)
        right_pwm = _speed_to_pwm(right)
        left_speed_true = _pwm_to_wheel_mps(left_pwm) * self.rng.gauss(1.0, MOTOR_SPEED_NOISE_STD)
        right_speed_true = _pwm_to_wheel_mps(right_pwm) * self.rng.gauss(1.0, MOTOR_SPEED_NOISE_STD)

        # ---- 참(진짜) 위치 갱신 - 실제로 이만큼 움직임 ----
        v = (left_speed_true + right_speed_true) / 2
        w = (right_speed_true - left_speed_true) / self.wheel_sep
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt

        # ---- HC-020K 엔코더가 "측정하는" 값 - 20틱/회전 해상도로 양자화됨 ----
        left_dist_true = left_speed_true * dt
        right_dist_true = right_speed_true * dt
        self._left_tick_frac += left_dist_true / self.meters_per_tick
        self._right_tick_frac += right_dist_true / self.meters_per_tick
        left_ticks = int(self._left_tick_frac)   # 정수부만 "실제로 카운트됨"
        right_ticks = int(self._right_tick_frac)
        self._left_tick_frac -= left_ticks       # 나머지 소수부는 다음 스텝으로 이월(다음 틱에 언젠간 반영됨)
        self._right_tick_frac -= right_ticks

        left_dist_measured = left_ticks * self.meters_per_tick
        right_dist_measured = right_ticks * self.meters_per_tick
        v_measured = (left_dist_measured + right_dist_measured) / 2
        w_measured = (right_dist_measured - left_dist_measured) / self.wheel_sep

        # 오도메트리 적분엔 참 theta가 아니라, 로봇이 "측정만으로" 누적해온 방향을 씀
        # (실제 로봇은 참 방향을 절대 모름 - 이게 진짜 오도메트리 누적오차의 근원)
        #
        # 버그수정(2026-08-04): w_measured는 "거리"(left_dist_measured/right_dist_measured)로
        # 계산되므로 이미 "이번 스텝 동안의 각도 변화량"(라디안)임 - 각속도(rad/s)가 아님.
        # 바로 위 참위치 갱신의 w(속도 기반, rad/s)와 헷갈려서 여기에도 *dt를 붙였었는데,
        # 그러면 오도메트리 회전추정치가 실제보다 20배(=1/dt) 작게 나가서, 회전이 섞인
        # 구간마다 believed_theta가 실제보다 훨씬 느리게 갱신되고, 그 결과 odom_dx/dy에
        # 쓰이는 방향(cos/sin(believed_theta))도 계속 어긋나며 오차가 누적됨. 이게
        # 스캔매칭 탐색범위(12도)를 넘어서게 만들어서 위치추정이 통째로 발산하는
        # 원인이었음(실측 확인됨 - 노이즈를 꺼도 재현됨, 노이즈와 무관한 버그였음).
        odom_dx = v_measured * math.cos(self.believed_theta)
        odom_dy = v_measured * math.sin(self.believed_theta)
        odom_dtheta = w_measured
        self.believed_theta += odom_dtheta  # 다음 스텝을 위해 측정방향도 갱신

        return odom_dx, odom_dy, odom_dtheta


def run_scenario(sc, max_mapping_steps=20 * 60 * 5, max_nav_steps=20 * 180, on_step=None):
    """
    on_step(phase_name, robot, ctx) 콜백 시그니처가 바뀌었습니다:
      이전: ctx = {"nav": CDynamicNavigator 또는 None, "slam": CSlam 또는 None}
      이후: ctx = {"session": CMappingSession}
    매핑/내비게이션 전체가 세션 하나로 합쳐졌기 때문입니다. 이 콜백을 쓰는 다른
    headless 테스트 스크립트가 있다면 ctx["nav"]/ctx["slam"] 접근 부분을 고쳐야 합니다.
    """
    obstacles = sc["obstacles"]  # 라이다가 실제로 보는(=진짜 세계) 장애물
    known_obstacles = sc.get("known_obstacles", obstacles)  # 고정맵이 "미리 알고" 시작하는 장애물
    grid_w, grid_h = sc["grid_size"]
    start = sc["start"]
    goal = sc["goal"]
    goal_theta_deg = sc["goal_theta_deg"]

    robot = SimRobot(start[0], start[1], seed=42)
    step_count = 0
    pending_odom = (0.0, 0.0, 0.0)

    if sc["map_source"] == "slam":
        session = CMappingSession(
            size_x=grid_w, size_y=grid_h, resolution=0.05,
            start=(start[0], start[1], 0.0), goal=goal, robot_radius=0.10,
            goal_theta_deg=goal_theta_deg, max_mapping_steps=max_mapping_steps,
        )
    else:
        grid, rows, cols = make_grid_from_circles(grid_w, grid_h, 0.05, known_obstacles)
        session = CMappingSession.from_fixed_map(
            grid, rows, cols, 0.05,
            start=(start[0], start[1], 0.0), goal=goal, robot_radius=0.10,
            goal_theta_deg=goal_theta_deg,
        )

    if session.phase == SessionPhase.PLAN_FAILED:
        return {
            "name": sc["name"], "success": False, "reason": "A* 계획 실패",
            "final_pos_error": None, "final_angle_error": None,
            "steps": step_count, "waypoints": 0,
        }

    done = False
    for _ in range(max_mapping_steps + max_nav_steps):
        scan = lidar_scan(robot.x, robot.y, robot.theta, obstacles, rng=robot.rng)
        left, right, done = session.step(scan, *pending_odom)
        pending_odom = robot.step(left, right)
        step_count += 1

        if on_step:
            phase_name = "mapping" if session.phase == SessionPhase.MAPPING else "navigate"
            on_step(phase_name, robot, {"session": session})

        if session.phase == SessionPhase.PLAN_FAILED:
            return {
                "name": sc["name"], "success": False,
                "reason": "A* 계획 실패 (매핑은 끝났지만 목표까지 갈 방법을 못 찾음)",
                "final_pos_error": None, "final_angle_error": None,
                "steps": step_count, "waypoints": 0,
            }

        if done:
            break

    pos_error = math.hypot(robot.x - goal[0], robot.y - goal[1])
    angle_error = None
    if goal_theta_deg is not None:
        goal_theta = math.radians(goal_theta_deg)
        diff = robot.theta - goal_theta
        angle_error = abs(math.degrees(math.atan2(math.sin(diff), math.cos(diff))))

    return {
        "name": sc["name"],
        "success": bool(done),
        "reason": "정상 도착" if done else "시간초과(미도착)",
        "final_pos_error": pos_error,
        "final_angle_error": angle_error,
        "steps": step_count,
        "waypoints": len(session.get_waypoints()),
        "final_pose": (robot.x, robot.y, robot.theta),
    }
