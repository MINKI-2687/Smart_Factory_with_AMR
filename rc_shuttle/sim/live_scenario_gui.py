"""
live_scenario_gui.py
scenarios.py에 정의된 시나리오 중 하나를 골라서 matplotlib로 실시간 애니메이션.

이 파일에는 "판단 로직"이 없습니다. SLAM 기반 시나리오(map_source=="slam")는
session_cwrap.CMappingSession(순수 C: mapping_session.h) 하나에 스캔+오도메트리를
넣고 나온 바퀴명령을 시뮬레이터 물리에 적용하기만 합니다. 프론티어 선택, 회피이력
관리, A*계획 성공여부 확인, 매핑->내비게이션 전환 - 전부 C쪽에서 처리되고 여기선
그 결과(phase, pose, target, waypoints, 확률격자)를 읽어서 그리기만 합니다.

고정맵 시나리오(map_source!="slam")는 애초에 프론티어 탐색이 없어서
CDynamicNavigator(nav_cwrap)를 그대로 직접 씁니다.

사용법: python3 live_scenario_gui.py [시나리오이름]
  인자 없이 실행하면 목록을 보여줌.
"""
import sys
import math

from scenarios import SCENARIOS
from run_scenario import lidar_scan, SimRobot
from nav_cwrap import make_grid_from_circles
from session_cwrap import CMappingSession, SessionPhase


def find_scenario(name):
    for sc in SCENARIOS:
        if sc["name"] == name:
            return sc
    return None


class LiveScenarioGui:
    STEPS_PER_FRAME = 8

    def __init__(self, sc):
        self.sc = sc
        self.obstacles = sc["obstacles"]
        # 매핑 중엔 없다가, navigate 단계 시작하는 순간 갑자기 나타나는 "예상 밖 장애물"
        self.dynamic_obstacles = sc.get("dynamic_obstacles", [])
        self._dynamic_obstacles_added = False
        self.known_obstacles = sc.get("known_obstacles", self.obstacles)
        self.grid_w, self.grid_h = sc["grid_size"]
        self.start = sc["start"]
        self.goal = sc["goal"]
        self.goal_theta_deg = sc["goal_theta_deg"]

        self.robot = SimRobot(self.start[0], self.start[1], seed=42)
        self.path_hist = [(self.robot.x, self.robot.y)]
        self.step_count = 0
        self.done = False
        self.result_text = ""
        self.plan_failed = False

        self.is_slam = (sc["map_source"] == "slam")
        self.session = None
        self._pending_odom = (0.0, 0.0, 0.0)

        if self.is_slam:
            self.session = CMappingSession(
                size_x=self.grid_w, size_y=self.grid_h, resolution=0.05,
                start=(self.start[0], self.start[1], 0.0),
                goal=self.goal, robot_radius=0.10,
                goal_theta_deg=self.goal_theta_deg,
            )
        else:
            # 고정맵도 라이다 스캔매칭으로 위치보정을 받음(순수 오도메트리만 쓰면
            # 방 하나 가로지르는 정도에서도 ~40cm급 드리프트가 쌓이는 게 실측 확인됨).
            # make_grid_from_circles가 이제 벽도 자동 포함하므로(obstacle_list.h 패치)
            # 스캔매칭이 벽까지 기준으로 삼을 수 있음.
            grid, rows, cols = make_grid_from_circles(self.grid_w, self.grid_h, 0.05, self.known_obstacles)
            self.session = CMappingSession.from_fixed_map(
                grid, rows, cols, 0.05,
                start=(self.start[0], self.start[1], 0.0),
                goal=self.goal, robot_radius=0.10,
                goal_theta_deg=self.goal_theta_deg,
            )
            if self.session.phase == SessionPhase.PLAN_FAILED:
                self.plan_failed = True
                self.done = True
                self.result_text = "A* 경로계획 실패 (고정맵으로 목표 도달 불가)"
                print(self.result_text)

        import matplotlib.pyplot as plt
        import matplotlib.animation as animation
        self._plt = plt
        self._animation = animation

        self.fig, self.ax = plt.subplots(figsize=(7, 7))
        self.ax.set_xlim(-0.2, self.grid_w + 0.2)
        self.ax.set_ylim(-0.2, self.grid_h + 0.2)
        self.ax.set_aspect('equal')
        self.title = self.ax.set_title("")

        _unknown_labeled = False
        static_obstacles = [o for o in self.obstacles if o not in self.dynamic_obstacles]
        for ox, oy, r in static_obstacles:
            is_known = any(abs(ox - kx) < 1e-6 and abs(oy - ky) < 1e-6 for kx, ky, kr in self.known_obstacles)
            color = 'gray' if is_known else 'orange'
            label = None
            if not is_known and not _unknown_labeled:
                label = '예상 밖 장애물(매핑 중 존재)'
                _unknown_labeled = True
            circ = plt.Circle((ox, oy), r, color=color, alpha=0.5, zorder=1, label=label)
            self.ax.add_patch(circ)

        self._dynamic_patches = []
        for i, (ox, oy, r) in enumerate(self.dynamic_obstacles):
            circ = plt.Circle((ox, oy), r, color='red', alpha=0.0, zorder=2,
                               label='동적 장애물(매핑 후 등장)' if i == 0 else None)
            self.ax.add_patch(circ)
            self._dynamic_patches.append(circ)

        self.grid_img = None
        if self.is_slam:
            extent = [0, self.grid_w, 0, self.grid_h]
            prob_grid, _, _ = self.session.get_prob_grid()
            self.grid_img = self.ax.imshow(prob_grid, origin='lower', extent=extent,
                                            cmap='Greys', vmin=0, vmax=1, alpha=0.6, zorder=0)

        self.path_line, = self.ax.plot([], [], '-', color='blue', linewidth=2, zorder=3, label='robot path')
        self.waypoint_scatter = self.ax.scatter([], [], s=20, c='green', marker='x', zorder=4, label='A* waypoints')
        self.robot_dot, = self.ax.plot([self.start[0]], [self.start[1]], 'o', color='blue', markersize=9, zorder=5)
        self.ax.plot([self.goal[0]], [self.goal[1]], '*', color='red', markersize=15, zorder=5, label='goal')

        self.ax.legend(loc='upper right', fontsize=8)
        self.fig.suptitle(f"{sc['name']}  (map_source={sc['map_source']})", fontsize=11)

    def _report_done(self):
        pos_err = math.hypot(self.robot.x - self.goal[0], self.robot.y - self.goal[1])
        ang_err_txt = ""
        if self.goal_theta_deg is not None:
            goal_theta = math.radians(self.goal_theta_deg)
            diff = self.robot.theta - goal_theta
            ang_err = abs(math.degrees(math.atan2(math.sin(diff), math.cos(diff))))
            ang_err_txt = f", angle error={ang_err:.1f}deg"
        self.result_text = f"DONE at step={self.step_count}, pos error={pos_err*100:.1f}cm{ang_err_txt}"
        print(self.result_text)

    def _step_once(self):
        if self.done:
            return

        scan = lidar_scan(self.robot.x, self.robot.y, self.robot.theta, self.obstacles, rng=self.robot.rng)

        phase_before = self.session.phase
        odx, ody, odth = self._pending_odom
        left, right, done = self.session.step(scan, odx, ody, odth)
        self._pending_odom = self.robot.step(left, right)

        phase_after = self.session.phase
        if self.is_slam and phase_before != SessionPhase.NAVIGATE and phase_after == SessionPhase.NAVIGATE:
            # 매핑 -> 내비게이션 전환된 그 순간에 "예상 밖 동적 장애물"을 실제로 등장시킴
            if self.dynamic_obstacles and not self._dynamic_obstacles_added:
                self.obstacles = self.obstacles + self.dynamic_obstacles
                self._dynamic_obstacles_added = True

        if phase_after == SessionPhase.PLAN_FAILED:
            self.done = True
            self.plan_failed = True
            self.result_text = f"PLAN_FAILED at step={self.step_count} (A* 경로계획 실패 - 목표까지 갈 방법을 못 찾음)"
            print(self.result_text)
        elif done:
            self.done = True
            self._report_done()

        self.step_count += 1
        self.path_hist.append((self.robot.x, self.robot.y))

    def update_frame(self, _frame):
        for _ in range(self.STEPS_PER_FRAME):
            if self.done:
                break
            self._step_once()

        if self.grid_img is not None and self.is_slam and self.session.phase == SessionPhase.MAPPING:
            prob_grid, _, _ = self.session.get_prob_grid()
            self.grid_img.set_data(prob_grid)

        if self._dynamic_obstacles_added:
            for patch in self._dynamic_patches:
                patch.set_alpha(0.6)

        xs = [p[0] for p in self.path_hist]
        ys = [p[1] for p in self.path_hist]
        self.path_line.set_data(xs, ys)
        self.robot_dot.set_data([self.robot.x], [self.robot.y])

        waypoints = None
        if not self.plan_failed and self.session.phase in (SessionPhase.NAVIGATE, SessionPhase.DONE):
            waypoints = self.session.get_waypoints()
        if waypoints:
            self.waypoint_scatter.set_offsets(waypoints)

        if self.done:
            status = self.result_text
        else:
            status = f"phase={self.session.phase.name.lower()}"
        self.title.set_text(f"step={self.step_count}  {status}")

        return self.grid_img, self.path_line, self.robot_dot, self.waypoint_scatter

    def run(self):
        self.ani = self._animation.FuncAnimation(
            self.fig, self.update_frame, interval=30, blit=False, cache_frame_data=False
        )
        self._plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 live_scenario_gui.py <scenario name>")
        print("\nAvailable scenarios:")
        for sc in SCENARIOS:
            print(f"  {sc['name']}")
        sys.exit(0)

    sc = find_scenario(sys.argv[1])
    if sc is None:
        print(f"Scenario not found: {sys.argv[1]}")
        sys.exit(1)

    demo = LiveScenarioGui(sc)
    demo.run()
