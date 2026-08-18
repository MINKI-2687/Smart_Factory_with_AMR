"""
shuttle_gui.py
main_shuttle.c(A<->B 왕복, 트리거 대기)를 소프트웨어로 그대로 재현하는 GUI.
판단 로직은 전부 session_cwrap.CMappingSession(=mapping_session.h, C)이 하고,
이 파일은 시각화 + "트리거" 버튼 클릭 처리만 담당합니다.

버튼을 누르는 게 실제 감압센서가 "짐 얹힘"/"짐 내려감"을 감지하는 순간을 흉내냅니다.

사용법: python3 shuttle_gui.py
"""
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from session_cwrap import CMappingSession, SessionPhase

# ---- 시나리오 설정 (원하는 대로 바꿔서 테스트) ----
ROOM_SIZE = 3.0
OBSTACLES = [(1.00, 1.00, 0.20), (1.75, 2.00, 0.20), (2.00, 0.90, 0.20)]
START = (0.3, 0.3, 0.0)
POINT_A = (0.5, 0.5, 0.0)      # (x, y, 주차각도deg)
POINT_B = (2.5, 2.5, 180.0)
USE_SLAM = True                 # False면 OBSTACLES를 그대로 고정맵으로 사용

STEPS_PER_FRAME = 8
K_V, K_W, DT = 0.01, 0.01, 0.05


def raycast(rx, ry, angle, max_range):
    best = max_range
    dirx, diry = math.cos(angle), math.sin(angle)
    if dirx > 1e-9:
        t = (ROOM_SIZE - rx) / dirx
        if 0 < t < best: best = t
    if dirx < -1e-9:
        t = (0.0 - rx) / dirx
        if 0 < t < best: best = t
    if diry > 1e-9:
        t = (ROOM_SIZE - ry) / diry
        if 0 < t < best: best = t
    if diry < -1e-9:
        t = (0.0 - ry) / diry
        if 0 < t < best: best = t
    for (cx, cy, r) in OBSTACLES:
        dx, dy = cx - rx, cy - ry
        proj = dx * dirx + dy * diry
        if proj < 0:
            continue
        perp2 = (dx * dx + dy * dy) - proj * proj
        if perp2 > r * r:
            continue
        t = proj - math.sqrt(r * r - perp2)
        if 0 < t < best:
            best = t
    return best


def simulate_lidar(rx, ry, rtheta):
    scan = []
    a = -180.0
    while a < 180.0:
        wa = rtheta + math.radians(a)
        d = raycast(rx, ry, wa, 3.0)
        if d < 2.999:
            scan.append((a, d * 1000.0))
        a += 2.0
    return scan


class ShuttleGui:
    def __init__(self):
        if USE_SLAM:
            self.session = CMappingSession(
                size_x=ROOM_SIZE, size_y=ROOM_SIZE, resolution=0.05,
                start=START, goal=(POINT_A[0], POINT_A[1]), robot_radius=0.10,
                goal_theta_deg=POINT_A[2], max_mapping_steps=20 * 60 * 5,
            )
        else:
            from nav_cwrap import make_grid_from_circles
            grid, rows, cols = make_grid_from_circles(ROOM_SIZE, ROOM_SIZE, 0.05, OBSTACLES)
            self.session = CMappingSession.from_fixed_map(
                grid, rows, cols, 0.05,
                start=START, goal=(POINT_A[0], POINT_A[1]), robot_radius=0.10,
                goal_theta_deg=POINT_A[2],
            )

        self.true_x, self.true_y, self.true_theta = START
        self.current_leg = 0  # 0: A로 향하는 중, 1: B로 향하는 중
        self.points = [POINT_A, POINT_B]
        self.labels = ["A지점 (짐 얹힘 트리거)", "B지점 (짐 내려감 트리거)"]
        self.waiting_for_trigger = False
        self.step_count = 0
        self.leg_count = 0
        self.path_hist = [(self.true_x, self.true_y)]
        self.stopped = False

        self.fig, self.ax = plt.subplots(figsize=(7, 7.5))
        self.ax.set_xlim(-0.2, ROOM_SIZE + 0.2)
        self.ax.set_ylim(-0.2, ROOM_SIZE + 0.2)
        self.ax.set_aspect('equal')
        self.title = self.ax.set_title("")

        for (ox, oy, r) in OBSTACLES:
            self.ax.add_patch(plt.Circle((ox, oy), r, color='gray', alpha=0.5, zorder=1))

        self.ax.plot([POINT_A[0]], [POINT_A[1]], '*', color='green', markersize=18, zorder=5, label='A지점')
        self.ax.plot([POINT_B[0]], [POINT_B[1]], '*', color='red', markersize=18, zorder=5, label='B지점')
        self.path_line, = self.ax.plot([], [], '-', color='blue', linewidth=2, zorder=3, label='로봇 경로')
        self.robot_dot, = self.ax.plot([self.true_x], [self.true_y], 'o', color='blue', markersize=10, zorder=6)
        self.ax.legend(loc='upper right', fontsize=8)

        # ---- 트리거 버튼 (실제 감압센서를 흉내내는 부분) ----
        from matplotlib.widgets import Button
        button_ax = self.fig.add_axes([0.35, 0.02, 0.3, 0.06])
        self.trigger_button = Button(button_ax, '트리거 발생 (짐 얹힘/내림)')
        self.trigger_button.on_clicked(self._on_trigger_clicked)
        self.trigger_button.ax.set_visible(False)  # 대기 중일 때만 보이게

    def _on_trigger_clicked(self, _event):
        if not self.waiting_for_trigger or self.stopped:
            return
        next_leg = 1 - self.current_leg
        goal_x, goal_y, goal_theta_deg = self.points[next_leg]
        ok = self.session.retarget(goal=(goal_x, goal_y), goal_theta_deg=goal_theta_deg)
        print(f"[trigger] {self.labels[self.current_leg]} 트리거 발생 -> {self.labels[next_leg]}로 재계획: "
              f"{'성공' if ok else 'A*계획 실패!'}")
        if not ok:
            self.stopped = True
            return
        self.current_leg = next_leg
        self.waiting_for_trigger = False
        self.trigger_button.ax.set_visible(False)

    def _step_once(self):
        if self.stopped or self.waiting_for_trigger:
            return

        scan = simulate_lidar(self.true_x, self.true_y, self.true_theta)
        left, right, done = self.session.step(scan, 0, 0, 0)

        if self.session.phase == SessionPhase.PLAN_FAILED:
            print("[shuttle] PLAN_FAILED - 중단")
            self.stopped = True
            return

        v = (left + right) / 2.0 * K_V
        w = (right - left) / 2.0 * K_W
        self.true_x += v * math.cos(self.true_theta) * DT
        self.true_y += v * math.sin(self.true_theta) * DT
        self.true_theta = math.atan2(math.sin(self.true_theta + w * DT), math.cos(self.true_theta + w * DT))

        if done:
            self.leg_count += 1
            label = self.labels[self.current_leg]
            print(f"[shuttle] {label} 도착! (누적 {self.leg_count}구간) pos=({self.true_x:.3f},{self.true_y:.3f})")
            self.waiting_for_trigger = True
            self.trigger_button.ax.set_visible(True)

        self.step_count += 1
        self.path_hist.append((self.true_x, self.true_y))

    def update_frame(self, _frame):
        for _ in range(STEPS_PER_FRAME):
            if self.stopped or self.waiting_for_trigger:
                break
            self._step_once()

        xs = [p[0] for p in self.path_hist]
        ys = [p[1] for p in self.path_hist]
        self.path_line.set_data(xs, ys)
        self.robot_dot.set_data([self.true_x], [self.true_y])

        if self.stopped:
            status = "중단됨 (A*계획 실패)"
        elif self.waiting_for_trigger:
            status = f"{self.labels[self.current_leg]} 도착 - 트리거 대기 중 (버튼을 눌러주세요)"
        else:
            status = f"phase={self.session.phase.name.lower()}, {self.labels[self.current_leg]}로 이동 중"
        self.title.set_text(f"step={self.step_count}  누적 {self.leg_count}구간\n{status}")

        return self.path_line, self.robot_dot

    def run(self):
        self.ani = animation.FuncAnimation(
            self.fig, self.update_frame, interval=30, blit=False, cache_frame_data=False
        )
        plt.show()


if __name__ == "__main__":
    gui = ShuttleGui()
    gui.run()
