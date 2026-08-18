"""
test_fixed_map.py
No SLAM mapping - builds a fixed map directly from the FIXED_MAP_ROWS ASCII grid
below (1 = obstacle/wall, 0 = free space) and starts navigating right away. Uses
the already-validated fixed-map + lidar-localization path (robust even with full
realistic noise: ~1-3cm error, confirmed earlier this session), so this sidesteps
the still-unresolved SLAM-mapping drift issue entirely.

Tests:
  - navigation to point A and point B (A<->B shuttle with a trigger button)
  - parking distance/heading error, shown both as "est" (what the robot believes,
    what real hardware would use) and "true" (physics ground truth)
  - obstacle avoidance: an "Add Unexpected Obstacle" button spawns an obstacle that
    is NOT in the known map, to test detection + replanning

All decision logic is in session_cwrap.CMappingSession (= mapping_session.h, C).
Uses real sensor/actuator noise from run_scenario.py (lidar +/-30mm & 0.72deg,
encoder tick quantization, PWM deadzone) - no shortcuts.

Usage: python3 test_fixed_map.py
"""
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button

from session_cwrap import CMappingSession, SessionPhase
from run_scenario import SimRobot, LIDAR_NOISE_STD_M, LIDAR_ANGULAR_RESOLUTION_DEG

# ---- Scenario settings (edit freely) ----
# Fixed map, given directly as an ASCII grid (1 = obstacle/wall, 0 = free space).
# Header is "rows cols resolution_m". Row 0 of this list is grid row 0 (row =
# int(y / resolution)), so row 0 sits at y=0 and the last row sits at the max-y
# wall. If the rendered map looks flipped top-to-bottom vs. what you expect,
# just do FIXED_MAP_ROWS = FIXED_MAP_ROWS[::-1].
RESOLUTION = 0.05
FIXED_MAP_ROWS = [
    "111111111111111111111111111111",
    "100000000000000000000000000001",
    "100000000000000000000000000001",
    "100000000000000000000000000001",
    "100000000000000000000000000001",
    "100000000000000000000000000001",
    "100000000000000000000000000001",
    "100000000000000000000000000001",
    "100000000000000000000000000001",
    "100000000001111111100000000001",
    "100000000001111111100000000001",
    "100000000001111111100000000001",
    "100000000001111111100000000001",
    "100000000001111111100000000001",
    "100000000001111111100000000001",
    "100000000001111111100000000001",
    "100000000001111111100000000001",
    "111111111111111111111111111111",
]

FIXED_MAP_ROWS = FIXED_MAP_ROWS[::-1]


def build_fixed_grid(map_rows, resolution):
    """Parse an ASCII grid (list of equal-length '0'/'1' strings) into a
    grid_2d (list of lists of int), plus rows/cols/room size in meters."""
    rows = len(map_rows)
    cols = len(map_rows[0])
    assert all(len(r) == cols for r in map_rows), "all map rows must be the same length"
    grid_2d = [[1 if ch == '1' else 0 for ch in row] for row in map_rows]
    return grid_2d, rows, cols, cols * resolution, rows * resolution


ROOM_WIDTH = len(FIXED_MAP_ROWS[0]) * RESOLUTION   # 1.5 m
ROOM_HEIGHT = len(FIXED_MAP_ROWS) * RESOLUTION      # 0.9 m

POINT_A = (0.15, 0.15, 0.0)     # (x, y, parking angle deg) - bottom-left corner
POINT_B = (1.35, 0.15, 180.0)   # bottom-right corner
ROBOT_START = POINT_A          # robot physically starts parked at A

# An obstacle NOT in the known map - only appears in the simulated physical world
# once you click "Add Unexpected Obstacle". Placed in the open strip between
# A and B so it forces a detour/replan.
UNEXPECTED_OBSTACLE = (0.75, 0.25, 0.08)

STEPS_PER_FRAME = 8

# 엔코더가 아직 물리적으로 없는 상황을 그대로 재현: True면 로봇이 실제로 움직인 만큼의
# 오도메트리(엔코더 틱 기반, run_scenario.SimRobot.step()의 반환값)를 session에 넘기고,
# False면 매 스텝 오도메트리 변화량을 그대로 (0,0,0)으로 고정해서 넘김 - 즉 위치추정을
# 라이다 스캔매칭(session 내부의 slam_localize_step)에만 맡김. robot.step() 자체는
# (물리엔진 갱신을 위해) 그대로 계속 호출하고, 반환되는 오도메트리 값만 버림.
# robot_runner.h의 RobotConfig.has_encoder=false와 동일한 상황을 시뮬레이션에서 검증하기
# 위한 용도.
USE_ENCODER = False


def grid_raycast(grid_2d, rows, cols, resolution, x, y, angle, max_range):
    step = resolution * 0.5
    dist = step
    dx, dy = math.cos(angle), math.sin(angle)
    while dist < max_range:
        px, py = x + dx * dist, y + dy * dist
        col, row = int(px / resolution), int(py / resolution)
        if row < 0 or row >= rows or col < 0 or col >= cols:
            return dist
        if grid_2d[row][col]:
            return dist
        dist += step
    return max_range


def grid_lidar_scan(grid_2d, rows, cols, resolution, rx, ry, rtheta,
                     extra_obstacles=None, max_range=4.0, rng=None):
    import random
    rng = rng or random
    scan = []
    a = -180.0
    while a < 180.0:
        wa = rtheta + math.radians(a)
        d = grid_raycast(grid_2d, rows, cols, resolution, rx, ry, wa, max_range)

        if extra_obstacles:
            dirx, diry = math.cos(wa), math.sin(wa)
            for (cx, cy, r) in extra_obstacles:
                dx, dy = cx - rx, cy - ry
                proj = dx * dirx + dy * diry
                if proj < 0:
                    continue
                perp2 = (dx * dx + dy * dy) - proj * proj
                if perp2 > r * r:
                    continue
                t = proj - math.sqrt(r * r - perp2)
                if 0 < t < d:
                    d = t

        if d < max_range - 1e-6:
            noisy = d + rng.gauss(0, LIDAR_NOISE_STD_M)
            noisy = max(0.01, noisy)
            scan.append((a, noisy * 1000.0))
        a += LIDAR_ANGULAR_RESOLUTION_DEG
    return scan


class TestFixedMapGui:
    def __init__(self):
        self.grid_2d, self.rows, self.cols, _w, _h = build_fixed_grid(
            FIXED_MAP_ROWS, RESOLUTION)

        self.session = CMappingSession.from_fixed_map(
            self.grid_2d, self.rows, self.cols, RESOLUTION,
            start=ROBOT_START, goal=(POINT_A[0], POINT_A[1]), robot_radius=0.10,
            goal_theta_deg=POINT_A[2],
        )
        print(f"[test] Initial plan to point A: "
              f"{'OK' if self.session.phase == SessionPhase.NAVIGATE else 'A* PLANNING FAILED!'}")

        self.robot = SimRobot(ROBOT_START[0], ROBOT_START[1], math.radians(ROBOT_START[2]), seed=7)
        self.pending_odom = (0.0, 0.0, 0.0)

        self.extra_obstacles = []
        self.obstacle_spawned = False

        self.step_count = 0
        self.leg_count = 0
        self.current_leg = 0  # 0: heading to A, 1: heading to B
        self.waiting_for_trigger = False
        self.stopped = (self.session.phase != SessionPhase.NAVIGATE)
        self.path_hist = [(self.robot.x, self.robot.y)]

        self._setup_plot()

    def _setup_plot(self):
        self.fig, self.ax = plt.subplots(figsize=(9, 6.4))
        self.ax.set_xlim(-0.2, ROOM_WIDTH + 0.2)
        self.ax.set_ylim(-0.2, ROOM_HEIGHT + 0.2)
        self.ax.set_aspect('equal')
        self.title = self.ax.set_title("")

        # Draw the fixed map itself (1 = wall/obstacle, 0 = free space).
        self.ax.imshow(self.grid_2d, extent=(0, ROOM_WIDTH, 0, ROOM_HEIGHT),
                        origin='lower', cmap='Greys', vmin=0, vmax=1, alpha=0.5, zorder=1)

        self.unexpected_patch = plt.Circle((UNEXPECTED_OBSTACLE[0], UNEXPECTED_OBSTACLE[1]),
                                            UNEXPECTED_OBSTACLE[2], color='orange', alpha=0.0, zorder=2,
                                            label='Unexpected obstacle')
        self.ax.add_patch(self.unexpected_patch)

        self.ax.plot([POINT_A[0]], [POINT_A[1]], '*', color='green', markersize=18,
                     zorder=5, label='Point A')
        self.ax.plot([POINT_B[0]], [POINT_B[1]], '*', color='red', markersize=18,
                     zorder=5, label='Point B')
        self.path_line, = self.ax.plot([], [], '-', color='blue', linewidth=2, zorder=3, label='robot path')
        self.robot_dot, = self.ax.plot([self.robot.x], [self.robot.y], 'o', color='blue', markersize=10, zorder=6)
        self.ax.legend(loc='upper right', fontsize=7)

        trigger_ax = self.fig.add_axes([0.35, 0.02, 0.28, 0.05])
        self.trigger_button = Button(trigger_ax, 'Trigger (load on/off)')
        self.trigger_button.on_clicked(self._on_trigger_clicked)
        self.trigger_button.ax.set_visible(False)

        obstacle_ax = self.fig.add_axes([0.02, 0.02, 0.28, 0.05])
        self.obstacle_button = Button(obstacle_ax, 'Add Unexpected Obstacle')
        self.obstacle_button.on_clicked(self._on_spawn_obstacle)

    def _on_spawn_obstacle(self, _event):
        if self.obstacle_spawned:
            return
        self.extra_obstacles.append(UNEXPECTED_OBSTACLE)
        self.obstacle_spawned = True
        self.unexpected_patch.set_alpha(0.6)
        print(f"[test] Unexpected obstacle spawned at {UNEXPECTED_OBSTACLE[:2]} "
              f"- watch for a replan when the robot's lidar sees it")

    def _on_trigger_clicked(self, _event):
        if not self.waiting_for_trigger or self.stopped:
            return
        next_leg = 1 - self.current_leg
        target = POINT_B if next_leg == 1 else POINT_A
        ok = self.session.retarget(goal=(target[0], target[1]), goal_theta_deg=target[2])
        print(f"[trigger] -> replanning to {'Point B' if next_leg == 1 else 'Point A'}: "
              f"{'OK' if ok else 'FAILED'}")
        if not ok:
            self.stopped = True
            return
        self.current_leg = next_leg
        self.waiting_for_trigger = False
        self.trigger_button.ax.set_visible(False)

    def _step(self):
        scan = grid_lidar_scan(self.grid_2d, self.rows, self.cols, RESOLUTION,
                                self.robot.x, self.robot.y, self.robot.theta,
                                extra_obstacles=self.extra_obstacles, rng=self.robot.rng)
        left, right, done = self.session.step(scan, *self.pending_odom)
        measured_odom = self.robot.step(left, right)
        self.pending_odom = measured_odom if USE_ENCODER else (0.0, 0.0, 0.0)
        self.step_count += 1
        self.path_hist.append((self.robot.x, self.robot.y))

        if self.session.phase == SessionPhase.PLAN_FAILED:
            print("[test] PLAN_FAILED - stopping")
            self.stopped = True
            return

        if done:
            self.leg_count += 1
            label = "Point A" if self.current_leg == 0 else "Point B"
            print(f"[test] Arrived at {label}! (leg {self.leg_count} total)")
            self.waiting_for_trigger = True
            self.trigger_button.ax.set_visible(True)

    def _current_target(self):
        return POINT_A if self.current_leg == 0 else POINT_B

    def _error_text(self):
        tx, ty, ttheta_deg = self._current_target()
        est_x, est_y, est_theta = self.session.get_pose()

        est_dist = math.hypot(est_x - tx, est_y - ty)
        true_dist = math.hypot(self.robot.x - tx, self.robot.y - ty)

        def heading_err_deg(theta_rad):
            diff = theta_rad - math.radians(ttheta_deg)
            return abs(math.degrees(math.atan2(math.sin(diff), math.cos(diff))))

        est_head = heading_err_deg(est_theta)
        true_head = heading_err_deg(self.robot.theta)

        label = "A" if self.current_leg == 0 else "B"
        return (f"Target {label}: dist est={est_dist:.3f}m true={true_dist:.3f}m | "
                f"heading est={est_head:.1f}deg true={true_head:.1f}deg")

    def update_frame(self, _frame):
        if not self.stopped and not self.waiting_for_trigger and self.session.phase == SessionPhase.NAVIGATE:
            for _ in range(STEPS_PER_FRAME):
                if self.stopped or self.waiting_for_trigger:
                    break
                self._step()

        xs = [p[0] for p in self.path_hist]
        ys = [p[1] for p in self.path_hist]
        self.path_line.set_data(xs, ys)
        self.robot_dot.set_data([self.robot.x], [self.robot.y])

        if self.stopped:
            status = "STOPPED"
        elif self.waiting_for_trigger:
            label = "Point A" if self.current_leg == 0 else "Point B"
            status = f"Arrived at {label} - waiting for trigger  [leg {self.leg_count}] " + self._error_text()
        else:
            status = f"Navigating  [leg {self.leg_count}]  {self._error_text()}"
        encoder_tag = "encoder=ON" if USE_ENCODER else "encoder=OFF(lidar-only)"
        self.title.set_text(f"step={self.step_count}  [{encoder_tag}]\n{status}")
        return self.path_line, self.robot_dot

    def run(self):
        self.ani = animation.FuncAnimation(
            self.fig, self.update_frame, interval=30, blit=False, cache_frame_data=False
        )
        plt.show()


if __name__ == "__main__":
    gui = TestFixedMapGui()
    gui.run()
