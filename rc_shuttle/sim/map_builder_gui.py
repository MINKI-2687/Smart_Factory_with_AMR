"""
map_builder_gui.py
STEP 1 of 2 (mapping only - separate from navigation on purpose, see navigate_gui.py).

Runs SLAM mapping with REALISTIC sensor/actuator models (reused from run_scenario.py:
real lidar noise +/-30mm, real 0.72deg angular resolution, real encoder tick
quantization, real PWM deadzone) - no shortcuts. Once mapping is done, click on the
map to set point A and point B (position + facing direction). Saves:
  - <MAP_FILE>          : occupancy grid, text format (also loadable by main_shuttle.c)
  - <MAP_FILE>.points.txt : point A / point B coordinates + angles
  - <MAP_FILE>.png      : a snapshot image of the map + points, for quick visual reference

Usage: python3 map_builder_gui.py
"""
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from session_cwrap import CMappingSession, SessionPhase
from run_scenario import SimRobot, lidar_scan
import map_file_io

# ---- Scenario settings ----
ROOM_SIZE = 3.0
OBSTACLES = [(1.00, 1.00, 0.20), (1.75, 2.00, 0.20), (2.00, 0.90, 0.20)]
START = (0.3, 0.3, 0.0)
OCC_THRESHOLD = 0.6
RESOLUTION = 0.05

MAP_FILE = "factory_map.txt"
POINTS_FILE = "factory_map.points.txt"
PNG_FILE = "factory_map.png"

STEPS_PER_FRAME = 8
ARROW_LEN = 0.3

PICK_A_POS, PICK_A_DIR, PICK_B_POS, PICK_B_DIR, PICK_DONE = range(5)


class MapBuilderGui:
    def __init__(self):
        self.session = CMappingSession.for_mapping_only(
            size_x=ROOM_SIZE, size_y=ROOM_SIZE, resolution=RESOLUTION,
            start=START, robot_radius=0.10, max_mapping_steps=20 * 60 * 10,
            occ_threshold=OCC_THRESHOLD,
        )
        self.robot = SimRobot(START[0], START[1], START[2], seed=42)
        self.pending_odom = (0.0, 0.0, 0.0)
        self.step_count = 0
        self.path_hist = [(self.robot.x, self.robot.y)]
        self.stopped = False
        self.saved = False

        self.pick_state = None
        self.point_a = None
        self.point_b = None
        self._pending_pos = None

        self.fig, self.ax = plt.subplots(figsize=(7, 7.6))
        self.ax.set_xlim(-0.2, ROOM_SIZE + 0.2)
        self.ax.set_ylim(-0.2, ROOM_SIZE + 0.2)
        self.ax.set_aspect('equal')
        self.title = self.ax.set_title("")

        for (ox, oy, r) in OBSTACLES:
            self.ax.add_patch(plt.Circle((ox, oy), r, color='gray', alpha=0.5, zorder=1))

        extent = [0, ROOM_SIZE, 0, ROOM_SIZE]
        prob_grid, _, _ = self.session.get_prob_grid()
        self.grid_img = self.ax.imshow(prob_grid, origin='lower', extent=extent,
                                        cmap='Greys', vmin=0, vmax=1, alpha=0.6, zorder=0)

        self.path_line, = self.ax.plot([], [], '-', color='blue', linewidth=2, zorder=3, label='robot path')
        self.robot_dot, = self.ax.plot([self.robot.x], [self.robot.y], 'o', color='blue', markersize=10, zorder=6)
        self.a_marker, = self.ax.plot([], [], '*', color='green', markersize=18, zorder=5, label='Point A')
        self.b_marker, = self.ax.plot([], [], '*', color='red', markersize=18, zorder=5, label='Point B')
        self.a_arrow = None
        self.b_arrow = None
        self.preview_arrow = None
        self.ax.legend(loc='upper right', fontsize=8)

        self.fig.canvas.mpl_connect('button_press_event', self._on_click)
        self.fig.canvas.mpl_connect('motion_notify_event', self._on_move)

    def _step_mapping(self):
        scan = lidar_scan(self.robot.x, self.robot.y, self.robot.theta, OBSTACLES, rng=self.robot.rng)
        left, right, done = self.session.step(scan, *self.pending_odom)
        self.pending_odom = self.robot.step(left, right)
        self.step_count += 1
        self.path_hist.append((self.robot.x, self.robot.y))

        if self.session.phase == SessionPhase.MAP_READY:
            print(f"[map_builder] Mapping complete (step={self.step_count}) - "
                  f"click on the map to set point A's position")
            self.pick_state = PICK_A_POS

    def _on_click(self, event):
        if self.pick_state is None or event.inaxes != self.ax or event.xdata is None:
            return
        x, y = event.xdata, event.ydata

        if self.pick_state == PICK_A_POS:
            self._pending_pos = (x, y)
            self.a_marker.set_data([x], [y])
            self.pick_state = PICK_A_DIR
            print(f"[map_builder] Point A position=({x:.2f},{y:.2f}) - click to set facing direction")

        elif self.pick_state == PICK_A_DIR:
            px, py = self._pending_pos
            theta_deg = math.degrees(math.atan2(y - py, x - px))
            self.point_a = (px, py, theta_deg)
            self._draw_arrow('a', px, py, theta_deg, 'green')
            print(f"[map_builder] Point A confirmed: ({px:.2f},{py:.2f}), {theta_deg:.1f}deg - "
                  f"click to set point B's position")
            self.pick_state = PICK_B_POS

        elif self.pick_state == PICK_B_POS:
            self._pending_pos = (x, y)
            self.b_marker.set_data([x], [y])
            self.pick_state = PICK_B_DIR
            print(f"[map_builder] Point B position=({x:.2f},{y:.2f}) - click to set facing direction")

        elif self.pick_state == PICK_B_DIR:
            px, py = self._pending_pos
            theta_deg = math.degrees(math.atan2(y - py, x - px))
            self.point_b = (px, py, theta_deg)
            self._draw_arrow('b', px, py, theta_deg, 'red')
            print(f"[map_builder] Point B confirmed: ({px:.2f},{py:.2f}), {theta_deg:.1f}deg")
            self.pick_state = PICK_DONE
            self._save_everything()

    def _on_move(self, event):
        if self.pick_state not in (PICK_A_DIR, PICK_B_DIR):
            return
        if event.inaxes != self.ax or event.xdata is None or self._pending_pos is None:
            return
        px, py = self._pending_pos
        theta_deg = math.degrees(math.atan2(event.ydata - py, event.xdata - px))
        color = 'green' if self.pick_state == PICK_A_DIR else 'red'
        self._draw_arrow('preview', px, py, theta_deg, color, alpha=0.4)
        self.fig.canvas.draw_idle()

    def _draw_arrow(self, which, x, y, theta_deg, color, alpha=1.0):
        dx = ARROW_LEN * math.cos(math.radians(theta_deg))
        dy = ARROW_LEN * math.sin(math.radians(theta_deg))
        arrow = self.ax.annotate('', xy=(x + dx, y + dy), xytext=(x, y),
                                  arrowprops=dict(arrowstyle='->', color=color, lw=2, alpha=alpha),
                                  zorder=7)
        if which == 'a':
            if self.a_arrow: self.a_arrow.remove()
            self.a_arrow = arrow
        elif which == 'b':
            if self.b_arrow: self.b_arrow.remove()
            self.b_arrow = arrow
        else:
            if self.preview_arrow: self.preview_arrow.remove()
            self.preview_arrow = arrow

    def _save_everything(self):
        if self.preview_arrow:
            self.preview_arrow.remove()
            self.preview_arrow = None

        prob_grid, rows, cols = self.session.get_prob_grid()
        binary_grid = [[1 if p >= OCC_THRESHOLD else 0 for p in row] for row in prob_grid]
        map_file_io.save_map(MAP_FILE, binary_grid, RESOLUTION)
        map_file_io.save_points(POINTS_FILE, {"A": self.point_a, "B": self.point_b})

        self.title.set_text("Saved! See console for details. You can close this window now.")
        self.fig.canvas.draw()
        self.fig.savefig(PNG_FILE, dpi=120)
        print(f"[map_builder] Snapshot saved: {PNG_FILE}")
        print(f"[map_builder] Done. Run navigate_gui.py next (it loads {MAP_FILE} and {POINTS_FILE}).")
        self.saved = True
        self.stopped = True

    def update_frame(self, _frame):
        if not self.stopped and self.session.phase == SessionPhase.MAPPING:
            for _ in range(STEPS_PER_FRAME):
                self._step_mapping()
                if self.session.phase != SessionPhase.MAPPING:
                    break

        if self.session.phase == SessionPhase.MAPPING:
            prob_grid, _, _ = self.session.get_prob_grid()
            self.grid_img.set_data(prob_grid)

        xs = [p[0] for p in self.path_hist]
        ys = [p[1] for p in self.path_hist]
        self.path_line.set_data(xs, ys)
        self.robot_dot.set_data([self.robot.x], [self.robot.y])

        if not self.saved:
            status = self._status_text()
            self.title.set_text(f"step={self.step_count}  {status}")
        return self.grid_img, self.path_line, self.robot_dot

    def _status_text(self):
        if self.session.phase == SessionPhase.MAPPING:
            return "Mapping (frontier exploration in progress)"
        if self.pick_state == PICK_A_POS:
            return "Click on the map to set point A's position"
        if self.pick_state == PICK_A_DIR:
            return "Click to set point A's facing direction"
        if self.pick_state == PICK_B_POS:
            return "Click on the map to set point B's position"
        if self.pick_state == PICK_B_DIR:
            return "Click to set point B's facing direction"
        return ""

    def run(self):
        self.ani = animation.FuncAnimation(
            self.fig, self.update_frame, interval=30, blit=False, cache_frame_data=False
        )
        plt.show()


if __name__ == "__main__":
    gui = MapBuilderGui()
    gui.run()
