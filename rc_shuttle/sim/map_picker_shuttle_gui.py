"""
map_picker_shuttle_gui.py
1) Animate SLAM mapping (frontier exploration builds the map)
2) Once the map is complete, click on it to set point A and point B
   - Click 1: position (x, y)
   - Click 2: direction (draws an arrow) -> becomes the parking angle (theta)
3) Once both points are set, immediately start an A<->B shuttle simulation
   (a trigger button advances to the next leg, same idea as shuttle_gui.py)

All decision logic lives in session_cwrap.CMappingSession (= mapping_session.h, C).
This file only handles visualization and click/button events.

Usage: python3 map_picker_shuttle_gui.py
"""
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button

from session_cwrap import CMappingSession, SessionPhase

# ---- Scenario settings ----
ROOM_SIZE = 3.0
OBSTACLES = [(1.00, 1.00, 0.20), (1.75, 2.00, 0.20), (2.00, 0.90, 0.20)]
START = (0.3, 0.3, 0.0)

STEPS_PER_FRAME = 8
K_V, K_W, DT = 0.01, 0.01, 0.05
ARROW_LEN = 0.3  # direction arrow length (m, fixed for display only)


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


PICK_A_POS, PICK_A_DIR, PICK_B_POS, PICK_B_DIR, PICK_DONE = range(5)


class MapPickerShuttleGui:
    def __init__(self):
        self.session = CMappingSession.for_mapping_only(
            size_x=ROOM_SIZE, size_y=ROOM_SIZE, resolution=0.05,
            start=START, robot_radius=0.10, max_mapping_steps=20 * 60 * 5,
        )
        self.true_x, self.true_y, self.true_theta = START
        self.step_count = 0
        self.path_hist = [(self.true_x, self.true_y)]
        self.stopped = False

        self.pick_state = None
        self.point_a = None
        self.point_b = None
        self._pending_pos = None

        self.current_leg = 0
        self.waiting_for_trigger = False
        self.leg_count = 0

        self.fig, self.ax = plt.subplots(figsize=(7, 7.8))
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
        self.robot_dot, = self.ax.plot([self.true_x], [self.true_y], 'o', color='blue', markersize=10, zorder=6)
        self.a_marker, = self.ax.plot([], [], '*', color='green', markersize=18, zorder=5, label='Point A')
        self.b_marker, = self.ax.plot([], [], '*', color='red', markersize=18, zorder=5, label='Point B')
        self.a_arrow = None
        self.b_arrow = None
        self.preview_arrow = None
        self.ax.legend(loc='upper right', fontsize=8)

        self.cid_click = self.fig.canvas.mpl_connect('button_press_event', self._on_click)
        self.cid_move = self.fig.canvas.mpl_connect('motion_notify_event', self._on_move)

        button_ax = self.fig.add_axes([0.35, 0.02, 0.3, 0.05])
        self.trigger_button = Button(button_ax, 'Trigger (load on/off)')
        self.trigger_button.on_clicked(self._on_trigger_clicked)
        self.trigger_button.ax.set_visible(False)

    def _step_mapping(self):
        scan = simulate_lidar(self.true_x, self.true_y, self.true_theta)
        left, right, done = self.session.step(scan, 0, 0, 0)
        linear = (left + right) / 2.0
        angular = (right - left) / 2.0
        v, w = linear * K_V, angular * K_W
        self.true_x += v * math.cos(self.true_theta) * DT
        self.true_y += v * math.sin(self.true_theta) * DT
        self.true_theta = math.atan2(math.sin(self.true_theta + w * DT), math.cos(self.true_theta + w * DT))
        self.step_count += 1
        self.path_hist.append((self.true_x, self.true_y))

        if self.session.phase == SessionPhase.MAP_READY:
            print(f"[map_picker] Mapping complete (step={self.step_count}) - "
                  f"click on the map to set point A's position")
            self.pick_state = PICK_A_POS

    def _on_click(self, event):
        if event.inaxes != self.ax or event.xdata is None:
            return
        x, y = event.xdata, event.ydata

        if self.pick_state == PICK_A_POS:
            self._pending_pos = (x, y)
            self.a_marker.set_data([x], [y])
            self.pick_state = PICK_A_DIR
            print(f"[map_picker] Point A position=({x:.2f},{y:.2f}) set - "
                  f"now click to set the facing direction")

        elif self.pick_state == PICK_A_DIR:
            px, py = self._pending_pos
            theta_deg = math.degrees(math.atan2(y - py, x - px))
            self.point_a = (px, py, theta_deg)
            self._draw_arrow('a', px, py, theta_deg, 'green')
            print(f"[map_picker] Point A confirmed: ({px:.2f},{py:.2f}), {theta_deg:.1f}deg - "
                  f"now click to set point B's position")
            self.pick_state = PICK_B_POS

        elif self.pick_state == PICK_B_POS:
            self._pending_pos = (x, y)
            self.b_marker.set_data([x], [y])
            self.pick_state = PICK_B_DIR
            print(f"[map_picker] Point B position=({x:.2f},{y:.2f}) set - "
                  f"now click to set the facing direction")

        elif self.pick_state == PICK_B_DIR:
            px, py = self._pending_pos
            theta_deg = math.degrees(math.atan2(y - py, x - px))
            self.point_b = (px, py, theta_deg)
            self._draw_arrow('b', px, py, theta_deg, 'red')
            print(f"[map_picker] Point B confirmed: ({px:.2f},{py:.2f}), {theta_deg:.1f}deg")
            self.pick_state = PICK_DONE
            self._start_shuttle()

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

    def _start_shuttle(self):
        if self.preview_arrow:
            self.preview_arrow.remove()
            self.preview_arrow = None
        ok = self.session.start_navigation(goal=(self.point_a[0], self.point_a[1]),
                                            goal_theta_deg=self.point_a[2])
        print(f"[map_picker] Starting navigation to point A: {'OK' if ok else 'A* PLANNING FAILED!'}")
        if not ok:
            self.stopped = True

    def _step_shuttle(self):
        scan = simulate_lidar(self.true_x, self.true_y, self.true_theta)
        left, right, done = self.session.step(scan, 0, 0, 0)

        if self.session.phase == SessionPhase.PLAN_FAILED:
            print("[shuttle] PLAN_FAILED - stopping")
            self.stopped = True
            return

        v = (left + right) / 2.0 * K_V
        w = (right - left) / 2.0 * K_W
        self.true_x += v * math.cos(self.true_theta) * DT
        self.true_y += v * math.sin(self.true_theta) * DT
        self.true_theta = math.atan2(math.sin(self.true_theta + w * DT), math.cos(self.true_theta + w * DT))
        self.step_count += 1
        self.path_hist.append((self.true_x, self.true_y))

        if done:
            self.leg_count += 1
            label = "Point A" if self.current_leg == 0 else "Point B"
            print(f"[shuttle] Arrived at {label}! (leg {self.leg_count} total)")
            self.waiting_for_trigger = True
            self.trigger_button.ax.set_visible(True)

    def _on_trigger_clicked(self, _event):
        if not self.waiting_for_trigger or self.stopped:
            return
        next_leg = 1 - self.current_leg
        target = self.point_b if next_leg == 1 else self.point_a
        ok = self.session.retarget(goal=(target[0], target[1]), goal_theta_deg=target[2])
        label_next = "Point B" if next_leg == 1 else "Point A"
        print(f"[trigger] -> replanning to {label_next}: {'OK' if ok else 'FAILED'}")
        if not ok:
            self.stopped = True
            return
        self.current_leg = next_leg
        self.waiting_for_trigger = False
        self.trigger_button.ax.set_visible(False)

    def update_frame(self, _frame):
        if not self.stopped:
            for _ in range(STEPS_PER_FRAME):
                if self.stopped:
                    break
                if self.session.phase == SessionPhase.MAPPING:
                    self._step_mapping()
                elif self.pick_state in (PICK_A_POS, PICK_A_DIR, PICK_B_POS, PICK_B_DIR):
                    break
                elif self.waiting_for_trigger:
                    break
                elif self.session.phase == SessionPhase.NAVIGATE:
                    self._step_shuttle()
                elif self.session.phase == SessionPhase.DONE:
                    break

        if self.session.phase == SessionPhase.MAPPING:
            prob_grid, _, _ = self.session.get_prob_grid()
            self.grid_img.set_data(prob_grid)

        xs = [p[0] for p in self.path_hist]
        ys = [p[1] for p in self.path_hist]
        self.path_line.set_data(xs, ys)
        self.robot_dot.set_data([self.true_x], [self.true_y])

        status = self._status_text()
        self.title.set_text(f"step={self.step_count}  {status}")
        return self.grid_img, self.path_line, self.robot_dot

    def _status_text(self):
        if self.stopped:
            return "STOPPED"
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
        if self.waiting_for_trigger:
            label = "Point A" if self.current_leg == 0 else "Point B"
            return f"Arrived at {label} - waiting for trigger (click the button)  [leg {self.leg_count}]"
        return f"Shuttling [leg {self.leg_count}]"

    def run(self):
        self.ani = animation.FuncAnimation(
            self.fig, self.update_frame, interval=30, blit=False, cache_frame_data=False
        )
        plt.show()


if __name__ == "__main__":
    gui = MapPickerShuttleGui()
    gui.run()
