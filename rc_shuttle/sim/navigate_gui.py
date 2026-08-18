"""
navigate_gui.py
STEP 2 of 2 (navigation only - loads a map saved by map_builder_gui.py, no mapping here).

Because this program starts fresh with zero prior knowledge of where the robot
physically is, you must tell it the robot's actual starting pose (ROBOT_START below).
This is the practical stand-in for "localization" here: our scan matcher only
refines a position that's already roughly known (a small search window around a
predicted pose) - it cannot solve "figure out where I am from nothing" (that's a
much harder problem, global relocalization / the "kidnapped robot" problem, which
is not implemented). So in practice: place the robot at a known reference position
(e.g. point A, as if it were parked there when powered on) and tell the program that.

Shows real-time distance/heading error to the current target, both:
  - "est"  = what the robot's own localization believes (this is what the real
             hardware would actually use to decide when it's "done")
  - "true" = the actual physics ground truth (only available in simulation - this
             is how you catch localization drifting away from reality)

Includes an "Add Unexpected Obstacle" button to test the obstacle-detection +
replanning path without needing to touch the map file.

Usage: python3 navigate_gui.py
"""
import math
import random
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button

from session_cwrap import CMappingSession, SessionPhase
from run_scenario import SimRobot, LIDAR_NOISE_STD_M, LIDAR_ANGULAR_RESOLUTION_DEG
import map_file_io

MAP_FILE = "factory_map.txt"
POINTS_FILE = "factory_map.points.txt"

# The robot's actual physical starting pose - MUST match reality (see module docstring).
# Defaults to point A, matching "robot was parked at A when the program starts".
ROBOT_START_OVERRIDE = None  # e.g. (0.5, 0.5, 0.0) to override, or None to use point A

# An obstacle that is NOT in the saved map, to test "unexpected obstacle -> replan".
# It only appears in the simulated physical world once you click the button.
UNEXPECTED_OBSTACLE = (1.5, 1.5, 0.20)

STEPS_PER_FRAME = 8


def grid_raycast(grid_2d, rows, cols, resolution, x, y, angle, max_range):
    """March along the ray in small steps and check the occupancy grid directly -
    faster and more accurate than approximating a loaded grid as many small circles."""
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
    """Same noise model as run_scenario.lidar_scan (+/-30mm gaussian, 0.72deg
    resolution), but raycasts against a loaded occupancy grid instead of a circle
    list, plus any extra circle obstacles (used for the "unexpected obstacle" test)."""
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


class NavigateGui:
    def __init__(self):
        self.grid_2d, self.rows, self.cols, self.resolution = map_file_io.load_map(MAP_FILE)
        points = map_file_io.load_points(POINTS_FILE)
        self.point_a = points["A"]
        self.point_b = points["B"]
        self.room_size_x = self.cols * self.resolution
        self.room_size_y = self.rows * self.resolution

        start = ROBOT_START_OVERRIDE if ROBOT_START_OVERRIDE else self.point_a
        print(f"[navigate] Robot start pose = {start}  "
              f"(make sure the real/simulated robot is physically placed here!)")

        self.session = CMappingSession.from_fixed_map(
            self.grid_2d, self.rows, self.cols, self.resolution,
            start=start, goal=(self.point_a[0], self.point_a[1]), robot_radius=0.10,
            goal_theta_deg=self.point_a[2],
        )
        self.robot = SimRobot(start[0], start[1], math.radians(start[2]), seed=7)
        self.pending_odom = (0.0, 0.0, 0.0)

        # Extra (not-in-map) obstacles, used only for the "unexpected obstacle" test.
        # Starts empty; the loaded map itself is raycast directly via grid_lidar_scan.
        self.extra_obstacles = []
        self.obstacle_spawned = False

        self.step_count = 0
        self.leg_count = 0
        self.current_leg = 0  # 0: heading to A, 1: heading to B
        self.waiting_for_trigger = False
        self.stopped = False
        self.path_hist = [(self.robot.x, self.robot.y)]

        self._setup_plot()

    def _setup_plot(self):
        self.fig, self.ax = plt.subplots(figsize=(7, 7.8))
        self.ax.set_xlim(-0.2, self.room_size_x + 0.2)
        self.ax.set_ylim(-0.2, self.room_size_y + 0.2)
        self.ax.set_aspect('equal')
        self.title = self.ax.set_title("")

        import numpy as np
        grid_img_data = np.array(self.grid_2d, dtype=float)
        extent = [0, self.room_size_x, 0, self.room_size_y]
        self.ax.imshow(grid_img_data, origin='lower', extent=extent,
                        cmap='Greys', vmin=0, vmax=1, alpha=0.5, zorder=0)

        self.unexpected_patch = plt.Circle((UNEXPECTED_OBSTACLE[0], UNEXPECTED_OBSTACLE[1]),
                                            UNEXPECTED_OBSTACLE[2], color='orange', alpha=0.0, zorder=2,
                                            label='Unexpected obstacle')
        self.ax.add_patch(self.unexpected_patch)

        self.ax.plot([self.point_a[0]], [self.point_a[1]], '*', color='green', markersize=18,
                     zorder=5, label='Point A')
        self.ax.plot([self.point_b[0]], [self.point_b[1]], '*', color='red', markersize=18,
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
        print(f"[navigate] Unexpected obstacle spawned at {UNEXPECTED_OBSTACLE[:2]} "
              f"- watch for a replan when the robot's lidar sees it")

    def _on_trigger_clicked(self, _event):
        if not self.waiting_for_trigger or self.stopped:
            return
        next_leg = 1 - self.current_leg
        target = self.point_b if next_leg == 1 else self.point_a
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
        scan = grid_lidar_scan(self.grid_2d, self.rows, self.cols, self.resolution,
                                self.robot.x, self.robot.y, self.robot.theta,
                                extra_obstacles=self.extra_obstacles, rng=self.robot.rng)
        left, right, done = self.session.step(scan, *self.pending_odom)
        self.pending_odom = self.robot.step(left, right)
        self.step_count += 1
        self.path_hist.append((self.robot.x, self.robot.y))

        if self.session.phase == SessionPhase.PLAN_FAILED:
            print("[navigate] PLAN_FAILED - stopping")
            self.stopped = True
            return

        if done:
            self.leg_count += 1
            label = "Point A" if self.current_leg == 0 else "Point B"
            print(f"[navigate] Arrived at {label}! (leg {self.leg_count} total)")
            self.waiting_for_trigger = True
            self.trigger_button.ax.set_visible(True)

    def _current_target(self):
        return self.point_a if self.current_leg == 0 else self.point_b

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
            status = f"Arrived at {label} - waiting for trigger  [leg {self.leg_count}]"
        else:
            status = f"Navigating  [leg {self.leg_count}]  {self._error_text()}"
        self.title.set_text(f"step={self.step_count}\n{status}")
        return self.path_line, self.robot_dot

    def run(self):
        self.ani = animation.FuncAnimation(
            self.fig, self.update_frame, interval=30, blit=False, cache_frame_data=False
        )
        plt.show()


if __name__ == "__main__":
    gui = NavigateGui()
    gui.run()
