"""헤드리스로 test_fixed_map.py와 동일한 시나리오를 돌려서,
USE_ENCODER=True(기존) vs False(엔코더 없음, 라이다 스캔매칭만) 두 경우의
도착 성공 여부/최종 오차를 비교 검증."""
import math
import random

from session_cwrap import CMappingSession, SessionPhase
from run_scenario import SimRobot, LIDAR_NOISE_STD_M, LIDAR_ANGULAR_RESOLUTION_DEG

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
][::-1]

POINT_A = (0.15, 0.15, 0.0)
POINT_B = (1.35, 0.15, 180.0)


def build_fixed_grid(map_rows, resolution):
    rows = len(map_rows)
    cols = len(map_rows[0])
    grid_2d = [[1 if ch == '1' else 0 for ch in row] for row in map_rows]
    return grid_2d, rows, cols


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


def grid_lidar_scan(grid_2d, rows, cols, resolution, rx, ry, rtheta, max_range=4.0, rng=None):
    rng = rng or random
    scan = []
    a = -180.0
    while a < 180.0:
        wa = rtheta + math.radians(a)
        d = grid_raycast(grid_2d, rows, cols, resolution, rx, ry, wa, max_range)
        if d < max_range - 1e-6:
            noisy = max(0.01, d + rng.gauss(0, LIDAR_NOISE_STD_M))
            scan.append((a, noisy * 1000.0))
        a += LIDAR_ANGULAR_RESOLUTION_DEG
    return scan


def run_one(use_encoder, seed, max_steps=4000):
    grid_2d, rows, cols = build_fixed_grid(FIXED_MAP_ROWS, RESOLUTION)
    session = CMappingSession.from_fixed_map(
        grid_2d, rows, cols, RESOLUTION,
        start=POINT_A, goal=(POINT_B[0], POINT_B[1]), robot_radius=0.10,
        goal_theta_deg=POINT_B[2],
    )
    if session.phase != SessionPhase.NAVIGATE:
        return dict(ok=False, reason="A* plan failed")

    robot = SimRobot(POINT_A[0], POINT_A[1], math.radians(POINT_A[2]), seed=seed)
    pending_odom = (0.0, 0.0, 0.0)

    max_est_true_gap = 0.0
    for step in range(max_steps):
        scan = grid_lidar_scan(grid_2d, rows, cols, RESOLUTION, robot.x, robot.y, robot.theta,
                                rng=robot.rng)
        left, right, done = session.step(scan, *pending_odom)
        measured_odom = robot.step(left, right)
        pending_odom = measured_odom if use_encoder else (0.0, 0.0, 0.0)

        est_x, est_y, _ = session.get_pose()
        gap = math.hypot(est_x - robot.x, est_y - robot.y)
        max_est_true_gap = max(max_est_true_gap, gap)

        if session.phase == SessionPhase.PLAN_FAILED:
            return dict(ok=False, reason=f"PLAN_FAILED at step {step}", max_est_true_gap=max_est_true_gap)
        if done:
            true_dist = math.hypot(robot.x - POINT_B[0], robot.y - POINT_B[1])
            heading_err = abs(math.degrees(math.atan2(
                math.sin(robot.theta - math.radians(POINT_B[2])),
                math.cos(robot.theta - math.radians(POINT_B[2])))))
            return dict(ok=True, steps=step, true_dist=true_dist, heading_err=heading_err,
                        max_est_true_gap=max_est_true_gap)
    return dict(ok=False, reason="timeout", max_est_true_gap=max_est_true_gap)


if __name__ == "__main__":
    import sys
    n_seeds = int(sys.argv[1]) if len(sys.argv) > 1 else 10
    max_steps = int(sys.argv[2]) if len(sys.argv) > 2 else 4000
    for use_encoder in (True, False):
        label = "encoder=ON " if use_encoder else "encoder=OFF"
        results = [run_one(use_encoder, seed=s, max_steps=max_steps) for s in range(n_seeds)]
        n_ok = sum(1 for r in results if r["ok"])
        print(f"[{label}] success {n_ok}/{n_seeds}", flush=True)
        for i, r in enumerate(results):
            if r["ok"]:
                print(f"  seed={i}: steps={r['steps']:4d} true_dist_err={r['true_dist']*100:5.1f}cm "
                      f"heading_err={r['heading_err']:5.1f}deg max_est_true_gap={r['max_est_true_gap']*100:5.1f}cm", flush=True)
            else:
                print(f"  seed={i}: FAIL ({r['reason']}) max_est_true_gap={r.get('max_est_true_gap',0)*100:5.1f}cm", flush=True)
