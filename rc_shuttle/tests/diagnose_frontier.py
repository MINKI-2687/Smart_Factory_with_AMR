import math
import os
import sys
# 리팩토링(2026-08-11): "/home/claude" 하드코딩을 저장소 기준 상대경로로 교체
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "sim"))
from session_cwrap import CMappingSession, SessionPhase
from run_scenario import SimRobot, lidar_scan

ROOM_SIZE = 3.0
OBSTACLES = [(1.00, 1.00, 0.20), (1.75, 2.00, 0.20), (2.00, 0.90, 0.20)]
START = (0.3, 0.3, 0.0)

session = CMappingSession.for_mapping_only(
    size_x=ROOM_SIZE, size_y=ROOM_SIZE, resolution=0.05,
    start=START, robot_radius=0.10, max_mapping_steps=20 * 60 * 10,
)
robot = SimRobot(START[0], START[1], START[2], seed=42)
pending_odom = (0.0, 0.0, 0.0)

last_target = None
step = 0
max_steps = 20 * 60 * 15
while session.phase == SessionPhase.MAPPING and step < max_steps:
    scan = lidar_scan(robot.x, robot.y, robot.theta, OBSTACLES, rng=robot.rng)
    left, right, done = session.step(scan, *pending_odom)
    pending_odom = robot.step(left, right)

    fd = session.get_frontier_debug()
    if fd["target"] != last_target:
        est_x, est_y, est_theta = session.get_pose()
        drift = math.hypot(robot.x - est_x, robot.y - est_y)
        print(f"[step {step:5d}] NEW target={fd['target']} "
              f"est=({est_x:.3f},{est_y:.3f}) true=({robot.x:.3f},{robot.y:.3f}) drift={drift:.3f}m "
              f"miss={fd['miss_count']} avoid_hist={fd['avoid_history_count']}")
        last_target = fd["target"]

    step += 1

print(f"\n[step {step}] mapping finished, phase={session.phase.name}")
fd = session.get_frontier_debug()
est_x, est_y, est_theta = session.get_pose()
drift = math.hypot(robot.x - est_x, robot.y - est_y)
print(f"final: est=({est_x:.3f},{est_y:.3f}) true=({robot.x:.3f},{robot.y:.3f}) drift={drift:.3f}m")
print(f"final frontier debug: {fd}")

prob_grid, rows, cols = session.get_prob_grid()
free_count = sum(1 for row in prob_grid for p in row if p < 0.45)
unknown_count = sum(1 for row in prob_grid for p in row if 0.45 <= p < 0.6)
occ_count = sum(1 for row in prob_grid for p in row if p >= 0.6)
total = rows * cols
print(f"\ngrid stats: free={free_count} ({100*free_count/total:.1f}%) "
      f"unknown={unknown_count} ({100*unknown_count/total:.1f}%) "
      f"occupied={occ_count} ({100*occ_count/total:.1f}%)")
