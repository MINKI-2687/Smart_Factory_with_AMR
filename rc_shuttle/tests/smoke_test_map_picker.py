import math
import os
import sys
# 리팩토링(2026-08-11): "/home/claude" 하드코딩을 저장소 기준 상대경로로 교체
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "sim"))
from session_cwrap import CMappingSession, SessionPhase

OBSTACLES = [(1.00, 1.00, 0.20), (1.75, 2.00, 0.20), (2.00, 0.90, 0.20)]
ROOM = 3.0

def raycast(rx, ry, angle, max_range):
    best = max_range
    dirx, diry = math.cos(angle), math.sin(angle)
    if dirx > 1e-9:
        t = (ROOM - rx) / dirx
        if 0 < t < best: best = t
    if dirx < -1e-9:
        t = (0.0 - rx) / dirx
        if 0 < t < best: best = t
    if diry > 1e-9:
        t = (ROOM - ry) / diry
        if 0 < t < best: best = t
    if diry < -1e-9:
        t = (0.0 - ry) / diry
        if 0 < t < best: best = t
    for (cx, cy, r) in OBSTACLES:
        dx, dy = cx - rx, cy - ry
        proj = dx * dirx + dy * diry
        if proj < 0: continue
        perp2 = (dx*dx + dy*dy) - proj*proj
        if perp2 > r*r: continue
        t = proj - math.sqrt(r*r - perp2)
        if 0 < t < best: best = t
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

session = CMappingSession.for_mapping_only(
    size_x=ROOM, size_y=ROOM, resolution=0.05,
    start=(0.3, 0.3, 0.0), robot_radius=0.10, max_mapping_steps=20*60*5,
)

true_x, true_y, true_theta = 0.3, 0.3, 0.0
K_V, K_W, DT = 0.01, 0.01, 0.05

step = 0
max_steps = 20 * 60 * 8
while session.phase == SessionPhase.MAPPING and step < max_steps:
    scan = simulate_lidar(true_x, true_y, true_theta)
    left, right, done = session.step(scan, 0, 0, 0)
    linear = (left + right) / 2.0
    angular = (right - left) / 2.0
    v, w = linear * K_V, angular * K_W
    true_x += v * math.cos(true_theta) * DT
    true_y += v * math.sin(true_theta) * DT
    true_theta = math.atan2(math.sin(true_theta + w * DT), math.cos(true_theta + w * DT))
    step += 1

print(f"[step {step}] 매핑 종료, phase={session.phase.name}")
assert session.phase == SessionPhase.MAP_READY, "MAP_READY로 안 넘어감!"

prob_grid, rows, cols = session.get_prob_grid()
print(f"지도 크기: {rows} x {cols}")

picked_point_a = (0.5, 0.5, 0.0)
ok = session.start_navigation(goal=(picked_point_a[0], picked_point_a[1]), goal_theta_deg=picked_point_a[2])
print(f"start_navigation(A={picked_point_a[:2]}) -> {'성공' if ok else '실패'}, phase={session.phase.name}")
assert ok and session.phase == SessionPhase.NAVIGATE

step2 = 0
while session.phase == SessionPhase.NAVIGATE and step2 < max_steps:
    scan = simulate_lidar(true_x, true_y, true_theta)
    left, right, done = session.step(scan, 0, 0, 0)
    linear = (left + right) / 2.0
    angular = (right - left) / 2.0
    v, w = linear * K_V, angular * K_W
    true_x += v * math.cos(true_theta) * DT
    true_y += v * math.sin(true_theta) * DT
    true_theta = math.atan2(math.sin(true_theta + w * DT), math.cos(true_theta + w * DT))
    step2 += 1
    if done:
        break

pos_err = math.hypot(true_x - picked_point_a[0], true_y - picked_point_a[1])
print(f"[step {step2}] A지점 도착 phase={session.phase.name} pos_err={pos_err:.3f}m")
print("\n=== 전체 흐름 검증 성공 ===")
