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

session = CMappingSession(
    size_x=ROOM, size_y=ROOM, resolution=0.05,
    start=(0.3, 0.3, 0.0), goal=(2.5, 2.5), robot_radius=0.10,
    goal_theta_deg=90.0, max_mapping_steps=20*60*5,
)

true_x, true_y, true_theta = 0.3, 0.3, 0.0
K_V, K_W, DT = 0.01, 0.01, 0.05

max_total_steps = 20 * 60 * 8
phase_changes = []
last_phase = None
step = 0
for step in range(max_total_steps):
    scan = simulate_lidar(true_x, true_y, true_theta)
    left, right, done = session.step(scan, 0, 0, 0)  # 완벽한 오도메트리 가정(odom delta는 여기선 안 씀 - true pose로 직접 시뮬)

    ph = session.phase
    if ph != last_phase:
        phase_changes.append((step, ph.name))
        last_phase = ph

    if ph == SessionPhase.PLAN_FAILED:
        print(f"[step {step}] PLAN_FAILED - A* 계획 실패로 세션 중단")
        break

    linear = (left + right) / 2.0
    angular = (right - left) / 2.0
    v, w = linear * K_V, angular * K_W
    true_x += v * math.cos(true_theta) * DT
    true_y += v * math.sin(true_theta) * DT
    true_theta = math.atan2(math.sin(true_theta + w * DT), math.cos(true_theta + w * DT))

    if done:
        print(f"[step {step}] DONE (phase={ph.name})")
        break
else:
    print(f"[step {step}] 타임아웃 (8분 동안 안 끝남)")

print("\n=== 단계 전환 기록 ===")
for s, name in phase_changes:
    print(f"  step {s}: -> {name}")

print(f"\n최종 위치: ({true_x:.3f}, {true_y:.3f}), 각도={math.degrees(true_theta):.1f}도")
print(f"목표(2.5,2.5,90도)와의 위치오차: {math.hypot(true_x-2.5, true_y-2.5):.3f}m")
wps = session.get_waypoints()
print(f"웨이포인트 개수: {len(wps)}")
