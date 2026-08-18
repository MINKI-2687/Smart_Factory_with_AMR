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

POINT_A = (0.5, 0.5, 0.0)
POINT_B = (2.5, 2.5, 180.0)

session = CMappingSession(
    size_x=ROOM, size_y=ROOM, resolution=0.05,
    start=(0.3, 0.3, 0.0), goal=(POINT_A[0], POINT_A[1]), robot_radius=0.10,
    goal_theta_deg=POINT_A[2], max_mapping_steps=20*60*5,
)

true_x, true_y, true_theta = 0.3, 0.3, 0.0
K_V, K_W, DT = 0.01, 0.01, 0.05

N_LEGS_TO_TEST = 4  # A->B->A->B (2왕복)
leg_labels = ["A", "B"]
current_leg = 0  # 첫 목표가 이미 A로 설정됨
leg_count = 0
step = 0
max_total_steps = 20 * 60 * 20  # 넉넉하게

while leg_count < N_LEGS_TO_TEST and step < max_total_steps:
    scan = simulate_lidar(true_x, true_y, true_theta)
    left, right, done = session.step(scan, 0, 0, 0)

    if session.phase == SessionPhase.PLAN_FAILED:
        print(f"[step {step}] PLAN_FAILED (leg={leg_labels[current_leg]}) - 중단")
        break

    linear = (left + right) / 2.0
    angular = (right - left) / 2.0
    v, w = linear * K_V, angular * K_W
    true_x += v * math.cos(true_theta) * DT
    true_y += v * math.sin(true_theta) * DT
    true_theta = math.atan2(math.sin(true_theta + w * DT), math.cos(true_theta + w * DT))

    if done:
        leg_count += 1
        pos_err = math.hypot(true_x - (POINT_A[0] if current_leg == 0 else POINT_B[0]),
                              true_y - (POINT_A[1] if current_leg == 0 else POINT_B[1]))
        print(f"[step {step}] {leg_labels[current_leg]}지점 도착! (누적 {leg_count}구간) "
              f"pos=({true_x:.3f},{true_y:.3f}) err={pos_err:.3f}m")

        # ---- 트리거 시뮬레이션(실제로는 감압센서, 여기선 즉시 발생시킴) ----
        next_leg = 1 - current_leg
        next_point = POINT_B if next_leg == 1 else POINT_A
        ok = session.retarget(goal=(next_point[0], next_point[1]), goal_theta_deg=next_point[2])
        print(f"  -> retarget to {leg_labels[next_leg]} {next_point[:2]} : {'성공' if ok else '실패'}")
        if not ok:
            break
        current_leg = next_leg

    step += 1

print(f"\n=== 총 {leg_count}구간 완료 (목표 {N_LEGS_TO_TEST}구간) ===")
print(f"최종 phase: {session.phase.name}")
