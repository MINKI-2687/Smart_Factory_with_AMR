#!/usr/bin/env python3
"""make_slot_map.py - 주차 슬롯이 있는 세트장 지도를 1cm 해상도로 생성.

왜 1cm인가: 슬롯 폭 17cm는 5cm 격자로 표현이 안 됨(15 아니면 20). 또 스캔매칭
정밀도의 바닥이 대략 '지도 해상도의 절반'이라, 5cm 지도로는 위치오차 2~3cm가 한계였음.
1cm로 올리면 그 바닥이 ~0.5cm로 내려감.

치수 근거:
  - 방 바깥 1.50 x 0.90 m, 벽 두께 5cm -> 안쪽 1.40 x 0.80 m
    (현재 5cm 지도의 안쪽 치수와 동일. 그 지도가 실기에서 스캔 일치도 97%가 나왔으므로
     실제 세트장과 맞는 값이라고 보고 유지함.)
  - 차체 15.1 x 25.5 cm
  - 슬롯: 평행구간 폭 17cm x 깊이 26cm (차체 길이 25.5cm보다 깊게 -> 코와 꼬리가 모두
    평행구간 안에 들어와야 각도가 잡힘. 전체를 V자로 만들면 꼬리가 ±11.5cm 흔들려
    차체가 최대 24도까지 비뚤어짐)
  - 입구: 폭 35cm, 깊이 14cm의 V자 (경사 33도 - 50cm 입구는 경사 50도라 걸릴 위험)
"""

RES = 0.01
OUT_W, OUT_H = 1.50, 0.90      # 벽 포함 바깥 치수
WALL = 0.05                    # 벽 두께

ROBOT_W, ROBOT_L = 0.151, 0.255

import sys
SLOT_W = 0.17                  # 평행구간 폭 (명령행 인자로 덮어쓸 수 있음)
for _i, _a in enumerate(sys.argv):
    if _a == "--slot-w" and _i + 1 < len(sys.argv):
        SLOT_W = float(sys.argv[_i + 1])
SLOT_PARALLEL_D = 0.26         # 평행구간 깊이
MOUTH_W = 0.35                 # V자 입구 폭
FUNNEL_D = 0.14                # V자 깊이
BLOCK_H = SLOT_PARALLEL_D + FUNNEL_D   # 가운데 블록 높이 = 슬롯 전체 깊이 = 0.40

cols = int(round(OUT_W / RES))
rows = int(round(OUT_H / RES))
grid = [[0] * cols for _ in range(rows)]


def fill(x0, x1, y0, y1):
    r0, r1 = int(round(y0 / RES)), int(round(y1 / RES))
    c0, c1 = int(round(x0 / RES)), int(round(x1 / RES))
    for r in range(max(0, r0), min(rows, r1)):
        for c in range(max(0, c0), min(cols, c1)):
            grid[r][c] = 1


# ---- 바깥 벽 ----
fill(0, OUT_W, 0, WALL)
fill(0, OUT_W, OUT_H - WALL, OUT_H)
fill(0, WALL, 0, OUT_H)
fill(OUT_W - WALL, OUT_W, 0, OUT_H)

# ---- 안쪽 좌표계 ----
IN_X0, IN_X1 = WALL, OUT_W - WALL          # 0.05 ~ 1.45
IN_Y0 = WALL                               # 0.05
BAY_W = MOUTH_W                            # 입구 폭 = 베이 폭 35cm
block_x0 = IN_X0 + BAY_W                   # 0.40
block_x1 = IN_X1 - BAY_W                   # 1.10
block_y1 = IN_Y0 + BLOCK_H                 # 0.45

# ---- 가운데 블록 ----
fill(block_x0, block_x1, IN_Y0, block_y1)

# ---- 좌/우 슬롯의 채움벽 ----
# 베이 중앙에 슬롯을 두고, 양옆을 채움. 위 14cm는 V자로 벌림.
slot_centers = [IN_X0 + BAY_W / 2.0, IN_X1 - BAY_W / 2.0]   # 0.225, 1.275

for cx in slot_centers:
    bay_x0, bay_x1 = cx - BAY_W / 2.0, cx + BAY_W / 2.0
    par_y1 = IN_Y0 + SLOT_PARALLEL_D                        # 0.31
    # 평행구간: 슬롯 바깥쪽을 통째로 채움
    fill(bay_x0, cx - SLOT_W / 2.0, IN_Y0, par_y1)
    fill(cx + SLOT_W / 2.0, bay_x1, IN_Y0, par_y1)
    # V자 구간: 위로 갈수록 채움벽이 얇아짐(=통로가 넓어짐)
    n = int(round(FUNNEL_D / RES))
    for i in range(n):
        y0 = par_y1 + i * RES
        t = (i + 0.5) / n                                    # 0..1
        half = (SLOT_W + (MOUTH_W - SLOT_W) * t) / 2.0
        fill(bay_x0, cx - half, y0, y0 + RES)
        fill(cx + half, bay_x1, y0, y0 + RES)

# ---- 주차 자세 계산 ----
# 코가 안쪽 바닥벽(y=IN_Y0)에 닿을 때 차체 중심의 y
park_y = IN_Y0 + ROBOT_L / 2.0
parks = [(cx, park_y) for cx in slot_centers]

with open("set_map.txt", "w") as f:
    f.write(f"{rows} {cols} {RES:.6f}\n")
    for r in range(rows):
        f.write("".join("1" if v else "0" for v in grid[r]) + "\n")

print(f"set_map.txt 생성: {rows}행 x {cols}열, 해상도 {RES}m ({OUT_W} x {OUT_H} m)")
print(f"  안쪽 공간      : x {IN_X0:.2f}~{IN_X1:.2f}, y {IN_Y0:.2f}~{OUT_H-WALL:.2f}")
print(f"  가운데 블록    : x {block_x0:.2f}~{block_x1:.2f} (폭 {block_x1-block_x0:.2f}), 높이 {BLOCK_H:.2f}")
print(f"  슬롯 평행구간  : 폭 {SLOT_W*100:.0f}cm, 깊이 {SLOT_PARALLEL_D*100:.0f}cm")
print(f"  슬롯 V자 입구  : 폭 {MOUTH_W*100:.0f}cm, 깊이 {FUNNEL_D*100:.0f}cm")
print(f"  위쪽 통로 높이 : {OUT_H - WALL - block_y1:.2f} m")
print()
print(f"  A 주차자세 : ({parks[0][0]:.3f}, {parks[0][1]:.3f}), -90도 (아래를 보고 진입)")
print(f"  B 주차자세 : ({parks[1][0]:.3f}, {parks[1][1]:.3f}), -90도")

# ---- 그림으로 확인 ----
print()
print("지도 (가로 3칸/세로 3칸을 한 글자로 축약):")
for r in range(rows - 1, -1, -3):
    print("  " + "".join("#" if any(grid[r][c:c + 3]) else "." for c in range(0, cols, 3)))
