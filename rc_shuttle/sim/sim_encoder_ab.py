#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_encoder_ab.py - 라이다만 쓸 때 vs 라이다+엔코더 쓸 때의
                    내비게이션 / 슬롯 주차 성능 비교 시뮬레이터

무엇을 재는가
-------------
  테스트 A (내비게이션) : A슬롯 대기점 -> B슬롯 대기점 주행.
                          도착 위치 오차, 경로 이탈, 스텝 수.
  테스트 B (슬롯 주차)   : 대기점에서 정렬(slot_align) 후 슬롯 진입(slot_drive).
                          최종 주차 오차(가로/세로/각도), 성공률.

C 코드의 무엇을 그대로 옮겼는가
-------------------------------
  - slam.h : likelihood field(LF_SIGMA_M=0.06), 이중선형 보간, 12배 스케일,
             거친탐색(±0.15m/3cm, ±12deg/3deg) -> 미세탐색(5mm/0.5deg),
             오도메트리 벌점(POS 40, THETA 15), MIN_SCORE_SPREAD=1.0
  - robot_runner.h : stop-and-go 스텝(속도 고정, 이동'시간'으로 양 조절),
             회전 스텝 클램프, slot_align / slot_drive 제어식, 라이다 전방오프셋
  - fpga_link.h : 엔코더 20틱/rev -> 차동회전 7.42deg/틱 양자화

이 시뮬레이터가 알려주지 못하는 것 (반드시 같이 읽을 것)
--------------------------------------------------------
  1) 지도 정합도. "지도 == 실물"을 가정함. 폼보드가 우그러져서 생기는 편향(bias)은
     여기 절대 안 나타남. 실기 절대정확도를 보장하지 않음.
  2) 회전 중 라이다 스캔 뭉개짐(1회전 102ms), 모터 데드존/비선형, 통신 지연/유실.
  3) C 원본을 파이썬으로 다시 옮긴 것이라 바이너리 그 자체를 시험하는 게 아님.
     대신 '270도 회전 버그'를 재현하는지로 이식이 맞는지 검산함(--selftest).

실행:
  python3 sim_encoder_ab.py                 # 전체 비교 + 그래프 저장
  python3 sim_encoder_ab.py --trials 20     # 시행 횟수 변경
  python3 sim_encoder_ab.py --selftest      # 이식 검산만 빠르게
"""

import argparse
import heapq
import math
import sys

import numpy as np

# ----------------------------------------------------------------------------
# 실기 설정과 동일한 상수 (robot_runner.h / fpga_link.h)
# ----------------------------------------------------------------------------
CONTROLLER_MAX   = 80
MAX_WHEEL_MPS    = 0.165
WHEEL_SEP        = 0.160
WHEEL_DIAM       = 0.066
TICKS_PER_REV    = 20

SG_STEP_SPEED    = 34
SLOT_STEP_SPEED  = 22
SLOT_TOUCH_SPEED = 18
SG_MOVE_SEC_MIN  = 0.07
SG_MOVE_SEC_MAX  = 0.45

SEARCH_WIN_M     = 0.15
SEARCH_STEP_M    = 0.03
SEARCH_WIN_DEG   = 12.0
SEARCH_STEP_DEG  = 3.0
FINE_STEP_M      = 0.005
FINE_STEP_DEG    = 0.5
LF_SIGMA_M       = 0.06
ODOM_PENALTY_POS = 40.0
ODOM_PEN_THETA   = 15.0
MIN_SCORE_SPREAD = 1.0

LIDAR_BEAMS      = 240      # 실기 RPLIDAR C1 한 바퀴 유효 빔 수와 비슷한 수준
MATCH_BEAMS      = 72       # 점수 계산에 실제로 쓰는 빔 수(균등 다운샘플).
                            # 스캔매칭 비용은 빔 수에 비례하는데, 인접 빔은 거의 같은
                            # 벽면을 보므로 정보가 중복됨. 72개면 5도 간격이라 이 크기
                            # 지도에서 점수 지형이 거의 안 변함. 실기에서도 그대로
                            # 유효한 최적화(라즈베리파이 스텝당 매칭 비용 3배 절감).
FINE_HALF        = 4        # 미세탐색 반경(칸). C는 6이지만 9x9x9로 줄여 2.9배 가속
LIDAR_NOISE_MM   = 30.0
LIDAR_MAX_M      = 3.0
LIDAR_OFFSET_M   = 0.06     # 라이다가 회전중심보다 앞으로 나간 거리(실측값 넣을 것)

ROBOT_LEN        = 0.255
ROBOT_RADIUS     = 0.16
SLOT_STAGING_Y   = 0.65
SLOT_ALIGN_TOL   = 1.0      # deg
SLOT_STOP_TOL_M  = 0.015
K_LATERAL        = 60.0
K_HEADING        = 25.0
MAX_DIFF         = 10
SLOT_STALL_EPS_M = 0.004
SLOT_STALL_STEPS = 3
SLOT_SEATED_MARGIN_M = 0.05

# make_slot_map.py 기준 주차자세
PARK_A = (0.225, 0.1775, -90.0)
PARK_B = (1.275, 0.1775, -90.0)

TICK_QUANTUM_RAD = 2.0 * (math.pi * WHEEL_DIAM / TICKS_PER_REV) / WHEEL_SEP  # 7.42deg


def rot_rate(speed):
    """제자리 회전 각속도 [rad/s]"""
    return 2.0 * abs(speed) * MAX_WHEEL_MPS / CONTROLLER_MAX / WHEEL_SEP


def lin_rate(speed):
    """직진 속도 [m/s]"""
    return abs(speed) * MAX_WHEEL_MPS / CONTROLLER_MAX


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


# ----------------------------------------------------------------------------
# 지도 + likelihood field  (slam.h 이식)
# ----------------------------------------------------------------------------
class MapModel:
    def __init__(self, path, sigma=None):
        self.sigma = sigma
        with open(path) as f:
            rows, cols, res = f.readline().split()
            self.rows, self.cols, self.res = int(rows), int(cols), float(res)
            grid = np.zeros((self.rows, self.cols), dtype=np.uint8)
            for r in range(self.rows):
                line = f.readline().rstrip("\n")
                grid[r, :len(line)] = np.frombuffer(line.encode(), dtype=np.uint8) - ord('0')
        self.grid = grid.astype(bool)

        # likelihood field: 가장 가까운 점유셀까지의 거리 -> 가우시안
        from scipy.ndimage import distance_transform_edt
        dist_cells = distance_transform_edt(~self.grid)
        dm = dist_cells * self.res
        sigma = getattr(self, "sigma", None) or LF_SIGMA_M
        self.sigma = sigma
        self.lf = np.exp(-(dm * dm) / (2.0 * sigma * sigma))
        # 자유공간 마스크: 벽에서 충분히 떨어진 = 지도가 "여긴 비어있다"고 아는 곳.
        # 스캔 점이 여기 떨어지면 그 자세를 반증하는 증거임.
        self.free = (dm > 3.0 * sigma)

        self.width = self.cols * self.res
        self.height = self.rows * self.res

    def sample_lf(self, wx, wy):
        """이중선형 보간 (slam__lf_sample 이식). wx, wy는 임의 shape의 배열."""
        fc = wx / self.res - 0.5
        fr = wy / self.res - 0.5
        c0 = np.floor(fc).astype(np.int32)
        r0 = np.floor(fr).astype(np.int32)
        tx = fc - c0
        ty = fr - r0
        acc = np.zeros(wx.shape, dtype=np.float64)
        for dr in (0, 1):
            wr = ty if dr else (1.0 - ty)
            for dc in (0, 1):
                wc = tx if dc else (1.0 - tx)
                r = np.clip(r0 + dr, 0, self.rows - 1)
                c = np.clip(c0 + dc, 0, self.cols - 1)
                inside = ((r0 + dr) >= 0) & ((r0 + dr) < self.rows) & \
                         ((c0 + dc) >= 0) & ((c0 + dc) < self.cols)
                acc += np.where(inside, wr * wc * self.lf[r, c], 0.0)
        return acc

    def raycast(self, x, y, angles, rng):
        """격자 레이캐스팅으로 스캔 합성. angles는 월드 각도 배열."""
        step = self.res * 0.5
        n_steps = int(LIDAR_MAX_M / step)
        dx = np.cos(angles) * step
        dy = np.sin(angles) * step
        px = np.full(angles.shape, x, dtype=np.float64)
        py = np.full(angles.shape, y, dtype=np.float64)
        hit = np.zeros(angles.shape, dtype=bool)
        dist = np.full(angles.shape, LIDAR_MAX_M, dtype=np.float64)
        for k in range(1, n_steps + 1):
            px = np.where(hit, px, px + dx)
            py = np.where(hit, py, py + dy)
            c = (px / self.res).astype(np.int32)
            r = (py / self.res).astype(np.int32)
            out = (r < 0) | (r >= self.rows) | (c < 0) | (c >= self.cols)
            occ = np.zeros(angles.shape, dtype=bool)
            ok = ~out & ~hit
            occ[ok] = self.grid[r[ok], c[ok]]
            new_hit = (~hit) & (occ | out)
            dist = np.where(new_hit, k * step, dist)
            hit = hit | new_hit
            if hit.all():
                break
        return dist, hit


# ----------------------------------------------------------------------------
# 라이다 스캔 합성 (차체중심 좌표계로 변환까지 = apply_lidar_frame_fix 이식)
# ----------------------------------------------------------------------------
def make_scan(mm, bx, by, btheta, rng, beam_keep=1.0, noise_mult=1.0):
    """bx,by,btheta는 '차체중심' 자세. 라이다는 그보다 LIDAR_OFFSET_M 앞에 있음."""
    lx = bx + LIDAR_OFFSET_M * math.cos(btheta)
    ly = by + LIDAR_OFFSET_M * math.sin(btheta)

    local = np.linspace(-math.pi, math.pi, LIDAR_BEAMS, endpoint=False)
    dist, hit = mm.raycast(lx, ly, btheta + local, rng)

    keep = hit & (rng.random(LIDAR_BEAMS) < beam_keep)
    d = dist[keep] + rng.normal(0.0, LIDAR_NOISE_MM * noise_mult / 1000.0, keep.sum())
    a = local[keep]
    d = np.maximum(d, 0.05)

    # 라이다 -> 차체중심 평행이동 (극좌표 재계산)
    px = d * np.cos(a) + LIDAR_OFFSET_M
    py = d * np.sin(a)
    return np.hypot(px, py), np.arctan2(py, px)


# ----------------------------------------------------------------------------
# 스캔매칭 (scan_match_correct_pose 이식)
# ----------------------------------------------------------------------------
def _score(mm, cand_x, cand_y, cand_th, d, a):
    """cand_*: (N,) 후보들. d,a: (M,) 스캔. 반환 (N,) 점수."""
    wa = cand_th[:, None] + a[None, :]
    ex = cand_x[:, None] + d[None, :] * np.cos(wa)
    ey = cand_y[:, None] + d[None, :] * np.sin(wa)
    return 12.0 * mm.sample_lf(ex, ey).sum(axis=1)


def scan_match(mm, px, py, pth, d, a):
    if len(d) > MATCH_BEAMS:                     # 균등 다운샘플
        idx = np.linspace(0, len(d) - 1, MATCH_BEAMS).astype(np.int32)
        d, a = d[idx], a[idx]
    win_th = math.radians(SEARCH_WIN_DEG)
    step_th = math.radians(SEARCH_STEP_DEG)
    offs_xy = np.arange(-SEARCH_WIN_M, SEARCH_WIN_M + 1e-9, SEARCH_STEP_M)
    offs_th = np.arange(-win_th, win_th + 1e-9, step_th)

    DX, DY, DT = np.meshgrid(offs_xy, offs_xy, offs_th, indexing="ij")
    DX, DY, DT = DX.ravel(), DY.ravel(), DT.ravel()
    raw = _score(mm, px + DX, py + DY, pth + DT, d, a)
    pen = ODOM_PENALTY_POS * (DX ** 2 + DY ** 2) + ODOM_PEN_THETA * DT ** 2
    sc = raw - pen

    if (raw.max() - raw.min()) < MIN_SCORE_SPREAD:
        return px, py, pth          # 구분이 안 되면 예측값 유지

    i = int(np.argmax(sc))
    bx, by, bth = px + DX[i], py + DY[i], pth + DT[i]

    # 미세탐색 (5mm / 0.5deg)
    fs = np.arange(-FINE_HALF, FINE_HALF + 1) * FINE_STEP_M
    ft = np.arange(-FINE_HALF, FINE_HALF + 1) * math.radians(FINE_STEP_DEG)
    FX, FY, FT = np.meshgrid(fs, fs, ft, indexing="ij")
    cx = bx + FX.ravel()
    cy = by + FY.ravel()
    ct = bth + FT.ravel()
    raw2 = _score(mm, cx, cy, ct, d, a)
    pen2 = (ODOM_PENALTY_POS * ((cx - px) ** 2 + (cy - py) ** 2)
            + ODOM_PEN_THETA * (ct - pth) ** 2)
    j = int(np.argmax(raw2 - pen2))
    return cx[j], cy[j], ct[j]


# ----------------------------------------------------------------------------
# 로봇 시뮬레이션 (실제 자세 + 엔코더 모델)
# ----------------------------------------------------------------------------
class Robot:
    def __init__(self, x, y, th, cfg, rng):
        self.x, self.y, self.th = x, y, th
        self.cfg = cfg
        self.rng = rng
        self.res_rot = 0.0          # 엔코더 양자화 잔차
        self.res_lin = 0.0
        self.pending = (0.0, 0.0)   # 직전 스텝에 실제로 움직인 (전진, 회전)

    def drive(self, left, right, sec):
        """좌우 컨트롤러 명령을 sec초 동안. 슬립/노이즈 포함해 실제 자세를 갱신."""
        vl = lin_rate(left) * (1 if left >= 0 else -1)
        vr = lin_rate(right) * (1 if right >= 0 else -1)
        v = (vl + vr) / 2.0
        w = (vr - vl) / WHEEL_SEP

        slip_rot = self.cfg["slip_rot"]
        slip_lin = self.cfg["slip_lin"]
        # 슬립은 매번 조금씩 다름(계통 + 랜덤)
        sr = slip_rot * (1.0 + self.rng.normal(0, 0.3))
        sl = slip_lin * (1.0 + self.rng.normal(0, 0.3))

        d_lin = v * sec * (1.0 - sl)
        d_rot = w * sec * (1.0 - sr)

        self.th = wrap(self.th + d_rot)
        self.x += d_lin * math.cos(self.th)
        self.y += d_lin * math.sin(self.th)
        self.pending = (d_lin, d_rot)

    def read_odom(self):
        """엔코더가 본 (dx_local, dtheta). 틱 양자화 적용. 엔코더 없으면 (0,0)."""
        if not self.cfg["has_encoder"]:
            self.pending = (0.0, 0.0)
            return 0.0, 0.0
        d_lin, d_rot = self.pending
        self.pending = (0.0, 0.0)

        # 회전: 틱 단위로만 보임
        m = d_rot + self.res_rot
        n = math.trunc(m / TICK_QUANTUM_RAD)
        q_rot = n * TICK_QUANTUM_RAD
        self.res_rot = m - q_rot

        # 직진: 틱당 이동거리 단위
        mpt = math.pi * WHEEL_DIAM / TICKS_PER_REV
        m2 = d_lin + self.res_lin
        n2 = math.trunc(m2 / mpt)
        q_lin = n2 * mpt
        self.res_lin = m2 - q_lin
        return q_lin, q_rot


def clamp_rot_sec(cfg, speed, sec):
    """robot_runner__clamp_rotation_sec 이식"""
    if not cfg["clamp"]:
        return sec
    frac = cfg["clamp_frac"]
    max_sec = math.radians(SEARCH_WIN_DEG) * frac / rot_rate(speed)
    max_sec = max(max_sec, SG_MOVE_SEC_MIN)
    return min(sec, max_sec)


def localize(mm, est, rob, cfg):
    """한 스텝 위치추정. est=(x,y,th) 직전 추정. 반환 새 추정."""
    d, a = make_scan(mm, rob.x, rob.y, rob.th, rob.rng,
                     cfg["beam_keep"], cfg["noise_mult"])
    if len(d) < 10:
        return est
    odom_lin, odom_rot = rob.read_odom()
    pth = wrap(est[2] + odom_rot)
    px = est[0] + odom_lin * math.cos(pth)
    py = est[1] + odom_lin * math.sin(pth)
    return scan_match(mm, px, py, pth, d, a)


# ----------------------------------------------------------------------------
# 테스트 A: 내비게이션 (A* 경로 + pure pursuit, stop-and-go)
# ----------------------------------------------------------------------------
def astar(mm, start, goal, inflate_cells):
    from scipy.ndimage import binary_dilation
    occ = binary_dilation(mm.grid, iterations=inflate_cells)
    sr, sc = int(start[1] / mm.res), int(start[0] / mm.res)
    gr, gc = int(goal[1] / mm.res), int(goal[0] / mm.res)
    if occ[gr, gc]:
        occ[max(0, gr - 3):gr + 4, max(0, gc - 3):gc + 4] = False
    openq = [(0.0, (sr, sc))]
    came, gsc = {}, {(sr, sc): 0.0}
    nbrs = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    while openq:
        _, cur = heapq.heappop(openq)
        if cur == (gr, gc):
            break
        for dr, dc in nbrs:
            nr, nc = cur[0] + dr, cur[1] + dc
            if not (0 <= nr < mm.rows and 0 <= nc < mm.cols) or occ[nr, nc]:
                continue
            ng = gsc[cur] + math.hypot(dr, dc)
            if ng < gsc.get((nr, nc), 1e18):
                gsc[(nr, nc)] = ng
                came[(nr, nc)] = cur
                h = math.hypot(nr - gr, nc - gc)
                heapq.heappush(openq, (ng + h, (nr, nc)))
    if (gr, gc) not in came and (gr, gc) != (sr, sc):
        return None
    path, cur = [], (gr, gc)
    while cur in came:
        path.append(((cur[1] + 0.5) * mm.res, (cur[0] + 0.5) * mm.res))
        cur = came[cur]
    path.reverse()
    return path[::10] + [goal]      # 10셀마다 웨이포인트


def run_navigation(mm, cfg, seed, start, goal):
    rng = np.random.default_rng(seed)
    rob = Robot(start[0], start[1], math.radians(start[2]), cfg, rng)
    est = (start[0], start[1], math.radians(start[2]))

    wps = astar(mm, (start[0], start[1]), goal, int(ROBOT_RADIUS / mm.res * 0.5))
    if wps is None:
        return None
    traj = [(rob.x, rob.y)]
    wi = 0
    for step in range(400):
        est = localize(mm, est, rob, cfg)
        # 다음 웨이포인트 선택
        while wi < len(wps) - 1 and math.hypot(wps[wi][0] - est[0], wps[wi][1] - est[1]) < 0.10:
            wi += 1
        tx, ty = wps[wi]
        if wi == len(wps) - 1 and math.hypot(tx - est[0], ty - est[1]) < 0.05:
            break
        alpha = wrap(math.atan2(ty - est[1], tx - est[0]) - est[2])

        if abs(alpha) > math.radians(20):       # 제자리 회전
            speed = SG_STEP_SPEED
            sec = min(abs(alpha) / rot_rate(speed), SG_MOVE_SEC_MAX)
            sec = max(sec, SG_MOVE_SEC_MIN)
            sec = clamp_rot_sec(cfg, speed, sec)
            s = 1 if alpha > 0 else -1
            rob.drive(-s * speed, s * speed, sec)
        else:                                    # 전진 + 완만한 조향
            speed = SG_STEP_SPEED
            diff = max(-MAX_DIFF, min(MAX_DIFF, alpha * 30.0))
            sec = min(0.20, SG_MOVE_SEC_MAX)
            rob.drive(int(speed - diff), int(speed + diff), sec)
        traj.append((rob.x, rob.y))

    return {
        "true": (rob.x, rob.y, rob.th),
        "est": est,
        "goal_err_mm": math.hypot(rob.x - goal[0], rob.y - goal[1]) * 1000.0,
        "est_err_mm": math.hypot(rob.x - est[0], rob.y - est[1]) * 1000.0,
        "steps": step + 1,
        "traj": traj,
    }


# ----------------------------------------------------------------------------
# 테스트 B: 슬롯 정렬 + 진입 (slot_align / slot_drive 이식)
# ----------------------------------------------------------------------------
def run_parking(mm, cfg, seed, start, park):
    rng = np.random.default_rng(seed)
    rob = Robot(start[0], start[1], math.radians(start[2]), cfg, rng)
    est = (start[0], start[1], math.radians(start[2]))
    goal_th = math.radians(park[2])
    traj = [(rob.x, rob.y)]

    # --- 1단계: 정렬 ---
    aligned = False
    for _ in range(40):
        est = localize(mm, est, rob, cfg)
        err = wrap(est[2] - goal_th)
        if abs(math.degrees(err)) <= SLOT_ALIGN_TOL:
            aligned = True
            break
        speed = SG_STEP_SPEED
        sec = min(max(abs(err) / rot_rate(speed), SG_MOVE_SEC_MIN), SG_MOVE_SEC_MAX)
        sec = clamp_rot_sec(cfg, speed, sec)
        s = -1 if err > 0 else 1
        rob.drive(-s * speed, s * speed, sec)
        traj.append((rob.x, rob.y))

    # --- 2단계: 슬롯 진입 ---
    lane_x, stop_y = park[0], park[1]
    last_y, stall = 1e9, 0
    touched = False
    for _step in range(120):
        est = localize(mm, est, rob, cfg)
        x, y, th = est
        if y <= stop_y + SLOT_STOP_TOL_M:
            break
        if abs(y - last_y) < SLOT_STALL_EPS_M:
            stall += 1
        else:
            stall = 0
        last_y = y
        near_depth = y <= stop_y + SLOT_SEATED_MARGIN_M
        if stall >= SLOT_STALL_STEPS and _step > 3 and near_depth:
            touched = True
            break

        lateral = x - lane_x
        heading = wrap(th - goal_th)
        # C: diff = -(K_LATERAL*lateral_err + K_HEADING*heading_err)  (부호 주의)
        diff = -(K_LATERAL * lateral + K_HEADING * heading)
        diff = max(-MAX_DIFF, min(MAX_DIFF, diff))

        remain = y - stop_y
        speed = SLOT_STEP_SPEED
        if remain < 0.06:
            t = max(0.0, remain / 0.06)
            speed = int(round(SLOT_TOUCH_SPEED + (SLOT_STEP_SPEED - SLOT_TOUCH_SPEED) * t))
        rob.drive(int(round(speed - diff)), int(round(speed + diff)),
                  SG_MOVE_SEC_MIN * 1.5)
        traj.append((rob.x, rob.y))

    lat_mm = abs(rob.x - park[0]) * 1000.0
    depth_mm = abs(rob.y - park[1]) * 1000.0
    ang_deg = abs(math.degrees(wrap(rob.th - goal_th)))
    # 슬롯 폭 170mm, 차폭 130mm -> 좌우 여유 ±20mm. 각도는 평행구간에서 ±5deg가 한계.
    ok = (lat_mm <= 20.0) and (ang_deg <= 5.0) and (depth_mm <= 60.0)
    return {
        "lat_mm": lat_mm, "depth_mm": depth_mm, "ang_deg": ang_deg,
        "aligned": aligned, "touched": touched, "ok": ok,
        "est_err_mm": math.hypot(rob.x - est[0], rob.y - est[1]) * 1000.0,
        "traj": traj,
    }


# ----------------------------------------------------------------------------
def make_cfg(has_encoder, clamp=True, clamp_frac=None, beam_keep=1.0,
             noise_mult=1.0, slip_rot=0.15, slip_lin=0.05):
    if clamp_frac is None:
        clamp_frac = 1.0 if has_encoder else 0.6
    return dict(has_encoder=has_encoder, clamp=clamp, clamp_frac=clamp_frac,
                beam_keep=beam_keep, noise_mult=noise_mult,
                slip_rot=slip_rot, slip_lin=slip_lin)


def pct(vals):
    v = np.array(vals, dtype=float)
    return f"{v.mean():7.1f} {np.median(v):7.1f} {v.max():7.1f}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default="set_map.txt")
    ap.add_argument("--trials", type=int, default=12)
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()

    mm = MapModel(args.map)
    print(f"지도: {args.map}  {mm.cols}x{mm.rows}셀 "
          f"({mm.width:.2f} x {mm.height:.2f} m, {mm.res*1000:.0f}mm/셀)")
    print(f"회전 각속도 {math.degrees(rot_rate(SG_STEP_SPEED)):.1f} deg/s, "
          f"탐색창 ±{SEARCH_WIN_DEG:.0f} deg, 엔코더 분해능 "
          f"{math.degrees(TICK_QUANTUM_RAD):.2f} deg/틱\n")

    # ---- 이식 검산: 원래 코드(클램프 없음, 엔코더 없음)가 270도 버그를 재현하는가 ----
    print("=" * 74)
    print("[검산] C 시뮬레이션에서 나온 '90도 대신 270도' 버그가 여기서도 재현되는가")
    print("=" * 74)
    for name, cfg in [("원래 코드(엔코더X, 클램프X)", make_cfg(False, clamp=False)),
                      ("패치 후 (엔코더X, 클램프O)", make_cfg(False, clamp=True))]:
        errs = []
        for t in range(6):
            r = run_parking(mm, cfg, 100 + t, (PARK_A[0], SLOT_STAGING_Y, 0.0), PARK_A)
            errs.append(r["ang_deg"])
        print(f"  {name}: 정렬 후 각도오차 평균 {np.mean(errs):6.1f} deg "
              f"(최악 {max(errs):.1f})")
    print()
    if args.selftest:
        return

    configs = [("라이다만",        make_cfg(False)),
               ("라이다+엔코더",   make_cfg(True))]

    # ---------------- 테스트 A: 내비게이션 ----------------
    print("=" * 74)
    print("테스트 A: 내비게이션  A슬롯 대기점 -> B슬롯 대기점")
    print("=" * 74)
    print(f"{'설정':<16}{'도착오차(mm) 평균/중앙/최악':>34}{'추정오차':>10}{'스텝':>7}")
    nav_res = {}
    for name, cfg in configs:
        errs, ests, steps, trajs = [], [], [], []
        for t in range(args.trials):
            r = run_navigation(mm, cfg, 200 + t,
                               (PARK_A[0], SLOT_STAGING_Y, 0.0),
                               (PARK_B[0], SLOT_STAGING_Y))
            if r is None:
                continue
            errs.append(r["goal_err_mm"]); ests.append(r["est_err_mm"])
            steps.append(r["steps"]); trajs.append(r["traj"])
        nav_res[name] = dict(errs=errs, trajs=trajs)
        print(f"{name:<16}{pct(errs):>34}{np.mean(ests):9.1f}{np.mean(steps):7.0f}")
    print()

    # ---------------- 테스트 B: 슬롯 주차 ----------------
    print("=" * 74)
    print("테스트 B: 슬롯 주차 (대기점에서 정렬 -> 진입)")
    print("=" * 74)
    print(f"{'설정':<16}{'좌우오차(mm)':>16}{'각도(deg)':>12}"
          f"{'깊이(mm)':>11}{'성공률':>9}")
    park_res = {}
    for name, cfg in configs:
        lat, ang, dep, ok, trajs = [], [], [], 0, []
        for t in range(args.trials):
            # 대기점 도착 자세에 현실적인 산포를 줌(내비 도착 오차 수준)
            rng = np.random.default_rng(300 + t)
            sx = PARK_B[0] + rng.normal(0, 0.02)
            sy = SLOT_STAGING_Y + rng.normal(0, 0.02)
            sth = rng.uniform(-180, 180)
            r = run_parking(mm, cfg, 300 + t, (sx, sy, sth), PARK_B)
            lat.append(r["lat_mm"]); ang.append(r["ang_deg"]); dep.append(r["depth_mm"])
            ok += 1 if r["ok"] else 0
            trajs.append(r["traj"])
        park_res[name] = dict(lat=lat, ang=ang, trajs=trajs)
        print(f"{name:<16}{np.mean(lat):10.1f}±{np.std(lat):4.1f}"
              f"{np.mean(ang):9.2f}±{np.std(ang):4.2f}"
              f"{np.mean(dep):8.1f}   {ok}/{args.trials:<5}")
    print()

    # ---------------- 테스트 C: 스캔 품질이 나쁠 때 ----------------
    print("=" * 74)
    print("테스트 C: 슬롯 주차 - 라이다 스캔 열화 (빔 40%, 노이즈 2배)")
    print("          벽을 잘라 특징이 줄었거나 반사가 나쁜 상황을 흉내낸 것")
    print("=" * 74)
    print(f"{'설정':<16}{'좌우오차(mm)':>16}{'각도(deg)':>12}"
          f"{'깊이(mm)':>11}{'성공률':>9}")
    for name, has_enc in [("라이다만", False), ("라이다+엔코더", True)]:
        cfg = make_cfg(has_enc, beam_keep=0.40, noise_mult=2.0)
        lat, ang, dep, ok = [], [], [], 0
        for t in range(args.trials):
            rng = np.random.default_rng(400 + t)
            sx = PARK_B[0] + rng.normal(0, 0.02)
            sy = SLOT_STAGING_Y + rng.normal(0, 0.02)
            sth = rng.uniform(-180, 180)
            r = run_parking(mm, cfg, 400 + t, (sx, sy, sth), PARK_B)
            lat.append(r["lat_mm"]); ang.append(r["ang_deg"]); dep.append(r["depth_mm"])
            ok += 1 if r["ok"] else 0
        print(f"{name:<16}{np.mean(lat):10.1f}±{np.std(lat):4.1f}"
              f"{np.mean(ang):9.2f}±{np.std(ang):4.2f}"
              f"{np.mean(dep):8.1f}   {ok}/{args.trials:<5}")
    print()

    if not args.no_plot:
        plot(mm, nav_res, park_res)


def plot(mm, nav_res, park_res):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    extent = [0, mm.width, 0, mm.height]

    for ax, (name, d) in zip(axes[0], nav_res.items()):
        ax.imshow(mm.grid, cmap="gray_r", origin="lower", extent=extent, alpha=0.35)
        for tr in d["trajs"]:
            t = np.array(tr)
            ax.plot(t[:, 0], t[:, 1], lw=0.9, alpha=0.7)
        ax.set_title(f"내비게이션 궤적 - {name}")
        ax.set_aspect("equal"); ax.set_xlim(0, mm.width); ax.set_ylim(0, mm.height)

    for ax, (name, d) in zip(axes[1], park_res.items()):
        ax.imshow(mm.grid, cmap="gray_r", origin="lower", extent=extent, alpha=0.35)
        for tr in d["trajs"]:
            t = np.array(tr)
            ax.plot(t[:, 0], t[:, 1], lw=0.9, alpha=0.7)
        ax.plot(PARK_B[0], PARK_B[1], "r*", ms=14)
        ax.set_title(f"슬롯 주차 궤적 - {name}")
        ax.set_aspect("equal"); ax.set_xlim(0.9, 1.5); ax.set_ylim(0, 0.8)

    for ax in axes.ravel():
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    fig.suptitle("라이다만 vs 라이다+엔코더", fontsize=13)
    fig.tight_layout()
    fig.savefig("sim_encoder_ab.png", dpi=130)
    print("그래프 저장: sim_encoder_ab.png")


if __name__ == "__main__":
    sys.exit(main())
