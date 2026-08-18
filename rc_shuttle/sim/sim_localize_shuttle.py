#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_localize_shuttle.py - 두 가지를 봄

  [1] 초기 전역 위치추정(slam_global_localize)이 어느 위치에서 실패하는가
      -> 지도 전체를 훑어서 "실패 지도"를 그림. 오른쪽 위 구석/중앙 통로가
         정말 취약한지, 취약하다면 왜인지(어디로 착각하는지) 확인.

  [2] A <-> B 왕복이 제대로 도는가
      -> 여러 바퀴 돌려서 매 구간 도착/주차 결과를 기록.

C 코드에서 옮긴 것
------------------
  - slam_global_localize: 1단계 전역 성긴탐색(5cm, 3deg) -> 2단계 미세탐색(1cm, 0.5deg)
  - 점수판: likelihood field(sigma 0.06), 일치도 = 점수/점개수*100
  - 나머지(스캔매칭/주행/주차)는 sim_encoder_ab.py에서 가져다 씀

라이다 결측 모델 (여기가 실기와 시뮬의 가장 큰 차이라서 따로 넣음)
------------------------------------------------------------------
  실기 라이다는 벽을 비스듬히 맞을수록(입사각이 클수록) 반사가 약해서 점을 못 만듦.
  구석에 서면 가까운 두 벽만 또렷하고 먼 벽은 스치듯 맞아서 대량 결측이 남.
  그러면 스캔이 "구석 모양"만 남아서 네 구석이 서로 닮아 보임 -> 오인식.
  이걸 모델링하지 않으면 시뮬에서는 늘 성공해서 실기 문제를 재현하지 못함.

실행:
  python3 sim_localize_shuttle.py --test localize
  python3 sim_localize_shuttle.py --test shuttle --laps 3
"""

import argparse
import math

import numpy as np

import sim_encoder_ab as S


# ----------------------------------------------------------------------------
# 입사각 기반 결측 모델
# ----------------------------------------------------------------------------
def make_scan_realistic(mm, bx, by, btheta, rng, glancing=True, noise_mm=30.0):
    """입사각이 클수록 점이 안 잡히는 실제 라이다 특성을 반영한 스캔."""
    lx = bx + S.LIDAR_OFFSET_M * math.cos(btheta)
    ly = by + S.LIDAR_OFFSET_M * math.sin(btheta)
    local = np.linspace(-math.pi, math.pi, S.LIDAR_BEAMS, endpoint=False)
    dist, hit = mm.raycast(lx, ly, btheta + local, rng)

    keep = hit.copy()
    if glancing:
        # 이웃 빔과의 거리 변화율로 입사각을 추정.
        # 수직 입사면 d(range)/d(angle)~0, 스치듯 맞으면 매우 큼.
        dtheta = 2 * math.pi / S.LIDAR_BEAMS
        grad = np.abs(np.gradient(np.where(hit, dist, np.nan)))
        grad = np.nan_to_num(grad, nan=1e3)
        # tan(입사각) ~ (dr/dphi)/r
        tan_inc = grad / dtheta / np.maximum(dist, 1e-3)
        cos_inc = 1.0 / np.sqrt(1.0 + tan_inc ** 2)
        # 반사 세기 ~ cos(입사각)/r^2 -> 확률로 환산(0.15는 실측 감으로 잡은 정규화값)
        p = np.clip(cos_inc / np.maximum(dist, 0.1) ** 2 * 0.15, 0.0, 1.0)
        keep = hit & (rng.random(S.LIDAR_BEAMS) < p)

    d = dist[keep] + rng.normal(0.0, noise_mm / 1000.0, int(keep.sum()))
    a = local[keep]
    d = np.maximum(d, 0.05)
    px = d * np.cos(a) + S.LIDAR_OFFSET_M
    py = d * np.sin(a)
    return np.hypot(px, py), np.arctan2(py, px)


# ----------------------------------------------------------------------------
# slam_global_localize 이식
# ----------------------------------------------------------------------------
def _score_batch(mm, cx, cy, ct, d, a, chunk=4000, free_penalty=0.0):
    out = np.empty(cx.shape, dtype=np.float64)
    for i in range(0, len(cx), chunk):
        s = slice(i, i + chunk)
        wa = ct[s][:, None] + a[None, :]
        ex = cx[s][:, None] + d[None, :] * np.cos(wa)
        ey = cy[s][:, None] + d[None, :] * np.sin(wa)
        sc = mm.sample_lf(ex, ey)
        if free_penalty > 0.0:
            # 지도가 '비어있다'고 아는 칸에 스캔 점이 떨어지면 감점.
            # 지도 밖으로 나간 점도 같이 감점(그 자세면 안 보였어야 할 점이므로).
            c = (ex / mm.res).astype(np.int32)
            r = (ey / mm.res).astype(np.int32)
            inside = (r >= 0) & (r < mm.rows) & (c >= 0) & (c < mm.cols)
            rc = np.clip(r, 0, mm.rows - 1)
            cc = np.clip(c, 0, mm.cols - 1)
            bad = np.where(inside, mm.free[rc, cc], True)
            sc = sc - free_penalty * bad
        out[s] = sc.sum(axis=1)
    return out


_FREE_CACHE = {}


def cell_free_mask(mm):
    """현재 C 코드의 slam__cell_is_free - '중심 셀 하나만' 비어있는지 확인"""
    key = id(mm), "cellonly"
    if key not in _FREE_CACHE:
        _FREE_CACHE[key] = ~mm.grid
    return _FREE_CACHE[key]


def free_mask(mm, clearance_frac=0.6):
    """로봇이 물리적으로 있을 수 있는 셀만 True (벽 안/벽에 붙은 곳 제외)"""
    key = id(mm), clearance_frac
    if key not in _FREE_CACHE:
        from scipy.ndimage import binary_dilation
        it = max(1, int(S.ROBOT_RADIUS / mm.res * clearance_frac))
        _FREE_CACHE[key] = ~binary_dilation(mm.grid, iterations=it)
    return _FREE_CACHE[key]


def global_localize(mm, d, a, hint=None, hint_radius=None, match_beams=90,
                    return_runnerup=False, free_only=False, coarse_step=0.05,
                    free_penalty=0.0):
    """C의 slam_global_localize와 같은 2단계 탐색.
    hint/hint_radius를 주면 그 반경 안만 탐색(C에는 없는 옵션 - 아래에서 효과 비교)."""
    if len(d) > match_beams:
        idx = np.linspace(0, len(d) - 1, match_beams).astype(np.int32)
        d, a = d[idx], a[idx]
    if len(d) < 8:
        return None

    xs = np.arange(0.0, mm.width + 1e-9, coarse_step)
    ys = np.arange(0.0, mm.height + 1e-9, coarse_step)
    ts = np.radians(np.arange(0.0, 358.0, 3.0))
    CX, CY, CT = np.meshgrid(xs, ys, ts, indexing="ij")
    CX, CY, CT = CX.ravel(), CY.ravel(), CT.ravel()

    if free_only:
        fm = free_mask(mm) if free_only == "radius" else cell_free_mask(mm)
        rr = np.clip((CY / mm.res).astype(np.int32), 0, mm.rows - 1)
        cc = np.clip((CX / mm.res).astype(np.int32), 0, mm.cols - 1)
        m = fm[rr, cc]
        CX, CY, CT = CX[m], CY[m], CT[m]
        if len(CX) == 0:
            return None

    if hint is not None and hint_radius is not None:
        m = np.hypot(CX - hint[0], CY - hint[1]) <= hint_radius
        CX, CY, CT = CX[m], CY[m], CT[m]
        if len(CX) == 0:
            return None

    sc = _score_batch(mm, CX, CY, CT, d, a, free_penalty=free_penalty)
    i = int(np.argmax(sc))
    bx, by, bt, bs = CX[i], CY[i], CT[i], sc[i]

    runner = None
    if return_runnerup:
        # 최고점에서 15cm/30deg 넘게 떨어진 별개 봉우리 중 최고
        far = (np.hypot(CX - bx, CY - by) > 0.15) | \
              (np.abs(np.arctan2(np.sin(CT - bt), np.cos(CT - bt))) > math.radians(30))
        if far.any():
            j = int(np.argmax(np.where(far, sc, -1e18)))
            runner = (CX[j], CY[j], CT[j], sc[j])

    fxs = np.arange(bx - 0.05, bx + 0.05 + 1e-9, 0.01)
    fys = np.arange(by - 0.05, by + 0.05 + 1e-9, 0.01)
    fts = bt + np.radians(np.arange(-4.0, 4.01, 0.5))
    FX, FY, FT = np.meshgrid(fxs, fys, fts, indexing="ij")
    FX, FY, FT = FX.ravel(), FY.ravel(), FT.ravel()
    fsc = _score_batch(mm, FX, FY, FT, d, a, free_penalty=free_penalty)
    k = int(np.argmax(fsc))
    if fsc[k] < bs:
        FX, FY, FT, fsc, k = np.array([bx]), np.array([by]), np.array([bt]), np.array([bs]), 0

    fit = fsc[k] / len(d) * 100.0
    return dict(x=FX[k], y=FY[k], theta=FT[k], fit=fit, best=bs,
                runner=runner, n_beams=len(d))


# ----------------------------------------------------------------------------
# [1] 전역 위치추정 실패 지도
# ----------------------------------------------------------------------------
def test_localize(mm, args):
    rng = np.random.default_rng(7)
    # 로봇이 실제로 있을 수 있는 자리만 (벽에서 로봇반경 이상 떨어진 곳)
    from scipy.ndimage import binary_dilation
    blocked = binary_dilation(mm.grid, iterations=int(S.ROBOT_RADIUS / mm.res * 0.6))

    xs = np.arange(0.15, mm.width - 0.10, 0.10)
    ys = np.arange(0.15, mm.height - 0.10, 0.10)
    thetas = [0, 90, 180, 270]

    for label, glancing in [("이상적 스캔(결측 없음)", False),
                            ("실제 라이다(입사각 결측 모델)", True)]:
        print("=" * 74)
        print(f"[1] 초기 전역 위치추정 - {label}")
        print("=" * 74)
        fails = []
        total = 0
        grid_fail = np.zeros((len(ys), len(xs)))
        grid_n = np.zeros((len(ys), len(xs)))
        for iy, y in enumerate(ys):
            for ix, x in enumerate(xs):
                if blocked[int(y / mm.res), int(x / mm.res)]:
                    grid_n[iy, ix] = -1
                    continue
                for thd in thetas:
                    th = math.radians(thd)
                    d, a = make_scan_realistic(mm, x, y, th, rng, glancing=glancing)
                    r = global_localize(mm, d, a, return_runnerup=True)
                    total += 1
                    grid_n[iy, ix] += 1
                    if r is None:
                        fails.append((x, y, thd, None, 0.0, 0))
                        grid_fail[iy, ix] += 1
                        continue
                    perr = math.hypot(r["x"] - x, r["y"] - y)
                    aerr = abs(math.degrees(S.wrap(r["theta"] - th)))
                    if perr > 0.10 or aerr > 10.0:
                        fails.append((x, y, thd, (r["x"], r["y"],
                                     math.degrees(r["theta"])), r["fit"], r["n_beams"]))
                        grid_fail[iy, ix] += 1
        print(f"  실패 {len(fails)} / {total}  ({100.0*len(fails)/max(total,1):.1f}%)")
        if fails:
            print(f"  {'참 위치':>22} {'-> 착각한 위치':>26} {'일치도':>8} {'빔수':>6}")
            for f in fails[:14]:
                est = f"({f[3][0]:.2f},{f[3][1]:.2f},{f[3][2]:6.1f})" if f[3] else "실패(빔부족)"
                print(f"  ({f[0]:.2f},{f[1]:.2f},{f[2]:3.0f}deg) -> {est:>26} "
                      f"{f[4]:7.0f}% {f[5]:6d}")
            if len(fails) > 14:
                print(f"  ... 외 {len(fails)-14}건")
        print()
        if glancing:
            plot_failmap(mm, xs, ys, grid_fail, grid_n, len(thetas))
            hint_experiment(mm, fails, rng)


def hint_experiment(mm, fails, rng):
    """--start 힌트로 탐색범위를 제한하면 실패가 사라지는가"""
    if not fails:
        return
    print("-" * 74)
    print("  [대책 검증] --start 힌트 반경 30cm로 탐색범위를 제한했을 때")
    print("-" * 74)
    fixed = 0
    tried = 0
    for (x, y, thd, _est, _fit, _n) in fails[:25]:
        th = math.radians(thd)
        d, a = make_scan_realistic(mm, x, y, th, rng, glancing=True)
        r = global_localize(mm, d, a, hint=(x, y), hint_radius=0.30)
        tried += 1
        if r is None:
            continue
        if math.hypot(r["x"] - x, r["y"] - y) <= 0.10 and \
           abs(math.degrees(S.wrap(r["theta"] - th))) <= 10.0:
            fixed += 1
    print(f"  기존 실패 {tried}건 중 {fixed}건 복구 "
          f"({100.0*fixed/max(tried,1):.0f}%)\n")


def plot_failmap(mm, xs, ys, grid_fail, grid_n, n_theta):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    rate = np.where(grid_n > 0, grid_fail / np.maximum(grid_n, 1) * 100.0, np.nan)
    fig, ax = plt.subplots(figsize=(9, 6))
    ax.imshow(mm.grid, cmap="gray_r", origin="lower",
              extent=[0, mm.width, 0, mm.height], alpha=0.30)
    im = ax.pcolormesh(xs, ys, rate, cmap="Reds", vmin=0, vmax=100, alpha=0.75,
                       shading="nearest")
    fig.colorbar(im, ax=ax, label="초기 위치추정 실패율 [%]")
    ax.set_aspect("equal"); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title("초기 전역 위치추정이 실패하는 지점 (방향 4가지 평균)")
    fig.tight_layout()
    fig.savefig("localize_failmap.png", dpi=130)
    print("  실패 지도 저장: localize_failmap.png\n")


# ----------------------------------------------------------------------------
# [2] A <-> B 왕복 테스트
# ----------------------------------------------------------------------------
def test_shuttle(mm, args):
    print("=" * 74)
    print(f"[2] A <-> B 왕복 {args.laps}바퀴 (구간 {args.laps*2}회)")
    print("=" * 74)

    for cfg_name, cfg in [("라이다만", S.make_cfg(False)),
                          ("라이다+엔코더", S.make_cfg(True))]:
        print(f"\n--- {cfg_name} ---")
        print(f"{'구간':<10}{'도착오차':>10}{'주차 좌우':>11}{'각도':>9}"
              f"{'깊이':>8}{'판정':>7}")
        # A 슬롯에 주차된 상태에서 시작
        cur = (S.PARK_A[0], S.PARK_A[1], S.PARK_A[2])
        ok_all = 0
        for leg in range(args.laps * 2):
            target = S.PARK_B if leg % 2 == 0 else S.PARK_A
            name = "A->B" if leg % 2 == 0 else "B->A"

            # 1) 슬롯 후진 이탈 -> 대기점
            back = (cur[0], S.SLOT_STAGING_Y, cur[2])
            # 2) 대기점 -> 목표 대기점 주행
            nav = S.run_navigation(mm, cfg, 900 + leg, back,
                                   (target[0], S.SLOT_STAGING_Y))
            if nav is None:
                print(f"{name:<10}{'경로없음':>10}")
                break
            tx, ty, tth = nav["true"]
            # 3) 정렬 + 슬롯 진입
            park = S.run_parking(mm, cfg, 900 + leg,
                                 (tx, ty, math.degrees(tth)), target)
            ok_all += 1 if park["ok"] else 0
            print(f"{name:<10}{nav['goal_err_mm']:9.1f}mm"
                  f"{park['lat_mm']:10.1f}mm{park['ang_deg']:8.2f}°"
                  f"{park['depth_mm']:7.1f}mm{'OK' if park['ok'] else '실패':>7}")
            cur = (target[0], target[1], target[2])
        print(f"  => {ok_all}/{args.laps*2} 구간 주차 성공")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default="set_map.txt")
    ap.add_argument("--test", choices=["localize", "shuttle", "all"], default="all")
    ap.add_argument("--laps", type=int, default=3)
    args = ap.parse_args()

    mm = S.MapModel(args.map)
    print(f"지도: {args.map}  {mm.width:.2f} x {mm.height:.2f} m\n")
    if args.test in ("localize", "all"):
        test_localize(mm, args)
    if args.test in ("shuttle", "all"):
        test_shuttle(mm, args)


if __name__ == "__main__":
    main()
