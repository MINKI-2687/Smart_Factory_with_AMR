#!/usr/bin/env python3
"""live_view.py - main_shuttle의 출력을 실시간으로 그려주는 뷰어.

C 코드는 이미 stdout으로 MAP / TARGET / WAYPOINTS / STATE / SCAN 줄을 뱉고 있으므로,
C 쪽은 한 줄도 안 고치고 그 출력만 받아서 그림.

사용법 (셋 중 아무거나):
  (A) 실행할 명령을 통째로 넘기기 - 제일 편함:
      python3 live_view.py --run "./main_shuttle --map-source load ..."
  (B) 파이프로 연결:
      ./lidar_dump /dev/ttyUSB1 | python3 live_view.py --polar
  (C) 저장해둔 로그 다시 재생:
      python3 live_view.py < 저장한로그.txt

키:  m = 스캔 미러링 토글,  f = 지도 일치율 계산 on/off,  p = 일시정지

================================================================================
2026-08-11a 전면 개정 - "가끔 GUI가 멈추는 등 답답하다"
================================================================================
원인이 넷이었고 넷 다 구조적으로 고쳤다. 예전 개정(2026-08-09b)은 '파싱 비용'을
잡았는데, 실제로 재보니 SCAN 한 줄 파싱은 0.14ms 라 그건 원인이 아니었다.
진짜 원인은 전부 "GUI 스레드가 자기 일이 아닌 것 때문에 막히는" 구조였다.

(1) [핵심] GUI 스레드에서 터미널로 print 를 하고 있었다.
    drain() -> state.feed() -> print(line, file=sys.stderr) 경로다.
    터미널이 잠깐이라도 쓰기를 안 받으면(윈도우 콘솔에서 텍스트를 드래그 선택하면
    출력이 실제로 멈춘다. PuTTY/X11 포워딩에서 흐름제어가 걸려도 같다)
    그 순간 GUI 스레드가 write() 에서 통째로 잠긴다. 화면이 굳고 창도 안 움직인다.
    -> 이제 화면 갱신 스레드는 절대 I/O 를 하지 않는다. 터미널 출력은 전용
       스레드가 맡고, 그 큐가 꽉 차면 오래된 줄부터 버린다.

(2) 읽기 스레드가 막히면 '로봇이 멈춘다'는 더 큰 문제가 있었다.
    C 프로그램 stdout -> 파이프 -> 여기. 파이프 버퍼는 보통 64KB 뿐이라,
    이쪽이 안 읽으면 C 의 printf 가 블록되고 로봇이 그 자리에 서 버린다.
    즉 뷰어가 느리면 로봇까지 느려진다. 그래서 읽기 스레드는 오직 읽기만 하고,
    파싱은 파싱 스레드가, 터미널 출력은 출력 스레드가 맡는다.
    어느 큐든 넘치면 버린다 - 파이프를 막는 것보다 로그를 버리는 게 언제나 낫다.

(3) 큐가 무제한이었다(queue.Queue()). SCAN 한 줄이 약 10KB 라 밀리기 시작하면
    수십 MB 까지 자라고, 그 메모리 압박 자체가 GC 지연과 스와핑을 만든다.
    -> deque(maxlen=...) 로 상한.

(4) 한 프레임 그리는 데 interval(100ms)보다 오래 걸리면 Tk/Qt 이벤트 루프가
    굶어서 창이 아예 반응하지 않는다(이동/크기조절/닫기 불가 = "멈춤").
    X11 포워딩이나 라즈베리파이 화면에서는 한 프레임에 200~400ms 도 나온다.
    -> 실제 프레임 간격을 재서 갱신 주기를 자동으로 늘렸다 줄인다(적응형).
       그리고 새 데이터가 없으면 아티스트를 건드리지 않는다(파싱/좌표변환/일치율
       계산이 통째로 빠진다). 다만 blit=False 이므로 matplotlib 자체의 캔버스
       래스터화까지 없어지지는 않는다 - '프레임이 공짜가 된다'가 아니라
       '프레임 비용의 파이썬 쪽이 없어진다'가 정확한 표현이다.
       화면 좌상단에 실측 FPS 와 밀린 줄 수를 띄워 눈으로 확인할 수 있게 했다.

여전히 느리면 이 순서로: --max-points 200  ->  --interval 200  ->  --no-fit
"""
import sys
import time
import argparse
import math
import threading
from collections import deque

import numpy as np
import matplotlib

# 백엔드는 창을 만들기 전에 정해야 한다. --backend 로 강제 지정 가능.
for _i, _a in enumerate(sys.argv):
    if _a == "--backend" and _i + 1 < len(sys.argv):
        matplotlib.use(sys.argv[_i + 1])
        break

import matplotlib.pyplot as plt

# ============================================================
# 한글 폰트 처리
#
# (1) findfont(..., fallback_to_default=False) 는 matplotlib 버전에 따라 예외를
#     안 던지고 그냥 대체 폰트 경로를 돌려준다 -> 설치된 폰트 목록에서 직접 찾는다.
# (2) 좌상단 정보상자가 family='monospace' 라 font.family 설정을 덮어쓴다
#     -> font.monospace 목록 맨 앞에도 한글 폰트를 넣는다.
# 한글 폰트가 하나도 없으면 그림 안 글자를 전부 영어로 바꾼다(T() 헬퍼).
# ============================================================
import warnings
warnings.filterwarnings("ignore", message=r"Glyph \d+ .* missing from font")
warnings.filterwarnings("ignore", message=r".*missing from font\(s\).*")

KO_FONT = None
try:
    from matplotlib import font_manager as _fm
    _installed = {f.name for f in _fm.fontManager.ttflist}
    for _f in ("NanumGothic", "NanumBarunGothic", "NanumGothicCoding", "D2Coding",
               "Noto Sans CJK KR", "Noto Sans KR", "Source Han Sans KR",
               "Malgun Gothic", "AppleGothic", "Apple SD Gothic Neo",
               "UnDotum", "UnBatang", "Baekmuk Gulim"):
        if _f in _installed:
            KO_FONT = _f
            break
except Exception:
    KO_FONT = None

if KO_FONT:
    matplotlib.rcParams["font.family"] = KO_FONT
    matplotlib.rcParams["font.sans-serif"] = (
        [KO_FONT] + list(matplotlib.rcParams.get("font.sans-serif", [])))
    matplotlib.rcParams["font.monospace"] = (
        [KO_FONT] + list(matplotlib.rcParams.get("font.monospace", [])))
matplotlib.rcParams["axes.unicode_minus"] = False


def T(ko, en):
    """그림 안에 넣을 글자. 한글 폰트가 없으면 영어로 대체한다."""
    return ko if KO_FONT else en


from matplotlib.animation import FuncAnimation


_ANIM_KEEPALIVE = []   # 애니메이션 객체가 GC 되지 않도록 붙잡아두는 곳

STALE_WARN_SEC = 3.0   # 이 시간 넘게 STATE 가 없으면 화면에 경고


# ============================================================
# 터미널 출력 전용 스레드
#
# 왜 따로 두나: sys.stderr.write() 는 상황에 따라 '얼마든지' 오래 막힐 수 있다.
# 그런데 이 프로그램에는 절대 막히면 안 되는 스레드가 둘 있다.
#   - 읽기 스레드 : 막히면 파이프가 차서 C 프로그램(=로봇)이 선다
#   - GUI 스레드  : 막히면 창이 얼어붙는다
# 그래서 출력은 이 스레드가 전담하고, 큐가 꽉 차면 오래된 줄부터 버린다.
# 버려도 되는 이유: 사람이 읽는 로그일 뿐이고, 화면 정보는 스냅샷으로 따로 간다.
# ============================================================
class Echo:
    def __init__(self, maxlen=4000):
        self.q = deque(maxlen=maxlen)
        self.cv = threading.Condition()
        self.dropped = 0
        self.stopping = False
        self.t = threading.Thread(target=self._run, daemon=True)
        self.t.start()

    def put(self, line):
        with self.cv:
            if len(self.q) == self.q.maxlen:
                self.dropped += 1          # deque 가 알아서 앞을 버린다
            self.q.append(line)
            self.cv.notify()

    def _run(self):
        while True:
            with self.cv:
                while not self.q and not self.stopping:
                    self.cv.wait(0.5)
                if self.stopping and not self.q:
                    return
                chunk = "\n".join(self.q) + "\n"
                self.q.clear()
            try:
                sys.stderr.write(chunk)     # 여기서 막혀도 이 스레드만 막힌다
                sys.stderr.flush()
            except Exception:
                pass

    def stop(self):
        with self.cv:
            self.stopping = True
            self.cv.notify()


# ============================================================
# 화면에 그릴 최신 상태 (파싱 스레드가 채우고, GUI 스레드가 읽는다)
#
# 잠금 규칙: 필드는 항상 '통째로 교체'만 한다(제자리 수정 금지).
# 그래야 GUI 스레드가 lock 안에서 참조만 복사해 나가면 그 뒤로는 안전하다.
# numpy 배열도 새로 만들어 대입하므로 같은 규칙이 적용된다.
# ============================================================
class LogState:
    def __init__(self, echo=None):
        self.lock = threading.Lock()
        self.echo = echo

        self.grid = None          # 2D numpy (rows, cols), 1=점유
        self.grid_dil = None      # 3x3 팽창본 - 일치율 계산용 (한 번만 계산)
        self.resolution = 0.05
        self.rows = 0
        self.cols = 0
        self.pose = None          # (x, y, theta)
        self.scan_world = np.empty((0, 2))
        self.waypoints = []
        self.target = None
        self.step = 0
        self.wp_idx = 0
        self.wp_count = 0
        self.phase = ""
        self.lines_seen = 0
        self.dropped = 0          # 따라잡기 위해 버린 STATE/SCAN 줄 수
        self.last_state_time = None
        self.parse_errors = 0
        self.notes = deque(maxlen=4)
        self.eof = False

        # 변경 감지용 일련번호 - 안 바뀌었으면 GUI 가 다시 안 그린다
        self.seq_map = 0
        self.seq_pose = 0
        self.seq_scan = 0
        self.seq_wp = 0
        self.seq_note = 0

    # ------------------------------------------------------------------
    def feed(self, line):
        """한 줄 처리. 파싱 스레드에서만 호출된다. 예외를 위로 던지지 않는다."""
        line = line.rstrip("\n")
        try:
            if line.startswith("SCAN "):
                self._parse_scan(line)
            elif line.startswith("STATE "):
                self._parse_state(line)
            elif line.startswith("MAP "):
                self._parse_map(line)
            elif line.startswith("WAYPOINTS "):
                self._parse_waypoints(line)
            elif line.startswith("TARGET "):
                p = line.split()
                if len(p) >= 4:
                    with self.lock:
                        self.target = (float(p[1]), float(p[2]), float(p[3]))
            else:
                if line.strip():
                    # 터미널 출력은 전용 스레드로 넘긴다 (여기서 직접 print 금지)
                    if self.echo is not None:
                        self.echo.put(line)
                    with self.lock:
                        self.notes.append(line.strip())
                        self.seq_note += 1
        except (ValueError, IndexError, KeyError):
            with self.lock:
                self.parse_errors += 1

    def _parse_map(self, line):
        p = line.split()
        # MAP <rows> <cols> <res> <비트문자열>
        if len(p) < 5:
            return
        rows, cols = int(p[1]), int(p[2])
        res = float(p[3])
        bits = p[4]
        if len(bits) < rows * cols:
            return
        # 파이썬 이중 루프로 13,500칸을 돌지 않고 numpy 한 방에
        buf = np.frombuffer(bits[:rows * cols].encode("ascii"), dtype=np.uint8)
        g = (buf == ord('1')).astype(np.uint8).reshape(rows, cols)
        # 일치율 계산용 3x3 팽창본을 여기서 딱 한 번만 만들어 둔다
        d = g.copy()
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                d |= np.roll(np.roll(g, dr, axis=0), dc, axis=1)
        with self.lock:
            self.rows, self.cols, self.resolution = rows, cols, res
            self.grid, self.grid_dil = g, d
            self.seq_map += 1

    def _parse_scan(self, line):
        p = line.split()
        if len(p) < 2:
            return
        n = int(p[1])
        vals = p[2:2 + 2 * n]
        if len(vals) % 2:
            vals = vals[:-1]
        if len(vals) < 2:
            arr = np.empty((0, 2))
        else:
            try:
                arr = np.asarray(vals, dtype=float).reshape(-1, 2)
            except ValueError:
                return            # 파이프 중간에서 잘린 줄 - 직전 스캔 유지
        with self.lock:
            self.scan_world = arr
            self.seq_scan += 1

    def _parse_state(self, line):
        # STATE <phase> <step> <x> <y> <theta> <wp_idx> <wp_count> <done>
        p = line.split()
        if len(p) < 6:
            return
        x, y, th = float(p[3]), float(p[4]), float(p[5])
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(th)):
            return
        with self.lock:
            self.phase = p[1]
            self.step = int(p[2])
            self.pose = (x, y, th)
            # slot/align 같은 주차 단계는 wp 칸이 0 0 0 이므로 있으면 읽고 없으면 유지
            if len(p) >= 8:
                self.wp_idx, self.wp_count = int(p[6]), int(p[7])
            self.last_state_time = time.monotonic()
            self.seq_pose += 1

    def _parse_waypoints(self, line):
        p = line.split()
        if len(p) < 2:
            return
        n = int(p[1])
        vals = p[2:2 + 2 * n]
        if len(vals) % 2:
            vals = vals[:-1]
        wps = [(float(vals[i]), float(vals[i + 1])) for i in range(0, len(vals), 2)]
        with self.lock:
            self.waypoints = wps
            self.seq_wp += 1

    # ------------------------------------------------------------------
    def snapshot(self):
        """GUI 스레드가 부르는 유일한 함수. lock 안에서 참조만 복사한다(O(1))."""
        with self.lock:
            return dict(
                grid=self.grid, grid_dil=self.grid_dil, rows=self.rows,
                cols=self.cols, resolution=self.resolution,
                pose=self.pose, scan_world=self.scan_world,
                waypoints=self.waypoints, target=self.target,
                step=self.step, phase=self.phase,
                wp_idx=self.wp_idx, wp_count=self.wp_count,
                lines_seen=self.lines_seen, dropped=self.dropped,
                last_state_time=self.last_state_time,
                notes=list(self.notes), eof=self.eof,
                seq=(self.seq_map, self.seq_pose, self.seq_scan,
                     self.seq_wp, self.seq_note),
            )


# ------------------------------------------------------------------
def scan_robot_frame(pose, scan_world):
    """SCAN(월드좌표)을 pose 로 되돌려 로봇 기준 (거리, 각도) 배열로 복원."""
    if pose is None or not len(scan_world):
        return np.empty((0, 2))
    x, y, th = pose
    d = scan_world - np.array([x, y])
    dist = np.hypot(d[:, 0], d[:, 1])
    ang = np.arctan2(d[:, 1], d[:, 0]) - th
    return np.column_stack([dist, ang])


def scan_points(pose, scan_world, mirror):
    """화면에 그릴 스캔 월드좌표. mirror=True 면 각도 부호를 뒤집어 다시 계산.

    RPLIDAR 는 각도가 시계방향으로 증가하는데 코드는 반시계 수학규약
    (x+d*cos, y+d*sin)에 그대로 넣는다. 규약이 어긋나 있으면 스캔 전체가 좌우로
    뒤집혀서, 대칭에 가까운 방에서는 '그럭저럭 맞는' 엉뚱한 자세로 수렴하고
    pose 가 계속 튄다. 뒤집었을 때 스캔이 지도 벽에 훨씬 잘 붙으면 그게 원인이다.
    """
    if not mirror:
        return scan_world
    if pose is None:
        return np.empty((0, 2))
    x, y, th = pose
    rp = scan_robot_frame(pose, scan_world)
    if not len(rp):
        return np.empty((0, 2))
    wa = th - rp[:, 1]
    return np.column_stack([x + rp[:, 0] * np.cos(wa),
                            y + rp[:, 0] * np.sin(wa)])


def scan_fit_score(snap, pts):
    """스캔 점들이 지도의 점유셀에 얼마나 잘 얹히는지 (0~100%).
    미러링 가설을 눈이 아니라 숫자로도 확인하려고 넣음.
    이웃 판정은 미리 만들어둔 팽창 지도(grid_dil) 조회 한 번으로 끝난다."""
    if snap["grid_dil"] is None or not len(pts):
        return None
    res = snap["resolution"]
    c = (pts[:, 0] / res).astype(np.int32)
    r = (pts[:, 1] / res).astype(np.int32)
    ok = (r >= 0) & (r < snap["rows"]) & (c >= 0) & (c < snap["cols"])
    if not ok.any():
        return 0.0
    hit = int(snap["grid_dil"][r[ok], c[ok]].sum())
    return 100.0 * hit / len(pts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mirror", action="store_true",
                    help="시작할 때부터 스캔 각도를 뒤집어서 표시 (실행 중 'm'키로 토글)")
    ap.add_argument("--polar", action="store_true",
                    help="지도 없이 RoboStudio처럼 극좌표로만 표시")
    ap.add_argument("--interval", type=int, default=100,
                    help="화면 갱신 주기(ms). 느린 화면에서는 자동으로 늘어난다")
    ap.add_argument("--max-interval", type=int, default=800,
                    help="적응형 갱신이 늘릴 수 있는 상한(ms)")
    ap.add_argument("--max-points", type=int, default=0,
                    help="한 프레임에 그릴 스캔 점 수 상한 (0=전부). "
                         "원격 화면에서 느리면 200 정도로 줄이면 눈에 띄게 빨라진다")
    ap.add_argument("--no-fit", action="store_true",
                    help="지도 일치율 계산을 끔 ('f'키로도 토글)")
    ap.add_argument("--backend", default=None,
                    help="matplotlib 백엔드 강제 지정 (예: TkAgg, QtAgg)")
    ap.add_argument("--run", default=None,
                    help='실행할 명령 전체를 따옴표로. 예: --run "./main_shuttle ..."')
    args = ap.parse_args()

    echo = Echo()
    state = LogState(echo)
    reader = open_input(args.run, state, echo)
    mirror = [args.mirror]
    show_fit = [not args.no_fit]
    paused = [False]

    # ---- 적응형 갱신 상태 ----
    base_iv = float(args.interval)
    cur_iv = [base_iv]
    last_tick = [time.monotonic()]
    fps_est = [0.0]
    anim_box = [None]

    def adapt(now):
        """실제 프레임 간격을 보고 갱신 주기를 조절한다.

        matplotlib 타이머는 콜백이 오래 걸려도 계속 다음 프레임을 밀어넣기 때문에,
        그리기가 주기보다 느리면 이벤트 루프가 굶어서 창이 반응하지 않는다.
        (사용자가 말한 '멈춤'이 대부분 이것이다. 실제로 죽은 게 아니라 이벤트를
         처리할 틈이 없는 상태다.) 그래서 실측 간격이 주기보다 훨씬 크면 주기를
        늘려 이벤트 루프에 숨통을 틔워주고, 여유가 생기면 다시 줄인다."""
        dt = now - last_tick[0]
        last_tick[0] = now
        if dt > 0:
            fps_est[0] = 0.7 * fps_est[0] + 0.3 * (1.0 / dt)
        want = cur_iv[0]
        if dt * 1000.0 > cur_iv[0] * 1.8:
            want = min(cur_iv[0] * 1.35, float(args.max_interval))
        elif dt * 1000.0 < cur_iv[0] * 1.20 and cur_iv[0] > base_iv:
            want = max(cur_iv[0] * 0.90, base_iv)
        if abs(want - cur_iv[0]) > 1.0:
            cur_iv[0] = want
            a = anim_box[0]
            if a is not None and getattr(a, "event_source", None) is not None:
                try:
                    a.event_source.interval = int(cur_iv[0])
                except Exception:
                    pass

    # ==================================================================
    # 극좌표 모드
    # ==================================================================
    if args.polar:
        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111, projection='polar')
        ax.set_theta_zero_location('N')
        ax.set_theta_direction(-1)     # 시계방향 = RPLIDAR/RoboStudio와 같은 방향
        sc = ax.scatter([], [], s=3, c='red')
        title = ax.set_title("")
        last_seq = [None]

        def update_polar(_):
            now = time.monotonic()
            adapt(now)
            if paused[0]:
                return ()
            snap = state.snapshot()
            if snap["seq"] == last_seq[0]:
                return ()
            last_seq[0] = snap["seq"]
            rp = scan_robot_frame(snap["pose"], snap["scan_world"])
            if len(rp):
                draw = rp
                if args.max_points and len(draw) > args.max_points:
                    draw = draw[:: max(1, len(draw) // args.max_points)]
                sc.set_offsets(np.column_stack([draw[:, 1], draw[:, 0] * 1000.0]))
                ax.set_rmax(float(rp[:, 0].max()) * 1100.0)
            title.set_text(
                T(f"라이다 극좌표 (로봇 기준)  점={len(rp)}  step={snap['step']}  "
                  f"{fps_est[0]:.1f}fps",
                  f"Lidar polar (robot frame)  pts={len(rp)}  step={snap['step']}  "
                  f"{fps_est[0]:.1f}fps"))
            return sc, title

        # anim 을 변수에 담아둬야 한다 - 안 그러면 가비지 컬렉션으로 사라져서
        # "Animation was deleted without rendering anything" 경고와 함께 빈 창만 뜬다.
        anim = FuncAnimation(fig, guard(update_polar, echo), interval=args.interval,
                             blit=False, cache_frame_data=False)
        anim_box[0] = anim
        _ANIM_KEEPALIVE.append(anim)
        plt.show()
        shutdown(reader, echo)
        return anim

    # ==================================================================
    # 지도 모드
    # ==================================================================
    fig, ax = plt.subplots(figsize=(11, 7))
    ax.set_aspect('equal')

    map_img = [None]
    map_extent = [None]
    warned_off = [False]
    fit_cache = [None, None]
    frame_no = [0]
    last_seq = [None]
    last_fit_seq = [None]

    scan_sc = ax.scatter([], [], s=4, c='red', label=T('라이다 스캔', 'Lidar scan'), zorder=5)
    wp_ln, = ax.plot([], [], 'o--', color='deepskyblue', ms=5, lw=1.2,
                     label=T('경로(웨이포인트)', 'Path (waypoints)'), zorder=4)
    robot_ln, = ax.plot([], [], 'o', color='lime', ms=11, label=T('로봇', 'Robot'), zorder=6)
    # 방향선은 '로봇이 앞이라고 믿는 방향'(pose의 theta). 이게 실제 차체 앞면과
    # 반대로 보이면 라이다가 틀어져 장착된 것 -> main_shuttle의 --lidar-yaw 로 보정.
    heading_ln, = ax.plot([], [], '-', color='lime', lw=2.5, zorder=6,
                          label=T('로봇이 앞이라 믿는 방향', 'Believed heading'))
    head_tip, = ax.plot([], [], '>', color='lime', ms=9, zorder=7)
    target_ln, = ax.plot([], [], '*', color='yellow', ms=18, label=T('목표', 'Goal'), zorder=6)
    title = ax.set_title("")
    info = ax.text(0.01, 0.99, "", transform=ax.transAxes, va='top', ha='left',
                   fontsize=9, family='monospace',
                   bbox=dict(boxstyle='round', fc='black', alpha=0.65), color='white', zorder=10)

    def on_key(event):
        if event.key == 'm':
            mirror[0] = not mirror[0]
            last_seq[0] = None          # 강제 재그리기
        elif event.key == 'f':
            show_fit[0] = not show_fit[0]
            last_seq[0] = None
        elif event.key == 'p':
            paused[0] = not paused[0]

    fig.canvas.mpl_connect('key_press_event', on_key)

    def update(_):
        now = time.monotonic()
        adapt(now)
        frame_no[0] += 1
        if paused[0]:
            return ()

        snap = state.snapshot()

        # 새 데이터가 없으면 아무것도 하지 않는다.
        stale_age = (now - snap["last_state_time"]) if snap["last_state_time"] else 0.0
        need_stale_redraw = (stale_age > STALE_WARN_SEC and frame_no[0] % 10 == 0)
        if snap["seq"] == last_seq[0] and not need_stale_redraw:
            return ()
        last_seq[0] = snap["seq"]

        if snap["grid"] is not None and map_img[0] is None:
            extent = [0, snap["cols"] * snap["resolution"],
                      0, snap["rows"] * snap["resolution"]]
            map_extent[0] = extent
            map_img[0] = ax.imshow(snap["grid"], origin='lower', extent=extent,
                                   cmap='Greys', alpha=0.85, zorder=1)
            ax.set_xlim(extent[0] - 0.05, extent[1] + 0.05)
            ax.set_ylim(extent[2] - 0.05, extent[3] + 0.05)
            ax.legend(loc='lower right', fontsize=8)

        pts = scan_points(snap["pose"], snap["scan_world"], mirror[0])
        draw_pts = pts
        if args.max_points and len(draw_pts) > args.max_points:
            draw_pts = draw_pts[:: max(1, len(draw_pts) // args.max_points)]
        # 빈 스캔이 와도 이전 점을 남기지 않는다 - 남으면 '멈춘 화면'처럼 보인다
        scan_sc.set_offsets(draw_pts if len(draw_pts) else np.empty((0, 2)))

        if snap["waypoints"]:
            wp_ln.set_data([p[0] for p in snap["waypoints"]],
                           [p[1] for p in snap["waypoints"]])

        off_map = False
        if snap["pose"]:
            x, y, th = snap["pose"]
            robot_ln.set_data([x], [y])
            # 선 끝에 화살촉을 찍어 어느 쪽이 '앞'인지 확실히 보이게 함
            hx, hy = x + 0.16 * math.cos(th), y + 0.16 * math.sin(th)
            heading_ln.set_data([x, hx], [y, hy])
            head_tip.set_data([hx], [hy])
            off_map = ensure_visible(ax, map_extent[0], x, y)
            if off_map and not warned_off[0]:
                warned_off[0] = True
                echo.put("[live_view] 경고: 추정 자세가 지도 밖으로 나갔습니다. "
                         "--lidar-yaw / --lidar-offset 을 확인하세요.")

        if snap["target"]:
            target_ln.set_data([snap["target"][0]], [snap["target"][1]])

        # 일치율은 스캔이 실제로 바뀌었을 때만, 그것도 4프레임에 한 번만.
        if show_fit[0] and snap["seq"][2] != last_fit_seq[0] and frame_no[0] % 4 == 1:
            last_fit_seq[0] = snap["seq"][2]
            fit_cache[0] = scan_fit_score(
                snap, scan_points(snap["pose"], snap["scan_world"], False))
            fit_cache[1] = scan_fit_score(
                snap, scan_points(snap["pose"], snap["scan_world"], True))
        fit_normal, fit_mirror = fit_cache

        lines = []
        if snap["pose"]:
            lines.append(f"pose = ({snap['pose'][0]:.3f}, {snap['pose'][1]:.3f}, "
                         f"{math.degrees(snap['pose'][2]):6.1f}deg)")
        else:
            lines.append(T("pose = (아직 STATE 수신 없음)", "pose = (no STATE yet)"))
        lines.append(f"phase={snap['phase'] or '-':10s} step={snap['step']}  "
                     f"wp={snap['wp_idx']}/{snap['wp_count']}  "
                     f"scan={len(snap['scan_world'])}" + T("점", "pts"))
        lines.append(T(f"표시중: {'미러링 ON' if mirror[0] else '원본'}   ('m'키로 토글)",
                       f"view: {'MIRRORED' if mirror[0] else 'raw'}   (press 'm' to toggle)"))
        if show_fit[0] and fit_normal is not None and fit_mirror is not None:
            if KO_FONT:
                better = "미러링" if fit_mirror > fit_normal + 5 else (
                         "원본" if fit_normal > fit_mirror + 5 else "비슷함")
                lines.append(f"지도 일치율:  원본 {fit_normal:5.1f}%   미러링 {fit_mirror:5.1f}%"
                             f"   -> {better}")
            else:
                better = "mirrored" if fit_mirror > fit_normal + 5 else (
                         "raw" if fit_normal > fit_mirror + 5 else "similar")
                lines.append(f"map fit:  raw {fit_normal:5.1f}%   mirrored {fit_mirror:5.1f}%"
                             f"   -> {better}")
        # 화면이 '느린 것'인지 '멈춘 것'인지 눈으로 구분하게 해주는 줄
        lines.append(T(f"화면 {fps_est[0]:4.1f}fps (주기 {int(cur_iv[0])}ms)  "
                       f"밀린줄 {reader.backlog()}  버린로그 {echo.dropped}",
                       f"view {fps_est[0]:4.1f}fps (every {int(cur_iv[0])}ms)  "
                       f"queued {reader.backlog()}  log dropped {echo.dropped}"))
        if snap["last_state_time"] is not None and stale_age > STALE_WARN_SEC:
            lines.append(T(f"[!] {stale_age:.1f}초째 STATE 갱신 없음 (로봇이 계산/대기 중)",
                           f"[!] no STATE update for {stale_age:.1f}s"))
        if snap["eof"]:
            lines.append(T("[!] 프로그램이 종료되었습니다", "[!] process exited"))
        if off_map:
            lines.append(T("[!] 자세가 지도 밖 - 위치추정 확인 필요",
                           "[!] pose outside map - check localization"))
        for n in snap["notes"][-3:]:
            # C 프로그램 로그를 그대로 띄우는 줄. 한글 폰트가 없으면 한글이
            # 통째로 깨지고 경고까지 나므로, 그때는 이 줄을 띄우지 않는다
            # (터미널에는 그대로 나오므로 정보 손실은 없다).
            if KO_FONT or all(ord(c) < 0x1100 for c in n):
                lines.append(n[:70])
        info.set_text("\n".join(lines))

        title.set_text(T(f"main_shuttle 실시간 뷰  (읽은 줄 {snap['lines_seen']})",
                         f"main_shuttle live view  (lines read {snap['lines_seen']})"))
        return scan_sc, wp_ln, robot_ln, heading_ln, head_tip, target_ln, title, info

    anim = FuncAnimation(fig, guard(update, echo), interval=args.interval, blit=False,
                         cache_frame_data=False)
    anim_box[0] = anim
    _ANIM_KEEPALIVE.append(anim)
    plt.show()
    shutdown(reader, echo)
    return anim


def shutdown(reader, echo):
    echo.put("[live_view] 창을 닫았습니다 - 실행 중이던 프로그램을 종료합니다.")
    reader.stop()
    echo.stop()
    time.sleep(0.2)


def guard(fn, echo=None):
    """애니메이션 콜백에서 예외가 나면 matplotlib 타이머가 죽어 화면이 그대로 굳는다.
    한 프레임을 건너뛰는 건 괜찮지만 멈추는 건 안 되므로 감싸준다.

    2026-08-11b: 예전에는 여기서 sys.stderr.write() 를 직접 했다. 그런데 그건
    이 파일이 없애려던 바로 그 경로다 - GUI 스레드가 터미널 쓰기에서 막히면
    창이 얼어붙는다. 예외는 드물지만, 예외가 연속으로 나는 상황(예: 스캔 형식이
    깨짐)에서는 매 프레임 발생하므로 실제로 위험하다. 출력 스레드로 넘긴다."""
    seen = [0]
    def wrapped(frame):
        try:
            return fn(frame)
        except Exception as exc:          # noqa: BLE001 - 화면을 살려두는 게 우선
            seen[0] += 1
            if echo is not None and seen[0] <= 20:
                echo.put(f"[live_view] 그리기 오류(무시하고 계속) {seen[0]}회: {exc!r}")
            return ()
    return wrapped


def ensure_visible(ax, extent, x, y):
    """pose 가 지도 밖으로 발산해도 로봇이 화면에서 사라지지 않게 축을 넓힘.
    반환 True = 지금 자세가 지도 밖(= 위치추정이 무너졌다는 신호).

    2026-08-11b: 예전에는 한 번 넓히면 영영 그대로였다. 자세가 잠깐 튀었다가
    돌아와도 화면이 계속 축소된 채라 로봇이 콩알만 하게 보였다. 지도 안으로
    돌아오면 원래 범위로 되돌린다."""
    if extent is None:
        return False
    x0, x1, y0, y1 = extent
    if x0 - 0.1 <= x <= x1 + 0.1 and y0 - 0.1 <= y <= y1 + 0.1:
        want = (x0 - 0.05, x1 + 0.05, y0 - 0.05, y1 + 0.05)
        if (ax.get_xlim() + ax.get_ylim()) != want:
            ax.set_xlim(want[0], want[1])
            ax.set_ylim(want[2], want[3])
        return False
    cx0, cx1 = ax.get_xlim()
    cy0, cy1 = ax.get_ylim()
    m = 0.2
    nx0, nx1 = min(cx0, x - m), max(cx1, x + m)
    ny0, ny1 = min(cy0, y - m), max(cy1, y + m)
    if (nx0, nx1, ny0, ny1) != (cx0, cx1, cy0, cy1):
        ax.set_xlim(nx0, nx1)
        ax.set_ylim(ny0, ny1)
    return True


class LineReader:
    """입력(파이프/파일/자식프로세스)을 읽고 파싱까지 백그라운드에서 끝낸다.

    예전 구조와의 차이가 핵심이다.
      예전: 읽기 스레드는 큐에 넣기만 -> GUI 스레드가 매 프레임 큐를 비우고 파싱하고
            터미널 출력까지 했다. 그래서 터미널이 막히면 GUI 가 통째로 멈췄고,
            큐가 무제한이라 밀리면 메모리가 수십 MB 로 불었다.
      지금: 읽기 / 파싱 / 터미널출력을 세 스레드로 나눴다.
            GUI 스레드는 snapshot() 한 번(참조 복사)이면 끝이라 O(1) 이고,
            밀린 양과 무관하게 프레임 비용이 일정하다.

    읽기와 파싱을 굳이 나눈 이유: 파싱이 순간적으로 느려질 수 있는데(지도 팽창 계산),
    그동안 파이프를 안 비우면 C 프로그램(=로봇)의 printf 가 블록되어 로봇이 선다.
    읽기 스레드는 오직 deque 에 append 만 하므로 절대 느려지지 않는다.
    큐가 넘치면 오래된 줄부터 버린다 - 파이프를 막는 것보다 언제나 낫다.
    """

    QUEUE_MAX = 20000

    def __init__(self, stream, state, echo, proc=None):
        self.stream = stream
        self.state = state
        self.echo = echo
        self.proc = proc
        self.q = deque(maxlen=self.QUEUE_MAX)
        self.cv = threading.Condition()
        self.stopping = False
        self.t_read = threading.Thread(target=self._read_loop, daemon=True)
        self.t_parse = threading.Thread(target=self._parse_loop, daemon=True)
        self.t_read.start()
        self.t_parse.start()

    def backlog(self):
        return len(self.q)

    def _read_loop(self):
        try:
            for line in self.stream:
                with self.cv:
                    full = (len(self.q) == self.QUEUE_MAX)
                    self.q.append(line)
                    self.cv.notify()
                if full:
                    with self.state.lock:
                        self.state.dropped += 1
                if self.stopping:
                    break
        except Exception:
            pass
        with self.cv:
            self.state.eof = True
            self.cv.notify()

    def _parse_loop(self):
        while True:
            with self.cv:
                while not self.q and not self.stopping and not self.state.eof:
                    self.cv.wait(0.2)
                if not self.q:
                    if self.stopping or self.state.eof:
                        return
                    continue
                # 밀려 있으면 화면에 필요한 건 '마지막' STATE/SCAN 하나뿐이므로
                # 중간 것들은 파싱하지 않고 버린다(꺼내는 것 자체는 공짜다).
                batch = list(self.q)
                self.q.clear()
            last_state = last_scan = None
            for line in batch:
                if line.startswith("STATE "):
                    if last_state is not None:
                        self.state.dropped += 1
                    last_state = line
                elif line.startswith("SCAN "):
                    if last_scan is not None:
                        self.state.dropped += 1
                    last_scan = line
                else:
                    self.state.feed(line)      # MAP/TARGET/WAYPOINTS/사람로그는 전부
            # 순서 중요: 자세를 먼저 반영해야 미러링 복원이 같은 프레임 기준으로 맞음
            if last_state is not None:
                self.state.feed(last_state)
            if last_scan is not None:
                self.state.feed(last_scan)
            with self.state.lock:
                self.state.lines_seen += len(batch)

    def stop(self):
        self.stopping = True
        with self.cv:
            self.cv.notify()
        if self.proc is not None and self.proc.poll() is None:
            self.proc.terminate()


def open_input(run_cmd, state, echo):
    """--run 이 있으면 그 명령을 실행해 stdout 을 읽고, 없으면 stdin 을 읽음."""
    if run_cmd:
        import subprocess, shlex
        sys.stderr.write(f"[live_view] 실행: {run_cmd}\n")
        proc = subprocess.Popen(shlex.split(run_cmd), stdout=subprocess.PIPE,
                                stderr=None, text=True, bufsize=1)
        return LineReader(proc.stdout, state, echo, proc)
    if sys.stdin.isatty():
        sys.stderr.write(__doc__ + "\n")
        sys.stderr.write("[live_view] 오류: 읽을 입력이 없습니다. "
                         "--run 을 쓰거나 파이프로 연결하세요.\n")
        sys.exit(1)
    return LineReader(sys.stdin, state, echo)


if __name__ == "__main__":
    main()
