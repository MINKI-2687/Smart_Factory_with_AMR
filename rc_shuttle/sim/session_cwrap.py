"""
session_cwrap.py
libsession.so(진짜 C로 컴파일된 mapping_session.h)를 ctypes로 감싼 래퍼.

이 파일에는 판단/로직이 전혀 없습니다 - 전부 C 함수를 그대로 호출만 합니다.
"매핑->프론티어탐색->지도확정->A*계획->회피/도킹" 전체 결정 로직은 100% C쪽
(mapping_session.h)에만 존재합니다. 여기서 재구현하면 안 됩니다.
"""
import ctypes
import os
from enum import IntEnum

_lib_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "libsession.so")
_lib = ctypes.CDLL(_lib_path)

V = ctypes.c_void_p
D = ctypes.c_double
DP = ctypes.POINTER(ctypes.c_double)
I = ctypes.c_int
IP = ctypes.POINTER(ctypes.c_int)

_lib.session_capi_create.restype = V
_lib.session_capi_create.argtypes = [D, D, D, D, D, D, D, D, I, D, D, D, I, I]

UB = ctypes.c_ubyte
UBP = ctypes.POINTER(ctypes.c_ubyte)
_lib.session_capi_create_fixed_map.restype = V
_lib.session_capi_create_fixed_map.argtypes = [UBP, I, I, D, D, D, D, D, D, I, D, D]

_lib.session_capi_create_mapping_only.restype = V
_lib.session_capi_create_mapping_only.argtypes = [D, D, D, D, D, D, D, D, I, I]

_lib.session_capi_start_navigation.restype = I
_lib.session_capi_start_navigation.argtypes = [V, D, D, I, D]

_lib.session_capi_destroy.argtypes = [V]

_lib.session_capi_step.argtypes = [V, DP, DP, I, D, D, D, IP, IP, IP]

_lib.session_capi_get_phase.restype = I
_lib.session_capi_get_phase.argtypes = [V]

_lib.session_capi_get_pose.argtypes = [V, DP, DP, DP]

_lib.session_capi_get_frontier_target.restype = I
_lib.session_capi_get_frontier_target.argtypes = [V, DP, DP]

_lib.session_capi_get_rows.restype = I
_lib.session_capi_get_rows.argtypes = [V]
_lib.session_capi_get_cols.restype = I
_lib.session_capi_get_cols.argtypes = [V]
_lib.session_capi_get_resolution.restype = D
_lib.session_capi_get_resolution.argtypes = [V]

_lib.session_capi_get_prob_grid.argtypes = [V, DP]

_lib.session_capi_get_waypoint_count.restype = I
_lib.session_capi_get_waypoint_count.argtypes = [V]
_lib.session_capi_get_waypoint.argtypes = [V, I, DP, DP]
_lib.session_capi_get_current_wp_idx.restype = I
_lib.session_capi_get_current_wp_idx.argtypes = [V]

_lib.session_capi_retarget.restype = I
_lib.session_capi_retarget.argtypes = [V, D, D, I, D]

_lib.session_capi_get_frontier_debug.argtypes = [V, IP, DP, DP, IP, IP, IP]

_lib.session_capi_get_used_graph_slam_correction.restype = I
_lib.session_capi_get_used_graph_slam_correction.argtypes = [V]


class SessionPhase(IntEnum):
    MAPPING = 0
    NAVIGATE = 1
    DONE = 2
    PLAN_FAILED = 3
    MAP_READY = 4


class CMappingSession:
    """매핑(SLAM+프론티어) -> 지도확정 -> A*계획 -> 내비게이션(회피+도킹) 전체를
    담당하는 세션. 결정 로직은 전부 C(mapping_session.h)에 있고, 이 클래스는 얇은
    ctypes 다리 역할만 합니다.

    사용법:
        session = CMappingSession(size_x=3.0, size_y=3.0, resolution=0.05,
                                   start=(0.3, 0.3, 0.0), goal=(2.5, 2.5),
                                   goal_theta_deg=90.0, robot_radius=0.10)
        while True:
            scan = ...  # [(angle_deg, dist_mm), ...]
            left, right, done = session.step(scan, odom_dx, odom_dy, odom_dtheta)
            if session.phase == SessionPhase.PLAN_FAILED:
                ...실패 처리...
                break
            if done:
                break
    """

    def __init__(self, size_x, size_y, resolution, start, goal, robot_radius,
                 goal_theta_deg=None, occ_threshold=0.6,
                 frontier_max_miss=3, max_mapping_steps=20 * 60 * 10):
        import math
        has_goal_theta = goal_theta_deg is not None
        goal_theta_rad = math.radians(goal_theta_deg) if has_goal_theta else 0.0
        self._handle = _lib.session_capi_create(
            size_x, size_y, resolution,
            start[0], start[1], start[2],
            goal[0], goal[1],
            1 if has_goal_theta else 0, goal_theta_rad,
            robot_radius, occ_threshold,
            frontier_max_miss, max_mapping_steps)

    def __del__(self):
        if getattr(self, "_handle", None):
            _lib.session_capi_destroy(self._handle)
            self._handle = None

    @classmethod
    def from_fixed_map(cls, known_grid, rows, cols, resolution, start, goal, robot_radius,
                        goal_theta_deg=None):
        """known_grid: list-of-lists(0/1), 반드시 벽 포함(nav_cwrap.make_grid_from_circles로
        만들면 자동 포함됨). 매핑(프론티어탐색) 단계 없이 바로 라이다보정+내비게이션 시작.
        오도메트리만 쓰면 방 하나 가로지르는 정도에서도 ~40cm급 드리프트가 쌓이는 게
        실측 확인됐으므로, 고정맵이어도 반드시 이 경로를 통해 라이다 위치보정을 받아야 함."""
        import math
        obj = cls.__new__(cls)
        has_goal_theta = goal_theta_deg is not None
        goal_theta_rad = math.radians(goal_theta_deg) if has_goal_theta else 0.0
        flat = (UB * (rows * cols))(*[known_grid[r][c] for r in range(rows) for c in range(cols)])
        obj._handle = _lib.session_capi_create_fixed_map(
            flat, rows, cols, resolution,
            start[0], start[1], start[2],
            goal[0], goal[1],
            1 if has_goal_theta else 0, goal_theta_rad,
            robot_radius)
        return obj

    @classmethod
    def for_mapping_only(cls, size_x, size_y, resolution, start, robot_radius,
                          occ_threshold=0.6, frontier_max_miss=3, max_mapping_steps=20 * 60 * 5):
        """목표를 아직 모를 때 씀(GUI에서 지도를 보여준 뒤 클릭으로 고르게 하고 싶을 때).
        매핑이 끝나면 자동으로 A*계획을 안 하고 phase가 MAP_READY에서 멈춤.
        이후 start_navigation(goal, goal_theta_deg)을 호출해서 실제 목표를 넣어줘야 진행됨."""
        obj = cls.__new__(cls)
        obj._handle = _lib.session_capi_create_mapping_only(
            size_x, size_y, resolution,
            start[0], start[1], start[2],
            robot_radius, occ_threshold, frontier_max_miss, max_mapping_steps)
        return obj

    def start_navigation(self, goal, goal_theta_deg=None):
        """phase가 MAP_READY일 때만 유효(for_mapping_only로 만든 세션). 사용자가 GUI에서
        고른 목표로 실제 내비게이션을 시작시킴. 성공하면 phase가 NAVIGATE로 전환됨
        (True 반환), A*계획 실패하면 PLAN_FAILED로 전환(False 반환)."""
        import math
        has_goal_theta = goal_theta_deg is not None
        goal_theta_rad = math.radians(goal_theta_deg) if has_goal_theta else 0.0
        ok = _lib.session_capi_start_navigation(self._handle, goal[0], goal[1],
                                                 1 if has_goal_theta else 0, goal_theta_rad)
        return ok == 1

    def step(self, scan, odom_dx, odom_dy, odom_dtheta):
        n = len(scan)
        angles = (D * n)(*[a for a, d in scan])
        dists = (D * n)(*[d for a, d in scan])
        left, right, done = I(), I(), I()
        _lib.session_capi_step(self._handle, angles, dists, n,
                                odom_dx, odom_dy, odom_dtheta,
                                ctypes.byref(left), ctypes.byref(right), ctypes.byref(done))
        return left.value, right.value, done.value == 1

    @property
    def phase(self):
        return SessionPhase(_lib.session_capi_get_phase(self._handle))

    def get_pose(self):
        x, y, theta = D(), D(), D()
        _lib.session_capi_get_pose(self._handle, ctypes.byref(x), ctypes.byref(y), ctypes.byref(theta))
        return x.value, y.value, theta.value

    def get_frontier_target(self):
        """탐사 목표가 있으면 (x,y), 없으면 None."""
        x, y = D(), D()
        found = _lib.session_capi_get_frontier_target(self._handle, ctypes.byref(x), ctypes.byref(y))
        return (x.value, y.value) if found == 1 else None

    def get_prob_grid(self):
        """점유확률(0~1) 격자를 list-of-lists로 반환 (시각화용)."""
        rows = _lib.session_capi_get_rows(self._handle)
        cols = _lib.session_capi_get_cols(self._handle)
        buf = (D * (rows * cols))()
        _lib.session_capi_get_prob_grid(self._handle, buf)
        return [[buf[r * cols + c] for c in range(cols)] for r in range(rows)], rows, cols

    def get_resolution(self):
        return _lib.session_capi_get_resolution(self._handle)

    def get_waypoints(self):
        n = _lib.session_capi_get_waypoint_count(self._handle)
        x, y = D(), D()
        wps = []
        for i in range(n):
            _lib.session_capi_get_waypoint(self._handle, i, ctypes.byref(x), ctypes.byref(y))
            wps.append((x.value, y.value))
        return wps

    def get_current_wp_idx(self):
        return _lib.session_capi_get_current_wp_idx(self._handle)

    def retarget(self, goal, goal_theta_deg=None):
        """셔틀 왕복용 - DONE 상태(한 지점 도킹 완료)에서 새 목표로 재설정.
        지도/위치추정은 그대로 유지되고 목표만 바뀜. 성공하면 phase가 다시
        NAVIGATE로 전환됨(True 반환), A*계획 실패하면 PLAN_FAILED로 전환(False 반환)."""
        import math
        has_goal_theta = goal_theta_deg is not None
        goal_theta_rad = math.radians(goal_theta_deg) if has_goal_theta else 0.0
        ok = _lib.session_capi_retarget(self._handle, goal[0], goal[1],
                                         1 if has_goal_theta else 0, goal_theta_rad)
        return ok == 1

    def get_frontier_debug(self):
        """Debug-only: returns dict with FrontierExplorer internals
        (has_target, target, miss_count, avoid_history_count, steps_on_current_target)."""
        has_target, tx, ty = I(), D(), D()
        miss_count, avoid_history_count, steps_on_target = I(), I(), I()
        _lib.session_capi_get_frontier_debug(
            self._handle, ctypes.byref(has_target), ctypes.byref(tx), ctypes.byref(ty),
            ctypes.byref(miss_count), ctypes.byref(avoid_history_count), ctypes.byref(steps_on_target))
        return {
            "has_target": has_target.value == 1,
            "target": (tx.value, ty.value),
            "miss_count": miss_count.value,
            "avoid_history_count": avoid_history_count.value,
            "steps_on_current_target": steps_on_target.value,
        }

    def used_graph_slam_correction(self):
        """True if graph-SLAM (loop closure) correction was actually applied to the
        final map/pose when mapping finished (it can fail silently and fall back to
        the raw map if there weren't enough re-observed landmarks)."""
        return _lib.session_capi_get_used_graph_slam_correction(self._handle) == 1
