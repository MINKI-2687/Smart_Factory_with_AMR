"""
nav_cwrap.py
ctypes wrapper around libnav.so (real C code: dynamic_navigator.h + goal_pid_controller.h).
"""
import ctypes
import os

_lib_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "libnav.so")
_lib = ctypes.CDLL(_lib_path)

D = ctypes.c_double
DP = ctypes.POINTER(ctypes.c_double)
I = ctypes.c_int
IP = ctypes.POINTER(ctypes.c_int)
UB = ctypes.c_ubyte
UBP = ctypes.POINTER(ctypes.c_ubyte)
V = ctypes.c_void_p

_lib.nav_capi_make_grid_from_circles.restype = UBP
_lib.nav_capi_make_grid_from_circles.argtypes = [D, D, D, DP, DP, DP, I, IP, IP]

_lib.nav_capi_free_buffer.argtypes = [ctypes.c_void_p]

_lib.nav_capi_create.restype = V
_lib.nav_capi_create.argtypes = [UBP, I, I, D, D, D, D]

_lib.nav_capi_destroy.argtypes = [V]
_lib.nav_capi_set_goal.restype = I
_lib.nav_capi_set_goal.argtypes = [V, D, D, D, D, I, D]
_lib.nav_capi_get_waypoint_count.restype = I
_lib.nav_capi_get_waypoint_count.argtypes = [V]
_lib.nav_capi_get_waypoint.argtypes = [V, I, DP, DP]
_lib.nav_capi_get_current_wp_idx.restype = I
_lib.nav_capi_get_current_wp_idx.argtypes = [V]
_lib.nav_capi_get_rows.restype = I
_lib.nav_capi_get_rows.argtypes = [V]
_lib.nav_capi_get_cols.restype = I
_lib.nav_capi_get_cols.argtypes = [V]
_lib.nav_capi_get_resolution.restype = D
_lib.nav_capi_get_resolution.argtypes = [V]
_lib.nav_capi_get_known_grid.argtypes = [V, UBP]
_lib.nav_capi_get_replanned_this_call.restype = I
_lib.nav_capi_get_replanned_this_call.argtypes = [V]
_lib.nav_capi_update.argtypes = [V, D, D, D, DP, DP, I, IP, IP, IP]

_lib.safenav_capi_create.restype = V
_lib.safenav_capi_create.argtypes = [D]
_lib.safenav_capi_destroy.argtypes = [V]
_lib.safenav_capi_compute.argtypes = [V, D, D, D, D, D, DP, DP, I, IP, IP, IP]


def make_grid_from_circles(size_x, size_y, resolution, circles):
    """circles: [(cx, cy, radius), ...]. Returns a Python list-of-lists (0/1)."""
    n = len(circles)
    cx = (D * n)(*[c[0] for c in circles])
    cy = (D * n)(*[c[1] for c in circles])
    cr = (D * n)(*[c[2] for c in circles])
    rows, cols = I(), I()
    buf = _lib.nav_capi_make_grid_from_circles(size_x, size_y, resolution, cx, cy, cr, n,
                                                ctypes.byref(rows), ctypes.byref(cols))
    grid = []
    for r in range(rows.value):
        grid.append([buf[r * cols.value + c] for c in range(cols.value)])
    _lib.nav_capi_free_buffer(buf)
    return grid, rows.value, cols.value


def _scan_to_arrays(scan):
    n = len(scan)
    angles = (D * n)(*[a for a, d in scan])
    dists = (D * n)(*[d for a, d in scan])
    return angles, dists, n


_lib.nav_capi_set_pure_pursuit.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_double]
_lib.nav_capi_get_goal_unreachable.restype = ctypes.c_int
_lib.nav_capi_get_goal_unreachable.argtypes = [ctypes.c_void_p]


class CDynamicNavigator:
    def __init__(self, grid, rows, cols, resolution, robot_radius, origin_x=0.0, origin_y=0.0):
        flat = (UB * (rows * cols))(*[grid[r][c] for r in range(rows) for c in range(cols)])
        self._handle = _lib.nav_capi_create(flat, rows, cols, resolution, robot_radius, origin_x, origin_y)
        self.rows = rows
        self.cols = cols
        self.resolution = resolution

    def __del__(self):
        if getattr(self, "_handle", None):
            _lib.nav_capi_destroy(self._handle)
            self._handle = None

    def set_pure_pursuit(self, enable, lookahead_m=0.22):
        """실기 RobotConfig.pure_pursuit / lookahead_m 에 해당."""
        _lib.nav_capi_set_pure_pursuit(self._handle, 1 if enable else 0, lookahead_m)

    def set_goal(self, start, goal, goal_theta=None):
        has_theta = goal_theta is not None
        ok = _lib.nav_capi_set_goal(self._handle, start[0], start[1], goal[0], goal[1],
                                     1 if has_theta else 0, goal_theta if has_theta else 0.0)
        return ok == 1

    def goal_unreachable(self):
        """done=True여도 이게 True면 '도착'이 아니라 '경로가 막혀 포기'한 것."""
        return _lib.nav_capi_get_goal_unreachable(self._handle) == 1

    def get_waypoints(self):
        n = _lib.nav_capi_get_waypoint_count(self._handle)
        wps = []
        x, y = D(), D()
        for i in range(n):
            _lib.nav_capi_get_waypoint(self._handle, i, ctypes.byref(x), ctypes.byref(y))
            wps.append((x.value, y.value))
        return wps

    def get_current_wp_idx(self):
        return _lib.nav_capi_get_current_wp_idx(self._handle)

    def replanned_this_call(self):
        return _lib.nav_capi_get_replanned_this_call(self._handle) == 1

    def get_known_grid(self):
        n = self.rows * self.cols
        buf = (UB * n)()
        _lib.nav_capi_get_known_grid(self._handle, buf)
        grid = []
        for r in range(self.rows):
            grid.append([buf[r * self.cols + c] for c in range(self.cols)])
        return grid

    def update(self, x, y, theta, scan):
        angles, dists, n = _scan_to_arrays(scan)
        left, right, done = I(), I(), I()
        _lib.nav_capi_update(self._handle, x, y, theta, angles, dists, n,
                              ctypes.byref(left), ctypes.byref(right), ctypes.byref(done))
        return left.value, right.value, done.value == 1


class CSafeNavigator:
    """Standalone local-avoidance controller (used during the SLAM mapping phase, no A*)."""

    def __init__(self, goal_tolerance=0.15):
        self._handle = _lib.safenav_capi_create(goal_tolerance)

    def __del__(self):
        if getattr(self, "_handle", None):
            _lib.safenav_capi_destroy(self._handle)
            self._handle = None

    def compute(self, x, y, theta, goal_x, goal_y, scan):
        angles, dists, n = _scan_to_arrays(scan)
        left, right, done = I(), I(), I()
        _lib.safenav_capi_compute(self._handle, x, y, theta, goal_x, goal_y, angles, dists, n,
                                   ctypes.byref(left), ctypes.byref(right), ctypes.byref(done))
        return left.value, right.value, done.value == 1
