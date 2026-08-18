"""
slam_cwrap.py
libslam.so(진짜 C로 컴파일된 slam.h)를 ctypes로 감싸서 파이썬처럼 쓸 수 있게 하는 래퍼.

주의: 이건 파이썬으로 SLAM을 "다시 구현"한 게 아니라, C 코드를 컴파일한 바이너리를
그대로 호출하는 것 - 계산 자체는 100% C가 함. 파이썬은 그 결과만 받아서 화면에 그림.

사용 전 준비:
  gcc -std=c11 -O2 -shared -fPIC -o libslam.so slam_capi.c -lm
"""
import ctypes
import os

_lib_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "libslam.so")
_lib = ctypes.CDLL(_lib_path)

_lib.slam_capi_create.restype = ctypes.c_void_p
_lib.slam_capi_create.argtypes = [ctypes.c_double] * 12

_lib.slam_capi_destroy.argtypes = [ctypes.c_void_p]

_lib.slam_capi_create_from_grid.restype = ctypes.c_void_p
_lib.slam_capi_create_from_grid.argtypes = ([ctypes.POINTER(ctypes.c_ubyte),
                                             ctypes.c_int, ctypes.c_int] + [ctypes.c_double] * 10)

_lib.slam_capi_update.argtypes = [
    ctypes.c_void_p,
    ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.c_int,
    ctypes.c_double, ctypes.c_double, ctypes.c_double,
    ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double),
]

_lib.slam_capi_update_map_only.argtypes = [
    ctypes.c_void_p,
    ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.c_int,
]

_lib.slam_capi_update_map_at_pose.argtypes = [
    ctypes.c_void_p,
    ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.c_int,
    ctypes.c_double, ctypes.c_double, ctypes.c_double,
]

_lib.slam_capi_localize_only.argtypes = [
    ctypes.c_void_p,
    ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.c_int,
    ctypes.c_double, ctypes.c_double, ctypes.c_double,
    ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double),
]

_lib.slam_capi_get_pose.argtypes = [
    ctypes.c_void_p,
    ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double),
]

_lib.slam_capi_get_rows.restype = ctypes.c_int
_lib.slam_capi_get_rows.argtypes = [ctypes.c_void_p]
_lib.slam_capi_get_cols.restype = ctypes.c_int
_lib.slam_capi_get_cols.argtypes = [ctypes.c_void_p]
_lib.slam_capi_get_resolution.restype = ctypes.c_double
_lib.slam_capi_get_resolution.argtypes = [ctypes.c_void_p]

_lib.slam_capi_get_log_odds.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_double)]

_lib.slam_capi_enable_likelihood_field.argtypes = [ctypes.c_void_p]


class CSlam:
    """진짜 C의 OccupancyGridSLAM을 파이썬 객체처럼 쓸 수 있게 감싼 클래스."""

    def __init__(self, size_x, size_y, resolution, origin_x=0.0, origin_y=0.0,
                 start_pose=(0.0, 0.0, 0.0),
                 search_window_m=0.15, search_window_theta_deg=12,
                 search_step_m=0.03, search_step_theta_deg=3):
        self._handle = _lib.slam_capi_create(
            size_x, size_y, resolution, origin_x, origin_y,
            start_pose[0], start_pose[1], start_pose[2],
            search_window_m, search_window_theta_deg, search_step_m, search_step_theta_deg,
        )
        self.rows = _lib.slam_capi_get_rows(self._handle)
        self.cols = _lib.slam_capi_get_cols(self._handle)
        self.resolution = _lib.slam_capi_get_resolution(self._handle)
        self.origin_x = origin_x
        self.origin_y = origin_y
        self._first_scan_done = False

    @classmethod
    def from_grid(cls, grid, rows, cols, resolution, origin_x=0.0, origin_y=0.0,
                  start_pose=(0.0, 0.0, 0.0),
                  search_window_m=0.15, search_window_theta_deg=12,
                  search_step_m=0.03, search_step_theta_deg=3):
        """이미 알고 있는 격자로 바로 초기화 - 실기의 --map-source load와 동일한 경로.
        합성 스캔으로 지도를 채우는(flood fill) 방식과 달리 지도가 정확히 일치하고,
        시작 자세도 준 값 그대로 들어가므로 초기 위치추정이 어긋나지 않음."""
        obj = cls.__new__(cls)
        flat = (ctypes.c_ubyte * (rows * cols))(
            *[1 if grid[r][c] else 0 for r in range(rows) for c in range(cols)])
        obj._handle = _lib.slam_capi_create_from_grid(
            flat, rows, cols, resolution, origin_x, origin_y,
            start_pose[0], start_pose[1], start_pose[2],
            search_window_m, search_window_theta_deg, search_step_m, search_step_theta_deg)
        obj.rows = _lib.slam_capi_get_rows(obj._handle)
        obj.cols = _lib.slam_capi_get_cols(obj._handle)
        obj.resolution = _lib.slam_capi_get_resolution(obj._handle)
        obj.origin_x, obj.origin_y = origin_x, origin_y
        obj._first_scan_done = True     # 지도가 이미 있으므로 첫 스캔으로 채울 필요 없음
        return obj

    def __del__(self):
        if getattr(self, "_handle", None):
            _lib.slam_capi_destroy(self._handle)
            self._handle = None

    @staticmethod
    def _scan_to_arrays(scan):
        n = len(scan)
        angles = (ctypes.c_double * n)(*[a for a, d in scan])
        dists = (ctypes.c_double * n)(*[d for a, d in scan])
        return angles, dists, n

    def update(self, scan, odom_dx, odom_dy, odom_dtheta):
        """scan: [(angle_deg, dist_mm), ...]. 반환: (x, y, theta) 보정된 pose."""
        angles, dists, n = self._scan_to_arrays(scan)

        if not self._first_scan_done:
            _lib.slam_capi_update_map_only(self._handle, angles, dists, n)
            self._first_scan_done = True
            return self.get_pose()

        x, y, theta = ctypes.c_double(), ctypes.c_double(), ctypes.c_double()
        _lib.slam_capi_update(self._handle, angles, dists, n,
                               odom_dx, odom_dy, odom_dtheta,
                               ctypes.byref(x), ctypes.byref(y), ctypes.byref(theta))
        return x.value, y.value, theta.value

    def localize_only(self, scan, odom_dx, odom_dy, odom_dtheta):
        """지도는 안 건드리고 위치만 스캔매칭으로 보정. 반환: (x, y, theta)."""
        angles, dists, n = self._scan_to_arrays(scan)
        x, y, theta = ctypes.c_double(), ctypes.c_double(), ctypes.c_double()
        _lib.slam_capi_localize_only(self._handle, angles, dists, n,
                                      odom_dx, odom_dy, odom_dtheta,
                                      ctypes.byref(x), ctypes.byref(y), ctypes.byref(theta))
        return x.value, y.value, theta.value

    def update_map_at_pose(self, scan, x, y, theta):
        """Map-only update at an externally-tracked pose (no correction).
        Used for the 'B condition' (odometry-only, no scan matching) comparison."""
        angles, dists, n = self._scan_to_arrays(scan)
        _lib.slam_capi_update_map_at_pose(self._handle, angles, dists, n, x, y, theta)

    def get_pose(self):
        x, y, theta = ctypes.c_double(), ctypes.c_double(), ctypes.c_double()
        _lib.slam_capi_get_pose(self._handle, ctypes.byref(x), ctypes.byref(y), ctypes.byref(theta))
        return x.value, y.value, theta.value

    def enable_likelihood_field(self):
        """실기(robot_runner.h)와 동일한 부드러운 점수판을 켬 - 고정맵/불러온맵에서만
        의미가 있음(지도가 매 스텝 바뀌는 SLAM 매핑 모드에는 쓰지 말 것)."""
        _lib.slam_capi_enable_likelihood_field(self._handle)

    def get_log_odds_grid(self):
        """rows x cols 크기의 2차원 리스트로 log-odds 값을 반환 (matplotlib 등에서 그리기 좋게)."""
        n = self.rows * self.cols
        buf = (ctypes.c_double * n)()
        _lib.slam_capi_get_log_odds(self._handle, buf)
        grid = []
        for r in range(self.rows):
            row = [buf[r * self.cols + c] for c in range(self.cols)]
            grid.append(row)
        return grid
