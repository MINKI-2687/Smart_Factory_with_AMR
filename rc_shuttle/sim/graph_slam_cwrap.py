"""
graph_slam_cwrap.py
libgraphslam.so(진짜 C로 컴파일된 graph_slam.h)를 ctypes로 감싸서 파이썬처럼 쓸 수 있게 하는 래퍼.
계산은 100% C가 함 - slam_cwrap.py와 동일한 철학.

사용 전 준비:
  gcc -std=c11 -O2 -shared -fPIC -o libgraphslam.so graph_slam_capi.c -lm
"""
import ctypes
import os

_lib_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "libgraphslam.so")
_lib = ctypes.CDLL(_lib_path)

V = ctypes.c_void_p
D = ctypes.c_double
I = ctypes.c_int
DP = ctypes.POINTER(ctypes.c_double)

_lib.gslam_capi_create.restype = V
_lib.gslam_capi_create.argtypes = [D, D, D, D, D]

_lib.gslam_capi_destroy.argtypes = [V]

_lib.gslam_capi_step.argtypes = [V, D, D, DP, DP, I, D, DP, DP, I]

_lib.gslam_capi_get_n_poses.restype = I
_lib.gslam_capi_get_n_poses.argtypes = [V]
_lib.gslam_capi_get_n_landmarks.restype = I
_lib.gslam_capi_get_n_landmarks.argtypes = [V]

_lib.gslam_capi_solve.restype = I
_lib.gslam_capi_solve.argtypes = [V, DP, DP, DP, DP]

_lib.gslam_capi_rebuild_grid.restype = I
_lib.gslam_capi_rebuild_grid.argtypes = [V, V]

_lib.gslam_capi_get_raw_pose.argtypes = [V, I, DP, DP]


def _arr(vals):
    a = (ctypes.c_double * len(vals))()
    for i, v in enumerate(vals):
        a[i] = v
    return a


class CGraphSlam:
    def __init__(self, start_x, start_y, motion_noise=0.0004, measurement_noise=0.0009,
                 landmark_match_threshold=0.3):
        self._handle = _lib.gslam_capi_create(start_x, start_y, motion_noise,
                                               measurement_noise, landmark_match_threshold)

    def __del__(self):
        if getattr(self, "_handle", None):
            _lib.gslam_capi_destroy(self._handle)

    def step(self, dx, dy, cluster_observations, robot_theta, scan=None):
        """
        cluster_observations: [(angle_rad, dist_m), ...] 로봇 기준 상대 랜드마크 관측
        scan: [(angle_deg, dist_mm), ...] 이번 스텝 라이다 스캔(지도재구성용, 없으면 None)
        """
        n_obs = len(cluster_observations)
        angles = _arr([o[0] for o in cluster_observations]) if n_obs else DP()
        dists = _arr([o[1] for o in cluster_observations]) if n_obs else DP()

        if scan:
            scan_angles = _arr([s[0] for s in scan])
            scan_dists = _arr([s[1] for s in scan])
            scan_n = len(scan)
        else:
            scan_angles, scan_dists, scan_n = DP(), DP(), 0

        _lib.gslam_capi_step(self._handle, dx, dy, angles, dists, n_obs, robot_theta,
                              scan_angles, scan_dists, scan_n)

    @property
    def n_poses(self):
        return _lib.gslam_capi_get_n_poses(self._handle)

    @property
    def n_landmarks(self):
        return _lib.gslam_capi_get_n_landmarks(self._handle)

    def solve(self):
        """반환: (corrected_poses [(x,y),...], corrected_landmarks [(x,y),...]) 또는 실패시 (None, None)"""
        n_poses = self.n_poses
        n_lm = self.n_landmarks
        px = (ctypes.c_double * n_poses)()
        py = (ctypes.c_double * n_poses)()
        lx = (ctypes.c_double * max(n_lm, 1))()
        ly = (ctypes.c_double * max(n_lm, 1))()
        ok = _lib.gslam_capi_solve(self._handle, px, py, lx, ly)
        if not ok:
            return None, None
        poses = [(px[i], py[i]) for i in range(n_poses)]
        landmarks = [(lx[i], ly[i]) for i in range(n_lm)]
        return poses, landmarks

    def rebuild_grid(self, cslam_grid):
        """cslam_grid: slam_cwrap.py의 CSlam 인스턴스. 성공하면 True."""
        return _lib.gslam_capi_rebuild_grid(self._handle, cslam_grid._handle) == 1

    def get_raw_pose(self, idx):
        x, y = ctypes.c_double(), ctypes.c_double()
        _lib.gslam_capi_get_raw_pose(self._handle, idx, ctypes.byref(x), ctypes.byref(y))
        return x.value, y.value
