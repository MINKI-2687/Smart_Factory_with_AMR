"""
frontier_cwrap.py
libfrontier.so(진짜 C로 컴파일된 frontier_exploration.h)를 ctypes로 감싼 래퍼.
계산은 100% C가 함. CSlam(slam_cwrap.py) 인스턴스의 _handle을 그대로 재사용.
"""
import ctypes
import os

_lib_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "libfrontier.so")
_lib = ctypes.CDLL(_lib_path)

V = ctypes.c_void_p
D = ctypes.c_double
I = ctypes.c_int
DP = ctypes.POINTER(ctypes.c_double)

_lib.frontier_capi_pick_target.restype = I
_lib.frontier_capi_pick_target.argtypes = [V, D, D, I, I, D, D, D, DP, DP]


def pick_next_frontier_target(slam_obj, robot_x, robot_y, min_cluster_size=3,
                               avoid_point=None, avoid_radius=0.3):
    """slam_obj: CSlam 인스턴스(slam_cwrap.py). 반환: (x,y) 또는 탐사끝이면 None.
    Python 버전(frontier_exploration.py)과 동일한 인터페이스 - 그대로 대체해서 쓸 수 있음."""
    out_x, out_y = D(), D()
    has_avoid = avoid_point is not None
    ax, ay = avoid_point if has_avoid else (0.0, 0.0)
    found = _lib.frontier_capi_pick_target(
        slam_obj._handle, robot_x, robot_y, min_cluster_size,
        1 if has_avoid else 0, ax, ay, avoid_radius,
        ctypes.byref(out_x), ctypes.byref(out_y))
    if found == 0:
        return None
    return (out_x.value, out_y.value)


_lib.frontier_capi_explorer_create.restype = V
_lib.frontier_capi_explorer_create.argtypes = [I]
_lib.frontier_capi_explorer_destroy.argtypes = [V]
_lib.frontier_capi_explorer_update.restype = I
_lib.frontier_capi_explorer_update.argtypes = [V, V, D, D, I]
_lib.frontier_capi_explorer_done.restype = I
_lib.frontier_capi_explorer_done.argtypes = [V]
_lib.frontier_capi_explorer_get_target.argtypes = [V, DP, DP]
_lib.frontier_capi_explorer_has_target.restype = I
_lib.frontier_capi_explorer_has_target.argtypes = [V]


class CFrontierExplorer:
    """탐사 상태(현재 목표, 실패횟수)를 들고 다니는 상태머신. "목표 도착" 신호를 받으면
    그 즉시 근처를 제외하고 다음 프론티어를 새로 골라줌 - 프론티어 방식을 쓰는 곳이면
    어디서든 필요한 로직이라, GUI 스크립트마다 직접 구현하지 말고 이걸 재사용하면 됨.

    사용법:
        explorer = CFrontierExplorer(max_miss=3)
        ...매 재계산 주기(또는 목표 도착 직후)마다...
        got_new = explorer.update(slam_obj, robot_x, robot_y, reached_current=도착했는지)
        if explorer.done(): ...탐사 끝, navigate 단계로 전환...
        elif explorer.has_target(): target = explorer.get_target()
    """
    def __init__(self, max_miss=3):
        self._handle = _lib.frontier_capi_explorer_create(max_miss)

    def __del__(self):
        if getattr(self, "_handle", None):
            _lib.frontier_capi_explorer_destroy(self._handle)

    def update(self, slam_obj, robot_x, robot_y, reached_current):
        return _lib.frontier_capi_explorer_update(
            self._handle, slam_obj._handle, robot_x, robot_y, 1 if reached_current else 0) == 1

    def done(self):
        return _lib.frontier_capi_explorer_done(self._handle) == 1

    def has_target(self):
        return _lib.frontier_capi_explorer_has_target(self._handle) == 1

    def get_target(self):
        x, y = D(), D()
        _lib.frontier_capi_explorer_get_target(self._handle, ctypes.byref(x), ctypes.byref(y))
        return (x.value, y.value)
