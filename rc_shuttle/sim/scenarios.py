"""
scenarios.py
여러 상황을 한 곳에 정의해두고, headless 테스트와 GUI 둘 다 이 정의를 그대로 재사용.

각 시나리오:
  name: 이름
  map_source: "fixed" 또는 "slam"
  obstacles: [(x,y,radius), ...] 진짜 정답 장애물(원형). fixed 모드는 이걸로 초기 격자를 만들고,
             slam 모드는 이걸로 "진짜 세계"를 흉내내서 라이다 스캔을 시뮬레이션함.
  grid_size: (가로, 세로) 미터
  start: (x, y)
  goal: (x, y)
  goal_theta_deg: None(방향 안 맞춤) 또는 각도(도킹까지 확인)
"""
import math


def wall(x0, y0, x1, y1, radius=0.05, spacing=0.08):
    """직선 벽을 원형 장애물들의 연속으로 생성 (S자 경로 등 만들 때 사용)."""
    length = math.hypot(x1 - x0, y1 - y0)
    n = max(2, int(length / spacing) + 1)
    circles = []
    for i in range(n):
        t = i / (n - 1)
        circles.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, radius))
    return circles


SCENARIOS = []

SCENARIOS.append({
    "name": "고정맵_단일장애물",
    "map_source": "fixed",
    "obstacles": [(1.5, 1.5, 0.2)],
    "grid_size": (3.0, 3.0),
    "start": (0.2, 0.2),
    "goal": (2.5, 2.5),
    "goal_theta_deg": None,
})

SCENARIOS.append({
    "name": "고정맵_주차90도",
    "map_source": "fixed",
    "obstacles": [(1.5, 1.5, 0.2)],
    "grid_size": (3.0, 3.0),
    "start": (0.2, 0.2),
    "goal": (2.5, 2.5),
    "goal_theta_deg": 90.0,
})

SCENARIOS.append({
    "name": "고정맵_주차180도",
    "map_source": "fixed",
    "obstacles": [(1.5, 1.5, 0.2)],
    "grid_size": (3.0, 3.0),
    "start": (0.2, 0.2),
    "goal": (2.5, 2.5),
    "goal_theta_deg": 180.0,
})

SCENARIOS.append({
    "name": "고정맵_다중장애물",
    "map_source": "fixed",
    "obstacles": [(0.9, 0.9, 0.15), (2.0, 0.8, 0.15), (1.5, 1.9, 0.15), (2.3, 2.2, 0.15)],
    "grid_size": (3.0, 3.0),
    "start": (0.2, 0.2),
    "goal": (2.8, 2.8),
    "goal_theta_deg": 0.0,
})

s_curve_obstacles = wall(1.0, 0.0, 1.0, 2.0) + wall(2.0, 1.0, 2.0, 3.0)
SCENARIOS.append({
    "name": "고정맵_S자경로",
    "map_source": "fixed",
    "obstacles": s_curve_obstacles,
    "grid_size": (3.0, 3.0),
    "start": (0.3, 0.3),
    "goal": (2.7, 2.5),
    "goal_theta_deg": None,
})

SCENARIOS.append({
    "name": "SLAM_단일장애물",
    "map_source": "slam",
    "obstacles": [(1.5, 1.5, 0.2)],
    "grid_size": (3.0, 3.0),
    "start": (0.2, 0.2),
    "goal": (2.5, 2.5),
    "goal_theta_deg": None,
})

# ---- SLAM 랜드마크 품질 비교용: 같은 위치, 작은 기둥형 장애물(반지름 0.05m) ----
# 큰 원(0.2m)은 라이다가 "가까운 면"만 보게 되어 클러스터중심이 진짜중심과 계통적으로
# 어긋나는 문제가 있었음(최대 20cm) - 작은 기둥은 이 편향이 훨씬 작아서(수cm) 그래프SLAM
# 랜드마크로 훨씬 적합함.
SCENARIOS.append({
    "name": "SLAM_단일장애물_작은기둥",
    "map_source": "slam",
    "obstacles": [(1.5, 1.5, 0.05)],
    "grid_size": (3.0, 3.0),
    "start": (0.2, 0.2),
    "goal": (2.5, 2.5),
    "goal_theta_deg": None,
})

SCENARIOS.append({
    "name": "SLAM_다중장애물",
    "map_source": "slam",
    "obstacles": [(0.9, 0.9, 0.15), (2.0, 0.8, 0.15), (1.5, 1.9, 0.15)],
    "grid_size": (3.0, 3.0),
    "start": (0.2, 0.2),
    "goal": (2.8, 2.6),
    "goal_theta_deg": None,
})

SCENARIOS.append({
    "name": "SLAM_주차90도",
    "map_source": "slam",
    "obstacles": [(1.5, 1.5, 0.2)],
    "grid_size": (3.0, 3.0),
    "start": (0.2, 0.2),
    "goal": (2.5, 2.5),
    "goal_theta_deg": 90.0,
})

# ---- SLAM + 사방 벽(실제 방처럼) + 내부 장애물 여러개 ----
# 어디에 있든 벽이 라이다 사거리 안에 있어서, 스캔매칭이 참고할 구조물이 항상 존재함
_room_walls = (
    wall(0.05, 0.05, 2.95, 0.05) +   # 아래벽
    wall(0.05, 2.95, 2.95, 2.95) +   # 위벽
    wall(0.05, 0.05, 0.05, 2.95) +   # 왼벽
    wall(2.95, 0.05, 2.95, 2.95)     # 오른벽
)
SCENARIOS.append({
    "name": "SLAM_벽있는방_다중장애물",
    "map_source": "slam",
    "obstacles": _room_walls + [(1.0, 1.0, 0.15), (2.0, 0.9, 0.15), (1.5, 2.0, 0.15)],
    "grid_size": (3.0, 3.0),
    "start": (0.3, 0.3),
    "goal": (2.5, 2.5),
    "goal_theta_deg": None,
})

# ---- 같은 벽 + 90도 주차까지 ----
SCENARIOS.append({
    "name": "SLAM_벽있는방_주차90도",
    "map_source": "slam",
    "obstacles": _room_walls + [(1.0, 1.0, 0.15), (2.0, 0.9, 0.15), (1.5, 2.0, 0.15)],
    "grid_size": (3.0, 3.0),
    "start": (0.3, 0.3),
    "goal": (2.5, 2.5),
    "goal_theta_deg": 90.0,
})

# ---- SLAM + 도킹각도 + 매핑 끝난 뒤 갑자기 나타나는 예상 밖 장애물 ----
# 오늘 배운 교훈 반영: 벽으로 둘러싸고, 장애물도 3개 이상(그래프SLAM 루프클로저가 잘 걸리도록)
SCENARIOS.append({
    "name": "SLAM_도킹_예상밖장애물",
    "map_source": "slam",
    "obstacles": _room_walls + [(1.0, 1.0, 0.15), (2.0, 0.9, 0.15), (1.5, 2.0, 0.15)],
    # 매핑 중엔 없다가, navigate 시작하는 순간 경로를 가로지르는 벽처럼 나타남(재탐색 강제 유발)
    "dynamic_obstacles": [(2.0, 2.2, 0.15)],  # 매핑 중엔 없다가, navigate 시작하는 순간 나타남
    "grid_size": (3.0, 3.0),
    "start": (0.3, 0.3),
    "goal": (2.5, 2.5),
    "goal_theta_deg": 90.0,
})

# ---- 9. 고정맵, 경로 옆에 예상 밖 장애물 하나 -> 국소회피만으로 해결(재탐색 불필요) ----
SCENARIOS.append({
    "name": "고정맵_예상밖_회피만으로해결",
    "map_source": "fixed",
    "known_obstacles": [(1.5, 1.5, 0.2)],                       # 로봇이 미리 아는 것
    "obstacles": [(1.5, 1.5, 0.2), (1.9, 1.1, 0.15)],           # 실제로 라이다가 보는 것(하나 더 있음)
    "grid_size": (3.0, 3.0),
    "start": (0.2, 0.2),
    "goal": (2.5, 2.5),
    "goal_theta_deg": None,
})

# ---- 10. 고정맵, 계획된 경로를 완전히 가로막는 예상 밖 장애물 -> 갇혀서 재탐색 필요 ----
# (예전에 test_dynamic_navigator.c "완전 봉쇄 후 우회" 테스트로 이미 검증됐던 설정 그대로 사용)
SCENARIOS.append({
    "name": "고정맵_예상밖_막혀서재탐색",
    "map_source": "fixed",
    "known_obstacles": [],
    "obstacles": [(1.0, 0.9, 0.35), (1.0, 0.4, 0.35)],
    "grid_size": (3.0, 1.8),
    "start": (0.1, 0.9),
    "goal": (2.8, 0.9),
    "goal_theta_deg": None,
})


# ---- 텍스트 맵 파싱 및 새로운 시나리오 추가 ----

def load_txt_map(txt_string):
    """
    텍스트 형태의 맵을 읽어 obstacles 리스트와 grid_size를 반환합니다.
    """
    lines = txt_string.strip().split('\n')
    header = lines[0].split()
    
    rows = int(header[0])
    cols = int(header[1])
    resolution = float(header[2])
    
    obstacles = []
    
    # 텍스트 맵의 '1'을 원형 장애물(격자 중심, 반지름은 해상도의 절반)로 변환
    # shape_map.png 이미지와 텍스트 배열을 매칭하기 위해 첫 번째 줄을 y축의 상단으로 간주합니다.
    for r in range(rows):
        line = lines[r + 1].strip()
        for c in range(cols):
            if line[c] == '1':
                x = c * resolution + (resolution / 2.0)
                y = (rows - 1 - r) * resolution + (resolution / 2.0)
                radius = resolution / 2.0
                obstacles.append((x, y, radius))
                
    grid_size = (cols * resolution, rows * resolution)
    return obstacles, grid_size

# 1. 맵 데이터 정의
txt_map_data = """18 30 0.050000
111111111111111111111111111111
100000000000000000000000000001
100000000000000000000000000001
100000000000000000000000000001
100000000000000000000000000001
100000000000000000000000000001
100000000000000000000000000001
100000000000000000000000000001
100000000000000000000000000001
100000000001111111100000000001
100000000001111111100000000001
100000000001111111100000000001
100000000001111111100000000001
100000000001111111100000000001
100000000001111111100000000001
100000000001111111100000000001
100000000001111111100000000001
111111111111111111111111111111"""



# 2. 장애물 및 맵 사이즈 파싱
shape_map_obstacles, shape_grid_size = load_txt_map(txt_map_data)

# 3. 시나리오 리스트에 추가
SCENARIOS.append({
    "name": "고정맵_txt로드_shape_map",
    "map_source": "fixed", # 필요에 따라 "slam"으로 변경 가능
    "obstacles": shape_map_obstacles,
    "grid_size": shape_grid_size,  # 계산 시 (1.5, 0.9)가 됩니다.
    "start": (0.15, 0.15),         # A 지점
    "goal": (1.35, 0.15),          # B 지점
    "goal_theta_deg": 180.0,       # B 지점의 방향
})


