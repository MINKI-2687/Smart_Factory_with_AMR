"""
lidar_clustering.py
라이다 원시 스캔([(angle_deg, dist_mm), ...])을 "장애물 덩어리"들로 묶어서,
DynamicNavigator.update()의 detected_obstacles 인자([[(x,y),...], ...] 형태)로
바로 넣을 수 있게 변환하는 모듈.

원리 (각도순 순차 클러스터링):
  라이다 스캔은 원래 각도순으로 들어오므로, "각도상 이웃한 두 점이 실제로도
  가까우면(예: 15cm 이내) 같은 물체", "갑자기 멀어지면 다른 물체(또는 빈 공간)"
  라고 보고 순서대로 묶어나감. 단순하지만 볼록한 형태의 장애물(기둥, 상자, 벽 등)
  에는 실전에서 충분히 잘 먹히는 방식.

  360도를 한 바퀴 다 도는 스캔이라, 마지막 클러스터와 첫 클러스터가 실제로는
  이어진 하나의 물체일 수 있음(0도/360도 경계에 걸친 경우) - 이것도 병합 처리함.
"""
import math


def cluster_lidar_scan(scan, robot_x, robot_y, robot_theta,
                        cluster_distance_threshold=0.15,
                        min_cluster_points=2,
                        max_range_m=3.0):
    """
    scan: [(angle_deg, dist_mm), ...] 로봇 기준 상대좌표 라이다 스캔
    robot_x, robot_y, robot_theta: 이 스캔을 찍은 시점의 로봇 위치/방향 (world 좌표계 변환용)
    cluster_distance_threshold: 이웃한 두 점을 "같은 물체"로 볼 최대거리(m)
    min_cluster_points: 이보다 점이 적은 클러스터는 노이즈로 보고 버림
    max_range_m: 이보다 먼 점은 아예 무시(너무 먼 벽 등은 재탐색 대상에서 제외)

    반환: [[(x1,y1),(x2,y2),...], ...] world 좌표 클러스터 목록
          (DynamicNavigator.update()의 detected_obstacles 인자에 그대로 넣으면 됨)
    """
    world_points = []
    for angle_deg, dist_mm in scan:
        if dist_mm is None or dist_mm <= 0:
            continue
        dist_m = dist_mm / 1000.0
        if dist_m > max_range_m:
            continue
        world_angle = robot_theta + math.radians(angle_deg)
        wx = robot_x + dist_m * math.cos(world_angle)
        wy = robot_y + dist_m * math.sin(world_angle)
        world_points.append((wx, wy))

    if not world_points:
        return []

    clusters = []
    current = [world_points[0]]
    for i in range(1, len(world_points)):
        px, py = world_points[i - 1]
        cx, cy = world_points[i]
        if math.hypot(cx - px, cy - py) <= cluster_distance_threshold:
            current.append(world_points[i])
        else:
            if len(current) >= min_cluster_points:
                clusters.append(current)
            current = [world_points[i]]
    if len(current) >= min_cluster_points:
        clusters.append(current)

    if len(clusters) >= 2:
        fx, fy = clusters[0][0]
        lx, ly = clusters[-1][-1]
        if math.hypot(fx - lx, fy - ly) <= cluster_distance_threshold:
            clusters[0] = clusters[-1] + clusters[0]
            clusters.pop()

    return clusters
