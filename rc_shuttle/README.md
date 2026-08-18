# 🚗 RC Shuttle — 자율주행 셔틀 로봇

> **Raspberry Pi + RPLidar C1 + FPGA 모터보드** 기반의 자율주행 셔틀 시스템  
> LiDAR 스캔매칭 위치추정 → A* 경로계획 → PID 추종 → 슬롯 정렬/주차를 수행합니다.

---

## ⚡ 빠른 시작 (Quick Start)

```bash
make          # 셔틀 본체 빌드 (main_shuttle)
make run      # 실시간 모니터링 뷰어와 함께 실행 (live_view.py)
make all      # 도구 + 단위 테스트 + 시뮬레이터(.so) 전체 빌드
make clean    # 빌드 산출물 정리
```

---

## 🔄 주행 파이프라인

```text
[RPLidar C1] ──> LiDAR Scan Frame 보정
                      │
                      ▼
[위치 추정]  ──> Scan Matching SLAM & Pose Gate 검증
                      │
                      ▼
[경로 계획]  ──> Grid Map (10mm 격자) + A* 글로벌 경로 탐색
                      │
                      ▼
[경로 추종]  ──> Dynamic Waypoint 추종 + 실시간 장애물 회피 (PID)
                      │
                      ▼
[슬롯 도킹]  ──> 슬롯 기하 측정 → 정렬(Align) → 진입 및 주차
```

---

## 📂 디렉토리 구성

| 폴더 | 설명 |
|---|---|
| `app/` | `main_shuttle.c` — CLI 파싱 및 메인 실행 루프 |
| `include/` | 모듈별 C 헤더 파일 (47개) |
| `src/` | 자율주행·SLAM·제어 핵심 C 소스 코드 (43개) |
| `viewer/` | `live_view.py` — 실시간 주행 궤적 및 맵 시각화 UI (Matplotlib) |
| `maps/` | `set_map.txt` (150×90 고정맵) 및 맵 생성기 |
| `sim/` | C 래퍼 기반 Python 오프라인 시뮬레이터 |
| `tests/` | 하드웨어 독립적인 C 단위 검증 프로그램 |
| `tools/` | 모터/엔코더/LiDAR 하드웨어 브링업 도구 |

---

## 🧩 핵심 제어 모듈 (`src/`)

| 범주 | 모듈명 | 주요 기능 |
|---|---|---|
| **위치/센서** | `robot_scan`, `robot_odom`, `robot_localize` | LiDAR 프레임 보정, 오도메트리 누적, 스캔매칭 위치추정 |
| **판단/보안** | `robot_pose_gate`, `robot_sense`, `robot_escape` | 추정 위치 신뢰도 검증, 차체 여유 감지, 끼임/탈출 기동 |
| **경로/추종** | `astar`, `planning_pipeline`, `path_follow`, `pid` | A* 경로 탐색, 웨이포인트 스무딩, 선/각속도 PID 추종 |
| **슬롯 주차** | `robot_slot_geom`, `robot_slot_align`, `robot_slot_drive` | 슬롯 감지, 진입 전 정렬, 정밀 밀착 주차 |
| **하드웨어** | `rplidar_c1`, `fpga_serial`, `trigger` | LiDAR 드라이버, FPGA 모터 UART 통신, GPIO 트리거 |
