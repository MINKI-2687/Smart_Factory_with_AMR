# RC 셔틀 — 고정맵 주행 + 슬롯 주차

라즈베리파이 + RPLidar C1 + FPGA 모터보드로 두 주차슬롯(A/B) 사이를 왕복하는 셔틀 로봇.
LiDAR 스캔매칭 위치추정 → A\* 경로계획 → PID 추종 → 슬롯 정렬/진입/착석.

```
make          # main_shuttle 빌드
make run      # live_view.py 로 실기 실행
make all      # 도구 + 테스트 + 시뮬용 .so 까지
```

---

## 디렉터리

| 경로 | 내용 |
|---|---|
| `app/` | `main_shuttle.c` — CLI 파싱(71개 옵션) → `RobotConfig` → `run_shuttle()` |
| `include/` | 헤더 47개. **선언만** 들어 있다 |
| `src/` | 구현 43개. 실행 경로의 전부 |
| `viewer/` | `live_view.py` — main_shuttle 의 stdout(MAP/TARGET/WAYPOINTS/STATE/SCAN)을 파싱해 matplotlib 로 표시. 프로젝트 모듈을 import 하지 않는 완전 독립 뷰어 |
| `maps/` | `set_map.txt`(150×90, 10mm 격자)와 그걸 만드는 `make_slot_map.py` |
| `sim/` | 파이썬 오프라인 시뮬레이션 스택. ctypes 래퍼 + `*_capi.c` + GUI 시뮬 8개. 실기 실행에는 안 쓰임 |
| `tests/` | 실기 없이 도는 C 검증 프로그램 7개 + 파이썬 스모크 테스트 |
| `tools/` | 하드웨어 브링업 도구 (`motor_test`, `encoder_cal`, `lidar_dump`, `wander_avoid`) |
| `attic/` | 구버전·사본·제거한 죽은 코드. 빌드에 안 들어감 |

---

## 실행 경로 모듈 (`src/`)

`main_shuttle.c` → `run_shuttle()` 한 줄기다.

**로봇 제어 (예전 `robot_runner.h` 8,375줄을 쪼갠 것)**

| 모듈 | 줄수 | 하는 일 |
|---|---|---|
| `robot_config` | 136 | `RobotConfig` 구조체 + 기본값 + SIGINT 핸들러 |
| `robot_scan` | 83 | 라이다 프레임 획득·좌우반전·장착각·오프셋 보정 |
| `robot_odom` | 302 | 오도메트리 델타, 이동 누적, 예측, 명령↔오도 대조 |
| `robot_rotrate` | 207 | 회전율 실측 추정기(`RotRateEst`) |
| `robot_motion` | 108 | 속도 모델, 선속도 배율 학습, 이동시간 계산 |
| `robot_sense` | 136 | 섹터 최소거리, 차체 여유, 회전 안전성, 차선 오차 |
| `robot_localize` | 106 | 전역 위치추정(후보 스윕) |
| `robot_relocalize` | 141 | 지도 일치율 판정, 넓게/전역 재위치추정 |
| `robot_pose_gate` | 131 | "이 자세를 믿어도 되나" 게이트, 이동 후 재정렬 |
| `robot_escape` | 185 | 밀어내기 탈출, 열린 방향으로 탈출 |
| `corner_escape` | 395 | 구석 끼임 탈출 (축 스윕 여유 계산 + 원호 기동) |
| `robot_slot_geom` | 483 | 슬롯 기하 측정/식별/재정박 |
| `robot_slot_align` | 1,258 | 슬롯 진입 전 각도·좌우 정렬 |
| `robot_slot_drive` | 1,240 | 슬롯 진입/밀착/착석 판정 |
| `robot_navigate` | 1,160 | 한 구간 주행 (A\* → 추종 → 회피) |
| `robot_map` | 207 | 지도 획득 (load / fixed / SLAM 매핑) |
| `robot_shuttle` | 734 | A↔B 왕복 메인 루프 |

**하위 모듈** — `slam`(스캔매칭·점유격자), `dynamic_navigator`, `astar`, `min_heap`, `path`,
`smoothing`, `planning_pipeline`, `path_follow`, `docking`, `goal_pid`, `pid`,
`safe_navigator`, `obstacle_avoidance`, `obstacle_list`, `grid_map`, `map_io`,
`lidar_clustering`, `lidar_thread`, `rplidar_c1`, `rplidar_reader`,
`fpga_serial`, `fpga_link`, `serial_port`, `trigger`, `angle_utils`, `frontier_exploration`

---

## 구조에서 알아둘 것

**헤더 온리가 아니다.** 예전에는 전부 `static inline` 이라 헤더 한 줄만 고쳐도 전체가
다시 컴파일됐고, 심지어 `corner_escape.h` 는 `robot_runner.h` **한복판(2116줄)에서**
include 돼야만 컴파일되는 상태였다. 지금은 선언(`include/`)과 정의(`src/`)가 분리돼 있어
include 순서에 의존하지 않는다.

**`_POSIX_C_SOURCE` 는 `CFLAGS` 에 있다.** 예전엔 `robot_runner.h` 맨 위에서 정의했다.
각 `.c` 가 독립 컴파일되는 지금은 그 방식이 통하지 않는다.

**`-flto` 를 쓴다.** `static inline` 이 사라지면서 컴파일러가 하던 인라인이 끊긴다.
제어 루프라 이게 그냥 손해라서 LTO 로 되살린다. 문제가 생기면 `make LTO=` 로 끄면 된다.

**`include/robot_runner.h` 는 우산 헤더다.** 옛 include 한 줄이 그대로 동작하게 남겨뒀다.
새로 쓰는 코드는 필요한 모듈만 직접 include 하는 게 낫다.

**모듈 간 순환 의존은 없다.** (`corner_escape` ↔ `robot_escape` 순환은
`robot_runner__escape_speed` 를 `robot_motion` 으로 옮겨 끊었다.)

---

---

## 거대 함수 분해 (2026-08-11)

원래 네 함수가 3,800줄을 차지했다. 전부 쪼갰다.

| 함수 | 이전 | 이후(본체) | 헬퍼 | 접두어 |
|---|---|---|---|---|
| `slot_drive` | 1,139 | 195 | 9 | `slot_drive__` |
| `navigate_one_leg_inner` | 1,147 | 159 | 10 | `nav__` |
| `slot_align_inner` | 753 | 54 | 8 | `align__` |
| `run_shuttle` | 734 | 171 | 9 | `shuttle__` |

방식은 넷 다 같다. **로직과 주석은 원본 줄을 그대로 옮겼고**, 지역변수만 구조체로 묶었다.

- **Ctx** — 한 번 정하면 안 바뀌는 값 (자원 포인터, 설정, 목표)
- **State** — 스텝을 넘어 사는 값 (카운터, 누적, 직전 상태)
- **Step** — 이번 스텝 관측 (스캔, 자세, 파생값)

제어흐름은 enum 으로 돌려준다. `RETRY` 가 원본의 `continue`, `FAIL`/`ABORT` 가
`return false` 또는 `clean_stop = false; break` 자리다. 즉 헬퍼가 무엇을 돌려주면
원본의 어디에 해당하는지가 1:1 로 대응한다.

**`run_shuttle` 만 예외가 하나 있다.** 원본에 `goto shuttle_arrived` 와
`goto shuttle_depart` 가 있었는데, 둘 다 앞으로만 뛰고 건너뛰는 구간이 한 덩어리라
`if (!skip_travel) { ... }` / `if (!depart_now) { ... }` 로 바꿨다. 다른 부분과 달리
이건 줄을 옮긴 게 아니라 제어흐름을 다시 쓴 것이므로, 동작이 의심되면 여기부터 볼 것.

지금 200줄 넘는 함수는 7개뿐이고, 300줄 넘는 건 `main()`(CLI 파싱 71개 옵션) 하나다.

---

## 남은 숙제

1. **`main()` 399줄 / `print_usage()` 177줄** — CLI 파싱이라 급하진 않지만
   옵션 그룹별로 나눌 수는 있다.
2. **`sim/` 의 GUI 시뮬 8개**가 서로 사본 관계다 (`test_fixed_map*.py` 계열).
   실기 검증에 안 쓰이므로 급하진 않지만 언젠가 하나로 합칠 것.
3. **`--map-source slam` 경로**(`robot_map` 의 매핑 단계 + `frontier_exploration`)는
   빌드에는 들어가지만 지금 실행 커맨드로는 절대 안 탄다. 계속 쓸 계획이 아니면 뺄 수 있다.
4. **실기 검증이 아직 안 됐다.** 이 리팩토링의 회귀 테스트는 CLI 파싱과 설정 해석까지만
   태운다. 분해한 네 함수의 내부 로직은 하드웨어가 없어 한 줄도 실행해보지 못했다.
   원본과 토큰 단위로 대조해 코드가 안 바뀐 것은 확인했지만, 그건 '동작이 맞다'는
   증명이 아니다. **실기에 올리기 전에 `tests/` 나 `sim/test_slot_map.py` 로 폐루프를
   한 번 돌려볼 것.**
