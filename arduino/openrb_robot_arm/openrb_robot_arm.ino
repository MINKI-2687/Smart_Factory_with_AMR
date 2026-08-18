/*
 * 로봇팔1 좌표 집기 + 로봇팔2 상자 파지/빈 슬롯 적재 통합 동작
 * Jetson UART 9바이트(X,Y,빈 슬롯) -> Init/Home -> 좌표 보정 자세에서 상자 집기
 * -> 정면 복귀 -> 수신 슬롯(1~9) 적재 -> Init/Home
 * 기준 코드: 로봇팔2 왼쪽 8점 실측 삼각형 보간 + 기존 상자 파지/슬롯 적재
 *
 * [수정 내역 - 동작 부자연스러움 / 적재 부정확 개선]
 *  1. 오타 수정: if (ok) -> if (ok)  (컴파일 오류였음)
 *  2. 도달 판정에 "연속 안정(settle)" 개념 추가 (waitForArmTol)
 *     - 허용치 안에 들어온 첫 순간 바로 통과하던 것을,
 *       SETTLE_STABLE_COUNT회 연속으로 안정될 때까지 대기하도록 변경
 *  3. 적재 전용 정밀 허용치(PLACE_TOLERANCE) / 경유점용 느슨한 허용치(WAYPOINT_TOLERANCE) 분리
 *  4. 적재 구간 전용 저속 프로파일(PLACE_SPEED / PLACE_ACCEL) 추가
 *     - setSpeed()만 있고 가속도는 고정이던 것을 setProfile()로 함께 제어
 *  5. 큰 관절 이동(안전자세 <-> 슬롯 진입/후퇴)을 관절공간 선형보간(moveSmoothAndWait)으로
 *     여러 스텝에 나눠 이동 -> 관절별 이동량 차이로 인한 팔 끝 휘어짐/튐 완화
 *  6. 적재 진입([8])은 반드시 최저속(PLACE_SPEED)으로 동작하도록 변경 (기존 MOVE_SPEED였음)
 *  7. 정지 후 짧은 대기(serviceDelay)를 추가해 진동 감쇠 후 놓기 동작 수행
 * ============================================================
 *  OpenRB-150 - OMX-F 로봇팔 제어 + Jetson 9바이트 좌표 수신
 *
 *  포트 구성:
 *    Serial  : USB 디버그 (115200)
 *    Serial1 : Dynamixel TTL (1Mbps, 내장 자동 방향전환)
 *    Serial3 : Jetson UART 수신/응답 (115200, D13=RX, D14=TX)
 *
 *  Jetson -> OpenRB TARGET 패킷 (9 Bytes):
 *    [0] 0xAA       Header 0
 *    [1] 0x55       Header 1
 *    [2] Command ID 명령 식별 번호
 *    [3] Slot       빈 슬롯 번호(1~9)
 *    [4] X High
 *    [5] X Low
 *    [6] Y High
 *    [7] Y Low
 *    [8] CRC8       Byte[2]~Byte[7] 6바이트 CRC
 *
 *  OpenRB -> Jetson 응답 패킷 (5 Bytes):
 *    [0] 0xAA  [1] 0x55  [2] Command ID  [3] Status  [4] CRC8
 *    Status: 0x06 ACK, 0x07 DONE, 0x15 NAK, 0xE0 BUSY, 0xE1 FAULT
 *
 *  핀 연결:
 *    Jetson USB-UART TX → OpenRB D13 (Serial3 RX)
 *    Jetson USB-UART RX ← OpenRB D14 (Serial3 TX)
 *    Jetson GND         ↔ OpenRB GND
 *    USB-UART 전원선은 연결하지 않는다.
 * ============================================================ */
#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial1
#define DEBUG_SERIAL Serial
#define JETSON_SERIAL Serial3   // OpenRB D13=RX, D14=TX

Dynamixel2Arduino dxl(DXL_SERIAL, -1);
const char FIRMWARE_VERSION[] = "R2-FIXED-PICK-J4-REVERSE3060-V34";


/* ************************************************************
 *  1. OMX-F 하드웨어 상수
 * ************************************************************ */

const uint8_t ID_J1   = 11;
const uint8_t ID_J2   = 12;
const uint8_t ID_J3   = 13;
const uint8_t ID_J4   = 14;
const uint8_t ID_J5   = 15;
const uint8_t ID_GRIP = 16;

const uint8_t SERVO_IDS[] = {ID_J1, ID_J2, ID_J3, ID_J4, ID_J5, ID_GRIP};
const uint8_t SERVO_COUNT = 6;
const uint8_t ARM_SERVO_COUNT = 5;  // ID11~ID15만 팔 프로파일 적용

const int MOVE_SPEED = 45;
const int MOVE_ACCEL = 15;
const int SLOW_SPEED = 15;

// [수정 4] 적재 구간 전용 저속/저가속 프로파일
const int PLACE_SPEED = 12;   // 적재 진입/후퇴 전용 (SLOW_SPEED보다 느리게)
const int PLACE_ACCEL = 5;    // 부드러운 가감속

// 선택 칸 높이 설정 전용 프로파일. 최종 삽입은 기존 PLACE_SPEED를 유지한다.
const int HEIGHT_SPEED = 20;
const int HEIGHT_ACCEL = 8;

// 그리퍼는 팔의 적재용 저속 프로파일과 분리한다.
// 집기 준비 2500, 실제 파지 2056, 적재 후 놓기 2300을 구분한다.
const int GRIP_SPEED = 40;
const int GRIP_ACCEL = 15;

const int LIM_MIN[5] = {   0,  683,  683,  910,    0 };
// J4 바깥쪽 방향을 감소 방향으로 다시 확인했으므로 확장했던 상한을
// 원래의 안전값 3210으로 복원한다.
const int LIM_MAX[5] = {4095, 3072, 3100, 3210, 4095 };

const int GRIP_PICK_PREP     = 2500;
const int GRIP_CLOSE         = 2056;
const int GRIP_PLACE_RELEASE = 2300;
const int GRIP_NEUTRAL       = 2048;
// 성공 위치에 도달한 뒤에도 열림 목표를 유지해 동작이 끝날 시간을 준다.
const unsigned long GRIP_FULL_OPEN_HOLD_MS = 1200UL;
const uint8_t GRIP_OPEN_STABLE_COUNT = 3;
const uint8_t GRIP_OPEN_MAX_ATTEMPTS = 2;
const unsigned long GRIP_OPEN_RETRY_DELAY_MS = 300UL;
// 준비 단계에서는 2500 도달을 40tick으로 확인하고, 상자에 도달한 뒤에는
// 2056으로 닫는 실제 파지 동작을 실행한다. 물체 접촉을 고려해 160tick 허용한다.
const int GRIP_PICK_PREP_TOLERANCE     = 40;
const int GRIP_CLOSE_TOLERANCE         = 160;
const int GRIP_PLACE_RELEASE_TOLERANCE = 40;

const int J5_NEUTRAL   = 2048;

// Init/Home 공통 자세: J1을 정면(2048)으로 맞추고 J2~J5는 기존값 유지
const int POSE_INIT[5] = {2048, 747, 3079, 3194, 2023};
const int POSE_HOME[5] = {2048, 747, 3079, 3194, 2023};

// Home 직후 첫 동작의 사진 실측값 (ID11~ID15 / ID16)
const int POSE_AFTER_HOME[5] = {2021, 1708, 2065, 3196, 2027};
const int GRIP_AFTER_HOME    = 2056;

// 로봇팔2에서 사용하던 왼쪽 집기 방향이다.
const int PICK_LEFT_BASE = 3072;


/* ************************************************************
 *  2. 역기구학 (Inverse Kinematics) - OMX 스펙 기반
 *
 *  OpenManipulator-X (RM-X52-TNM) 링크 길이:
 *    L1 = 77.0 mm  (베이스 → J2 회전축 높이)
 *    L2 = 128.0 mm (J2 → J3, 상완)
 *    L3 = 124.0 mm (J3 → J4, 전완)
 *    L4 = 126.0 mm (J4 → 그리퍼 끝)
 *
 *  카메라 설정 (위에서 아래를 내려다보는 Top-Down 뷰):
 *    - 해상도: 1920x1080 (Zybo Pcam)
 *    - 카메라 영상 중심 = 로봇팔 베이스 바로 위 (조정 가능)
 *    - 픽셀 좌표는 2x2 보정 행렬로 로봇 XY(mm) 좌표로 변환
 *
 *  XM430-W350 서보 1 tick = 360° / 4096 ≈ 0.088°
 *  중립(2048 tick) = 0° (정면/수직)
 * ************************************************************ */

#include <math.h>

/* --- OMX 링크 길이 (mm, ROBOTIS 공식 스펙) --- */
#define OMX_L1   77.0    // 베이스 높이 (J1 → J2 수직 오프셋)
#define OMX_L2  128.0    // 상완 (J2 → J3)
#define OMX_L3  124.0    // 전완 (J3 → J4)
#define OMX_L4  126.0    // 손목 → 그리퍼 끝

/* --- 카메라 → 실제 좌표 변환 파라미터 (★ 실측 후 조정 필요) --- */
#define CAM_W       1920       // Pcam 해상도 가로
#define CAM_H       1080       // Pcam 해상도 세로

// 카메라는 1920x1080이지만 로봇팔2의 왼쪽 집기 작업영역은
// 사진에서 새로 측정한 화면 좌측 절반 X=0~960, Y=0~1080이다.
// UART 좌표는 원본 Full HD 픽셀 좌표를 그대로 받는다.
#define AREA_GRID_COLS       16
#define AREA_GRID_ROWS       18
#define AREA_CELL_W_PX       60
#define AREA_CELL_H_PX      (CAM_H / AREA_GRID_ROWS)
#define AREA_MID_PERCENT     70

constexpr float AREA_FORWARD_START_CM = 11.5f;
constexpr float AREA_REAL_WIDTH_CM     = 16.0f;
constexpr float AREA_LATERAL_TOP_CM    =  4.5f;
constexpr float AREA_REAL_HEIGHT_CM    =  9.0f;

// 로봇팔 베이스가 카메라 영상에서 위치하는 픽셀 좌표 (★ 실측 조정)
#define BASE_PX     960.0      // 영상 중심 X (로봇팔 바로 위)
#define BASE_PY     540.0      // 영상 중심 Y (로봇팔 바로 위)

// 픽셀 차이 → 로봇 XY(mm) 2x2 보정 행렬.
// 기본값은 기존의 0.5 mm/px 변환과 동일하다.
// 카메라가 회전되어 있거나 X/Y 축 배율이 다르면 실측값으로 교체한다.
#define CAM_TO_ROBOT_A11   0.5
#define CAM_TO_ROBOT_A12   0.0
#define CAM_TO_ROBOT_A21   0.0
#define CAM_TO_ROBOT_A22  -0.5

// 동일한 X,Y에서 Z만 바꾸어 수직으로 접근한다. 모두 실측 조정 필요.
#define PICK_Z_MM       30.0   // 물체를 잡는 높이
#define PICK_MID_Z_MM   60.0   // 중간 하강 높이
#define PICK_HIGH_Z_MM  90.0   // 접근/이탈 높이

// 목표 위치 도달 판정
#define GRIP_TOLERANCE_TICK  40
#define ARM_MOVE_TIMEOUT_MS   10000UL
#define HOME_MOVE_TIMEOUT_MS  20000UL
#define GRIP_MOVE_TIMEOUT_MS  8000UL

// [수정 2/3] 적재 전용 타임아웃 및 연속 안정 판정 횟수
#define PLACE_TIMEOUT_MS      20000UL
#define SETTLE_STABLE_COUNT   4        // 연속 4회(약 80ms) 안정 시 도달 인정
#define WITHDRAW_RETRY_TIMEOUT_MS          30000UL
#define WITHDRAW_RETRY_WAYPOINT_TIMEOUT_MS 12000UL

// 7번은 위층 왼쪽 칸에서 열린 그리퍼의 하중 변화와 측벽 간섭 때문에
// 후퇴 판정이 자주 실패하므로 더 잘게 나누고 충분한 시간을 준다.
#define SLOT7_WITHDRAW_STEPS                10
#define SLOT7_WITHDRAW_RETRY_STEPS          14
#define SLOT7_WITHDRAW_TIMEOUT_MS            35000UL
#define SLOT7_WITHDRAW_RETRY_TIMEOUT_MS      45000UL
#define SLOT7_WITHDRAW_WAYPOINT_TIMEOUT_MS   12000UL

// 각 칸의 실측 놓기 자세에서 같은 높이로 최대한 후퇴시키는 변화량.
// 진입 전과 놓은 후에 동일한 후퇴 자세를 사용하여 같은 경로로 왕복한다.
#define PLACE_RETREAT_J2_TICK   390
#define PLACE_RETREAT_J3_TICK   390
#define PLACE_RETREAT_J4_TICK  -780

// 바닥 충돌 방지용 J4 후퇴 하한.
// 기존 1번 계산값 J4=915는 관절 최솟값 910에 너무 가까웠다.
#define PLACE_RETREAT_J4_SAFE_MIN 1600

// 기존 V11에서 1번에 적용했던 깊이 보정과, 측면 칸에 새로 추가하는 보정량.
// J2를 전진 방향으로 증가시키고 J4를 같은 양 감소시켜 손목 수평합을 유지한다.
#define SLOT1_PREVIOUS_EXTRA_DEPTH_TICK 30
#define SIDE_SLOT_ADDITIONAL_DEPTH_TICK 30

// 1~9번 모든 칸에서 빈칸 진입부터 그리퍼를 열 때까지 높이를 30tick 올린다.
// V17의 20tick보다 10tick 더 높게 설정한다.
// J3를 감소시키고 J4를 같은 양 증가시켜 손목 수평합은 유지한다.
#define SLOT_PLACE_RAISE_J3_TICK 30

// 놓은 뒤 후퇴할 때 위쪽 칸일수록 J3를 큰 tick 방향으로 움직여
// 그리퍼를 조금 낮춘 상태로 빠지게 하여 칸 천장과의 접촉을 방지한다.
#define WITHDRAW_BOTTOM_J3_DOWN_TICK 20
#define WITHDRAW_MIDDLE_J3_DOWN_TICK 40
#define WITHDRAW_TOP_J3_DOWN_TICK    80

// 정면 복귀 중 J2를 작은 tick 방향으로 이동시켜 앉은 자세를 만든다.
#define J2_SEATED_PERCENT 50

// 기존 일반 이동용 허용치 (경유점이 없는 단순 이동에 계속 사용)
const int JOINT_TOLERANCE[5] = {
  20,
  60,  // J2: 상승/수납함 접근 시 중력 부하를 고려
  60,
  90,
  20
};

// [수정 3] 적재 전용 정밀 허용치 (도달 판정을 더 엄격하게)
const int PLACE_TOLERANCE[5] = { 12, 40, 40, 45, 12 };

// 물체가 이미 칸 안에 들어간 최종 놓기 위치 전용 허용치.
// 엄격 판정 실패 뒤 실측 자세를 직접 재명령하고 이 범위에 들어오면
// 안전하게 그리퍼 열기까지 진행한다. 일반 이동 판정에는 사용하지 않는다.
const int INSERTION_RELEASE_TOLERANCE[5] = { 20, 50, 50, 100, 20 };

// 수평 보정 자세는 J4에 하중이 크게 걸리므로 별도 허용치를 사용한다.
// J4 오차를 너무 크게 허용하면 아래로 기울어진 상태로 칸에 진입하므로
// 수평이 맞지 않을 때는 다음 단계로 진행하지 않도록 제한한다.
const int HORIZONTAL_TOLERANCE[5] = { 12, 40, 40, 50, 20 };

// 보간 중에도 J4가 크게 뒤처지면 손목이 아래로 기울기 때문에
// J4 경유점 허용치를 60tick으로 제한한다.
const int WAYPOINT_TOLERANCE[5] = { 60, 100, 100, 60, 60 };

// 선반에서 멀리 떨어진 정면 손목 보정 구간 전용 경유점 허용치.
// 중간 자세는 조금 느슨하게 통과하되, 최종 자세는 위의
// HORIZONTAL_TOLERANCE로 다시 엄격하게 확인한다.
const int FRONT_LEVEL_WAYPOINT_TOLERANCE[5] = {
  60, 100, 100, 110, 60
};

// 놓기 후 Home 복귀는 저속에서 관절 이동량이 크므로 일반 경유점보다
// 충분한 허용시간을 주고, POSE_AFTER_HOME을 중간 안전 자세로 사용한다.
const int HOME_RETURN_WAYPOINT_TOLERANCE[5] = {
  80, 120, 120, 130, 80
};

// 물체를 놓은 뒤에는 정밀 삽입 오차보다 선반 밖으로 안전하게 빠지는 것이
// 우선이다. 7번 후퇴에서만 J2~J4의 부하 편차를 추가 허용한다.
const int SLOT7_WITHDRAW_TOLERANCE[5] = {
  30, 90, 90, 140, 30
};
const int SLOT7_WITHDRAW_WAYPOINT_TOLERANCE[5] = {
  80, 140, 140, 160, 80
};
#define HOME_RETURN_WAYPOINT_TIMEOUT_MS 12000UL
#define HOME_RETURN_STAGE_TIMEOUT_MS    30000UL

// 물체를 든 상태에서 J4 실제 위치가 명령보다 약 +90tick에서 멈추는 현상을
// 1~9번 모든 칸의 후퇴 수평/높이 설정에 동일하게 보상한다.
// 명령값만 낮추고 성공 판정은 원래 수평 목표로 한다.
#define J4_HORIZONTAL_LOAD_COMP_TICK 90
#define J4_LEVEL_TIMEOUT_MS 30000UL

// 7~9번 위층은 고정 90tick 보상이 과도하므로 실측 재보정값에 맞춰 줄인다.
#define J4_TOP_ROW_COMP_TICK 20
#define HEIGHT_WAYPOINT_TIMEOUT_MS 2500UL
#define HEIGHT_FINAL_TIMEOUT_MS    10000UL

/* --- Dynamixel XM430 상수 --- */
#define DXL_TICK_PER_DEG  (4096.0 / 360.0)   // ≈ 11.378 tick/도
#define DXL_CENTER        2048                 // 0도 = 2048 tick

/* --- 도/라디안 변환 --- */
#define DEG2RAD(d)  ((d) * M_PI / 180.0)
#define RAD2DEG(r)  ((r) * 180.0 / M_PI)

struct JointTicks {
  int j1, j2, j3, j4, j5;
};

// 사진으로 확인한 고정 집기 자세. true이면 UART X/Y 대신 이 자세로 집고,
// UART의 슬롯 번호만 기존 1~9번 적재 동작에 사용한다.
// false로 바꾸면 아래의 8점 좌표 보간 방식으로 즉시 복귀할 수 있다.
constexpr bool USE_FIXED_PICK_POSE = true;
// J3는 높이를 올린 1395를 유지한다. 확인된 J4 감소 방향으로 3120에서
// 60tick을 더 낮춘 3060을 사용한다. 최초 기준 3210보다 총 150tick
// (약 13.2도) 바깥쪽·위쪽으로 회전한 실사용 최대 보정값이다.
// J3의 기계 한계 683까지 한 번에 이동하면 집게 방향이 크게 바뀌고
// 바닥/프레임 충돌 위험이 있으므로 실제 집기가 가능한 큰 보정값으로 제한한다.
const JointTicks FIXED_PICK_POSE = {
  3020, 2500, 1395, 3060, 1882
};
constexpr int FIXED_PICK_GRIP_REFERENCE = 2500;

/* 로봇팔2가 왼쪽을 바라본 상태에서 측정한 좌표별 보정값.
 * 외곽 7점과 실제 수신 중심 (542,513)을 중심으로 삼각형 보간한다.
 * arm은 ID11~ID15의 집기 자세이고 gripTick은 당시 ID16 참고값이다.
 * 단, ID15(J5)는 집게 회전 방지를 위해 실측값 대신 2048로 고정한다.
 * 집기 준비는 GRIP_PICK_PREP(2500), 실제 파지는 GRIP_CLOSE(2056)를 사용한다.
 */
struct AreaPickCalibration {
  JointTicks arm;
  int gripTick;
};

struct AreaPickPoint {
  float x;
  float y;
  AreaPickCalibration calibration;
};

// 화면 외곽을 시계방향으로 배치한다. 연속한 두 외곽점과 중심점이
// 하나의 삼각형을 만들며 총 7개 구역으로 전체 작업영역을 덮는다.
constexpr uint8_t AREA_PICK_BOUNDARY_COUNT = 7;
const AreaPickPoint AREA_PICK_BOUNDARY[AREA_PICK_BOUNDARY_COUNT] = {
  {  0.0f,    0.0f, {{3290, 1825, 2582, 2646, 2147}, 2052}},
  {480.0f,    0.0f, {{3265, 2269, 2102, 2823, 2143}, 2071}},
  {960.0f,    0.0f, {{3209, 2450, 1778, 2748, 2185}, 2051}},
  {960.0f,  540.0f, {{3033, 2501, 1681, 2911, 1972}, 2058}},
  {960.0f, 1080.0f, {{2851, 2504, 1720, 2762, 1962}, 2052}},
  {480.0f, 1080.0f, {{2788, 2315, 2008, 2801, 2001}, 2097}},
  {  0.0f, 1080.0f, {{2669, 1969, 2460, 2739, 1932}, 2052}}
};

// 실제로 수신된 상자 중심 (542,513)에서 상자 중앙을 정확히 잡은 실측 자세.
// J5=2082는 측정 당시 값이지만 집게 회전 방지를 위해 실제 명령은 2048을 유지한다.
// ID16=2205는 8점 보간 모드의 참고값이며 실제 파지는 GRIP_CLOSE를 사용한다.
const AreaPickPoint AREA_PICK_CENTER = {
  542.0f, 513.0f, {{3138, 1982, 2492, 2582, 2082}, 2205}
};

constexpr float AREA_CAL_X_MAX = 960.0f;
constexpr float AREA_CAL_Y_MAX = 1080.0f;

// 새 8점은 각 좌표에서 직접 측정한 최종 자세이므로 추가 좌표 보정은 0이다.
constexpr int PICK_X_CORRECTION_PX = 0;

// (618,791)에서 집게가 바닥에 닿고 상자 아래를 잡는 현상을 보정한다.
// 해당 좌표에서는 40tick 전부 적용하고, 주변 좌표에서는 선형으로 감소시킨다.
constexpr int PICK_LOCAL_LIFT_CENTER_X_PX = 618;
constexpr int PICK_LOCAL_LIFT_CENTER_Y_PX = 791;
constexpr int PICK_LOCAL_LIFT_RADIUS_X_PX = 200;
constexpr int PICK_LOCAL_LIFT_RADIUS_Y_PX = 240;
constexpr int PICK_LOCAL_LIFT_TICK = 40;

// Y=513에서 약 3cm 위를 잡은 문제는 임의의 +360px 이동 대신, 해당 좌표에
// 정확한 중앙 파지 실측 자세를 배치해 보정한다. 그래야 네 모서리 값이 유지된다.
constexpr int PICK_Y_CORRECTION_PX = 0;

/*
 * 3x3 수납함 번호
 *
 *   ┌─────┬─────┬─────┐
 *   │  7  │  8  │  9  │  위층
 *   ├─────┼─────┼─────┤
 *   │  4  │  5  │  6  │  가운데층
 *   ├─────┼─────┼─────┤
 *   │  1  │  2  │  3  │  아래층
 *   └─────┴─────┴─────┘
 *
 * measuredPose는 사진에서 status 명령으로 측정한 ID11~ID15 실측값이다.
 * measuredGripTick은 당시 ID16 참고값이며, 실제 놓기 때는 기존처럼
 * 집기 준비는 GRIP_PICK_PREP, 파지는 GRIP_CLOSE, 놓기는 GRIP_PLACE_RELEASE로 제어한다.
 */
struct PlaceSlotPose {
  JointTicks measuredPose;
  int measuredGripTick;
};

// 7번 칸 J1=2254에서 오른쪽(작은 J1 방향)으로 5tick만 더 이동한다.
// 원래 실측 J1=2264를 기준으로 총 15tick 보정하므로 최종 J1은 2249이다.
constexpr int SLOT7_RIGHT_SHIFT_J1_TICK = 15;

const PlaceSlotPose PLACE_SLOTS[9] = {
  // 아래층: 1(왼쪽), 2(가운데), 3(오른쪽)
  {{2283, 1808, 2539, 1695, 2011}, 2026},
  {{2062, 1770, 2572, 1698, 2012}, 2026},
  {{1807, 1893, 2450, 1709, 2014}, 2026},
  // slot 4: 왼쪽 가운데
  {{2264, 1803, 2283, 1966, 2060}, 2026},
  // slot 5: 가운데
  {{2052, 1747, 2340, 1980, 2015}, 2027},
  // slot 6: 오른쪽 가운데
  {{1840, 1830, 2245, 1968, 2014}, 2027},
  // 위층: 7(왼쪽), 8(가운데), 9(오른쪽)
  {{2264 - SLOT7_RIGHT_SHIFT_J1_TICK, 1902, 1875, 2288, 1977}, 2028},
  {{2044, 1848, 1985, 2202, 1968}, 2027},
  {{1821, 1911, 1895, 2235, 1997}, 2026}
};

// 수동 claw/test 명령의 기본 놓기 위치. 자동 동작은 UART의 빈칸값을 사용한다.
uint8_t selectedPlaceSlot = 4;


/* ************************************************************
 *  3. 유틸리티 함수 + 역기구학 계산
 * ************************************************************ */

int clampJoint(uint8_t idx, int tick) {
  if (tick < LIM_MIN[idx]) return LIM_MIN[idx];
  if (tick > LIM_MAX[idx]) return LIM_MAX[idx];
  return tick;
}

/* 각도(도) → Dynamixel tick 변환 */
int angleToTick(float angle_deg) {
  return (int)lroundf(DXL_CENTER + angle_deg * DXL_TICK_PER_DEG);
}

bool isJointWithinLimit(uint8_t idx, int tick) {
  return tick >= LIM_MIN[idx] && tick <= LIM_MAX[idx];
}

/* --------------------------------------------------------
 *  역기구학: 픽셀 좌표 (px, py) → 관절 tick
 *
 *  Top-Down 카메라 좌표계:
 *    카메라 이미지 상에서 (px, py) 픽셀
 *    → 로봇 베이스 기준 실제 좌표 (world_x, world_y) mm
 *    → 극좌표 (r, theta) 로 변환
 *    → J1 = theta (베이스 회전)
 *    → J2, J3, J4 = 2D 평면 역기구학 (r, z 기준)
 *
 *  2D 평면 역기구학 (측면에서 본 팔):
 *
 *       J2 ──L2── J3 ──L3── J4 ──L4── [그리퍼]
 *        |                                 ↓
 *       L1 (높이)                     목표 (r, z)
 *        |
 *      [베이스]
 *
 *    목표점까지 수평 거리 = r
 *    목표점 높이 = z (= PICK_Z_MM)
 *    J4는 그리퍼가 수직 아래를 향하도록 고정
 * -------------------------------------------------------- */
bool calcJointsFromPixelAtZ(int16_t px, int16_t py,
                            float target_z, JointTicks &out) {

  if (px < 0 || px >= CAM_W || py < 0 || py >= CAM_H) {
    DEBUG_SERIAL.print("[IK FAIL] 영상 범위 밖 좌표: px=");
    DEBUG_SERIAL.print(px);
    DEBUG_SERIAL.print(" py=");
    DEBUG_SERIAL.println(py);
    return false;
  }

  /* [1] 픽셀 → 실제 좌표 (mm) 변환 */
  const float pixel_dx = (float)px - BASE_PX;
  const float pixel_dy = (float)py - BASE_PY;
  float world_x = CAM_TO_ROBOT_A11 * pixel_dx +
                  CAM_TO_ROBOT_A12 * pixel_dy;
  float world_y = CAM_TO_ROBOT_A21 * pixel_dx +
                  CAM_TO_ROBOT_A22 * pixel_dy;

  /* [2] 극좌표 변환: 수평 거리(r)와 베이스 회전 각도(theta) */
  float r     = sqrtf(world_x * world_x + world_y * world_y);
  float theta = atan2f(world_x, world_y);   // 정면(+Y) 기준 시계방향 각도

  /* [3] J1: 베이스 회전 (도 단위) */
  float j1_deg = RAD2DEG(theta);

  /* [4] 2D 평면 역기구학 (측면 뷰, r-z 평면)
   *     그리퍼가 수직 아래를 향하도록 J4 각도를 고정하면
   *     J4 끝의 위치 = 목표점이므로,
   *     J2-J3 이음관절이 닿아야 할 점 (wrist point)을 먼저 구한다.
   */
  // 손목점(Wrist Point): 그리퍼가 수직이면 L4만큼 위에 있는 점
  float wr = r;                              // 손목까지 수평 거리 = 동일
  float wz = target_z + OMX_L4 - OMX_L1;    // 손목 높이 = 목표z + L4(그리퍼) - L1(베이스)

  /* [5] 2-링크 IK (L2, L3으로 손목점 도달) */
  float dist_sq = wr * wr + wz * wz;
  float dist    = sqrtf(dist_sq);

  // 도달 가능 범위 체크
  float reach_max = OMX_L2 + OMX_L3;
  float reach_min = fabsf(OMX_L2 - OMX_L3);

  if (dist > reach_max || dist < reach_min) {
    DEBUG_SERIAL.print("[IK FAIL] 도달 불가! dist=");
    DEBUG_SERIAL.print(dist);
    DEBUG_SERIAL.print("mm (범위: ");
    DEBUG_SERIAL.print(reach_min);
    DEBUG_SERIAL.print("~");
    DEBUG_SERIAL.print(reach_max);
    DEBUG_SERIAL.println("mm)");
    return false;
  }

  // 코사인 법칙으로 J3 각도 계산
  float cos_j3 = (dist_sq - OMX_L2 * OMX_L2 - OMX_L3 * OMX_L3) /
                 (2.0f * OMX_L2 * OMX_L3);
  cos_j3 = constrain(cos_j3, -1.0f, 1.0f);
  float j3_angle = acosf(cos_j3);   // 라디안 (팔꿈치 각도, 0=완전 펼침)

  // J2 각도 계산
  float alpha = atan2f(wz, wr);     // 손목점까지의 앙각
  float beta  = atan2f(OMX_L3 * sinf(j3_angle),
                       OMX_L2 + OMX_L3 * cosf(j3_angle));
  float j2_angle = alpha + beta;     // J2 각도 (라디안)

  // J4 각도: 그리퍼가 수직 아래를 향하도록 보정
  // J4 = -(J2 + J3) + 90° 이면 그리퍼가 수직
  float j4_angle = -(j2_angle - M_PI / 2.0f) - j3_angle;

  /* [6] 라디안 → 도 → Dynamixel tick 변환
   *     OMX 서보 방향 규약:
   *     J1: 정면=2048, 왼쪽(반시계)=+
   *     J2: 수직=2048, 앞으로 기울이면=+
   *     J3: 펼침=2048, 접으면=-
   *     J4: 수평=2048
   */
  float j2_deg = RAD2DEG(j2_angle);
  float j3_deg = RAD2DEG(j3_angle);
  float j4_deg = RAD2DEG(j4_angle);

  const int rawJ1 = angleToTick(-j1_deg);  // 카메라 X축 반전 보정
  const int rawJ2 = angleToTick(j2_deg);
  const int rawJ3 = angleToTick(-j3_deg);
  const int rawJ4 = angleToTick(j4_deg);
  const int rawJ5 = DXL_CENTER;

  // 제한값으로 잘라내면 목표점과 다른 위치로 이동하므로 IK 자체를 거부한다.
  const int rawTicks[5] = {rawJ1, rawJ2, rawJ3, rawJ4, rawJ5};
  for (uint8_t i = 0; i < 5; ++i) {
    if (!isJointWithinLimit(i, rawTicks[i])) {
      DEBUG_SERIAL.print("[IK FAIL] J");
      DEBUG_SERIAL.print(i + 1);
      DEBUG_SERIAL.print(" 제한 초과: tick=");
      DEBUG_SERIAL.println(rawTicks[i]);
      return false;
    }
  }

  out.j1 = rawJ1;
  out.j2 = rawJ2;
  out.j3 = rawJ3;
  out.j4 = rawJ4;
  out.j5 = rawJ5;

  DEBUG_SERIAL.print("[IK] px=");    DEBUG_SERIAL.print(px);
  DEBUG_SERIAL.print(" py=");        DEBUG_SERIAL.print(py);
  DEBUG_SERIAL.print(" | world(");   DEBUG_SERIAL.print(world_x, 1);
  DEBUG_SERIAL.print(",");           DEBUG_SERIAL.print(world_y, 1);
  DEBUG_SERIAL.print(")mm r=");      DEBUG_SERIAL.print(r, 1);
  DEBUG_SERIAL.print("mm z=");       DEBUG_SERIAL.print(target_z, 1);
  DEBUG_SERIAL.print("mm θ=");       DEBUG_SERIAL.print(j1_deg, 1);
  DEBUG_SERIAL.println("°");
  DEBUG_SERIAL.print("       J1=");  DEBUG_SERIAL.print(out.j1);
  DEBUG_SERIAL.print(" J2=");        DEBUG_SERIAL.print(out.j2);
  DEBUG_SERIAL.print(" J3=");        DEBUG_SERIAL.print(out.j3);
  DEBUG_SERIAL.print(" J4=");        DEBUG_SERIAL.println(out.j4);

  return true;
}

bool calcJointsFromPixel(int16_t px, int16_t py, JointTicks &out) {
  return calcJointsFromPixelAtZ(px, py, PICK_Z_MM, out);
}

int interpolateTick(int from, int to, float ratio) {
  return (int)lroundf((float)from + ((float)to - (float)from) * ratio);
}

int getAreaPickPointValue(const AreaPickPoint &point, uint8_t motor) {
  switch (motor) {
    case 0: return point.calibration.arm.j1;
    case 1: return point.calibration.arm.j2;
    case 2: return point.calibration.arm.j3;
    case 3: return point.calibration.arm.j4;
    case 4: return point.calibration.arm.j5;
    default: return point.calibration.gripTick;
  }
}

// 세 실측점으로 이루어진 삼각형 안에 (px,py)가 있으면 barycentric 보간한다.
// 삼각형 꼭짓점에서는 해당 실측 모터값이 오차 없이 그대로 반환된다.
bool interpolateAreaTriangle(float px, float py,
                             const AreaPickPoint &a,
                             const AreaPickPoint &b,
                             const AreaPickPoint &c,
                             int result[6]) {
  const float denominator =
      (b.y - c.y) * (a.x - c.x) +
      (c.x - b.x) * (a.y - c.y);
  if (fabsf(denominator) < 0.0001f) return false;

  const float weightA =
      ((b.y - c.y) * (px - c.x) +
       (c.x - b.x) * (py - c.y)) / denominator;
  const float weightB =
      ((c.y - a.y) * (px - c.x) +
       (a.x - c.x) * (py - c.y)) / denominator;
  const float weightC = 1.0f - weightA - weightB;
  const float insideEpsilon = -0.0001f;
  if (weightA < insideEpsilon ||
      weightB < insideEpsilon ||
      weightC < insideEpsilon) {
    return false;
  }

  for (uint8_t motor = 0; motor < 6; ++motor) {
    result[motor] = (int)lroundf(
        getAreaPickPointValue(a, motor) * weightA +
        getAreaPickPointValue(b, motor) * weightB +
        getAreaPickPointValue(c, motor) * weightC);
  }
  return true;
}

int getLocalPickLiftTick(int16_t px, int16_t py) {
  const int distanceX = abs((int)px - PICK_LOCAL_LIFT_CENTER_X_PX);
  const int distanceY = abs((int)py - PICK_LOCAL_LIFT_CENTER_Y_PX);
  if (distanceX >= PICK_LOCAL_LIFT_RADIUS_X_PX ||
      distanceY >= PICK_LOCAL_LIFT_RADIUS_Y_PX) {
    return 0;
  }

  const float weightX =
      1.0f - (float)distanceX / PICK_LOCAL_LIFT_RADIUS_X_PX;
  const float weightY =
      1.0f - (float)distanceY / PICK_LOCAL_LIFT_RADIUS_Y_PX;
  return (int)lroundf(PICK_LOCAL_LIFT_TICK * weightX * weightY);
}

JointTicks interpolateJointPose(const JointTicks &from,
                                const JointTicks &to,
                                int percent) {
  const float ratio = (float)constrain(percent, 0, 100) / 100.0f;
  JointTicks result;
  result.j1 = interpolateTick(from.j1, to.j1, ratio);
  result.j2 = interpolateTick(from.j2, to.j2, ratio);
  result.j3 = interpolateTick(from.j3, to.j3, ratio);
  result.j4 = interpolateTick(from.j4, to.j4, ratio);
  result.j5 = interpolateTick(from.j5, to.j5, ratio);
  return result;
}

/* 수신한 상자 중심 (x,y)을 로봇팔2의 8개 실측점으로 보간한다.
 * 유효 작업영역은 Full HD 영상의 좌측 절반 X=0~960, Y=0~1080이다.
 * 실제 중심점과 연속한 외곽 두 점으로 만든 7개 삼각형 중 좌표가 포함된
 * 삼각형의 ID11~ID16 값을 직접 보간한다.
 */
bool calcMeasuredAreaPoseFromPixel(int16_t px, int16_t py,
                                   JointTicks &out, int &outGripReference) {
  if (px < 0 || (float)px > AREA_CAL_X_MAX ||
      py < 0 || (float)py > AREA_CAL_Y_MAX) {
    DEBUG_SERIAL.print("[AREA MAP FAIL] 왼쪽 작업영역 범위 밖: x=");
    DEBUG_SERIAL.print(px);
    DEBUG_SERIAL.print(" y=");
    DEBUG_SERIAL.println(py);
    return false;
  }

  // 새 실측 좌표를 그대로 사용한다. 추가 보정값이 생기더라도 경계에서는
  // 안전하게 포화되도록 correctedX/correctedY 경로는 유지한다.
  const int correctedX = constrain(
      (int)px + PICK_X_CORRECTION_PX, 0, (int)AREA_CAL_X_MAX);
  const int correctedY = constrain(
      (int)py + PICK_Y_CORRECTION_PX, 0, (int)AREA_CAL_Y_MAX);
  const int appliedXCorrection = correctedX - (int)px;
  const int appliedYCorrection = correctedY - (int)py;
  const float u = (float)correctedX / AREA_CAL_X_MAX;
  const float v = (float)correctedY / AREA_CAL_Y_MAX;
  int result[6];
  int8_t triangleIndex = -1;
  for (uint8_t i = 0; i < AREA_PICK_BOUNDARY_COUNT; ++i) {
    const uint8_t next = (i + 1) % AREA_PICK_BOUNDARY_COUNT;
    if (interpolateAreaTriangle(
            (float)correctedX, (float)correctedY,
            AREA_PICK_CENTER,
            AREA_PICK_BOUNDARY[i],
            AREA_PICK_BOUNDARY[next],
            result)) {
      triangleIndex = (int8_t)i;
      break;
    }
  }
  if (triangleIndex < 0) {
    DEBUG_SERIAL.println("[AREA MAP FAIL] 좌표가 8점 삼각형 영역에 없음");
    return false;
  }

  // 바닥 접촉이 확인된 (618,791) 주변에서는 파지 경로 전체를 조금 더 올린다.
  // J3/J4를 동일한 양만큼 반대 방향으로 보정해 손목 방향 변화는 억제한다.
  const int localLiftTick = getLocalPickLiftTick(px, py);
  result[2] -= localLiftTick;
  result[3] += localLiftTick;

  for (uint8_t motor = 0; motor < 4; ++motor) {
    if (!isJointWithinLimit(motor, result[motor])) {
      DEBUG_SERIAL.print("[AREA MAP FAIL] 8점 보간 후 J");
      DEBUG_SERIAL.print(motor + 1);
      DEBUG_SERIAL.print(" 제한 초과: ");
      DEBUG_SERIAL.println(result[motor]);
      return false;
    }
  }
  if (result[5] < 0 || result[5] > 4095) {
    DEBUG_SERIAL.println("[AREA MAP FAIL] 8점 보간 후 ID16 참고값 제한 초과");
    return false;
  }

  // 사진의 ID15 값은 위치 측정 당시의 손목 회전값일 뿐이다.
  // 집는 동안 집게 방향이 돌아가지 않도록 로봇팔2의 J5 중립값으로 고정한다.
  result[4] = J5_NEUTRAL;

  out.j1 = result[0];
  out.j2 = result[1];
  out.j3 = result[2];
  out.j4 = result[3];
  out.j5 = result[4];
  outGripReference = result[5];

  const uint8_t column = min(
      (int)(correctedX / AREA_CELL_W_PX), AREA_GRID_COLS - 1);
  const uint8_t row = min(
      (int)(correctedY / AREA_CELL_H_PX), AREA_GRID_ROWS - 1);
  const float areaXcm = AREA_FORWARD_START_CM + AREA_REAL_WIDTH_CM * u;
  const float areaYcm = AREA_LATERAL_TOP_CM - AREA_REAL_HEIGHT_CM * v;
  DEBUG_SERIAL.print("[LEFT AREA 8P TRI MAP] request=(");
  DEBUG_SERIAL.print(px);
  DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(py);
  DEBUG_SERIAL.print(") correctedX=");
  DEBUG_SERIAL.print(correctedX);
  DEBUG_SERIAL.print(" (requested ");
  DEBUG_SERIAL.print(PICK_X_CORRECTION_PX);
  DEBUG_SERIAL.print("px, applied ");
  DEBUG_SERIAL.print(appliedXCorrection);
  DEBUG_SERIAL.print("px) correctedY=");
  DEBUG_SERIAL.print(correctedY);
  DEBUG_SERIAL.print(" (requested +");
  DEBUG_SERIAL.print(PICK_Y_CORRECTION_PX);
  DEBUG_SERIAL.print("px, applied +");
  DEBUG_SERIAL.print(appliedYCorrection);
  DEBUG_SERIAL.print("px)");
  DEBUG_SERIAL.print(" grid[row=");
  DEBUG_SERIAL.print(row + 1);
  DEBUG_SERIAL.print(", col=");
  DEBUG_SERIAL.print(column + 1);
  DEBUG_SERIAL.print("] area=(");
  DEBUG_SERIAL.print(areaXcm, 2);
  DEBUG_SERIAL.print("cm,");
  DEBUG_SERIAL.print(areaYcm, 2);
  DEBUG_SERIAL.print("cm) triangle=");
  DEBUG_SERIAL.print(triangleIndex + 1);
  DEBUG_SERIAL.print("/");
  DEBUG_SERIAL.print(AREA_PICK_BOUNDARY_COUNT);
  DEBUG_SERIAL.print(" localLift=");
  DEBUG_SERIAL.print(localLiftTick);
  DEBUG_SERIAL.print(" J=");
  DEBUG_SERIAL.print(out.j1); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(out.j2); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(out.j3); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(out.j4); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(out.j5);
  DEBUG_SERIAL.print(" gripRef=");
  DEBUG_SERIAL.println(outGripReference);
  return true;
}

void setSpeed(int vel) {
  // 팔 관절(ID11~ID15)만 변경한다. ID16 그리퍼는 전용 프로파일 사용.
  for (uint8_t i = 0; i < ARM_SERVO_COUNT; i++)
    dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, SERVO_IDS[i], vel);
}

// [수정 4] 가속도를 함께 제어하기 위한 함수들
void setAccel(int acc) {
  // 팔 관절(ID11~ID15)만 변경한다. ID16 그리퍼는 전용 프로파일 사용.
  for (uint8_t i = 0; i < ARM_SERVO_COUNT; i++)
    dxl.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, SERVO_IDS[i], acc);
}

void setProfile(int vel, int acc) {
  setSpeed(vel);
  setAccel(acc);
}

void setGripperProfile(int vel, int acc) {
  dxl.writeControlTableItem(
      ControlTableItem::PROFILE_VELOCITY, ID_GRIP, vel);
  dxl.writeControlTableItem(
      ControlTableItem::PROFILE_ACCELERATION, ID_GRIP, acc);
}

void moveArm(int j1, int j2, int j3, int j4, int j5) {
  dxl.setGoalPosition(ID_J1, clampJoint(0, j1));
  dxl.setGoalPosition(ID_J2, clampJoint(1, j2));
  dxl.setGoalPosition(ID_J3, clampJoint(2, j3));
  dxl.setGoalPosition(ID_J4, clampJoint(3, j4));
  dxl.setGoalPosition(ID_J5, clampJoint(4, j5));
}

void moveJoints(const JointTicks &jt) {
  moveArm(jt.j1, jt.j2, jt.j3, jt.j4, jt.j5);
}

void movePose(const int pose[5]) {
  moveArm(pose[0], pose[1], pose[2], pose[3], pose[4]);
}

// 긴 대기 중에도 Jetson X/Y/빈 슬롯 UART 파서를 계속 서비스한다.
void jetsonParse();

void serviceDelay(unsigned long durationMs) {
  const unsigned long started = millis();
  while (millis() - started < durationMs) {
    jetsonParse();
    delay(2);
  }
}

// [수정 2] 허용치/연속 안정 횟수를 인자로 받는 일반화된 도달 대기 함수.
// verbose=false면 경유점 통과 시 실패 로그를 출력하지 않는다 (보간 중간 단계용).
bool waitForArmTol(const JointTicks &target, unsigned long timeoutMs,
                   const int tol[5], uint8_t stableNeed, bool verbose = true) {
  const uint8_t ids[5]  = {ID_J1, ID_J2, ID_J3, ID_J4, ID_J5};
  const int    goals[5] = {target.j1, target.j2, target.j3, target.j4, target.j5};
  const unsigned long started = millis();
  uint8_t stableCount = 0;

  while (millis() - started < timeoutMs) {
    jetsonParse();
    bool allReached = true;

    for (uint8_t i = 0; i < 5; ++i) {
      const uint32_t hwError =
          dxl.readControlTableItem(ControlTableItem::HARDWARE_ERROR_STATUS, ids[i]);
      if (hwError != 0) {
        DEBUG_SERIAL.print("[MOVE FAIL] J");
        DEBUG_SERIAL.print(i + 1);
        DEBUG_SERIAL.print(" HW_ERR=");
        DEBUG_SERIAL.println(hwError);
        return false;
      }

      const int present = (int)dxl.getPresentPosition(ids[i], UNIT_RAW);
      if (abs(present - goals[i]) > tol[i]) {
        allReached = false;
      }
    }

    if (allReached) {
      if (++stableCount >= stableNeed) return true;
    } else {
      stableCount = 0;
    }
    delay(20);
  }

  if (verbose) {
    DEBUG_SERIAL.println("[MOVE FAIL] 팔 이동 시간 초과");
    for (uint8_t i = 0; i < 5; ++i) {
      const int present =
          (int)dxl.getPresentPosition(ids[i], UNIT_RAW);
      const int error = abs(present - goals[i]);

      DEBUG_SERIAL.print("  J");
      DEBUG_SERIAL.print(i + 1);
      DEBUG_SERIAL.print(" target=");
      DEBUG_SERIAL.print(goals[i]);
      DEBUG_SERIAL.print(" present=");
      DEBUG_SERIAL.print(present);
      DEBUG_SERIAL.print(" error=");
      DEBUG_SERIAL.println(error);
    }
  }
  return false;
}

// 기존 시그니처 유지 (기존 호출부들이 그대로 동작하도록)
bool waitForArm(const JointTicks &target, unsigned long timeoutMs) {
  return waitForArmTol(target, timeoutMs, JOINT_TOLERANCE, 1);
}

bool moveJointsAndWait(const JointTicks &target,
                       unsigned long timeoutMs = ARM_MOVE_TIMEOUT_MS) {
  moveJoints(target);
  return waitForArm(target, timeoutMs);
}

JointTicks makeJointTicks(int j1, int j2, int j3, int j4, int j5) {
  JointTicks result = {j1, j2, j3, j4, j5};
  return result;
}

int getSeatedJ2Tick() {
  return POSE_AFTER_HOME[1] -
      (POSE_AFTER_HOME[1] - LIM_MIN[1]) * J2_SEATED_PERCENT / 100;
}

// J1 방향만 지정하고 J2는 50% 앉은 위치, J3~J5는
// POSE_AFTER_HOME을 유지하는 이동 자세이다.
JointTicks makeSeatedPoseAtBase(int baseTick) {
  return makeJointTicks(
      baseTick, getSeatedJ2Tick(), POSE_AFTER_HOME[2],
      POSE_AFTER_HOME[3], POSE_AFTER_HOME[4]);
}

JointTicks makePlaceRetreatPose(const JointTicks &placedPose) {
  JointTicks retreat = placedPose;

  // 요청된 J4 후퇴량을 먼저 계산하되 바닥 충돌 방지 하한에서 제한한다.
  const int requestedJ4 = placedPose.j4 + PLACE_RETREAT_J4_TICK;
  retreat.j4 = max(requestedJ4, PLACE_RETREAT_J4_SAFE_MIN);

  // J4가 안전 하한 때문에 덜 움직였으면 J2/J3도 같은 비율로 줄인다.
  // 이를 통해 J4만 제한되고 J2/J3는 과도하게 접히는 잘못된 자세를 막는다.
  const int requestedJ4Distance = abs(PLACE_RETREAT_J4_TICK);
  const int actualJ4Distance = abs(placedPose.j4 - retreat.j4);
  retreat.j2 += PLACE_RETREAT_J2_TICK * actualJ4Distance /
                requestedJ4Distance;
  retreat.j3 += PLACE_RETREAT_J3_TICK * actualJ4Distance /
                requestedJ4Distance;
  return retreat;
}

// 1~9번 모두 사진으로 측정한 해당 칸 모터값을 기준으로 사용한다.
// 측면 칸 1/3/4/6/7/9는 V11보다 30tick 더 깊게 넣는다.
// 1번은 기존 보정 30tick도 유지하므로 실측값 기준 총 60tick 보정이다.
JointTicks makeSlotInsertionPose(uint8_t slot,
                                 const JointTicks &placedPose) {
  JointTicks insertion = placedPose;
  int extraDepthTick = 0;

  if (slot == 1) {
    extraDepthTick += SLOT1_PREVIOUS_EXTRA_DEPTH_TICK;
  }
  if (slot == 1 || slot == 3 ||
      slot == 4 || slot == 6 ||
      slot == 7 || slot == 9) {
    extraDepthTick += SIDE_SLOT_ADDITIONAL_DEPTH_TICK;
  }

  insertion.j2 += extraDepthTick;
  insertion.j4 -= extraDepthTick;

  // V15처럼 전진하면서 원래 높이로 내려가지 않도록 최종 놓기 자세에도
  // J3 상승량을 유지한다. J4를 반대로 보정해 그리퍼 수평은 변하지 않는다.
  insertion.j3 -= SLOT_PLACE_RAISE_J3_TICK;
  insertion.j4 += SLOT_PLACE_RAISE_J3_TICK;
  return insertion;
}

// 50% 앉은 J2와 POSE_AFTER_HOME의 J3를 유지한 후퇴 위치에서
// 선택 칸 실측 자세와 같은 손목 피치(J2+J3+J4)를 만든다.
JointTicks makeMaxBackHorizontalPose(const JointTicks &placedPose) {
  JointTicks horizontal;
  horizontal.j1 = placedPose.j1;
  horizontal.j2 = getSeatedJ2Tick();
  horizontal.j3 = POSE_AFTER_HOME[2];
  horizontal.j4 =
      placedPose.j2 + placedPose.j3 + placedPose.j4 -
      horizontal.j2 - horizontal.j3;
  horizontal.j5 = placedPose.j5;
  return horizontal;
}

// J2는 최대 후퇴 수평 자세에 고정하고 J3/J4만으로 해당 줄 높이를 만든다.
// J3는 해당 칸에서 측정한 높이값을 그대로 사용한다. 기존처럼 후퇴 보정값을
// 더하면 1번은 J3가 2539가 아닌 2587이 되어 전진 전에 불필요하게 내려간다.
// J4는 J2+J3+J4 합을 보존해 그리퍼 수평 각도를 유지한다.
JointTicks makeHeightSetPose(const JointTicks &placedPose) {
  const JointTicks horizontalBack =
      makeMaxBackHorizontalPose(placedPose);
  const int horizontalPitch =
      placedPose.j2 + placedPose.j3 + placedPose.j4;

  JointTicks heightPose = horizontalBack;
  heightPose.j3 = placedPose.j3;
  heightPose.j4 =
      horizontalPitch - heightPose.j2 - heightPose.j3;
  heightPose.j5 = placedPose.j5;
  return heightPose;
}

// 전진 시작 높이를 30tick 올린다. insertionPose에도 같은 상승량을 적용하므로
// 전진 중과 그리퍼를 여는 순간까지 J3 높이가 내려가지 않는다.
JointTicks makeRaisedEntryHeightPose(const JointTicks &placedPose) {
  JointTicks raisedPose = makeHeightSetPose(placedPose);
  raisedPose.j3 -= SLOT_PLACE_RAISE_J3_TICK;
  raisedPose.j4 += SLOT_PLACE_RAISE_J3_TICK;
  return raisedPose;
}

// 그리퍼를 연 뒤 J2/J3/J4를 동시에 사용해 선반에서 빠지는 자세를 만든다.
// 아래층은 바닥 때문에 20tick만, 가운데/위층은 천장 여유를 위해 각각
// 40/80tick 내려 준다. J4를 반대로 보정해 손목 피치는 계속 유지한다.
JointTicks makeCeilingSafeWithdrawPose(
    uint8_t slot,
    const JointTicks &insertionPose,
    const JointTicks &heightPose) {
  int j3DownTick = WITHDRAW_BOTTOM_J3_DOWN_TICK;
  if (slot >= 7) {
    j3DownTick = WITHDRAW_TOP_J3_DOWN_TICK;
  } else if (slot >= 4) {
    j3DownTick = WITHDRAW_MIDDLE_J3_DOWN_TICK;
  }

  const int horizontalPitch =
      insertionPose.j2 + insertionPose.j3 + insertionPose.j4;
  JointTicks withdrawPose = heightPose;
  withdrawPose.j3 += j3DownTick;
  withdrawPose.j4 =
      horizontalPitch - withdrawPose.j2 - withdrawPose.j3;
  return withdrawPose;
}

bool moveFixedAndWait(int j1, int j2, int j3, int j4, int j5,
                      unsigned long timeoutMs = ARM_MOVE_TIMEOUT_MS) {
  JointTicks target = makeJointTicks(j1, j2, j3, j4, j5);
  moveArm(j1, j2, j3, j4, j5);
  return waitForArm(target, timeoutMs);
}

bool moveGripperAndWait(int targetTick, int toleranceTick);

bool isPoseWithinLimits(const JointTicks &pose) {
  const int ticks[5] = {pose.j1, pose.j2, pose.j3, pose.j4, pose.j5};
  for (uint8_t i = 0; i < 5; ++i) {
    if (!isJointWithinLimit(i, ticks[i])) return false;
  }
  return true;
}

bool isValidPlaceSlot(uint8_t slot) {
  return slot >= 1 && slot <= 9;
}

bool selectPlaceSlot(int slot) {
  if (slot < 1 || slot > 9) {
    DEBUG_SERIAL.println("[PLACE FAIL] 칸 번호는 1~9만 가능합니다.");
    return false;
  }

  selectedPlaceSlot = (uint8_t)slot;
  DEBUG_SERIAL.print("[PLACE] 최종 놓기 위치: ");
  DEBUG_SERIAL.print(selectedPlaceSlot);
  DEBUG_SERIAL.println("번 칸");
  return true;
}

bool validatePlaceSlot(uint8_t slot) {
  if (!isValidPlaceSlot(slot)) {
    DEBUG_SERIAL.println("[PLACE FAIL] 칸 번호는 1~9만 가능합니다.");
    return false;
  }

  const PlaceSlotPose &pose = PLACE_SLOTS[slot - 1];
  const JointTicks retreatPose = makePlaceRetreatPose(pose.measuredPose);
  const JointTicks insertionPose =
      makeSlotInsertionPose(slot, pose.measuredPose);
  const JointTicks heightPose =
      makeRaisedEntryHeightPose(pose.measuredPose);
  const JointTicks withdrawBasePose =
      makeHeightSetPose(pose.measuredPose);
  const JointTicks withdrawPose =
      makeCeilingSafeWithdrawPose(slot, insertionPose, withdrawBasePose);
  const JointTicks slotHorizontalPose =
      makeMaxBackHorizontalPose(pose.measuredPose);
  JointTicks frontHorizontalPose = slotHorizontalPose;
  frontHorizontalPose.j1 = POSE_AFTER_HOME[0];
  if (!isPoseWithinLimits(pose.measuredPose) ||
      !isPoseWithinLimits(retreatPose) ||
      !isPoseWithinLimits(insertionPose) ||
      !isPoseWithinLimits(heightPose) ||
      !isPoseWithinLimits(withdrawPose) ||
      !isPoseWithinLimits(frontHorizontalPose) ||
      !isPoseWithinLimits(slotHorizontalPose)) {
    DEBUG_SERIAL.print("[PLACE FAIL] ");
    DEBUG_SERIAL.print(slot);
    DEBUG_SERIAL.println(
        "번 칸 이동/수평/높이/진입/후퇴 자세가 관절 제한을 벗어났습니다.");
    return false;
  }
  return true;
}

void printPlaceSlotPose(uint8_t slot) {
  if (!validatePlaceSlot(slot)) return;
  const PlaceSlotPose &pose = PLACE_SLOTS[slot - 1];

  DEBUG_SERIAL.print("[SLOT ");
  DEBUG_SERIAL.print(slot);
  DEBUG_SERIAL.print("] measured ID11~15=");
  DEBUG_SERIAL.print(pose.measuredPose.j1); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(pose.measuredPose.j2); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(pose.measuredPose.j3); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(pose.measuredPose.j4); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.println(pose.measuredPose.j5);
  DEBUG_SERIAL.print("         measured ID16(reference)=");
  DEBUG_SERIAL.println(pose.measuredGripTick);
  const JointTicks retreatPose = makePlaceRetreatPose(pose.measuredPose);
  DEBUG_SERIAL.print("         approach/retreat ID11~15=");
  DEBUG_SERIAL.print(retreatPose.j1); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(retreatPose.j2); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(retreatPose.j3); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(retreatPose.j4); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.println(retreatPose.j5);
  const JointTicks insertionPose =
      makeSlotInsertionPose(slot, pose.measuredPose);
  DEBUG_SERIAL.print("         insertion(final) ID11~15=");
  DEBUG_SERIAL.print(insertionPose.j1); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(insertionPose.j2); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(insertionPose.j3); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(insertionPose.j4); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.println(insertionPose.j5);
  const JointTicks horizontalPose =
      makeMaxBackHorizontalPose(pose.measuredPose);
  DEBUG_SERIAL.print("         max-back horizontal ID11~15=");
  DEBUG_SERIAL.print(horizontalPose.j1); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(horizontalPose.j2); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(horizontalPose.j3); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(horizontalPose.j4); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.println(horizontalPose.j5);
  const JointTicks heightPose =
      makeRaisedEntryHeightPose(pose.measuredPose);
  DEBUG_SERIAL.print("         raised-place-height(J3 -");
  DEBUG_SERIAL.print(SLOT_PLACE_RAISE_J3_TICK);
  DEBUG_SERIAL.print("tick) ID11~15=");
  DEBUG_SERIAL.print(heightPose.j1); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(heightPose.j2); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(heightPose.j3); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(heightPose.j4); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.println(heightPose.j5);
  const JointTicks withdrawBasePose =
      makeHeightSetPose(pose.measuredPose);
  const JointTicks withdrawPose =
      makeCeilingSafeWithdrawPose(slot, insertionPose, withdrawBasePose);
  DEBUG_SERIAL.print("         withdraw(J2/J3/J4) ID11~15=");
  DEBUG_SERIAL.print(withdrawPose.j1); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(withdrawPose.j2); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(withdrawPose.j3); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(withdrawPose.j4); DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.println(withdrawPose.j5);
}

// [수정 5] 현재 서보 위치를 읽어와 JointTicks로 반환
bool readCurrentPose(JointTicks &out) {
  const uint8_t ids[5] = {ID_J1, ID_J2, ID_J3, ID_J4, ID_J5};
  int v[5];
  for (uint8_t i = 0; i < 5; ++i) {
    v[i] = (int)dxl.getPresentPosition(ids[i], UNIT_RAW);
  }
  out = makeJointTicks(v[0], v[1], v[2], v[3], v[4]);
  return true;
}

// [수정 5] 두 자세 사이를 t(0~1) 비율로 선형 보간
JointTicks lerpPose(const JointTicks &a, const JointTicks &b, float t) {
  JointTicks r;
  r.j1 = a.j1 + (int)lroundf((b.j1 - a.j1) * t);
  r.j2 = a.j2 + (int)lroundf((b.j2 - a.j2) * t);
  r.j3 = a.j3 + (int)lroundf((b.j3 - a.j3) * t);
  r.j4 = a.j4 + (int)lroundf((b.j4 - a.j4) * t);
  r.j5 = a.j5 + (int)lroundf((b.j5 - a.j5) * t);
  return r;
}

// [수정 5] 관절공간 선형보간으로 target까지 steps개 경유점을 거쳐 이동.
// 중간 경유점은 느슨한 허용치로 "근접 확인"만 하고 바로 다음 경유점으로 넘어가
// 관절별 이동 속도 차이로 인한 팔 끝 휘어짐/튐을 줄인다.
// 마지막 target에서만 finalTol + SETTLE_STABLE_COUNT로 정밀 도달 확인한다.
bool moveSmoothAndWait(const JointTicks &target, uint8_t steps,
                       const int finalTol[5],
                       unsigned long timeoutMs = PLACE_TIMEOUT_MS,
                       const int waypointTol[5] = WAYPOINT_TOLERANCE,
                       unsigned long waypointTimeoutMs = 4000UL) {
  JointTicks from;
  readCurrentPose(from);
  if (steps < 1) steps = 1;

  for (uint8_t s = 1; s < steps; ++s) {
    JointTicks mid = lerpPose(from, target, (float)s / (float)steps);
    if (!isPoseWithinLimits(mid)) {
      DEBUG_SERIAL.println("[SMOOTH FAIL] 경유점이 관절 제한 초과");
      return false;
    }
    moveJoints(mid);
    if (!waitForArmTol(
            mid, waypointTimeoutMs, waypointTol, 1, false)) {
      DEBUG_SERIAL.print("[SMOOTH FAIL] 경유점 도달 실패 step=");
      DEBUG_SERIAL.print(s);
      DEBUG_SERIAL.print("/");
      DEBUG_SERIAL.println(steps);
      // 실패한 관절을 바로 확인할 수 있도록 목표/현재/오차를 출력한다.
      waitForArmTol(mid, 100UL, waypointTol, 1, true);
      return false;
    }
  }

  JointTicks last = target;
  moveJoints(last);
  return waitForArmTol(target, timeoutMs, finalTol, SETTLE_STABLE_COUNT);
}

// J4에는 하중 보상 목표를 명령하지만 실제 도달 판정은 desiredPose로 한다.
// 보상 후에도 실제 손목이 수평 목표에 도달하지 않으면 안전하게 실패한다.
bool moveHorizontalWithJ4Compensation(const JointTicks &desiredPose) {
  JointTicks commandPose = desiredPose;
  commandPose.j4 = clampJoint(
      3, desiredPose.j4 - J4_HORIZONTAL_LOAD_COMP_TICK);

  DEBUG_SERIAL.print("[J4 DIRECT V4] desired=");
  DEBUG_SERIAL.print(desiredPose.j4);
  DEBUG_SERIAL.print(" command=");
  DEBUG_SERIAL.println(commandPose.j4);

  // 정면의 최대 후퇴 위치이므로 중간 경유점 판정을 사용하지 않는다.
  // 저속 프로파일로 보상 목표를 직접 명령한 뒤 실제 수평 위치만 검사한다.
  moveJoints(commandPose);
  return waitForArmTol(
      desiredPose, J4_LEVEL_TIMEOUT_MS, HORIZONTAL_TOLERANCE,
      SETTLE_STABLE_COUNT);
}

// 물체를 든 상태에서 1~9번 칸의 후퇴 수평/높이를 맞출 때 사용한다.
// 각 경유점은 원래 수평 궤적으로 계산하고 J4 명령에만 하중 보상을 적용한다.
// 최종 칸 진입에는 적용하지 않는다. 최종 실측값에서 -90tick을 적용하면
// 이번 로그처럼 J4가 target보다 약 90tick 작게 멈춰 그리퍼 열기 전에 중단된다.
bool moveSmoothWithJ4Compensation(const JointTicks &desiredTarget,
                                  uint8_t steps,
                                  const int finalTol[5]) {
  JointTicks from;
  readCurrentPose(from);
  if (steps < 1) steps = 1;

  for (uint8_t s = 1; s < steps; ++s) {
    const JointTicks desiredMid =
        lerpPose(from, desiredTarget, (float)s / (float)steps);
    JointTicks commandMid = desiredMid;
    commandMid.j4 = clampJoint(
        3, desiredMid.j4 - J4_HORIZONTAL_LOAD_COMP_TICK);

    if (!isPoseWithinLimits(commandMid)) {
      DEBUG_SERIAL.println("[J4 COMP FAIL] 보상 경유점이 관절 제한 초과");
      return false;
    }

    moveJoints(commandMid);
    // 판정 대상은 보상 명령값이 아니라 실제로 도달해야 할 원래 궤적이다.
    if (!waitForArmTol(
            desiredMid, 8000UL, WAYPOINT_TOLERANCE, 1, false)) {
      DEBUG_SERIAL.print("[J4 COMP FAIL] 실제 경유점 도달 실패 step=");
      DEBUG_SERIAL.print(s);
      DEBUG_SERIAL.print("/");
      DEBUG_SERIAL.println(steps);
      // 실패 시 목표/현재/오차를 한 번 더 출력해 다음 보정에 사용한다.
      waitForArmTol(
          desiredMid, 100UL, WAYPOINT_TOLERANCE, 1, true);
      return false;
    }
  }

  JointTicks commandTarget = desiredTarget;
  commandTarget.j4 = clampJoint(
      3, desiredTarget.j4 - J4_HORIZONTAL_LOAD_COMP_TICK);
  moveJoints(commandTarget);
  return waitForArmTol(
      desiredTarget, J4_LEVEL_TIMEOUT_MS, finalTol,
      SETTLE_STABLE_COUNT);
}

// 선택 줄의 높이를 맞추는 동안에는 J1/J2/J5의 현재 위치를 그대로 고정하고
// J3/J4만 보간한다. j4CompTick이 0보다 크면 명령에만 하중 보상을 적용하고
// 판정은 원래 궤적으로 한다. 제한 시간도 층별 값으로 전달받는다.
bool moveHeightWithJ3J4Only(const JointTicks &heightTarget,
                            uint8_t steps,
                            int j4CompTick,
                            unsigned long waypointTimeoutMs,
                            unsigned long finalTimeoutMs) {
  JointTicks from;
  readCurrentPose(from);
  if (steps < 1) steps = 1;

  JointTicks adjustedTarget = heightTarget;
  adjustedTarget.j1 = from.j1;
  adjustedTarget.j2 = from.j2;  // 높이 설정 중 J2를 절대 움직이지 않는다.
  adjustedTarget.j5 = from.j5;

  // 실제 J2와 계산 기준 J2의 작은 차이는 J4로 보정하여 수평합을 유지한다.
  adjustedTarget.j4 += heightTarget.j2 - adjustedTarget.j2;

  for (uint8_t s = 1; s <= steps; ++s) {
    JointTicks desiredMid = from;
    desiredMid.j3 = from.j3 +
        (adjustedTarget.j3 - from.j3) * s / steps;
    desiredMid.j4 = from.j4 +
        (adjustedTarget.j4 - from.j4) * s / steps;

    JointTicks commandMid = desiredMid;
    if (j4CompTick > 0) {
      commandMid.j4 = clampJoint(
          3, desiredMid.j4 - j4CompTick);
    }

    if (!isPoseWithinLimits(commandMid)) {
      DEBUG_SERIAL.println("[HEIGHT FAIL] J3/J4 경유점이 관절 제한 초과");
      return false;
    }

    moveJoints(commandMid);
    const bool lastStep = (s == steps);
    const int *tol = lastStep ? PLACE_TOLERANCE : WAYPOINT_TOLERANCE;
    const unsigned long timeoutMs = lastStep
        ? finalTimeoutMs : waypointTimeoutMs;
    const uint8_t stableNeed = lastStep ? SETTLE_STABLE_COUNT : 1;
    bool reached = waitForArmTol(
        desiredMid, timeoutMs, tol, stableNeed, false);

    // 고정 -90tick 보상이 자세에 따라 부족하거나 과할 수 있다.
    // 7번 로그처럼 J4가 64tick 뒤처져 경유점 허용치 60을 넘으면,
    // 실제 오차만큼 기존 명령을 보정하여 같은 경유점을 한 번 재시도한다.
    if (!reached && j4CompTick > 0) {
      JointTicks actualPose;
      readCurrentPose(actualPose);
      const int j4Residual = desiredMid.j4 - actualPose.j4;
      JointTicks retryCommand = commandMid;
      retryCommand.j4 = clampJoint(
          3, commandMid.j4 + j4Residual);

      DEBUG_SERIAL.print("[HEIGHT J4 AUTO] desired=");
      DEBUG_SERIAL.print(desiredMid.j4);
      DEBUG_SERIAL.print(" present=");
      DEBUG_SERIAL.print(actualPose.j4);
      DEBUG_SERIAL.print(" oldCommand=");
      DEBUG_SERIAL.print(commandMid.j4);
      DEBUG_SERIAL.print(" retryCommand=");
      DEBUG_SERIAL.println(retryCommand.j4);

      if (isPoseWithinLimits(retryCommand)) {
        moveJoints(retryCommand);
        reached = waitForArmTol(
            desiredMid, timeoutMs, tol, stableNeed, false);
      }
    }

    if (!reached) {
      DEBUG_SERIAL.print("[HEIGHT FAIL] J3/J4 높이 도달 실패 step=");
      DEBUG_SERIAL.print(s);
      DEBUG_SERIAL.print("/");
      DEBUG_SERIAL.println(steps);
      waitForArmTol(desiredMid, 100UL, tol, 1, true);
      return false;
    }
  }

  return true;
}

// 물체를 놓고 선반에서 J2/J3/J4로 빠져나온 뒤의 전용 Home 복귀 경로.
// 슬롯 자세에서 접힌 Home으로 곧바로 이동하지 않고, 익숙한 정면 안전 자세인
// POSE_AFTER_HOME을 한 번 거쳐 관절별 이동량을 두 구간으로 나눈다.
bool returnHomeAfterPlacement() {
  const JointTicks afterHomeTarget = makeJointTicks(
      POSE_AFTER_HOME[0], POSE_AFTER_HOME[1], POSE_AFTER_HOME[2],
      POSE_AFTER_HOME[3], POSE_AFTER_HOME[4]);
  const JointTicks homeTarget = makeJointTicks(
      POSE_HOME[0], POSE_HOME[1], POSE_HOME[2],
      POSE_HOME[3], POSE_HOME[4]);

  DEBUG_SERIAL.println(
      "[11A] 후퇴 위치 -> 정면 POSE_AFTER_HOME 안전 자세");
  bool afterHomeOk = moveSmoothAndWait(
      afterHomeTarget, 8, JOINT_TOLERANCE,
      HOME_RETURN_STAGE_TIMEOUT_MS,
      HOME_RETURN_WAYPOINT_TOLERANCE,
      HOME_RETURN_WAYPOINT_TIMEOUT_MS);
  if (!afterHomeOk) {
    DEBUG_SERIAL.println(
        "[HOME RETRY] 현재 자세 -> POSE_AFTER_HOME 재시도");
    dxl.setGoalPosition(ID_GRIP, GRIP_PLACE_RELEASE);
    serviceDelay(300);
    afterHomeOk = moveSmoothAndWait(
        afterHomeTarget, 10, JOINT_TOLERANCE,
        HOME_RETURN_STAGE_TIMEOUT_MS,
        HOME_RETURN_WAYPOINT_TOLERANCE,
        HOME_RETURN_WAYPOINT_TIMEOUT_MS);
  }
  if (!afterHomeOk) {
    DEBUG_SERIAL.println("[HOME FAIL] 정면 안전 자세 복귀 실패");
    return false;
  }
  serviceDelay(300);

  DEBUG_SERIAL.println("[11B] POSE_AFTER_HOME -> Home");
  bool homeOk = moveSmoothAndWait(
      homeTarget, 8, JOINT_TOLERANCE,
      HOME_RETURN_STAGE_TIMEOUT_MS,
      HOME_RETURN_WAYPOINT_TOLERANCE,
      HOME_RETURN_WAYPOINT_TIMEOUT_MS);
  if (!homeOk) {
    DEBUG_SERIAL.println("[HOME RETRY] POSE_AFTER_HOME -> Home 재시도");
    dxl.setGoalPosition(ID_GRIP, GRIP_PLACE_RELEASE);
    serviceDelay(300);
    homeOk = moveSmoothAndWait(
        homeTarget, 10, JOINT_TOLERANCE,
        HOME_RETURN_STAGE_TIMEOUT_MS,
        HOME_RETURN_WAYPOINT_TOLERANCE,
        HOME_RETURN_WAYPOINT_TIMEOUT_MS);
  }
  if (!homeOk) {
    DEBUG_SERIAL.println("[HOME FAIL] 최종 Home 자세 복귀 실패");
    return false;
  }

  return true;
}

bool moveHeldObjectToSlot(uint8_t slot) {
  if (!validatePlaceSlot(slot)) return false;
  const PlaceSlotPose &pose = PLACE_SLOTS[slot - 1];

  const JointTicks slotHorizontalPose =
      makeMaxBackHorizontalPose(pose.measuredPose);
  JointTicks frontHorizontalPose = slotHorizontalPose;
  frontHorizontalPose.j1 = POSE_AFTER_HOME[0];
  const JointTicks heightPose =
      makeRaisedEntryHeightPose(pose.measuredPose);
  const JointTicks insertionPose =
      makeSlotInsertionPose(slot, pose.measuredPose);
  const JointTicks withdrawBasePose =
      makeHeightSetPose(pose.measuredPose);
  const JointTicks withdrawPose =
      makeCeilingSafeWithdrawPose(slot, insertionPose, withdrawBasePose);
  const bool topRow = (slot >= 7);
  const uint8_t heightMoveSteps = topRow ? 4 : 6;
  const int heightJ4CompTick =
      topRow ? J4_TOP_ROW_COMP_TICK : J4_HORIZONTAL_LOAD_COMP_TICK;
  const int heightSpeed = topRow ? HEIGHT_SPEED : SLOW_SPEED;
  const int heightAccel = topRow ? HEIGHT_ACCEL : PLACE_ACCEL;
  const unsigned long heightWaypointTimeoutMs =
      topRow ? HEIGHT_WAYPOINT_TIMEOUT_MS : 8000UL;
  const unsigned long heightFinalTimeoutMs =
      topRow ? HEIGHT_FINAL_TIMEOUT_MS : J4_LEVEL_TIMEOUT_MS;

  DEBUG_SERIAL.print("[7] ");
  DEBUG_SERIAL.print(slot);
  DEBUG_SERIAL.println(
      "번 칸: V12 측면 깊이/천장회피/J4 자동보정");

  // 선반을 향하기 전에 정면의 최대 후퇴 위치에서 손목을 먼저 수평으로 만든다.
  // 이후 J2~J5를 고정하고 J1만 선택 칸 방향으로 회전하므로,
  // 그리퍼가 아래를 향한 상태로 선반 쪽을 지나가는 동작을 막는다.
  setProfile(SLOW_SPEED, PLACE_ACCEL);
  // 물체를 들었을 때의 +90tick J4 처짐 보상을 1~9번 모두 적용한다.
  const bool frontLevelOk =
      moveHorizontalWithJ4Compensation(frontHorizontalPose);
  if (!frontLevelOk) {
    setProfile(MOVE_SPEED, MOVE_ACCEL);
    DEBUG_SERIAL.println("[FAIL] 정면 손목 수평 보정 실패");
    return false;
  }
  const bool slotTurnOk =
      moveHorizontalWithJ4Compensation(slotHorizontalPose);
  if (!slotTurnOk) {
    setProfile(MOVE_SPEED, MOVE_ACCEL);
    DEBUG_SERIAL.println("[FAIL] 수평 유지 칸 방향 회전 실패");
    return false;
  }
  DEBUG_SERIAL.print("[7A] J2 고정, J3 -");
  DEBUG_SERIAL.print(SLOT_PLACE_RAISE_J3_TICK);
  DEBUG_SERIAL.println("tick 높이를 놓기까지 유지");
  DEBUG_SERIAL.print("[7A FAST] steps=");
  DEBUG_SERIAL.print(heightMoveSteps);
  DEBUG_SERIAL.print(" speed=");
  DEBUG_SERIAL.print(heightSpeed);
  DEBUG_SERIAL.print(" accel=");
  DEBUG_SERIAL.print(heightAccel);
  DEBUG_SERIAL.print(" J4comp=");
  DEBUG_SERIAL.println(heightJ4CompTick);
  setProfile(heightSpeed, heightAccel);
  const bool heightOk = moveHeightWithJ3J4Only(
      heightPose, heightMoveSteps, heightJ4CompTick,
      heightWaypointTimeoutMs, heightFinalTimeoutMs);
  if (!heightOk) {
    setProfile(MOVE_SPEED, MOVE_ACCEL);
    DEBUG_SERIAL.println("[FAIL] J3/J4 구역 높이 설정 실패");
    return false;
  }
  serviceDelay(300);   // 진동 감쇠 대기

  DEBUG_SERIAL.print("[8] ");
  DEBUG_SERIAL.print(slot);
  DEBUG_SERIAL.println(
      "번 칸 높이 확정, J3 상승 높이 유지하며 수평 전진");

  // [수정 6] 적재 진입은 반드시 최저속으로 (정밀도가 가장 중요한 구간)
  setProfile(PLACE_SPEED, PLACE_ACCEL);
  // 최종 삽입 자세는 사진으로 측정한 원래 모터값을 직접 명령한다.
  // 1~9번 모두 최종 진입에는 -90tick 보상을 적용하지 않는다.
  bool insertionOk =
      moveSmoothAndWait(insertionPose, 3, PLACE_TOLERANCE);
  if (!insertionOk) {
    DEBUG_SERIAL.println(
        "[INSERT RETRY] 실측 삽입 자세 직접 재명령 후 놓기 판정");
    moveJoints(insertionPose);
    insertionOk = waitForArmTol(
        insertionPose, 8000UL, INSERTION_RELEASE_TOLERANCE, 2, true);
  }
  if (!insertionOk) {
    setProfile(MOVE_SPEED, MOVE_ACCEL);
    DEBUG_SERIAL.println("[FAIL] J2/J3/J4 수평 전진 실패");
    return false;
  }
  serviceDelay(500);   // [수정 7] 완전 정지 후 놓기

  DEBUG_SERIAL.print("[9] ");
  DEBUG_SERIAL.print(slot);
  DEBUG_SERIAL.println("번 칸 실측 자세에서 놓기");
  bool releaseOk = moveGripperAndWait(
      GRIP_PLACE_RELEASE, GRIP_PLACE_RELEASE_TOLERANCE);
  if (!releaseOk && slot == 7) {
    // 7번은 위층 측벽에 상자가 닿으면 첫 열림 때 순간 부하가 커질 수 있다.
    // 현재 삽입 자세를 그대로 유지한 채 열림 동작만 한 번 더 수행한다.
    DEBUG_SERIAL.println(
        "[SLOT7 RELEASE RETRY] 삽입 자세 유지 후 그리퍼 열기 재시도");
    serviceDelay(500);
    releaseOk = moveGripperAndWait(
        GRIP_PLACE_RELEASE, GRIP_PLACE_RELEASE_TOLERANCE);
  }
  if (!releaseOk) {
    setProfile(MOVE_SPEED, MOVE_ACCEL);
    DEBUG_SERIAL.println("[FAIL] 놓기 중 집게 열기 실패");
    return false;
  }
  // 최종 삽입은 이미 무보상 실측 자세이므로, 그리퍼를 연 뒤 자세를
  // 다시 명령하거나 들어 올리지 않고 그대로 유지한다.
  serviceDelay(400);   // 물체가 자리 잡을 시간

  DEBUG_SERIAL.println(
      "[10] 그리퍼를 연 상태에서 J2/J3/J4 천장 회피 저속 후퇴");
  // 전진 반대 방향으로 J2/J3/J4를 동시에 움직인다. 가운데/위층은
  // J3를 조금 낮추고 J4를 반대로 보정하여 칸 천장과의 접촉을 피한다.
  setProfile(PLACE_SPEED, PLACE_ACCEL);
  const bool slot7Withdraw = (slot == 7);
  const uint8_t withdrawSteps =
      slot7Withdraw ? SLOT7_WITHDRAW_STEPS : 6;
  const int *withdrawFinalTolerance =
      slot7Withdraw ? SLOT7_WITHDRAW_TOLERANCE : JOINT_TOLERANCE;
  const int *withdrawWaypointTolerance =
      slot7Withdraw ? SLOT7_WITHDRAW_WAYPOINT_TOLERANCE
                    : WAYPOINT_TOLERANCE;
  const unsigned long withdrawTimeoutMs =
      slot7Withdraw ? SLOT7_WITHDRAW_TIMEOUT_MS : PLACE_TIMEOUT_MS;
  const unsigned long withdrawWaypointTimeoutMs =
      slot7Withdraw ? SLOT7_WITHDRAW_WAYPOINT_TIMEOUT_MS : 8000UL;
  if (slot7Withdraw) {
    DEBUG_SERIAL.println(
        "[SLOT7 WITHDRAW] 10단계 저속 후퇴 및 전용 허용치 적용");
  }
  bool withdrawOk = moveSmoothAndWait(
      withdrawPose, withdrawSteps, withdrawFinalTolerance,
      withdrawTimeoutMs, withdrawWaypointTolerance,
      withdrawWaypointTimeoutMs);
  if (!withdrawOk) {
    // 하중이 사라진 직후 관절 반동으로 첫 경유점 판정이 실패할 수 있다.
    // 현재 위치를 새 시작점으로 다시 읽고, 열린 그리퍼를 유지한 채
    // 경유점을 늘려 동일한 후퇴 목표를 한 번 더 저속으로 수행한다.
    DEBUG_SERIAL.println(
        "[WITHDRAW RETRY] 열린 그리퍼 유지, 현재 자세에서 저속 재후퇴");
    dxl.setGoalPosition(ID_GRIP, GRIP_PLACE_RELEASE);
    serviceDelay(300);
    setProfile(PLACE_SPEED, PLACE_ACCEL);
    const uint8_t retrySteps =
        slot7Withdraw ? SLOT7_WITHDRAW_RETRY_STEPS : 8;
    const unsigned long retryTimeoutMs =
        slot7Withdraw ? SLOT7_WITHDRAW_RETRY_TIMEOUT_MS
                      : WITHDRAW_RETRY_TIMEOUT_MS;
    const int *retryWaypointTolerance =
        slot7Withdraw ? SLOT7_WITHDRAW_WAYPOINT_TOLERANCE
                      : HOME_RETURN_WAYPOINT_TOLERANCE;
    withdrawOk = moveSmoothAndWait(
        withdrawPose, retrySteps, withdrawFinalTolerance,
        retryTimeoutMs,
        retryWaypointTolerance,
        WITHDRAW_RETRY_WAYPOINT_TIMEOUT_MS);
  }
  if (!withdrawOk) {
    setProfile(MOVE_SPEED, MOVE_ACCEL);
    DEBUG_SERIAL.println("[FAIL] J2/J3/J4 천장 회피 후퇴 재시도 실패");
    return false;
  }

  DEBUG_SERIAL.println(
      "[11] 후퇴 자세에서 열린 그리퍼를 유지하고 단계별 Home 복귀");
  setProfile(SLOW_SPEED, PLACE_ACCEL);
  if (!returnHomeAfterPlacement()) {
    setProfile(MOVE_SPEED, MOVE_ACCEL);
    DEBUG_SERIAL.println("[FAIL] 천장 회피 후퇴 후 Home 복귀 실패");
    return false;
  }

  // Home에 도착할 때까지 그리퍼는 열린 상태로 둔다.
  dxl.setGoalPosition(ID_GRIP, GRIP_NEUTRAL);
  serviceDelay(800);
  setProfile(MOVE_SPEED, MOVE_ACCEL);

  return true;
}

bool testPlaceSlot(uint8_t slot) {
  if (!validatePlaceSlot(slot)) return false;
  const PlaceSlotPose &pose = PLACE_SLOTS[slot - 1];
  const JointTicks slotHorizontalPose =
      makeMaxBackHorizontalPose(pose.measuredPose);
  JointTicks frontHorizontalPose = slotHorizontalPose;
  frontHorizontalPose.j1 = POSE_AFTER_HOME[0];
  const JointTicks heightPose =
      makeRaisedEntryHeightPose(pose.measuredPose);
  const JointTicks insertionPose =
      makeSlotInsertionPose(slot, pose.measuredPose);
  const JointTicks withdrawBasePose =
      makeHeightSetPose(pose.measuredPose);
  const JointTicks withdrawPose =
      makeCeilingSafeWithdrawPose(slot, insertionPose, withdrawBasePose);
  DEBUG_SERIAL.print("[SLOT TEST] ");
  DEBUG_SERIAL.print(slot);
  DEBUG_SERIAL.println("번 칸 저속 시험 시작");
  printPlaceSlotPose(slot);

  setSpeed(SLOW_SPEED);
  // placetest는 물체를 들지 않으므로 J4 하중 보상을 적용하지 않는다.
  bool ok = moveJointsAndWait(frontHorizontalPose) &&
            moveJointsAndWait(slotHorizontalPose);
  if (ok) {
    const bool topRow = (slot >= 7);
    setProfile(
        topRow ? HEIGHT_SPEED : SLOW_SPEED,
        topRow ? HEIGHT_ACCEL : PLACE_ACCEL);
    ok = moveHeightWithJ3J4Only(
        heightPose, topRow ? 4 : 6, 0,
        topRow ? HEIGHT_WAYPOINT_TIMEOUT_MS : 8000UL,
        topRow ? HEIGHT_FINAL_TIMEOUT_MS : J4_LEVEL_TIMEOUT_MS);
  }
  if (ok) {
    ok = moveSmoothAndWait(insertionPose, 3, PLACE_TOLERANCE);
  }

  // [수정 1] 오타 수정: ok -> ok (원본은 컴파일 오류였음)
  if (ok) serviceDelay(1500);

  // 실제 적재 동작과 동일하게 J2/J3/J4로 천장 회피 후퇴 후 Home 이동.
  if (!moveSmoothAndWait(
          withdrawPose, 6, JOINT_TOLERANCE, PLACE_TIMEOUT_MS,
          WAYPOINT_TOLERANCE, 8000UL) ||
      !returnHomeAfterPlacement()) {
    ok = false;
  }

  setSpeed(MOVE_SPEED);
  DEBUG_SERIAL.println(ok ? "[SLOT TEST] 완료" : "[SLOT TEST] 실패");
  return ok;
}

bool moveGripperAndWait(int targetTick, int toleranceTick) {
  // 적재 진입용 PLACE_SPEED/PLACE_ACCEL과 무관하게 그리퍼 전용 속도를 사용한다.
  setGripperProfile(GRIP_SPEED, GRIP_ACCEL);
  int lastPresent = (int)dxl.getPresentPosition(ID_GRIP, UNIT_RAW);
  uint32_t lastHwError = 0;

  for (uint8_t attempt = 1; attempt <= GRIP_OPEN_MAX_ATTEMPTS; ++attempt) {
    // 같은 목표를 재전송하면 순간 부하로 멈춘 경우 다시 움직일 수 있다.
    setGripperProfile(GRIP_SPEED, GRIP_ACCEL);
    dxl.setGoalPosition(ID_GRIP, targetTick);
    const unsigned long started = millis();
    uint8_t stableCount = 0;

    while (millis() - started < GRIP_MOVE_TIMEOUT_MS) {
      jetsonParse();
      lastHwError = dxl.readControlTableItem(
          ControlTableItem::HARDWARE_ERROR_STATUS, ID_GRIP);
      if (lastHwError != 0) {
        DEBUG_SERIAL.print("[GRIP FAIL] HW_ERR=");
        DEBUG_SERIAL.println(lastHwError);
        return false;
      }

      lastPresent = (int)dxl.getPresentPosition(ID_GRIP, UNIT_RAW);
      // 순간값이 아니라 목표와의 절대오차가 허용범위인 상태를 3회 연속 확인한다.
      if (abs(lastPresent - targetTick) <= toleranceTick) {
        if (++stableCount >= GRIP_OPEN_STABLE_COUNT) {
          // 전달받은 목표를 계속 유지해 준비/파지/놓기 동작이 끝날 시간을 준다.
          serviceDelay(GRIP_FULL_OPEN_HOLD_MS);
          lastPresent = (int)dxl.getPresentPosition(ID_GRIP, UNIT_RAW);
          if (abs(lastPresent - targetTick) <= toleranceTick) {
            DEBUG_SERIAL.print("[GRIP MOVE OK] attempt=");
            DEBUG_SERIAL.print(attempt);
            DEBUG_SERIAL.print(" target=");
            DEBUG_SERIAL.print(targetTick);
            DEBUG_SERIAL.print(" present=");
            DEBUG_SERIAL.print(lastPresent);
            DEBUG_SERIAL.print(" tolerance=");
            DEBUG_SERIAL.print(toleranceTick);
            DEBUG_SERIAL.print(" fullOpenHoldMs=");
            DEBUG_SERIAL.println(GRIP_FULL_OPEN_HOLD_MS);
            return true;
          }
          stableCount = 0;
        }
      } else {
        stableCount = 0;
      }
      delay(20);
    }

    if (attempt < GRIP_OPEN_MAX_ATTEMPTS) {
      DEBUG_SERIAL.print("[GRIP RETRY] 목표 명령 재전송 attempt=");
      DEBUG_SERIAL.print(attempt + 1);
      DEBUG_SERIAL.print(" lastPresent=");
      DEBUG_SERIAL.println(lastPresent);
      serviceDelay(GRIP_OPEN_RETRY_DELAY_MS);
    }
  }

  DEBUG_SERIAL.print("[GRIP FAIL] 이동 시간 초과 target=");
  DEBUG_SERIAL.print(targetTick);
  DEBUG_SERIAL.print(" present=");
  DEBUG_SERIAL.print(lastPresent);
  DEBUG_SERIAL.print(" error=");
  DEBUG_SERIAL.print(abs(lastPresent - targetTick));
  DEBUG_SERIAL.print(" HW_ERR=");
  DEBUG_SERIAL.println(lastHwError);
  return false;
}

void initServos() {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    uint8_t id = SERVO_IDS[i];
    if (dxl.ping(id)) {
      dxl.torqueOn(id);
      dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, id, MOVE_SPEED);
      dxl.writeControlTableItem(ControlTableItem::PROFILE_ACCELERATION, id, MOVE_ACCEL);
      // [수정 3 보조] 정밀 허용치를 조였을 때 정상상태 오차/타임아웃이 발생하면
      // 아래 게인 값을 조정한다 (기본값 대비 P를 다소 높임).
      // dxl.writeControlTableItem(ControlTableItem::POSITION_P_GAIN, id, 1000);
      // dxl.writeControlTableItem(ControlTableItem::POSITION_D_GAIN, id, 300);
    } else {
      DEBUG_SERIAL.print("[WARN] ID ");
      DEBUG_SERIAL.print(id);
      DEBUG_SERIAL.println(" 응답 없음");
    }
  }
}


/* ************************************************************
 *  4. 기준 자세 & 전원 관리
 * ************************************************************ */

void initPose() {
  DEBUG_SERIAL.println("=== [INIT POSE] 기준 자세(수직)로 이동 ===");
  setSpeed(SLOW_SPEED);
  movePose(POSE_INIT);  serviceDelay(3000);
  dxl.setGoalPosition(ID_GRIP, GRIP_NEUTRAL);  serviceDelay(800);
  setSpeed(MOVE_SPEED);
  DEBUG_SERIAL.println("=== [INIT POSE] 완료 ===");
}

void homePose() {
  DEBUG_SERIAL.println("=== [HOME POSE] 대기 자세로 이동 ===");
  movePose(POSE_HOME);  serviceDelay(2000);
  dxl.setGoalPosition(ID_GRIP, GRIP_NEUTRAL);  serviceDelay(800);
  DEBUG_SERIAL.println("=== [HOME POSE] 완료 ===");
}

// HOME과 INIT은 같은 실측 자세이므로 한 번만 이동한다.
void safeHoming() { homePose(); }

void powerOnAndReset() {
  DEBUG_SERIAL.println("[INFO] 토크 ON 진행 중...");
  initServos();
  safeHoming();
  DEBUG_SERIAL.println("[INFO] 토크 ON 및 복귀 완료.");
}

void powerOff() {
  for (uint8_t i = 0; i < SERVO_COUNT; i++) dxl.torqueOff(SERVO_IDS[i]);
  DEBUG_SERIAL.println("[INFO] 토크 OFF 완료.");
}

void rebootServos() {
  DEBUG_SERIAL.println("[INFO] 서보 리부트 중...");
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    dxl.reboot(SERVO_IDS[i]);
    delay(200);
  }
  delay(1000);
  initServos();
  DEBUG_SERIAL.println("[INFO] 리부트 완료.");
}

void printStatus() {
  DEBUG_SERIAL.println("--- 현재 상태 ---");
  for (uint8_t i = 0; i < SERVO_COUNT; i++) {
    uint8_t id = SERVO_IDS[i];
    DEBUG_SERIAL.print("ID ");
    DEBUG_SERIAL.print(id);
    if (!dxl.ping(id)) { DEBUG_SERIAL.println(" : 응답 없음"); continue; }
    DEBUG_SERIAL.print(" | pos ");
    DEBUG_SERIAL.print(dxl.getPresentPosition(id, UNIT_RAW));
    DEBUG_SERIAL.print(" | torque ");
    DEBUG_SERIAL.print(dxl.readControlTableItem(ControlTableItem::TORQUE_ENABLE, id));
    DEBUG_SERIAL.print(" | HW_ERR ");
    DEBUG_SERIAL.println(dxl.readControlTableItem(ControlTableItem::HARDWARE_ERROR_STATUS, id));
  }
}


/* ************************************************************
 *  5. Jetson 9바이트 TARGET 수신 + 5바이트 상태 응답
 *
 *  Jetson -> OpenRB:
 *    AA 55 ID SLOT X_H X_L Y_H Y_L CRC8(ID..Y_L)
 *
 *  OpenRB -> Jetson:
 *    AA 55 ID STATUS CRC8(ID, STATUS)
 *
 *  STATUS: ACK=06, DONE=07, NAK=15, BUSY=E0, FAULT=E1
 *  CRC-8: polynomial 0x07, initial value 0x00
 * ************************************************************ */

static constexpr uint8_t kSof0 = 0xAA;
static constexpr uint8_t kSof1 = 0x55;
static constexpr uint8_t kCommandSize = 9;
static constexpr uint8_t kResponseSize = 5;

static constexpr uint8_t kStatusAck   = 0x06;
static constexpr uint8_t kStatusDone  = 0x07;
static constexpr uint8_t kStatusNak   = 0x15;
static constexpr uint8_t kStatusBusy  = 0xE0;
static constexpr uint8_t kStatusFault = 0xE1;

struct TargetCommand {
  uint8_t id;
  uint8_t slot;
  uint16_t x;
  uint16_t y;

  // Arduino의 자동 함수 원형 생성보다 구조체 선언이 늦어지는 문제를
  // 피하기 위해 비교 기능을 구조체 내부에 둔다.
  bool matches(const TargetCommand& other) const {
    return id == other.id && slot == other.slot &&
           x == other.x && y == other.y;
  }
};

static uint8_t rxFrame[kCommandSize];
static uint8_t rxIndex = 0;

// 전원을 켜면 Jetson 명령을 바로 처리한다. USB 명령 "auto"로 끌 수 있다.
bool autoMode = true;
static bool robotBusy = false;
static bool haveActiveCommand = false;
static TargetCommand activeCommand = {0, 0, 0, 0};

// 마지막 DONE/FAULT 응답이 유실됐을 때 같은 명령을 다시 실행하지 않고
// 이전 결과만 재전송한다.
static bool haveLastResult = false;
static TargetCommand lastResultCommand = {0, 0, 0, 0};
static uint8_t lastResultStatus = kStatusFault;

volatile bool newTarget = false;
volatile int16_t targetX = 0;
volatile int16_t targetY = 0;
volatile uint8_t targetCommandId = 0;
volatile uint8_t targetEmptySlot = 4;

static uint8_t crc8Update(uint8_t crc, uint8_t data) {
  crc ^= data;
  for (uint8_t bit = 0; bit < 8; ++bit) {
    crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07)
                       : static_cast<uint8_t>(crc << 1);
  }
  return crc;
}

static uint8_t calcCRC8(const uint8_t* data, size_t length) {
  uint8_t crc = 0;
  for (size_t index = 0; index < length; ++index) {
    crc = crc8Update(crc, data[index]);
  }
  return crc;
}

static void sendResponse(uint8_t commandId, uint8_t status) {
  uint8_t packet[kResponseSize] = {kSof0, kSof1, commandId, status, 0};
  packet[4] = calcCRC8(&packet[2], 2);
  JETSON_SERIAL.write(packet, sizeof(packet));
  JETSON_SERIAL.flush();

  DEBUG_SERIAL.print("[UART TX] id=");
  DEBUG_SERIAL.print(commandId);
  DEBUG_SERIAL.print(" status=0x");
  if (status < 0x10) DEBUG_SERIAL.print('0');
  DEBUG_SERIAL.println(status, HEX);
}

static void acceptTargetFrame(const uint8_t* frame) {
  const uint8_t commandId = frame[2];
  const uint8_t receivedCrc = frame[8];
  const uint8_t expectedCrc = calcCRC8(&frame[2], 6);

  if (receivedCrc != expectedCrc) {
    DEBUG_SERIAL.print("[UART RX BAD] CRC recv=0x");
    DEBUG_SERIAL.print(receivedCrc, HEX);
    DEBUG_SERIAL.print(" expected=0x");
    DEBUG_SERIAL.println(expectedCrc, HEX);
    sendResponse(commandId, kStatusNak);
    return;
  }

  const TargetCommand command = {
    commandId,
    frame[3],
    static_cast<uint16_t>((static_cast<uint16_t>(frame[4]) << 8) | frame[5]),
    static_cast<uint16_t>((static_cast<uint16_t>(frame[6]) << 8) | frame[7])
  };

  DEBUG_SERIAL.print("[TARGET RX] id=");
  DEBUG_SERIAL.print(command.id);
  DEBUG_SERIAL.print(" slot=");
  DEBUG_SERIAL.print(command.slot);
  DEBUG_SERIAL.print(" center=(");
  DEBUG_SERIAL.print(command.x);
  DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(command.y);
  DEBUG_SERIAL.println(')');

  if (!isValidPlaceSlot(command.slot) || command.x >= CAM_W ||
      command.y >= CAM_H) {
    DEBUG_SERIAL.println(
        "[UART RX BAD] slot=1~9, camera X=0~1919, Y=0~1079만 허용");
    sendResponse(command.id, kStatusNak);
    return;
  }

  // DONE 또는 FAULT가 유실되어 Jetson이 같은 프레임을 재전송한 경우이다.
  if (haveLastResult && command.matches(lastResultCommand)) {
    DEBUG_SERIAL.println("[DUPLICATE RESULT] ACK + terminal status resend");
    sendResponse(command.id, kStatusAck);
    sendResponse(command.id, lastResultStatus);
    return;
  }

  // 자동 모드가 꺼져 있으면 명령을 보관하지 않고 BUSY로 거절한다.
  if (!autoMode) {
    DEBUG_SERIAL.println("[BUSY] auto mode is OFF");
    sendResponse(command.id, kStatusBusy);
    return;
  }

  // ACK 유실로 현재 명령이 다시 들어오면 ACK만 재전송한다.
  // 다른 명령은 현재 작업과 섞이지 않도록 BUSY로 거절한다.
  if (robotBusy || newTarget) {
    if (haveActiveCommand && command.matches(activeCommand)) {
      DEBUG_SERIAL.println("[DUPLICATE ACTIVE] ACK resend");
      sendResponse(command.id, kStatusAck);
    } else {
      DEBUG_SERIAL.println("[BUSY] another command rejected");
      sendResponse(command.id, kStatusBusy);
    }
    return;
  }

  activeCommand = command;
  haveActiveCommand = true;
  targetX = static_cast<int16_t>(command.x);
  targetY = static_cast<int16_t>(command.y);
  targetCommandId = command.id;
  targetEmptySlot = command.slot;
  newTarget = true;

  // 검증과 작업 큐 등록이 끝난 뒤 ACK를 보낸다.
  sendResponse(command.id, kStatusAck);
}

static void parseByte(uint8_t value) {
  if (rxIndex == 0) {
    if (value == kSof0) {
      rxFrame[0] = value;
      rxIndex = 1;
    }
    return;
  }

  if (rxIndex == 1) {
    if (value == kSof1) {
      rxFrame[1] = value;
      rxIndex = 2;
    } else if (value == kSof0) {
      rxFrame[0] = value;
      rxIndex = 1;
    } else {
      rxIndex = 0;
    }
    return;
  }

  rxFrame[rxIndex++] = value;
  if (rxIndex == kCommandSize) {
    acceptTargetFrame(rxFrame);
    rxIndex = 0;
  }
}

void jetsonParse() {
  while (JETSON_SERIAL.available() > 0) {
    parseByte(static_cast<uint8_t>(JETSON_SERIAL.read()));
  }
}


/* ************************************************************
 *  6. ★ 고정/좌표 기반 Pick & Place
 *     고정 모드 ON이면 실측 고정 자세, OFF이면 Jetson X/Y 8점 보간 사용
 * ************************************************************ */

bool pickAndPlace(int16_t px, int16_t py, uint8_t placeSlot) {
  DEBUG_SERIAL.print("=== 좌표 집기/빈칸 적재 시작 (x=");
  DEBUG_SERIAL.print(px);
  DEBUG_SERIAL.print(" y=");
  DEBUG_SERIAL.print(py);
  DEBUG_SERIAL.print(" slot=");
  DEBUG_SERIAL.print(placeSlot);
  DEBUG_SERIAL.println(") ===");

  if (!validatePlaceSlot(placeSlot)) return false;

  JointTicks pickPose;
  int gripReference = 0;
  if (USE_FIXED_PICK_POSE) {
    pickPose = FIXED_PICK_POSE;
    gripReference = FIXED_PICK_GRIP_REFERENCE;
    DEBUG_SERIAL.print(
        "[FIXED PICK] UART XY는 참고만 사용, 고정 자세 J=");
    DEBUG_SERIAL.print(pickPose.j1); DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(pickPose.j2); DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(pickPose.j3); DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(pickPose.j4); DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(pickPose.j5);
    DEBUG_SERIAL.print(" gripRef=");
    DEBUG_SERIAL.println(gripReference);
  } else {
    if (!calcMeasuredAreaPoseFromPixel(px, py, pickPose, gripReference)) {
      DEBUG_SERIAL.println("[FAIL] 중심좌표의 8점 보간 자세 계산 실패");
      return false;
    }
  }
  if (!isPoseWithinLimits(pickPose)) {
    DEBUG_SERIAL.println("[FAIL] 선택된 집기 자세가 관절 제한 초과");
    return false;
  }

  // 먼저 로봇팔2의 POSE_AFTER_HOME 형태를 유지한 채 정확히 왼쪽을 본다.
  // 이후 고정 모드 또는 8점 보간으로 선택된 J1~J5 자세로 이동한다.
  const JointTicks leftTransitPose = {
    PICK_LEFT_BASE,
    POSE_AFTER_HOME[1], POSE_AFTER_HOME[2],
    POSE_AFTER_HOME[3], POSE_AFTER_HOME[4]
  };
  const JointTicks baseSafePose = {
    pickPose.j1, 1900, 1800, 2800, J5_NEUTRAL
  };
  const JointTicks abovePose = {
    pickPose.j1, 1900, 1800, 2800, pickPose.j5
  };
  const JointTicks midPose =
      interpolateJointPose(abovePose, pickPose, AREA_MID_PERCENT);
  if (!isPoseWithinLimits(leftTransitPose) ||
      !isPoseWithinLimits(baseSafePose) ||
      !isPoseWithinLimits(abovePose) ||
      !isPoseWithinLimits(midPose)) {
    DEBUG_SERIAL.println("[FAIL] 좌표 접근 경유 자세가 관절 제한 초과");
    return false;
  }

  DEBUG_SERIAL.println("[1] Init/Home -> 정면 안전 자세");
  safeHoming();
  dxl.setGoalPosition(ID_GRIP, GRIP_AFTER_HOME);
  if (!moveFixedAndWait(
          POSE_AFTER_HOME[0], POSE_AFTER_HOME[1], POSE_AFTER_HOME[2],
          POSE_AFTER_HOME[3], POSE_AFTER_HOME[4],
          HOME_MOVE_TIMEOUT_MS)) return false;
  serviceDelay(800);

  DEBUG_SERIAL.println(
      "[2] POSE_AFTER_HOME 형태를 유지하며 왼쪽(J1=3072)으로 회전");
  setProfile(SLOW_SPEED, MOVE_ACCEL);
  if (!moveJointsAndWait(leftTransitPose, HOME_MOVE_TIMEOUT_MS)) {
    setProfile(MOVE_SPEED, MOVE_ACCEL);
    return false;
  }

  DEBUG_SERIAL.print("[3] 선택된 집기 위치 상공으로 이동: J1=");
  DEBUG_SERIAL.println(pickPose.j1);
  if (!moveJointsAndWait(baseSafePose, HOME_MOVE_TIMEOUT_MS) ||
      !moveJointsAndWait(abovePose, HOME_MOVE_TIMEOUT_MS)) {
    setProfile(MOVE_SPEED, MOVE_ACCEL);
    return false;
  }

  DEBUG_SERIAL.println("[4] 집기 위치 상공에서 ID16=2500 집기 준비");
  if (!moveGripperAndWait(
          GRIP_PICK_PREP, GRIP_PICK_PREP_TOLERANCE)) {
    setProfile(MOVE_SPEED, MOVE_ACCEL);
    return false;
  }

  DEBUG_SERIAL.println("[5] 보간된 중간 자세 -> 중심좌표 집기 자세");
  if (!moveJointsAndWait(midPose, HOME_MOVE_TIMEOUT_MS) ||
      !moveJointsAndWait(pickPose, HOME_MOVE_TIMEOUT_MS)) {
    setProfile(MOVE_SPEED, MOVE_ACCEL);
    return false;
  }

  DEBUG_SERIAL.print("[6] 상자에 도달한 후 ID16=2056으로 닫아 파지: ID16=");
  DEBUG_SERIAL.print(GRIP_CLOSE);
  DEBUG_SERIAL.print(" (좌표 보간 참고값=");
  DEBUG_SERIAL.print(gripReference);
  DEBUG_SERIAL.println(")");
  if (!moveGripperAndWait(
          GRIP_CLOSE, GRIP_CLOSE_TOLERANCE)) {
    setProfile(MOVE_SPEED, MOVE_ACCEL);
    return false;
  }

  DEBUG_SERIAL.println(
      "[7] 접근 경로의 역순으로 상승 후 로봇팔2 왼쪽 이동대기 자세");
  if (!moveJointsAndWait(midPose, HOME_MOVE_TIMEOUT_MS) ||
      !moveJointsAndWait(abovePose, HOME_MOVE_TIMEOUT_MS) ||
      !moveJointsAndWait(baseSafePose, HOME_MOVE_TIMEOUT_MS) ||
      !moveJointsAndWait(leftTransitPose, HOME_MOVE_TIMEOUT_MS)) {
    setProfile(MOVE_SPEED, MOVE_ACCEL);
    return false;
  }

  DEBUG_SERIAL.println(
      "[8] 왼쪽 이동대기 자세에서 정면 복귀 + J2 50% 앉은 자세");
  const JointTicks frontSeatedPose =
      makeSeatedPoseAtBase(POSE_AFTER_HOME[0]);
  setProfile(SLOW_SPEED, PLACE_ACCEL);
  const bool seatedOk =
      moveSmoothAndWait(frontSeatedPose, 4, JOINT_TOLERANCE);
  setProfile(MOVE_SPEED, MOVE_ACCEL);
  if (!seatedOk) return false;

  DEBUG_SERIAL.println(
      "[9] 정면에서 로봇팔2의 1~9번 빈칸 넣기 동작 시작");
  if (!moveHeldObjectToSlot(placeSlot)) return false;

  // moveHeldObjectToSlot() 안에서 J2 후퇴와 Home 복귀까지 완료한다.
  DEBUG_SERIAL.println("=== 좌표 집기/빈칸 적재 완료 ===");
  return true;
}


/* ************************************************************
 *  7. 수동 시험 (Jetson 없이 고정 집기 자세 사용)
 * ************************************************************ */

void clawMachineTask() {
  DEBUG_SERIAL.println("=== 수동 고정 자세 집기 작업 시작 ===");
  if (!pickAndPlace(542, 513, selectedPlaceSlot)) {
    DEBUG_SERIAL.println("[FAIL] 수동 작업 중단");
  }
}


/* ************************************************************
 *  8. 명령어 처리
 * ************************************************************ */

void enterFault(const char* message) {
  DEBUG_SERIAL.print("[FAULT] ");
  DEBUG_SERIAL.println(message);

  autoMode = false;
  newTarget = false;
  robotBusy = false;
}

void printHelp() {
  DEBUG_SERIAL.println("\n[명령어 대기 중]");
  DEBUG_SERIAL.println("  auto       : Jetson 패킷/빈칸 자동 모드 ON/OFF");
  DEBUG_SERIAL.println("  auto N     : 수동 기본 칸 N 지정 + 자동 모드 ON");
  DEBUG_SERIAL.println("  claw       : 고정 자세로 수동 집기/적재 작업");
  DEBUG_SERIAL.println("  on         : 토크 ON + 안전 초기화");
  DEBUG_SERIAL.println("  reset      : 안전 초기화 (INIT -> HOME)");
  DEBUG_SERIAL.println("  init       : 기준 자세 (전 관절 0도, 수직)");
  DEBUG_SERIAL.println("  home       : 접힌 대기 자세");
  DEBUG_SERIAL.println("  place N    : 수동 작업의 놓기 칸 지정 (1~9)");
  DEBUG_SERIAL.println("  slot N     : place N과 동일");
  DEBUG_SERIAL.println("  placetest N: N번 칸 경로 저속 시험 (집게 동작 없음)");
  DEBUG_SERIAL.println("  reboot     : 서보 리부트");
  DEBUG_SERIAL.println("  status     : 현재 위치 / 에러 확인");
  DEBUG_SERIAL.println("  off        : 토크 OFF");
  DEBUG_SERIAL.println("  test X Y N : 고정 집기/빈칸 수동 시험(X/Y는 참고 로그)");
}

void processCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();

  if (cmd.startsWith("auto ")) {
    int slot = cmd.substring(5).toInt();
    if (selectPlaceSlot(slot)) {
      autoMode = true;
      DEBUG_SERIAL.print("[INFO] 자동 모드 ON - UART 슬롯 우선, 수동 기본 칸 ");
      DEBUG_SERIAL.print(selectedPlaceSlot);
      DEBUG_SERIAL.println("번");
    }
  }
  else if (cmd == "auto") {
    autoMode = !autoMode;
    DEBUG_SERIAL.print("[INFO] 자동 모드 ");
    DEBUG_SERIAL.println(autoMode ? "ON - Jetson 9바이트 패킷 대기 중..." : "OFF");
    if (autoMode) {
      DEBUG_SERIAL.print("[INFO] 수동 기본 놓기 칸: ");
      DEBUG_SERIAL.println(selectedPlaceSlot);
      DEBUG_SERIAL.println("[INFO] 자동 작업은 UART 패킷의 빈칸 번호를 사용합니다.");
    }
  }
  else if (cmd == "claw")   clawMachineTask();
  else if (cmd == "on")     powerOnAndReset();
  else if (cmd == "reset")  safeHoming();
  else if (cmd == "init")   initPose();
  else if (cmd == "home")   homePose();
  else if (cmd.startsWith("slot")) {
    int firstDigit = cmd.indexOf(' ');
    int slot = (firstDigit >= 0) ? cmd.substring(firstDigit + 1).toInt()
                                 : cmd.substring(4).toInt();
    if (selectPlaceSlot(slot)) {
      printPlaceSlotPose(selectedPlaceSlot);
    }
  }
  else if (cmd.startsWith("placetest")) {
    int firstDigit = cmd.indexOf(' ');
    int slot = (firstDigit >= 0) ? cmd.substring(firstDigit + 1).toInt()
                                 : cmd.substring(9).toInt();
    if (slot >= 1 && slot <= 9) {
      testPlaceSlot((uint8_t)slot);
    } else {
      DEBUG_SERIAL.println("사용법: placetest 1  (범위 1~9)");
    }
  }
  else if (cmd.startsWith("place")) {
    int firstDigit = cmd.indexOf(' ');
    int slot = (firstDigit >= 0) ? cmd.substring(firstDigit + 1).toInt()
                                 : cmd.substring(5).toInt();
    if (selectPlaceSlot(slot)) {
      printPlaceSlotPose(selectedPlaceSlot);
      DEBUG_SERIAL.println(
          "[PLACE] 다음 수동 claw/test 동작에 적용됩니다.");
    }
  }
  else if (cmd == "reboot") rebootServos();
  else if (cmd == "status") printStatus();
  else if (cmd == "off")    powerOff();
  else if (cmd.startsWith("test")) {
    int sp1 = cmd.indexOf(' ', 4);
    if (sp1 > 0) {
      int sp2 = cmd.indexOf(' ', sp1 + 1);
      if (sp2 > 0) {
        int tx = cmd.substring(sp1 + 1, sp2).toInt();
        int sp3 = cmd.indexOf(' ', sp2 + 1);
        int ty = (sp3 > 0) ? cmd.substring(sp2 + 1, sp3).toInt()
                           : cmd.substring(sp2 + 1).toInt();
        int slot = (sp3 > 0) ? cmd.substring(sp3 + 1).toInt()
                             : selectedPlaceSlot;
        DEBUG_SERIAL.print("[TEST] 수동 좌표: px=");
        DEBUG_SERIAL.print(tx);
        DEBUG_SERIAL.print(" py=");
        DEBUG_SERIAL.print(ty);
        DEBUG_SERIAL.print(" slot=");
        DEBUG_SERIAL.println(slot);
        pickAndPlace((int16_t)tx, (int16_t)ty, (uint8_t)slot);
      } else {
        DEBUG_SERIAL.println("사용법: test 480 540 4");
      }
    } else {
      DEBUG_SERIAL.println("사용법: test 480 540 4");
    }
  }
  else if (cmd.length() > 0) {
    DEBUG_SERIAL.println("알 수 없는 명령어입니다.");
    printHelp();
  }
}


/* ************************************************************
 *  9. setup / loop
 * ************************************************************ */

void setup() {
  DEBUG_SERIAL.begin(115200);
  JETSON_SERIAL.begin(115200);    // Serial3 (D13=RX, D14=TX)
  delay(3000);

  dxl.begin(1000000);
  dxl.setPortProtocolVersion(2.0);

  initServos();
  safeHoming();

  DEBUG_SERIAL.println("========================================");
  DEBUG_SERIAL.println(" OpenRB-150 OMX-F + Jetson UART (통합)");
  DEBUG_SERIAL.print(" FW: ");
  DEBUG_SERIAL.println(FIRMWARE_VERSION);
  DEBUG_SERIAL.println(" UART: D13=RX, D14=TX, 115200 8-N-1");
  DEBUG_SERIAL.println(" TARGET: AA 55 ID SLOT X_H X_L Y_H Y_L CRC (9B)");
  DEBUG_SERIAL.println(" RESPONSE: AA 55 ID STATUS CRC (ACK/DONE/NAK/BUSY/FAULT)");
  DEBUG_SERIAL.println(" PICK: J4 바깥/위쪽 최대 보정 자세 J=3020,2500,1395,3060,1882");
  DEBUG_SERIAL.println(
      " PICK MODE: fixed=ON, gripRef=2500 (8-point map retained as fallback)");
  DEBUG_SERIAL.println(" GRIP: prepare=2500, close=2056, place release=2300");
  DEBUG_SERIAL.println(" 자동 모드: 전원 인가 후 기본 ON");
  DEBUG_SERIAL.println(" 수동 기본 놓기 위치: 4번 칸");
  DEBUG_SERIAL.println("========================================");
  printHelp();
}

void loop() {
  // Jetson 9바이트 패킷을 항상 파싱한다.
  jetsonParse();

  // 자동 모드: 좌표와 빈칸이 함께 도착하면 해당 작업을 즉시 수행한다.
  if (autoMode && newTarget) {
    const int16_t jobX = targetX;
    const int16_t jobY = targetY;
    const uint8_t jobSlot = targetEmptySlot;
    const uint8_t jobCommandId = targetCommandId;
    const TargetCommand jobCommand = activeCommand;
    newTarget = false;
    robotBusy = true;

    const bool jobOk = pickAndPlace(jobX, jobY, jobSlot);

    // 재전송된 동일 명령이 로봇을 다시 움직이지 않도록 결과를 먼저 저장한다.
    lastResultCommand = jobCommand;
    lastResultStatus = jobOk ? kStatusDone : kStatusFault;
    haveLastResult = true;
    robotBusy = false;
    haveActiveCommand = false;

    sendResponse(jobCommandId, lastResultStatus);

    if (!jobOk) {
      autoMode = false;
      DEBUG_SERIAL.println(
          "[FAULT TX] 자동 모드를 중지합니다. 상태 확인 후 reset/auto를 입력하세요."
      );
    } else {
      DEBUG_SERIAL.print("[DONE TX] command_id=");
      DEBUG_SERIAL.println(jobCommandId);
    }
  }

  // 디버그 명령어
  if (DEBUG_SERIAL.available()) {
    String cmd = DEBUG_SERIAL.readStringUntil('\n');
    processCommand(cmd);
  }
}
