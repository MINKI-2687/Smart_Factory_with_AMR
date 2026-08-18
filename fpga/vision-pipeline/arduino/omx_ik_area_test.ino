#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/*
 * OpenRB-150 / OMX-F clean AREA pick-and-place test
 *
 * Cycle:
 *   HOME -> safe approach -> target XY HIGH -> open gripper
 *   -> same XY MID -> same XY PICK -> close gripper
 *   -> same XY MID/HIGH -> fixed PLACE -> release -> HOME
 *
 * USB serial remains available for manual tests. Zybo/Vitis uses Serial3:
 *   TARGET: AA 55 01 Seq X_L X_H Y_L Y_H CRC8
 *   ACK:    AA 55 02 Seq CRC8
 * CRC-8 polynomial 0x07, initial value 0x00.
 */

#define DXL_SERIAL Serial1
#define DEBUG_SERIAL Serial
#define ZYBO_SERIAL Serial3

Dynamixel2Arduino dxl(DXL_SERIAL, -1);

namespace {

constexpr float kPi = 3.14159265358979323846f;

// Dynamixel IDs.
constexpr uint8_t kIdJ1 = 11;
constexpr uint8_t kIdJ2 = 12;
constexpr uint8_t kIdJ3 = 13;
constexpr uint8_t kIdJ4 = 14;
constexpr uint8_t kIdJ5 = 15;
constexpr uint8_t kIdGrip = 16;
const uint8_t kServoIds[] = {
    kIdJ1, kIdJ2, kIdJ3, kIdJ4, kIdJ5, kIdGrip};
constexpr uint8_t kArmJointCount = 5;
constexpr uint8_t kServoCount = 6;

// Motion profile. These values are faster than the diagnostic build while
// remaining below the supplied sketch's original velocity of 45.
constexpr int kMoveSpeed = 30;
constexpr int kHomeSpeed = 25;
constexpr int kMoveAcceleration = 20;
constexpr unsigned long kArmMoveTimeoutMs = 20000UL;
constexpr unsigned long kHomeMoveTimeoutMs = 25000UL;
constexpr unsigned long kArmSettledMs = 400UL;
constexpr int kMeaningfulMotionTick = 1;

// Limits and measured fixed poses retained from the supplied working code.
const int kJointMin[kArmJointCount] = {0, 683, 683, 910, 0};
const int kJointMax[kArmJointCount] = {4095, 3072, 3100, 3210, 4095};
const int kPoseHome[kArmJointCount] = {1018, 747, 3079, 3194, 2023};
const int kPoseHomeEntry[kArmJointCount] = {1024, 1850, 1750, 2875, 2048};
const int kPoseFrontSafe[kArmJointCount] = {2048, 1900, 1800, 2800, 2048};
constexpr int kJ5Neutral = 2048;

constexpr int kGripOpen = 2230;
constexpr int kGripClose = 1800;
constexpr int kGripNeutral = 2053;
constexpr int kGripToleranceTick = 40;
constexpr unsigned long kGripMoveTimeoutMs = 5000UL;
constexpr unsigned long kGripStableMs = 250UL;
constexpr unsigned long kGripCloseHoldMs = 1500UL;
constexpr unsigned long kGripReleaseRetryNeutralMs = 350UL;
constexpr unsigned long kFreshTargetRetryPeriodMs = 1000UL;

// AREA coordinates from the marked 320 x 180 mm work surface.
//   A: 0..1920 maps bottom -> top over 320 mm.
//   B: 0..1080 maps near -> far over 180 mm.
constexpr float kAreaAUnits = 1920.0f;
constexpr float kAreaBUnits = 1080.0f;
constexpr float kAreaSideLengthMm = 320.0f;
constexpr float kAreaDepthLengthMm = 180.0f;

// Effective J1-axis to AREA-near distance after the previous successful
// center calibration: 80 mm marked gap + 41.3 mm body offset - 20 mm trim.
constexpr float kAreaNearForwardMm = 101.3f;
constexpr float kDepthTrimMm = -2.0f;
constexpr float kSideTrimMm = 2.0f;
constexpr float kAreaYawDeg = 0.0f;

// The measured table is model Z=45 mm. The requested physical PICK height is
// 15 mm, so the center command is model Z=60 mm.
constexpr float kMeasuredTableModelZMm = 45.0f;
constexpr float kPickPhysicalClearanceMm = 15.0f;
constexpr float kPickBaseZMm =
    kMeasuredTableModelZMm + kPickPhysicalClearanceMm;
constexpr float kApproachZMm = 120.0f;

// At A=0/B=540 the old fixed-Z command reached about 10 mm lower than the
// center. Interpolate an upward command correction by radial reach while
// keeping the requested physical clearance at about 15 mm.
constexpr long kHeightReferenceA = 960;
constexpr long kHeightReferenceB = 540;
constexpr long kHeightEdgeA = 0;
constexpr long kHeightEdgeB = 540;
constexpr float kHeightEdgeRaiseMm = 10.0f;
constexpr float kHeightMaxRaiseMm = 20.0f;

// Measured fixed release pose. ID 16 is intentionally excluded because the
// normal release step must still command kGripOpen.
const int kPosePlaceRelease[kArmJointCount] = {
    1089, 2478, 1434, 2947, 2087};

// OMX-F geometry used by the already verified IK/FK model.
constexpr float kLinkBaseHeightMm = 97.5f;
constexpr float kLinkShoulderMm = 120.5204236f;
constexpr float kLinkElbowMm = 162.0f;
constexpr float kToolLengthMm = 120.63f;
constexpr float kShoulderOffsetRad = 1.2192606473f;
// Keep the proven 68 degree grasp angle through the normal work area. Only
// the far corners need the wrist to extend: smoothly reduce the pitch to 54
// degrees as radial reach grows from 280 to 324 mm.
constexpr float kToolPitchNearDeg = 68.0f;
constexpr float kToolPitchFarDeg = 54.0f;
constexpr float kPitchReductionStartRadiusMm = 280.0f;
constexpr float kPitchReductionEndRadiusMm = 324.0f;
constexpr float kTicksPerRadian = 4096.0f / (2.0f * kPi);
const int kJointZeroTick[kArmJointCount] = {2048, 2048, 2048, 2048, 2048};

struct JointTicks {
  int value[kArmJointCount];
};

struct RobotPoint {
  float forwardMm;
  float sideMm;
};

struct CartesianPlan {
  long areaA;
  long areaB;
  RobotPoint point;
  float correctionZMm;
  float targetZMm;
  float middleZMm;
  float highToolPitchDeg;
  float middleToolPitchDeg;
  float toolPitchDeg;
  int8_t elbowBranch;
  JointTicks high;
  JointTicks middle;
  JointTicks target;
};

bool gArmReady = false;
bool gGripperReady = false;
bool gAtHome = false;
bool gBusy = false;
bool gFault = false;
bool gHoldingObject = false;
bool gAutoMode = false;
bool gNewUartTarget = false;
bool gRequestFreshTargetAfterHome = false;
bool gAwaitingFreshTarget = false;
uint16_t gUartTargetA = 0;
uint16_t gUartTargetB = 0;
uint8_t gUartTargetSequence = 0;
uint8_t gFreshTargetRetrySequence = 0;
unsigned long gLastFreshTargetRetryMs = 0UL;

float degreesToRadians(float degrees) {
  return degrees * kPi / 180.0f;
}

JointTicks makePose(const int pose[kArmJointCount]) {
  JointTicks result;
  for (uint8_t index = 0; index < kArmJointCount; ++index) {
    result.value[index] = pose[index];
  }
  return result;
}

JointTicks makeFoldedPose(int baseTick) {
  JointTicks result = {{baseTick, 1900, 1800, 2800, kJ5Neutral}};
  return result;
}

bool poseIsValid(const JointTicks& pose, bool printError) {
  for (uint8_t index = 0; index < kArmJointCount; ++index) {
    if (pose.value[index] < kJointMin[index] ||
        pose.value[index] > kJointMax[index]) {
      if (printError) {
        DEBUG_SERIAL.print("[LIMIT] J");
        DEBUG_SERIAL.print(index + 1);
        DEBUG_SERIAL.print(" tick=");
        DEBUG_SERIAL.print(pose.value[index]);
        DEBUG_SERIAL.print(" allowed=");
        DEBUG_SERIAL.print(kJointMin[index]);
        DEBUG_SERIAL.print("..");
        DEBUG_SERIAL.println(kJointMax[index]);
      }
      return false;
    }
  }
  return true;
}

int jointRadiansToTick(uint8_t index, float radians) {
  return static_cast<int>(lroundf(
      static_cast<float>(kJointZeroTick[index]) +
      radians * kTicksPerRadian));
}

float jointTickToRadians(uint8_t index, int tick) {
  return
      (static_cast<float>(tick) -
       static_cast<float>(kJointZeroTick[index])) /
      kTicksPerRadian;
}

void printPose(const char* label, const JointTicks& pose) {
  DEBUG_SERIAL.print(label);
  for (uint8_t index = 0; index < kArmJointCount; ++index) {
    DEBUG_SERIAL.print(" J");
    DEBUG_SERIAL.print(index + 1);
    DEBUG_SERIAL.print('=');
    DEBUG_SERIAL.print(pose.value[index]);
  }
  DEBUG_SERIAL.println();
}

bool areaToRobot(long areaA, long areaB, RobotPoint& output) {
  if (areaA < 0 || areaA > 1920 || areaB < 0 || areaB > 1080) {
    DEBUG_SERIAL.println("[AREA] range: A=0..1920, B=0..1080");
    return false;
  }

  const float localSideMm =
      static_cast<float>(areaA) / kAreaAUnits * kAreaSideLengthMm -
      kAreaSideLengthMm / 2.0f;
  const float localDepthMm =
      static_cast<float>(areaB) / kAreaBUnits * kAreaDepthLengthMm;
  const float yaw = degreesToRadians(kAreaYawDeg);
  const float cosine = cosf(yaw);
  const float sine = sinf(yaw);

  output.forwardMm =
      kAreaNearForwardMm + kDepthTrimMm +
      cosine * localDepthMm - sine * localSideMm;
  output.sideMm =
      kSideTrimMm + sine * localDepthMm + cosine * localSideMm;
  return true;
}

float pointRadius(const RobotPoint& point) {
  return sqrtf(
      point.forwardMm * point.forwardMm +
      point.sideMm * point.sideMm);
}

float calculateToolPitchDeg(const RobotPoint& point) {
  const float spanMm =
      kPitchReductionEndRadiusMm - kPitchReductionStartRadiusMm;
  float ratio =
      (pointRadius(point) - kPitchReductionStartRadiusMm) / spanMm;
  ratio = constrain(ratio, 0.0f, 1.0f);
  return kToolPitchNearDeg +
         (kToolPitchFarDeg - kToolPitchNearDeg) * ratio;
}

bool calculatePickZ(
    const RobotPoint& targetPoint,
    float& targetZMm,
    float& correctionMm) {
  RobotPoint referencePoint;
  RobotPoint edgePoint;
  if (!areaToRobot(kHeightReferenceA, kHeightReferenceB, referencePoint) ||
      !areaToRobot(kHeightEdgeA, kHeightEdgeB, edgePoint)) {
    return false;
  }

  const float referenceRadiusMm = pointRadius(referencePoint);
  const float calibrationSpanMm =
      pointRadius(edgePoint) - referenceRadiusMm;
  if (calibrationSpanMm <= 0.001f) {
    DEBUG_SERIAL.println("[CONFIG] invalid height calibration span");
    return false;
  }

  correctionMm =
      (pointRadius(targetPoint) - referenceRadiusMm) /
      calibrationSpanMm * kHeightEdgeRaiseMm;
  correctionMm = constrain(correctionMm, 0.0f, kHeightMaxRaiseMm);
  targetZMm = kPickBaseZMm + correctionMm;
  return true;
}

bool solveIk(
    const RobotPoint& target,
    float targetZMm,
    float toolPitchDeg,
    int8_t elbowBranch,
    JointTicks& output,
    bool printError) {
  const float radialMm = pointRadius(target);
  const float baseAngle = atan2f(target.sideMm, target.forwardMm);
  const float toolPitchRad = degreesToRadians(toolPitchDeg);
  const float wristRadialMm =
      radialMm - kToolLengthMm * cosf(toolPitchRad);
  const float wristHeightMm =
      targetZMm - kLinkBaseHeightMm +
      kToolLengthMm * sinf(toolPitchRad);
  const float numerator =
      wristRadialMm * wristRadialMm +
      wristHeightMm * wristHeightMm -
      kLinkShoulderMm * kLinkShoulderMm -
      kLinkElbowMm * kLinkElbowMm;
  const float denominator =
      2.0f * kLinkShoulderMm * kLinkElbowMm;
  float elbowCosine = numerator / denominator;

  if (elbowCosine < -1.0001f || elbowCosine > 1.0001f) {
    if (printError) {
      DEBUG_SERIAL.print("[IK] unreachable r=");
      DEBUG_SERIAL.print(radialMm, 1);
      DEBUG_SERIAL.print(" z=");
      DEBUG_SERIAL.println(targetZMm, 1);
    }
    return false;
  }

  elbowCosine = constrain(elbowCosine, -1.0f, 1.0f);
  const float geometricElbow =
      static_cast<float>(elbowBranch) * acosf(elbowCosine);
  const float shoulderLinkAngle =
      atan2f(wristHeightMm, wristRadialMm) -
      atan2f(
          kLinkElbowMm * sinf(geometricElbow),
          kLinkShoulderMm +
              kLinkElbowMm * cosf(geometricElbow));
  const float elbowLinkAngle =
      shoulderLinkAngle + geometricElbow;

  const float jointAngles[kArmJointCount] = {
      baseAngle,
      kShoulderOffsetRad - shoulderLinkAngle,
      -kShoulderOffsetRad - geometricElbow,
      toolPitchRad + elbowLinkAngle,
      0.0f};

  for (uint8_t index = 0; index < kArmJointCount; ++index) {
    output.value[index] =
        jointRadiansToTick(index, jointAngles[index]);
  }
  output.value[4] = kJ5Neutral;
  return poseIsValid(output, printError);
}

long planDistanceFromFolded(const CartesianPlan& plan) {
  const JointTicks folded = makeFoldedPose(plan.high.value[0]);
  long distance = 0;
  for (uint8_t index = 0; index < kArmJointCount; ++index) {
    distance += labs(
        static_cast<long>(plan.high.value[index]) -
        static_cast<long>(folded.value[index]));
  }
  return distance;
}

bool buildPlanForBranch(
    long areaA,
    long areaB,
    float targetZMm,
    float correctionZMm,
    int8_t elbowBranch,
    CartesianPlan& plan,
    bool printError) {
  plan.areaA = areaA;
  plan.areaB = areaB;
  plan.targetZMm = targetZMm;
  plan.correctionZMm = correctionZMm;
  plan.middleZMm = (kApproachZMm + targetZMm) * 0.5f;
  plan.elbowBranch = elbowBranch;

  if (!areaToRobot(areaA, areaB, plan.point)) {
    return false;
  }
  plan.toolPitchDeg = calculateToolPitchDeg(plan.point);
  plan.highToolPitchDeg = plan.toolPitchDeg;
  plan.middleToolPitchDeg = plan.toolPitchDeg;
  if (targetZMm >= kApproachZMm) {
    if (printError) {
      DEBUG_SERIAL.println("[CONFIG] target Z must be below approach Z");
    }
    return false;
  }

  return solveIk(
             plan.point,
             kApproachZMm,
             plan.toolPitchDeg,
             elbowBranch,
             plan.high,
             printError) &&
         solveIk(
             plan.point,
             plan.middleZMm,
             plan.toolPitchDeg,
             elbowBranch,
             plan.middle,
             printError) &&
         solveIk(
             plan.point,
             targetZMm,
             plan.toolPitchDeg,
             elbowBranch,
             plan.target,
             printError);
}

bool selectPlan(
    long areaA,
    long areaB,
    float targetZMm,
    float correctionZMm,
    CartesianPlan& selected) {
  CartesianPlan negative;
  CartesianPlan positive;
  const bool negativeValid = buildPlanForBranch(
      areaA, areaB, targetZMm, correctionZMm, -1, negative, false);
  const bool positiveValid = buildPlanForBranch(
      areaA, areaB, targetZMm, correctionZMm, 1, positive, false);

  if (!negativeValid && !positiveValid) {
    DEBUG_SERIAL.println("[PLAN] coordinate is outside the OMX-F IK range");
    buildPlanForBranch(
        areaA, areaB, targetZMm, correctionZMm, -1, negative, true);
    buildPlanForBranch(
        areaA, areaB, targetZMm, correctionZMm, 1, positive, true);
    return false;
  }

  if (negativeValid && positiveValid) {
    selected = planDistanceFromFolded(negative) <=
                       planDistanceFromFolded(positive)
                   ? negative
                   : positive;
  } else {
    selected = negativeValid ? negative : positive;
  }
  return true;
}

bool makePickPlan(long areaA, long areaB, CartesianPlan& plan) {
  RobotPoint point;
  if (!areaToRobot(areaA, areaB, point)) {
    return false;
  }
  float targetZMm = kPickBaseZMm;
  float correctionMm = 0.0f;
  if (!calculatePickZ(point, targetZMm, correctionMm)) {
    return false;
  }
  return selectPlan(areaA, areaB, targetZMm, correctionMm, plan);
}

bool makePlacePlan(CartesianPlan& plan) {
  plan.areaA = -1;
  plan.areaB = -1;
  plan.correctionZMm = 0.0f;
  plan.target = makePose(kPosePlaceRelease);

  if (!poseIsValid(plan.target, true)) {
    return false;
  }

  const float baseAngle = jointTickToRadians(0, plan.target.value[0]);
  const float shoulderLinkAngle =
      kShoulderOffsetRad - jointTickToRadians(1, plan.target.value[1]);
  const float geometricElbow =
      -kShoulderOffsetRad - jointTickToRadians(2, plan.target.value[2]);
  const float elbowLinkAngle = shoulderLinkAngle + geometricElbow;
  const float toolPitchRad =
      jointTickToRadians(3, plan.target.value[3]) - elbowLinkAngle;
  const float wristRadialMm =
      kLinkShoulderMm * cosf(shoulderLinkAngle) +
      kLinkElbowMm * cosf(elbowLinkAngle);
  const float wristHeightMm =
      kLinkShoulderMm * sinf(shoulderLinkAngle) +
      kLinkElbowMm * sinf(elbowLinkAngle);
  const float radialMm =
      wristRadialMm + kToolLengthMm * cosf(toolPitchRad);

  plan.point.forwardMm = radialMm * cosf(baseAngle);
  plan.point.sideMm = radialMm * sinf(baseAngle);
  plan.targetZMm =
      kLinkBaseHeightMm + wristHeightMm -
      kToolLengthMm * sinf(toolPitchRad);
  plan.middleZMm = (kApproachZMm + plan.targetZMm) * 0.5f;
  plan.toolPitchDeg = toolPitchRad * 180.0f / kPi;
  plan.highToolPitchDeg = calculateToolPitchDeg(plan.point);
  plan.middleToolPitchDeg =
      (plan.highToolPitchDeg + plan.toolPitchDeg) * 0.5f;
  plan.elbowBranch = geometricElbow < 0.0f ? -1 : 1;

  if (plan.targetZMm >= kApproachZMm) {
    DEBUG_SERIAL.println(
        "[PLACE] measured release pose must be below approach Z");
    return false;
  }

  return solveIk(
             plan.point,
             kApproachZMm,
             plan.highToolPitchDeg,
             plan.elbowBranch,
             plan.high,
             true) &&
         solveIk(
             plan.point,
             plan.middleZMm,
             plan.middleToolPitchDeg,
             plan.elbowBranch,
             plan.middle,
             true);
}

void printPlan(const char* label, const CartesianPlan& plan) {
  DEBUG_SERIAL.println("----------------------------------------");
  DEBUG_SERIAL.print(label);
  if (plan.areaA < 0 || plan.areaB < 0) {
    DEBUG_SERIAL.println(" fixed measured joint pose");
  } else {
    DEBUG_SERIAL.print(" AREA A=");
    DEBUG_SERIAL.print(plan.areaA);
    DEBUG_SERIAL.print(" B=");
    DEBUG_SERIAL.println(plan.areaB);
  }
  DEBUG_SERIAL.print("robot forward=");
  DEBUG_SERIAL.print(plan.point.forwardMm, 1);
  DEBUG_SERIAL.print(" side=");
  DEBUG_SERIAL.print(plan.point.sideMm, 1);
  DEBUG_SERIAL.print(" r=");
  DEBUG_SERIAL.println(pointRadius(plan.point), 1);
  DEBUG_SERIAL.print("Z high/mid/target=");
  DEBUG_SERIAL.print(kApproachZMm, 1);
  DEBUG_SERIAL.print('/');
  DEBUG_SERIAL.print(plan.middleZMm, 1);
  DEBUG_SERIAL.print('/');
  DEBUG_SERIAL.print(plan.targetZMm, 1);
  DEBUG_SERIAL.print(" mm, height correction=");
  DEBUG_SERIAL.print(plan.correctionZMm, 1);
  DEBUG_SERIAL.print(" mm, pitch high/mid/target=");
  DEBUG_SERIAL.print(plan.highToolPitchDeg, 1);
  DEBUG_SERIAL.print('/');
  DEBUG_SERIAL.print(plan.middleToolPitchDeg, 1);
  DEBUG_SERIAL.print('/');
  DEBUG_SERIAL.print(plan.toolPitchDeg, 1);
  DEBUG_SERIAL.print(" deg, branch=");
  DEBUG_SERIAL.println(plan.elbowBranch);
  printPose("HIGH  ", plan.high);
  printPose("MID   ", plan.middle);
  printPose("TARGET", plan.target);
  DEBUG_SERIAL.println("----------------------------------------");
}

// Vitis UART protocol copied from the supplied, working integration code.
constexpr uint8_t kUartSof0 = 0xAA;
constexpr uint8_t kUartSof1 = 0x55;
constexpr uint8_t kUartTypeTarget = 0x01;
constexpr uint8_t kUartTypeAck = 0x02;
constexpr uint8_t kUartTypeRetry = 0x03;
constexpr size_t kUartTargetPayloadSize = 6;

uint8_t gUartParserState = 0;
uint8_t gUartPayload[kUartTargetPayloadSize] = {};
size_t gUartPayloadIndex = 0;
bool gHaveLastUartSequence = false;
uint8_t gLastUartSequence = 0;

uint8_t crc8Update(uint8_t crc, uint8_t data) {
  crc ^= data;
  for (uint8_t bit = 0; bit < 8; ++bit) {
    crc = (crc & 0x80)
              ? static_cast<uint8_t>((crc << 1) ^ 0x07)
              : static_cast<uint8_t>(crc << 1);
  }
  return crc;
}

uint8_t calculateCrc8(const uint8_t* data, size_t length) {
  uint8_t crc = 0;
  for (size_t index = 0; index < length; ++index) {
    crc = crc8Update(crc, data[index]);
  }
  return crc;
}

void sendUartAck(uint8_t sequence) {
  uint8_t packet[5] = {
      kUartSof0, kUartSof1, kUartTypeAck, sequence, 0};
  packet[4] = calculateCrc8(&packet[2], 2);
  ZYBO_SERIAL.write(packet, sizeof(packet));
  ZYBO_SERIAL.flush();
  DEBUG_SERIAL.print("[ACK TX] seq=");
  DEBUG_SERIAL.println(sequence);
}

void sendUartRetry(uint8_t sequence) {
  uint8_t packet[5] = {
      kUartSof0, kUartSof1, kUartTypeRetry, sequence, 0};
  packet[4] = calculateCrc8(&packet[2], 2);

  // Send a few copies because RETRY is a one-way recovery notification and
  // Vitis may be transitioning from ACK handling to its car-present wait.
  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    ZYBO_SERIAL.write(packet, sizeof(packet));
    ZYBO_SERIAL.flush();
    delay(20);
  }
  DEBUG_SERIAL.print("[RETRY TX] HOME ready; request fresh target after seq=");
  DEBUG_SERIAL.println(sequence);
}

void acceptUartTarget(const uint8_t* payload) {
  const uint8_t sequence = payload[1];
  const uint16_t areaA =
      static_cast<uint16_t>(payload[2]) |
      (static_cast<uint16_t>(payload[3]) << 8);
  const uint16_t areaB =
      static_cast<uint16_t>(payload[4]) |
      (static_cast<uint16_t>(payload[5]) << 8);

  DEBUG_SERIAL.print("[TARGET RX] seq=");
  DEBUG_SERIAL.print(sequence);
  DEBUG_SERIAL.print(" A=");
  DEBUG_SERIAL.print(areaA);
  DEBUG_SERIAL.print(" B=");
  DEBUG_SERIAL.println(areaB);

  // ACK every valid TARGET, including retransmissions, so Vitis can stop
  // retrying even while the arm is in a blocking motion sequence.
  sendUartAck(sequence);

  if (gHaveLastUartSequence && sequence == gLastUartSequence) {
    DEBUG_SERIAL.println("[UART] duplicate sequence; ACK only");
    return;
  }

  gAwaitingFreshTarget = false;
  gHaveLastUartSequence = true;
  gLastUartSequence = sequence;
  gUartTargetA = areaA;
  gUartTargetB = areaB;
  gUartTargetSequence = sequence;
  gNewUartTarget = true;
}

void parseUartByte(uint8_t byte) {
  switch (gUartParserState) {
    case 0:
      gUartParserState = byte == kUartSof0 ? 1 : 0;
      break;

    case 1:
      if (byte == kUartSof1) {
        gUartPayloadIndex = 0;
        gUartParserState = 2;
      } else {
        gUartParserState = byte == kUartSof0 ? 1 : 0;
      }
      break;

    case 2:
      gUartPayload[gUartPayloadIndex++] = byte;
      if (gUartPayloadIndex == kUartTargetPayloadSize) {
        gUartParserState = 3;
      }
      break;

    default:
      if (gUartPayload[0] != kUartTypeTarget) {
        DEBUG_SERIAL.print("[UART WARN] unknown type=0x");
        DEBUG_SERIAL.println(gUartPayload[0], HEX);
      } else {
        const uint8_t expectedCrc =
            calculateCrc8(gUartPayload, kUartTargetPayloadSize);
        if (byte != expectedCrc) {
          DEBUG_SERIAL.print("[UART WARN] CRC recv=0x");
          DEBUG_SERIAL.print(byte, HEX);
          DEBUG_SERIAL.print(" expected=0x");
          DEBUG_SERIAL.println(expectedCrc, HEX);
        } else {
          acceptUartTarget(gUartPayload);
        }
      }
      gUartParserState = 0;
      gUartPayloadIndex = 0;
      break;
  }
}

void zyboParse() {
  while (ZYBO_SERIAL.available() > 0) {
    parseUartByte(static_cast<uint8_t>(ZYBO_SERIAL.read()));
  }
}

void serviceFreshTargetRetry() {
  if (!gAwaitingFreshTarget || !gAutoMode || !gArmReady ||
      !gAtHome || gBusy || gFault) {
    return;
  }

  if (millis() - gLastFreshTargetRetryMs < kFreshTargetRetryPeriodMs) {
    return;
  }

  sendUartRetry(gFreshTargetRetrySequence);
  gLastFreshTargetRetryMs = millis();
}

void serviceDelay(unsigned long durationMs) {
  const unsigned long started = millis();
  while (millis() - started < durationMs) {
    zyboParse();
    delay(2);
  }
}

void torqueOffAll() {
  for (uint8_t index = 0; index < kServoCount; ++index) {
    dxl.torqueOff(kServoIds[index]);
  }
  gArmReady = false;
  gGripperReady = false;
  gAtHome = false;
  gBusy = false;
  gHoldingObject = false;
  gAutoMode = false;
  gNewUartTarget = false;
  gRequestFreshTargetAfterHome = false;
  gAwaitingFreshTarget = false;
  gHaveLastUartSequence = false;
  gUartParserState = 0;
  gUartPayloadIndex = 0;
}

bool pollEmergencyStop() {
  if (DEBUG_SERIAL.available() <= 0) {
    return false;
  }

  String command = DEBUG_SERIAL.readStringUntil('\n');
  command.trim();
  command.toLowerCase();
  if (command != "off" && command != "stop" && command != "!") {
    DEBUG_SERIAL.println("[BUSY] command ignored during motion");
    return false;
  }

  torqueOffAll();
  gFault = true;
  DEBUG_SERIAL.println("[EMERGENCY] torque OFF");
  return true;
}

bool armHasHardwareError() {
  for (uint8_t index = 0; index < kArmJointCount; ++index) {
    const uint32_t error = dxl.readControlTableItem(
        ControlTableItem::HARDWARE_ERROR_STATUS,
        kServoIds[index]);
    if (error != 0) {
      DEBUG_SERIAL.print("[HW ERROR] J");
      DEBUG_SERIAL.print(index + 1);
      DEBUG_SERIAL.print(" value=");
      DEBUG_SERIAL.println(error);
      return true;
    }
  }
  return false;
}

bool gripperHasHardwareError() {
  if (!gGripperReady) {
    return true;
  }

  const uint32_t error = dxl.readControlTableItem(
      ControlTableItem::HARDWARE_ERROR_STATUS, kIdGrip);
  if (error != 0) {
    DEBUG_SERIAL.print("[HW ERROR] gripper value=");
    DEBUG_SERIAL.println(error);
    return true;
  }
  return false;
}

bool readArmPose(JointTicks& pose) {
  if (!gArmReady) {
    return false;
  }
  for (uint8_t index = 0; index < kArmJointCount; ++index) {
    const uint8_t id = kServoIds[index];
    const uint32_t error = dxl.readControlTableItem(
        ControlTableItem::HARDWARE_ERROR_STATUS, id);
    if (error != 0) {
      DEBUG_SERIAL.print("[HW ERROR] J");
      DEBUG_SERIAL.print(index + 1);
      DEBUG_SERIAL.print(" value=");
      DEBUG_SERIAL.println(error);
      return false;
    }
    pose.value[index] = static_cast<int>(
        lroundf(dxl.getPresentPosition(id, UNIT_RAW)));
  }
  return true;
}

void setServoSpeed(int velocity) {
  for (uint8_t index = 0; index < kServoCount; ++index) {
    dxl.writeControlTableItem(
        ControlTableItem::PROFILE_VELOCITY,
        kServoIds[index],
        velocity);
  }
}

bool sendArmGoal(const JointTicks& target) {
  if (!gArmReady || !poseIsValid(target, true)) {
    return false;
  }
  for (uint8_t index = 0; index < kArmJointCount; ++index) {
    dxl.setGoalPosition(kServoIds[index], target.value[index]);
  }
  return true;
}

bool waitForArmSettled(
    const JointTicks& target,
    unsigned long timeoutMs,
    const char* label) {
  const unsigned long started = millis();
  unsigned long lastMotionMs = started;
  JointTicks previous = {{0, 0, 0, 0, 0}};
  JointTicks present = {{0, 0, 0, 0, 0}};
  bool havePrevious = false;

  while (millis() - started < timeoutMs) {
    zyboParse();
    if (pollEmergencyStop()) {
      return false;
    }
    if (!readArmPose(present)) {
      return false;
    }

    bool moved = false;
    for (uint8_t index = 0; index < kArmJointCount; ++index) {
      if (havePrevious &&
          abs(present.value[index] - previous.value[index]) >
              kMeaningfulMotionTick) {
        moved = true;
      }
      previous.value[index] = present.value[index];
    }

    if (moved) {
      lastMotionMs = millis();
    } else if (havePrevious &&
               millis() - lastMotionMs >= kArmSettledMs) {
      int largestResidual = 0;
      for (uint8_t index = 0; index < kArmJointCount; ++index) {
        const int residual =
            abs(target.value[index] - present.value[index]);
        if (residual > largestResidual) {
          largestResidual = residual;
        }
      }
      if (largestResidual > 80) {
        DEBUG_SERIAL.print("[POSE WARN] ");
        DEBUG_SERIAL.print(label);
        DEBUG_SERIAL.print(" settled with max residual=");
        DEBUG_SERIAL.println(largestResidual);
      }
      return true;
    }

    havePrevious = true;
    delay(20);
  }

  DEBUG_SERIAL.print("[MOVE FAIL] ");
  DEBUG_SERIAL.print(label);
  DEBUG_SERIAL.println(" did not settle");
  return false;
}

bool moveArmPose(
    const JointTicks& target,
    const char* label,
    unsigned long timeoutMs = kArmMoveTimeoutMs) {
  DEBUG_SERIAL.print("[MOVE] ");
  DEBUG_SERIAL.println(label);
  return sendArmGoal(target) &&
         waitForArmSettled(target, timeoutMs, label);
}

bool holdArmAtPresent() {
  JointTicks present;
  if (!readArmPose(present)) {
    return false;
  }
  return sendArmGoal(present);
}

bool openGripper() {
  if (!gGripperReady) {
    DEBUG_SERIAL.println("[GRIP FAIL] gripper is not ready");
    return false;
  }

  DEBUG_SERIAL.println("[GRIP] open");
  dxl.setGoalPosition(kIdGrip, kGripOpen);
  const unsigned long started = millis();
  unsigned long stableSince = 0UL;
  bool stable = false;

  while (millis() - started < kGripMoveTimeoutMs) {
    zyboParse();
    if (pollEmergencyStop()) {
      return false;
    }
    const uint32_t error = dxl.readControlTableItem(
        ControlTableItem::HARDWARE_ERROR_STATUS, kIdGrip);
    if (error != 0) {
      DEBUG_SERIAL.print("[GRIP FAIL] HW_ERR=");
      DEBUG_SERIAL.println(error);
      return false;
    }

    const int present = static_cast<int>(lroundf(
        dxl.getPresentPosition(kIdGrip, UNIT_RAW)));
    if (abs(present - kGripOpen) <= kGripToleranceTick) {
      if (!stable) {
        stable = true;
        stableSince = millis();
      } else if (millis() - stableSince >= kGripStableMs) {
        return true;
      }
    } else {
      stable = false;
    }
    delay(20);
  }

  DEBUG_SERIAL.println("[GRIP FAIL] open timeout");
  return false;
}

bool closeGripper() {
  if (!gGripperReady) {
    DEBUG_SERIAL.println("[GRIP FAIL] gripper is not ready");
    return false;
  }

  DEBUG_SERIAL.println("[GRIP] close and hold");
  dxl.setGoalPosition(kIdGrip, kGripClose);
  const unsigned long started = millis();
  while (millis() - started < kGripCloseHoldMs) {
    zyboParse();
    if (pollEmergencyStop()) {
      return false;
    }
    const uint32_t error = dxl.readControlTableItem(
        ControlTableItem::HARDWARE_ERROR_STATUS, kIdGrip);
    if (error != 0) {
      DEBUG_SERIAL.print("[GRIP FAIL] HW_ERR=");
      DEBUG_SERIAL.println(error);
      return false;
    }
    delay(20);
  }
  return true;
}

bool releaseGripperWithRecovery() {
  if (openGripper()) {
    return true;
  }

  if (!gArmReady || !gGripperReady || gFault ||
      gripperHasHardwareError()) {
    return false;
  }

  gRequestFreshTargetAfterHome = true;

  DEBUG_SERIAL.println(
      "[GRIP RECOVER] open timeout; neutral then retry open");
  dxl.setGoalPosition(kIdGrip, kGripNeutral);
  serviceDelay(kGripReleaseRetryNeutralMs);

  if (openGripper()) {
    DEBUG_SERIAL.println("[GRIP RECOVER] release retry succeeded");
    return true;
  }

  if (!gArmReady || !gGripperReady || gFault ||
      gripperHasHardwareError()) {
    return false;
  }

  // A position timeout without a Dynamixel hardware error is recoverable for
  // the continuous demonstration. Keep commanding open while the arm follows
  // the known PLACE lift path instead of locking the whole cycle.
  dxl.setGoalPosition(kIdGrip, kGripOpen);
  DEBUG_SERIAL.println(
      "[GRIP WARN] release position not confirmed; continuing safe HOME return");
  return true;
}

bool initializeServos() {
  bool allFound = true;
  gGripperReady = false;

  for (uint8_t index = 0; index < kServoCount; ++index) {
    const uint8_t id = kServoIds[index];
    if (!dxl.ping(id)) {
      DEBUG_SERIAL.print("[INIT FAIL] DXL ID ");
      DEBUG_SERIAL.print(id);
      DEBUG_SERIAL.println(" not found");
      allFound = false;
      continue;
    }
    dxl.torqueOn(id);
    dxl.writeControlTableItem(
        ControlTableItem::PROFILE_VELOCITY, id, kMoveSpeed);
    dxl.writeControlTableItem(
        ControlTableItem::PROFILE_ACCELERATION, id, kMoveAcceleration);
    if (id == kIdGrip) {
      gGripperReady = true;
    }
  }

  gArmReady = allFound;
  return allFound && gGripperReady;
}

bool moveHomePose() {
  setServoSpeed(kHomeSpeed);
  const bool reached = moveArmPose(
      makePose(kPoseHome), "HOME", kHomeMoveTimeoutMs);
  setServoSpeed(kMoveSpeed);
  if (reached) {
    dxl.setGoalPosition(kIdGrip, kGripNeutral);
    serviceDelay(500);
  }
  gAtHome = reached;
  return reached;
}

bool failCycle(const char* reason) {
  DEBUG_SERIAL.print("[CYCLE FAIL] ");
  DEBUG_SERIAL.println(reason);
  if (gArmReady && !armHasHardwareError()) {
    holdArmAtPresent();
  }
  gBusy = false;
  gAtHome = false;
  gFault = true;
  return false;
}

bool executeCycle(const CartesianPlan& pick, const CartesianPlan& place) {
  if (!gArmReady || !gGripperReady) {
    DEBUG_SERIAL.println("[CYCLE] use 'on' first");
    return false;
  }
  if (gBusy) {
    DEBUG_SERIAL.println("[CYCLE] already busy");
    return false;
  }
  if (gFault) {
    DEBUG_SERIAL.println("[CYCLE] fault active; use home or on");
    return false;
  }
  if (!gAtHome) {
    DEBUG_SERIAL.println("[CYCLE] a new move can start only from HOME");
    return false;
  }

  gBusy = true;
  gAtHome = false;
  gHoldingObject = false;
  gRequestFreshTargetAfterHome = false;
  setServoSpeed(kMoveSpeed);
  DEBUG_SERIAL.println("[CYCLE] HOME -> PICK -> PLACE -> HOME");

  if (!moveArmPose(makePose(kPoseHomeEntry), "HOME_ENTRY") ||
      !moveArmPose(makePose(kPoseFrontSafe), "FRONT_SAFE") ||
      !moveArmPose(
          makeFoldedPose(pick.high.value[0]),
          "rotate folded toward PICK") ||
      !moveArmPose(pick.high, "PICK HIGH")) {
    return failCycle("could not reach PICK approach");
  }

  if (!openGripper()) {
    return failCycle("gripper did not open");
  }
  if (!moveArmPose(pick.middle, "PICK MID") ||
      !moveArmPose(pick.target, "PICK Z=15mm physical target")) {
    return failCycle("PICK descent failed");
  }

  // Once the final PICK command has settled, close without a second TCP or
  // joint-tolerance rejection. This is the deterministic behavior requested
  // for the demonstration cycle.
  if (!holdArmAtPresent() || !closeGripper()) {
    return failCycle("gripper close failed");
  }
  gHoldingObject = true;

  if (!moveArmPose(pick.middle, "PICK MID lift") ||
      !moveArmPose(pick.high, "PICK HIGH lift") ||
      !moveArmPose(
          makeFoldedPose(pick.high.value[0]),
          "fold at PICK") ||
      !moveArmPose(
          makeFoldedPose(place.high.value[0]),
          "rotate folded toward PLACE") ||
      !moveArmPose(place.high, "PLACE HIGH") ||
      !moveArmPose(place.middle, "PLACE MID") ||
      !moveArmPose(place.target, "PLACE RELEASE")) {
    return failCycle("could not carry object to PLACE");
  }

  if (!holdArmAtPresent() || !releaseGripperWithRecovery()) {
    return failCycle("object release failed");
  }
  gHoldingObject = false;

  if (!moveArmPose(place.middle, "PLACE MID lift") ||
      !moveArmPose(place.high, "PLACE HIGH lift") ||
      !moveArmPose(makePose(kPoseHomeEntry), "HOME_ENTRY return") ||
      !moveHomePose()) {
    return failCycle("HOME return failed");
  }

  gBusy = false;
  gFault = false;
  DEBUG_SERIAL.println("[CYCLE DONE] ready for the next move");
  return true;
}

void processPendingUartTarget() {
  if (!gAutoMode || !gNewUartTarget || gBusy) {
    return;
  }
  if (!gArmReady || !gAtHome || gFault) {
    return;
  }

  const long areaA = static_cast<long>(gUartTargetA);
  const long areaB = static_cast<long>(gUartTargetB);
  const uint8_t sequence = gUartTargetSequence;
  gNewUartTarget = false;

  CartesianPlan pick;
  CartesianPlan place;
  if (!makePickPlan(areaA, areaB, pick) || !makePlacePlan(place)) {
    DEBUG_SERIAL.print("[AUTO FAIL] invalid TARGET seq=");
    DEBUG_SERIAL.println(sequence);
    gAutoMode = false;
    return;
  }

  DEBUG_SERIAL.print("[AUTO START] seq=");
  DEBUG_SERIAL.print(sequence);
  DEBUG_SERIAL.print(" A=");
  DEBUG_SERIAL.print(areaA);
  DEBUG_SERIAL.print(" B=");
  DEBUG_SERIAL.println(areaB);
  printPlan("PICK", pick);
  if (!executeCycle(pick, place)) {
    gAutoMode = false;
    DEBUG_SERIAL.println("[AUTO] stopped after motion failure");
    return;
  }

  if (gRequestFreshTargetAfterHome) {
    gAwaitingFreshTarget = true;
    gFreshTargetRetrySequence = sequence;
    gLastFreshTargetRetryMs = millis();
    sendUartRetry(gFreshTargetRetrySequence);
    gRequestFreshTargetAfterHome = false;
  }
}

void printStatus() {
  DEBUG_SERIAL.println("--- DYNAMIXEL STATUS ---");
  for (uint8_t index = 0; index < kServoCount; ++index) {
    const uint8_t id = kServoIds[index];
    DEBUG_SERIAL.print("ID ");
    DEBUG_SERIAL.print(id);
    DEBUG_SERIAL.print(" pos=");
    DEBUG_SERIAL.print(dxl.getPresentPosition(id, UNIT_RAW));
    DEBUG_SERIAL.print(" hwErr=");
    DEBUG_SERIAL.println(dxl.readControlTableItem(
        ControlTableItem::HARDWARE_ERROR_STATUS, id));
  }
  DEBUG_SERIAL.print("state ready/home/busy/fault/holding=");
  DEBUG_SERIAL.print(gArmReady);
  DEBUG_SERIAL.print('/');
  DEBUG_SERIAL.print(gAtHome);
  DEBUG_SERIAL.print('/');
  DEBUG_SERIAL.print(gBusy);
  DEBUG_SERIAL.print('/');
  DEBUG_SERIAL.print(gFault);
  DEBUG_SERIAL.print('/');
  DEBUG_SERIAL.println(gHoldingObject);
  DEBUG_SERIAL.print("uart auto/pending/retryWait/lastSeq=");
  DEBUG_SERIAL.print(gAutoMode);
  DEBUG_SERIAL.print('/');
  DEBUG_SERIAL.print(gNewUartTarget);
  DEBUG_SERIAL.print('/');
  DEBUG_SERIAL.print(gAwaitingFreshTarget);
  DEBUG_SERIAL.print('/');
  if (gHaveLastUartSequence) {
    DEBUG_SERIAL.println(gLastUartSequence);
  } else {
    DEBUG_SERIAL.println("none");
  }
}

void printConfiguration() {
  DEBUG_SERIAL.println("--- CLEAN IK CONFIG ---");
  DEBUG_SERIAL.print("AREA mm=");
  DEBUG_SERIAL.print(kAreaSideLengthMm, 0);
  DEBUG_SERIAL.print('x');
  DEBUG_SERIAL.println(kAreaDepthLengthMm, 0);
  DEBUG_SERIAL.print("AREA near forward=");
  DEBUG_SERIAL.print(kAreaNearForwardMm, 1);
  DEBUG_SERIAL.println(" mm");
  DEBUG_SERIAL.print("PICK physical clearance=");
  DEBUG_SERIAL.print(kPickPhysicalClearanceMm, 1);
  DEBUG_SERIAL.print(" mm, center model Z=");
  DEBUG_SERIAL.println(kPickBaseZMm, 1);
  DEBUG_SERIAL.print("edge radial height correction=");
  DEBUG_SERIAL.print(kHeightEdgeRaiseMm, 1);
  DEBUG_SERIAL.println(" mm");
  DEBUG_SERIAL.print("tool pitch near/far/radius start-end=");
  DEBUG_SERIAL.print(kToolPitchNearDeg, 1);
  DEBUG_SERIAL.print('/');
  DEBUG_SERIAL.print(kToolPitchFarDeg, 1);
  DEBUG_SERIAL.print(" deg/");
  DEBUG_SERIAL.print(kPitchReductionStartRadiusMm, 0);
  DEBUG_SERIAL.print('-');
  DEBUG_SERIAL.print(kPitchReductionEndRadiusMm, 0);
  DEBUG_SERIAL.println(" mm");
  DEBUG_SERIAL.print("fixed PLACE pose");
  for (uint8_t index = 0; index < kArmJointCount; ++index) {
    DEBUG_SERIAL.print(" J");
    DEBUG_SERIAL.print(index + 1);
    DEBUG_SERIAL.print('=');
    DEBUG_SERIAL.print(kPosePlaceRelease[index]);
  }
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.print("speed/home/acceleration=");
  DEBUG_SERIAL.print(kMoveSpeed);
  DEBUG_SERIAL.print('/');
  DEBUG_SERIAL.print(kHomeSpeed);
  DEBUG_SERIAL.print('/');
  DEBUG_SERIAL.println(kMoveAcceleration);
  DEBUG_SERIAL.println("UART Serial3=115200 8-N-1, D13=RX D14=TX");
  DEBUG_SERIAL.println("UART TARGET=AA 55 01 Seq A_L A_H B_L B_H CRC8");
  DEBUG_SERIAL.println("UART ACK=AA 55 02 Seq CRC8");
}

void printHelp() {
  DEBUG_SERIAL.println("Commands:");
  DEBUG_SERIAL.println("  on          torque on and HOME");
  DEBUG_SERIAL.println("  move A B    full PICK -> PLACE -> HOME cycle");
  DEBUG_SERIAL.println("  A B         same as move A B");
  DEBUG_SERIAL.println("  calc A B    calculate only");
  DEBUG_SERIAL.println("  home        return through HOME_ENTRY");
  DEBUG_SERIAL.println("  status      servo/state status");
  DEBUG_SERIAL.println("  config      coordinate and Z settings");
  DEBUG_SERIAL.println("  auto        enable Vitis TARGET execution");
  DEBUG_SERIAL.println("  auto off    disable Vitis TARGET execution");
  DEBUG_SERIAL.println("  release     open gripper at HOME");
  DEBUG_SERIAL.println("  off/stop/!  torque off immediately");
}

bool parseCoordinateCommand(
    const String& line,
    char operation[12],
    long& areaA,
    long& areaB) {
  char extra = '\0';
  return sscanf(
             line.c_str(),
             "%11s %ld %ld %c",
             operation,
             &areaA,
             &areaB,
             &extra) == 3;
}

void powerOnAndHome() {
  DEBUG_SERIAL.println("[POWER] initialize servos");
  gFault = false;
  if (!initializeServos()) {
    gFault = true;
    DEBUG_SERIAL.println("[POWER FAIL] servo initialization failed");
    return;
  }
  if (!moveHomePose()) {
    gFault = true;
    DEBUG_SERIAL.println("[POWER FAIL] HOME failed");
    return;
  }
  DEBUG_SERIAL.println("[POWER] HOME ready");
}

void returnHomeCommand() {
  if (!gArmReady || gBusy) {
    DEBUG_SERIAL.println("[HOME] arm is not ready or is busy");
    return;
  }
  gFault = false;
  if (!moveArmPose(makePose(kPoseHomeEntry), "HOME_ENTRY manual") ||
      !moveHomePose()) {
    gFault = true;
    return;
  }
  gHoldingObject = false;
  DEBUG_SERIAL.println("[HOME] ready");
}

void processCommand(String line) {
  line.trim();
  if (line.length() == 0) {
    return;
  }

  String lower = line;
  lower.toLowerCase();
  lower.replace(',', ' ');

  char operation[12] = {};
  long areaA = 0;
  long areaB = 0;
  bool coordinateCommand =
      parseCoordinateCommand(lower, operation, areaA, areaB);

  char extra = '\0';
  if (!coordinateCommand &&
      sscanf(lower.c_str(), "%ld %ld %c", &areaA, &areaB, &extra) == 2) {
    strcpy(operation, "move");
    coordinateCommand = true;
  }

  if (coordinateCommand &&
      (strcmp(operation, "move") == 0 ||
       strcmp(operation, "test") == 0 ||
       strcmp(operation, "calc") == 0)) {
    CartesianPlan pick;
    CartesianPlan place;
    if (!makePickPlan(areaA, areaB, pick) || !makePlacePlan(place)) {
      return;
    }
    printPlan("PICK", pick);
    if (strcmp(operation, "calc") != 0) {
      executeCycle(pick, place);
    }
    return;
  }

  if (lower == "on") {
    powerOnAndHome();
  } else if (lower == "home" || lower == "recover") {
    returnHomeCommand();
  } else if (lower == "status") {
    printStatus();
  } else if (lower == "config") {
    printConfiguration();
  } else if (lower == "auto" || lower == "auto on") {
    gAutoMode = true;
    DEBUG_SERIAL.println(
        "[AUTO] ON; waiting for a valid Vitis TARGET at HOME");
  } else if (lower == "auto off" || lower == "manual") {
    gAutoMode = false;
    gNewUartTarget = false;
    gAwaitingFreshTarget = false;
    DEBUG_SERIAL.println("[AUTO] OFF");
  } else if (lower == "release") {
    if (gArmReady && gAtHome && !gBusy && openGripper()) {
      gHoldingObject = false;
    } else {
      DEBUG_SERIAL.println("[RELEASE] allowed only at HOME");
    }
  } else if (lower == "off" || lower == "stop" || lower == "!") {
    torqueOffAll();
    DEBUG_SERIAL.println("[POWER] torque OFF");
  } else if (lower == "help") {
    printHelp();
  } else {
    DEBUG_SERIAL.println("Unknown command");
    printHelp();
  }
}

}  // namespace

void setup() {
  DEBUG_SERIAL.begin(115200);
  DEBUG_SERIAL.setTimeout(100);
  ZYBO_SERIAL.begin(115200);
  delay(3000);

  dxl.begin(1000000);
  dxl.setPortProtocolVersion(2.0);
  torqueOffAll();

  DEBUG_SERIAL.println("========================================");
  DEBUG_SERIAL.println(" OMX-F CLEAN IK + VITIS UART");
  DEBUG_SERIAL.println(" Serial3 D13=RX D14=TX, 115200 8-N-1");
  DEBUG_SERIAL.println("========================================");
  printConfiguration();
  printHelp();
}

void loop() {
  zyboParse();
  processPendingUartTarget();
  serviceFreshTargetRetry();

  if (DEBUG_SERIAL.available() > 0) {
    processCommand(DEBUG_SERIAL.readStringUntil('\n'));
  }
}
