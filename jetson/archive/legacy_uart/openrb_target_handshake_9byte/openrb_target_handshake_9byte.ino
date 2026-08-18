/*
 * OpenRB-150 simple 9-byte Jetson target handshake
 *
 * Wiring (verified project UART):
 *   Jetson USB-TTL TX -> OpenRB D13 / Serial3 RX
 *   Jetson USB-TTL RX <- OpenRB D14 / Serial3 TX
 *   GND               <-> GND
 *   USB-TTL +5V is not connected
 *
 * Jetson -> OpenRB (9 bytes):
 *   AA 55 ID SLOT X_H X_L Y_H Y_L CRC8(ID..Y_L)
 *
 * OpenRB -> Jetson (5 bytes):
 *   AA 55 ID STATUS CRC8(ID, STATUS)
 *
 * Status:
 *   06 ACK, 07 DONE, 15 NAK, E0 BUSY, E1 FAULT
 *
 * This sketch does not move the robot.  It waits SIMULATED_WORK_MS after ACK
 * and then sends DONE.  Replace finishSimulatedWork() with the real robot
 * call only after the UART handshake has been verified.
 */

#include <Arduino.h>

#define DEBUG_SERIAL Serial
#define JETSON_SERIAL Serial3

static constexpr uint32_t UART_BAUD = 115200;
static constexpr uint8_t HEADER0 = 0xAA;
static constexpr uint8_t HEADER1 = 0x55;
static constexpr uint8_t COMMAND_SIZE = 9;
static constexpr uint8_t RESPONSE_SIZE = 5;

static constexpr uint8_t STATUS_ACK = 0x06;
static constexpr uint8_t STATUS_DONE = 0x07;
static constexpr uint8_t STATUS_NAK = 0x15;
static constexpr uint8_t STATUS_BUSY = 0xE0;

static constexpr uint16_t CAMERA_WIDTH = 1920;
static constexpr uint16_t CAMERA_HEIGHT = 1080;
static constexpr uint32_t SIMULATED_WORK_MS = 1000;

struct TargetCommand {
  uint8_t id;
  uint8_t slot;
  uint16_t x;
  uint16_t y;
};

static uint8_t rxFrame[COMMAND_SIZE];
static uint8_t rxIndex = 0;

static bool busy = false;
static TargetCommand activeCommand = {0, 0, 0, 0};
static uint32_t workStartedMs = 0;

static bool haveLastCompleted = false;
static TargetCommand lastCompleted = {0, 0, 0, 0};

uint8_t crc8(const uint8_t* data, size_t length) {
  uint8_t crc = 0;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80)
                ? static_cast<uint8_t>((crc << 1) ^ 0x07)
                : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

bool sameCommand(const TargetCommand& a, const TargetCommand& b) {
  return a.id == b.id && a.slot == b.slot && a.x == b.x && a.y == b.y;
}

void sendResponse(uint8_t commandId, uint8_t status) {
  uint8_t response[RESPONSE_SIZE] = {
      HEADER0, HEADER1, commandId, status, 0};
  response[4] = crc8(&response[2], 2);
  JETSON_SERIAL.write(response, RESPONSE_SIZE);
  JETSON_SERIAL.flush();

  DEBUG_SERIAL.print("[TX] id=");
  DEBUG_SERIAL.print(commandId);
  DEBUG_SERIAL.print(" status=0x");
  if (status < 0x10) DEBUG_SERIAL.print('0');
  DEBUG_SERIAL.println(status, HEX);
}

void handleFrame(const uint8_t* frame) {
  const uint8_t commandId = frame[2];
  const uint8_t receivedCrc = frame[8];
  const uint8_t expectedCrc = crc8(&frame[2], 6);

  if (receivedCrc != expectedCrc) {
    DEBUG_SERIAL.println("[RX BAD] CRC mismatch");
    sendResponse(commandId, STATUS_NAK);
    return;
  }

  TargetCommand command = {
      commandId,
      frame[3],
      static_cast<uint16_t>((static_cast<uint16_t>(frame[4]) << 8) | frame[5]),
      static_cast<uint16_t>((static_cast<uint16_t>(frame[6]) << 8) | frame[7])};

  if (command.slot < 1 || command.slot > 9 ||
      command.x >= CAMERA_WIDTH || command.y >= CAMERA_HEIGHT) {
    DEBUG_SERIAL.println("[RX BAD] slot/coordinate out of range");
    sendResponse(command.id, STATUS_NAK);
    return;
  }

  DEBUG_SERIAL.print("[RX] id=");
  DEBUG_SERIAL.print(command.id);
  DEBUG_SERIAL.print(" slot=");
  DEBUG_SERIAL.print(command.slot);
  DEBUG_SERIAL.print(" center=(");
  DEBUG_SERIAL.print(command.x);
  DEBUG_SERIAL.print(',');
  DEBUG_SERIAL.print(command.y);
  DEBUG_SERIAL.println(')');

  // DONE response was lost: return the old result without moving again.
  if (haveLastCompleted && sameCommand(command, lastCompleted)) {
    DEBUG_SERIAL.println("[DUPLICATE DONE] ACK + DONE resend");
    sendResponse(command.id, STATUS_ACK);
    sendResponse(command.id, STATUS_DONE);
    return;
  }

  if (busy) {
    // ACK response was lost: acknowledge the same active command again.
    if (sameCommand(command, activeCommand)) {
      DEBUG_SERIAL.println("[DUPLICATE BUSY] ACK resend");
      sendResponse(command.id, STATUS_ACK);
    } else {
      DEBUG_SERIAL.println("[BUSY] another command rejected");
      sendResponse(command.id, STATUS_BUSY);
    }
    return;
  }

  activeCommand = command;
  busy = true;
  workStartedMs = millis();
  sendResponse(command.id, STATUS_ACK);
}

void readJetsonFrames() {
  while (JETSON_SERIAL.available() > 0) {
    const uint8_t value = static_cast<uint8_t>(JETSON_SERIAL.read());

    if (rxIndex == 0) {
      if (value == HEADER0) {
        rxFrame[0] = value;
        rxIndex = 1;
      }
      continue;
    }

    if (rxIndex == 1) {
      if (value == HEADER1) {
        rxFrame[1] = value;
        rxIndex = 2;
      } else if (value == HEADER0) {
        rxFrame[0] = value;
        rxIndex = 1;
      } else {
        rxIndex = 0;
      }
      continue;
    }

    rxFrame[rxIndex++] = value;
    if (rxIndex == COMMAND_SIZE) {
      handleFrame(rxFrame);
      rxIndex = 0;
    }
  }
}

void finishSimulatedWork() {
  if (!busy || millis() - workStartedMs < SIMULATED_WORK_MS) return;

  // Replace this simulated completion with the real robot action result.
  lastCompleted = activeCommand;
  haveLastCompleted = true;
  sendResponse(activeCommand.id, STATUS_DONE);
  busy = false;

  DEBUG_SERIAL.println("[DONE] simulated robot work completed");
}

void setup() {
  DEBUG_SERIAL.begin(115200);
  JETSON_SERIAL.begin(UART_BAUD);
  delay(1000);

  DEBUG_SERIAL.println("OpenRB 9-byte target handshake ready");
  DEBUG_SERIAL.println("Serial3: D13=RX, D14=TX, 115200 8-N-1");
  DEBUG_SERIAL.println("CMD: AA 55 ID SLOT X_H X_L Y_H Y_L CRC8");
}

void loop() {
  readJetsonFrames();
  finishSimulatedWork();
}
