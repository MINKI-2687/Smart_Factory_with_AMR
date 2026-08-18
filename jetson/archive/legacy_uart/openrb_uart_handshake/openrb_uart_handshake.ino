/* OpenRB-150 Serial2 <-> Jetson framed UART handshake test.
 *
 * Jetson -> OpenRB: AA 55 command_id slot CRC8(command_id, slot)
 * OpenRB -> Jetson: AA 55 command_id status CRC8(command_id, status)
 * status: 06=ACK, 07=DONE, 15=NAK, E1=FAULT
 *
 * The LED action is only a bench test. Replace executeSlot() with the actual
 * DYNAMIXEL motion and return only after that motion has really completed.
 */

#include <Arduino.h>

constexpr uint32_t UART_BAUD = 115200;
constexpr uint8_t SOF_1 = 0xAA;
constexpr uint8_t SOF_2 = 0x55;
constexpr uint8_t STATUS_ACK = 0x06;
constexpr uint8_t STATUS_DONE = 0x07;
constexpr uint8_t STATUS_NAK = 0x15;
constexpr uint8_t STATUS_FAULT = 0xE1;
constexpr size_t FRAME_SIZE = 5;

uint8_t rxFrame[FRAME_SIZE];
size_t rxIndex = 0;
bool haveLastCommand = false;
uint8_t lastCommandId = 0;
uint8_t lastSlot = 0;

uint8_t crc8(const uint8_t *data, size_t length)
{
  uint8_t crc = 0;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80U) ? static_cast<uint8_t>((crc << 1) ^ 0x07U)
                          : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

void sendStatus(uint8_t commandId, uint8_t status)
{
  uint8_t response[FRAME_SIZE] = {SOF_1, SOF_2, commandId, status, 0};
  response[4] = crc8(&response[2], 2);
  Serial2.write(response, sizeof(response));
  Serial2.flush();
}

bool executeSlot(uint8_t slot)
{
  (void)slot;
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  delay(150);
  return true;
}

void processFrame(const uint8_t *frame)
{
  const uint8_t commandId = frame[2];
  const uint8_t slot = frame[3];
  const bool valid =
      slot >= 1 && slot <= 9 && frame[4] == crc8(&frame[2], 2);

  if (!valid) {
    sendStatus(commandId, STATUS_NAK);
    return;
  }

  if (haveLastCommand && commandId == lastCommandId) {
    if (slot == lastSlot) {
      // Jetson retry: repeat responses without executing the motion twice.
      sendStatus(commandId, STATUS_ACK);
      sendStatus(commandId, STATUS_DONE);
    } else {
      sendStatus(commandId, STATUS_NAK);
    }
    return;
  }

  sendStatus(commandId, STATUS_ACK);
  if (!executeSlot(slot)) {
    sendStatus(commandId, STATUS_FAULT);
    return;
  }

  lastCommandId = commandId;
  lastSlot = slot;
  haveLastCommand = true;
  sendStatus(commandId, STATUS_DONE);
}

void consumeByte(uint8_t value)
{
  if (rxIndex == 0 && value != SOF_1) {
    return;
  }
  if (rxIndex == 1 && value != SOF_2) {
    rxIndex = (value == SOF_1) ? 1 : 0;
    rxFrame[0] = value;
    return;
  }

  rxFrame[rxIndex++] = value;
  if (rxIndex == FRAME_SIZE) {
    processFrame(rxFrame);
    rxIndex = 0;
  }
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial2.begin(UART_BAUD);
}

void loop()
{
  while (Serial2.available() > 0) {
    consumeByte(static_cast<uint8_t>(Serial2.read()));
  }
}
