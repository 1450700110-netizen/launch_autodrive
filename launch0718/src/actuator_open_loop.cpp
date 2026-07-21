#include "actuator_open_loop.h"

#include <Arduino.h>

#include "config.h"
#include "shared_rs485.h"

namespace
{
enum class ActuatorCommand
{
  Unknown,
  AutoMode,
  TargetLeft,
  TargetCenter,
  TargetRight
};

ActuatorCommand lastCommand = ActuatorCommand::Unknown;
unsigned long lastHallReadMs = 0;
unsigned long lastControlTxMs = 0;
unsigned long hallReadSentMs = 0;
unsigned long lastHallPrintMs = 0;
bool hallReadPending = false;
ActuatorFeedback feedback = {};

const uint8_t READ_HALL_CMD[] = {0xFF, 0x03, 0x10, 0x00, 0x00, 0x13, 0x15, 0x19};
const uint8_t AUTO_MODE_CMD[] = {0xFF, 0x10, 0x10, 0x11, 0x00, 0x01, 0x02, 0x00, 0x01, 0x3D, 0x74};
const uint8_t TARGET_RIGHT_CMD[] = {0xFF, 0x10, 0x10, 0x12, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x89, 0x51};
const uint8_t TARGET_CENTER_CMD[] = {0xFF, 0x10, 0x10, 0x12, 0x00, 0x02, 0x04, 0x00, 0x00, 0x01, 0x7D, 0x48, 0xE0};
const uint8_t TARGET_LEFT_CMD[] = {0xFF, 0x10, 0x10, 0x12, 0x00, 0x02, 0x04, 0x00, 0x00, 0x02, 0xFB, 0xC9, 0xB2};

uint16_t modbusCrc16(const uint8_t *data, size_t length)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i)
  {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit)
    {
      crc = (crc & 0x0001) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001) : static_cast<uint16_t>(crc >> 1);
    }
  }
  return crc;
}

void printHexByte(uint8_t value)
{
  if (value < 0x10)
  {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

void sendRawCommand(const char *name, const uint8_t *data, size_t length)
{
  if (!ENABLE_ACTUATOR_OPEN_LOOP)
  {
    return;
  }

  if (ENABLE_STATUS_TELEMETRY)
  {
    Serial.print("[ACT OPEN TX] ");
    Serial.print(name);
    Serial.print(" ");
    for (size_t i = 0; i < length; ++i)
    {
      printHexByte(data[i]);
      Serial.print(i + 1 == length ? '\n' : ' ');
    }
  }

  const size_t written = sharedRs485Write(data, length);
  if (written != length && ENABLE_STATUS_TELEMETRY)
  {
    Serial.printf("[ACT OPEN TX] short write %u/%u\n",
                  static_cast<unsigned>(written),
                  static_cast<unsigned>(length));
  }
}

void sendCommand(ActuatorCommand command, const char *name, const uint8_t *data, size_t length)
{
  if (!ENABLE_ACTUATOR_OPEN_LOOP || command == lastCommand)
  {
    return;
  }

  sendRawCommand(name, data, length);
  lastCommand = command;
}

bool parseHallFrame(const uint8_t *frame, size_t length)
{
  if (length != 43 || frame[0] != 0xFF || frame[1] != 0x03 || frame[2] != 0x26)
  {
    return false;
  }

  const uint16_t receivedCrc = static_cast<uint16_t>(frame[41]) |
                               (static_cast<uint16_t>(frame[42]) << 8);
  if (modbusCrc16(frame, 41) != receivedCrc)
  {
    return false;
  }

  const uint16_t hallHigh = (static_cast<uint16_t>(frame[11]) << 8) | frame[12];
  const uint16_t hallLow = (static_cast<uint16_t>(frame[13]) << 8) | frame[14];
  const uint32_t wireHall = (static_cast<uint32_t>(hallHigh) << 16) | hallLow;

  // This actuator returns the calibrated count in bits 8..23 (for example 0x0002FB00 -> 0x02FB).
  uint32_t rawHall = wireHall;
  if (rawHall > ACTUATOR_HALL_MAX && (rawHall & 0xFFU) == 0)
  {
    rawHall >>= 8;
  }
  if (rawHall < ACTUATOR_HALL_MIN || rawHall > ACTUATOR_HALL_MAX)
  {
    return false;
  }

  const uint32_t range = ACTUATOR_HALL_MAX - ACTUATOR_HALL_MIN;
  const uint32_t offset = rawHall - ACTUATOR_HALL_MIN;
  feedback.hallRaw = rawHall;
  feedback.positionTenthsMm = static_cast<uint16_t>((offset * ACTUATOR_STROKE_TENTHS_MM + range / 2) / range);
  feedback.positionPercent = static_cast<uint8_t>((offset * 100U + range / 2) / range);
  feedback.updatedMs = millis();
  feedback.valid = true;
  hallReadPending = false;

  if (ENABLE_STATUS_TELEMETRY && millis() - lastHallPrintMs >= 1000)
  {
    lastHallPrintMs = millis();
    Serial.printf("[ACT HALL] raw=%lu position=%u.%u mm percent=%u%%\n",
                  static_cast<unsigned long>(feedback.hallRaw),
                  feedback.positionTenthsMm / 10,
                  feedback.positionTenthsMm % 10,
                  feedback.positionPercent);
  }
  return true;
}

void handleActuatorFrame(const uint8_t *frame, size_t length)
{
  if (length == 0)
  {
    return;
  }

  const bool parsed = parseHallFrame(frame, length);
  if (!parsed && ENABLE_STATUS_TELEMETRY)
  {
    Serial.print("[ACT OPEN RX] ");
    for (size_t i = 0; i < length; ++i)
    {
      printHexByte(frame[i]);
      Serial.print(i + 1 == length ? '\n' : ' ');
    }
  }
}
} // namespace

void setupActuatorOpenLoop()
{
  if (!ENABLE_ACTUATOR_OPEN_LOOP)
  {
    return;
  }

  setActuatorRs485FrameHandler(handleActuatorFrame);
  if (ENABLE_STATUS_TELEMETRY)
  {
    Serial.printf("[ACT OPEN] shared RS485 TX=GPIO%d RX=GPIO%d baud=%lu\n",
                  PIN_ACTUATOR_RS485_TX,
                  PIN_ACTUATOR_RS485_RX,
                  ACTUATOR_RS485_BAUD);
  }

  delay(4000);
  sendCommand(ActuatorCommand::TargetCenter, "TARGET_CENTER", TARGET_CENTER_CMD, sizeof(TARGET_CENTER_CMD));
  delay(2000);
  sendCommand(ActuatorCommand::AutoMode, "AUTO_MODE", AUTO_MODE_CMD, sizeof(AUTO_MODE_CMD));
  delay(300);
  sendRawCommand("AUTO_MODE_RETRY", AUTO_MODE_CMD, sizeof(AUTO_MODE_CMD));
}

void handleActuatorOpenLoop()
{
  if (!ENABLE_ACTUATOR_OPEN_LOOP)
  {
    return;
  }

  const unsigned long now = millis();
  if (hallReadPending && now - hallReadSentMs >= ACTUATOR_HALL_RESPONSE_TIMEOUT_MS)
  {
    hallReadPending = false;
  }

  if (ENABLE_ACTUATOR_PERIODIC_HALL_READ && !hallReadPending &&
      now - lastControlTxMs >= ACTUATOR_CONTROL_GUARD_MS &&
      now - lastHallReadMs >= ACTUATOR_HALL_READ_INTERVAL_MS)
  {
    lastHallReadMs = now;
    sendRawCommand("READ_HALL", READ_HALL_CMD, sizeof(READ_HALL_CMD));
    hallReadSentMs = millis();
    hallReadPending = true;
  }
}

void actuatorOpenLoopExtend()
{
  sendRawCommand("AUTO_MODE_KEEP", AUTO_MODE_CMD, sizeof(AUTO_MODE_CMD));
  delay(80);
  sendCommand(ActuatorCommand::TargetLeft, "TARGET_LEFT", TARGET_LEFT_CMD, sizeof(TARGET_LEFT_CMD));
  lastControlTxMs = millis();
}

void actuatorOpenLoopRetract()
{
  sendRawCommand("AUTO_MODE_KEEP", AUTO_MODE_CMD, sizeof(AUTO_MODE_CMD));
  delay(80);
  sendCommand(ActuatorCommand::TargetRight, "TARGET_RIGHT", TARGET_RIGHT_CMD, sizeof(TARGET_RIGHT_CMD));
  lastControlTxMs = millis();
}

void actuatorOpenLoopBrake()
{
  sendRawCommand("AUTO_MODE_KEEP", AUTO_MODE_CMD, sizeof(AUTO_MODE_CMD));
  delay(80);
  sendCommand(ActuatorCommand::TargetCenter, "TARGET_CENTER", TARGET_CENTER_CMD, sizeof(TARGET_CENTER_CMD));
  lastControlTxMs = millis();
}

bool isActuatorBusGuardActive()
{
  return ENABLE_ACTUATOR_OPEN_LOOP &&
         (millis() - lastControlTxMs < ACTUATOR_CONTROL_GUARD_MS || hallReadPending);
}

ActuatorFeedback getActuatorFeedback()
{
  ActuatorFeedback result = feedback;
  if (result.valid && millis() - result.updatedMs > ACTUATOR_FEEDBACK_STALE_MS)
  {
    result.valid = false;
  }
  return result;
}
