#include "shared_rs485.h"

#include <SoftwareSerial.h>

#include "config.h"

namespace
{
enum class ReceiveProtocol
{
  None,
  Dgus,
  Actuator
};

constexpr size_t RX_FRAME_CAPACITY = 260;
constexpr unsigned long RX_FRAME_TIMEOUT_MS = 30;

SoftwareSerial sharedRs485;
ReceiveProtocol receiveProtocol = ReceiveProtocol::None;
uint8_t receiveFrame[RX_FRAME_CAPACITY];
size_t receiveFrameLength = 0;
size_t expectedFrameLength = 0;
unsigned long lastReceiveByteMs = 0;
SharedRs485FrameHandler dgusFrameHandler = nullptr;
SharedRs485FrameHandler actuatorFrameHandler = nullptr;

static_assert(PIN_DGUS_RS485_RX == PIN_ACTUATOR_RS485_RX,
              "DGUS and actuator must use the same RS485 RX pin");
static_assert(PIN_DGUS_RS485_TX == PIN_ACTUATOR_RS485_TX,
              "DGUS and actuator must use the same RS485 TX pin");
static_assert(DGUS_BAUD == ACTUATOR_RS485_BAUD,
              "DGUS and actuator must use the same RS485 baud rate");

void resetReceiveFrame()
{
  receiveProtocol = ReceiveProtocol::None;
  receiveFrameLength = 0;
  expectedFrameLength = 0;
}

void startReceiveFrame(ReceiveProtocol protocol, uint8_t firstByte)
{
  receiveProtocol = protocol;
  receiveFrame[0] = firstByte;
  receiveFrameLength = 1;
  expectedFrameLength = 0;
  lastReceiveByteMs = millis();
}

void dispatchReceiveFrame()
{
  if (receiveProtocol == ReceiveProtocol::Dgus && dgusFrameHandler != nullptr)
  {
    dgusFrameHandler(receiveFrame, receiveFrameLength);
  }
  else if (receiveProtocol == ReceiveProtocol::Actuator && actuatorFrameHandler != nullptr)
  {
    actuatorFrameHandler(receiveFrame, receiveFrameLength);
  }

  resetReceiveFrame();
}

void processWaitingByte(uint8_t value)
{
  if (value == 0x5A)
  {
    startReceiveFrame(ReceiveProtocol::Dgus, value);
  }
  else if (value == 0xFF)
  {
    startReceiveFrame(ReceiveProtocol::Actuator, value);
  }
}

void processDgusByte(uint8_t value)
{
  if (receiveFrameLength >= sizeof(receiveFrame))
  {
    resetReceiveFrame();
    processWaitingByte(value);
    return;
  }

  receiveFrame[receiveFrameLength++] = value;
  lastReceiveByteMs = millis();

  if (receiveFrameLength == 2 && receiveFrame[1] != 0xA5)
  {
    resetReceiveFrame();
    processWaitingByte(value);
    return;
  }

  if (receiveFrameLength == 3)
  {
    expectedFrameLength = 3U + receiveFrame[2];
    if (receiveFrame[2] == 0 || expectedFrameLength > sizeof(receiveFrame))
    {
      resetReceiveFrame();
      return;
    }
  }

  if (expectedFrameLength > 0 && receiveFrameLength == expectedFrameLength)
  {
    dispatchReceiveFrame();
  }
}

void updateActuatorExpectedLength()
{
  if (receiveFrameLength < 2)
  {
    return;
  }

  const uint8_t functionCode = receiveFrame[1];
  if ((functionCode & 0x80U) != 0)
  {
    expectedFrameLength = 5;
  }
  else if (functionCode == 0x06 || functionCode == 0x10)
  {
    expectedFrameLength = 8;
  }
  else if ((functionCode == 0x03 || functionCode == 0x04) && receiveFrameLength >= 3)
  {
    expectedFrameLength = 5U + receiveFrame[2];
  }

  if (expectedFrameLength > sizeof(receiveFrame))
  {
    resetReceiveFrame();
  }
}

void processActuatorByte(uint8_t value)
{
  if (receiveFrameLength >= sizeof(receiveFrame))
  {
    resetReceiveFrame();
    processWaitingByte(value);
    return;
  }

  receiveFrame[receiveFrameLength++] = value;
  lastReceiveByteMs = millis();
  updateActuatorExpectedLength();

  if (expectedFrameLength > 0 && receiveFrameLength == expectedFrameLength)
  {
    dispatchReceiveFrame();
  }
}

void processReceiveByte(uint8_t value)
{
  switch (receiveProtocol)
  {
  case ReceiveProtocol::Dgus:
    processDgusByte(value);
    break;
  case ReceiveProtocol::Actuator:
    processActuatorByte(value);
    break;
  case ReceiveProtocol::None:
  default:
    processWaitingByte(value);
    break;
  }
}
} // namespace

void setupSharedRs485()
{
  sharedRs485.begin(DGUS_BAUD,
                    SWSERIAL_8N1,
                    PIN_DGUS_RS485_RX,
                    PIN_DGUS_RS485_TX);
  if (ENABLE_STATUS_TELEMETRY)
  {
    Serial.printf("[RS485] shared SoftwareSerial TX=GPIO%d RX=GPIO%d baud=%lu\n",
                  PIN_DGUS_RS485_TX,
                  PIN_DGUS_RS485_RX,
                  DGUS_BAUD);
  }
}

void handleSharedRs485()
{
  while (sharedRs485.available() > 0)
  {
    const int value = sharedRs485.read();
    if (value >= 0)
    {
      processReceiveByte(static_cast<uint8_t>(value));
    }
  }

  if (receiveProtocol != ReceiveProtocol::None &&
      millis() - lastReceiveByteMs >= RX_FRAME_TIMEOUT_MS)
  {
    if (receiveProtocol == ReceiveProtocol::Actuator)
    {
      dispatchReceiveFrame();
    }
    else
    {
      resetReceiveFrame();
    }
  }
}

size_t sharedRs485Write(const uint8_t *data, size_t length)
{
  return sharedRs485.write(data, length);
}

void setDgusRs485FrameHandler(SharedRs485FrameHandler handler)
{
  dgusFrameHandler = handler;
}

void setActuatorRs485FrameHandler(SharedRs485FrameHandler handler)
{
  actuatorFrameHandler = handler;
}
