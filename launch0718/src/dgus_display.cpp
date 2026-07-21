#include "dgus_display.h"

#include "actuator_open_loop.h"
#include "boat_control.h"
#include "config.h"
#include "rtk_receiver.h"
#include "shared_rs485.h"
#include "telemetry.h"

namespace {
unsigned long lastDgusSendMs = 0;
unsigned long lastActuatorFeedbackDisplayMs = 0;
bool actuatorFeedbackDisplayInitialized = false;
bool lastActuatorFeedbackValid = false;
constexpr const char *TEST_LONGITUDE_TEXT = "117.12345678";
constexpr const char *TEST_LATITUDE_TEXT = "36.12345678";

bool screenAutoRequest = false;

void dgusWriteBytes(const uint8_t *data, size_t length)
{
  const size_t written = sharedRs485Write(data, length);
  if (written != length && ENABLE_STATUS_TELEMETRY)
  {
    Serial.printf("[DGUS TX] short write %u/%u\n",
                  static_cast<unsigned>(written),
                  static_cast<unsigned>(length));
  }
}

void handleDgusFrame(const uint8_t *frame, size_t frameBytes)
{
  if (frameBytes < 4 || frame[0] != 0x5A || frame[1] != 0xA5)
  {
    return;
  }

  const uint8_t length = frame[2];
  if (frameBytes != 3U + length)
  {
    return;
  }

  const uint8_t *payload = frame + 3;

  if (ENABLE_STATUS_TELEMETRY)
  {
    Serial.print("[DGUS RAW] 5A A5 ");
    if (length < 0x10) {
      Serial.print('0');
    }
    Serial.print(length, HEX);
    for (uint8_t i = 0; i < length; ++i) {
      Serial.print(' ');
      if (payload[i] < 0x10) {
        Serial.print('0');
      }
      Serial.print(payload[i], HEX);
    }
    Serial.println();
  }

  const uint8_t command = payload[0];
  if (command != 0x83) {
    if (ENABLE_STATUS_TELEMETRY) {
      Serial.printf("[DGUS RX] command=0x%02X len=%u\n", command, length);
    }
    return;
  }

  if (length < 6) {
    if (ENABLE_STATUS_TELEMETRY) {
      Serial.printf("[DGUS RX] short 0x83 frame len=%u\n", length);
    }
    return;
  }

  const uint16_t vp = (static_cast<uint16_t>(payload[1]) << 8) | payload[2];
  const uint8_t wordCount = payload[3];
  const uint16_t value = (static_cast<uint16_t>(payload[4]) << 8) | payload[5];

  if (ENABLE_STATUS_TELEMETRY) {
    Serial.printf("[DGUS RX] vp=0x%04X words=%u value=0x%04X\n", vp, wordCount, value);
  }

  if (vp == 0x2000 && wordCount == 1 && (value == 0 || value == 1)) {
    screenAutoRequest = value != 0;
    if (ENABLE_STATUS_TELEMETRY) {
      Serial.printf("[DGUS MODE REQUEST] %s received; GPIO11/GPIO12 selector controls actual mode\n",
                    screenAutoRequest ? "AUTO" : "MANUAL");
    }
  }
}
}

void setupDgusDisplay()
{
  if (!ENABLE_DGUS_DISPLAY) {
    return;
  }

  setDgusRs485FrameHandler(handleDgusFrame);

  if (ENABLE_STATUS_TELEMETRY)
  {
    printfBoth("DGUS RS485: shared TX=GPIO%d RX=GPIO%d baud=%lu\n",
               PIN_DGUS_RS485_TX,
               PIN_DGUS_RS485_RX,
               DGUS_BAUD);
  }
}

void dgusWriteWord(uint16_t vp, uint16_t value)
{
  const uint8_t frame[] = {
      0x5A, 0xA5,
      0x05,
      0x82,
      static_cast<uint8_t>(vp >> 8),
      static_cast<uint8_t>(vp & 0xFF),
      static_cast<uint8_t>(value >> 8),
      static_cast<uint8_t>(value & 0xFF),
  };
  dgusWriteBytes(frame, sizeof(frame));
}

void dgusWriteText(uint16_t vp, const char *text, uint8_t fieldBytes)
{
  if (fieldBytes > 240) {
    fieldBytes = 240;
  }

  uint8_t frame[4 + 2 + 240] = {
      0x5A,
      0xA5,
      static_cast<uint8_t>(3 + fieldBytes),
      0x82,
      static_cast<uint8_t>(vp >> 8),
      static_cast<uint8_t>(vp & 0xFF),
  };

  uint8_t i = 0;
  for (; i < fieldBytes && text[i] != '\0'; ++i) {
    frame[6 + i] = static_cast<uint8_t>(text[i]);
  }
  for (; i < fieldBytes; ++i) {
    frame[6 + i] = ' ';
  }

  dgusWriteBytes(frame, 6 + fieldBytes);
}

void updateDgusDisplay()
{
  if (!ENABLE_DGUS_DISPLAY) {
    return;
  }

  if (!ENABLE_DGUS_PERIODIC_TX) {
    return;
  }

  if (isActuatorBusGuardActive()) {
    return;
  }

  const unsigned long now = millis();
  const ActuatorFeedback actuator = getActuatorFeedback();
  if (!actuatorFeedbackDisplayInitialized ||
      actuator.updatedMs != lastActuatorFeedbackDisplayMs ||
      actuator.valid != lastActuatorFeedbackValid) {
    dgusWriteWord(DGUS_VP_ACTUATOR_POSITION, actuator.positionTenthsMm);
    delay(5);
    dgusWriteWord(DGUS_VP_ACTUATOR_HALL, static_cast<uint16_t>(actuator.hallRaw));
    delay(5);
    dgusWriteWord(DGUS_VP_ACTUATOR_VALID, actuator.valid ? 1 : 0);
    lastActuatorFeedbackDisplayMs = actuator.updatedMs;
    lastActuatorFeedbackValid = actuator.valid;
    actuatorFeedbackDisplayInitialized = true;
  }

  if (now - lastDgusSendMs < DGUS_TEST_INTERVAL_MS) {
    return;
  }
  lastDgusSendMs = now;

  const char *steering = getSteeringStatusText();
  const char *drive = getDriveStatusText();
  const RtkDisplayData rtk = getLatestRtkDisplayData();

  char lonText[DGUS_POSITION_TEXT_FIELD_BYTES + 1];
  char latText[DGUS_POSITION_TEXT_FIELD_BYTES + 1];
  char headingText[DGUS_NAV_TEXT_FIELD_BYTES + 1];
  char speedText[DGUS_NAV_TEXT_FIELD_BYTES + 1];

  if (rtk.valid) {
    snprintf(lonText, sizeof(lonText), "%.8f", rtk.lon);
    snprintf(latText, sizeof(latText), "%.8f", rtk.lat);
    snprintf(headingText, sizeof(headingText), "%.2f deg", rtk.headingDeg);
    snprintf(speedText, sizeof(speedText), "%.2f km/h", rtk.speedKmh);
  } else {
    snprintf(lonText, sizeof(lonText), "%s", TEST_LONGITUDE_TEXT);
    snprintf(latText, sizeof(latText), "%s", TEST_LATITUDE_TEXT);
    snprintf(headingText, sizeof(headingText), "39.00 deg");
    snprintf(speedText, sizeof(speedText), "30.00 km/h");
  }

  dgusWriteText(0x2010, steering, DGUS_TEXT_FIELD_BYTES);
  delay(5);
  dgusWriteText(0x2020, drive, DGUS_TEXT_FIELD_BYTES);
  delay(5);
  dgusWriteText(0x2040, lonText, DGUS_POSITION_TEXT_FIELD_BYTES);
  delay(5);
  dgusWriteText(0x2050, latText, DGUS_POSITION_TEXT_FIELD_BYTES);
  delay(5);
  dgusWriteText(0x2060, headingText, DGUS_NAV_TEXT_FIELD_BYTES);
  delay(5);
  dgusWriteText(0x2070, speedText, DGUS_NAV_TEXT_FIELD_BYTES);

  if (ENABLE_STATUS_TELEMETRY) {
    printfBoth("[DGUS TX] steering=%s drive=%s lon=%s lat=%s heading=%s speed=%s\n",
               steering,
               drive,
               lonText,
               latText,
               headingText,
               speedText);
  }
}

bool getScreenAutoRequest()
{
  return screenAutoRequest;
}
