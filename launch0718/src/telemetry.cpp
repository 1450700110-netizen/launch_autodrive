#include "telemetry.h"

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <SoftwareSerial.h>
#include <esp_mac.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "actuator_open_loop.h"
#include "boat_control.h"
#include "config.h"
#include "rtk_receiver.h"

namespace
{
constexpr uint16_t COMMAND_PERIODIC_REPORT = 0x3001;
constexpr uint8_t PROTOCOL_JSON_V1 = 0x40;

constexpr size_t FRAME_CAPACITY = 256;
constexpr size_t FRAME_HEADER_SIZE = 14;
constexpr size_t JSON_CAPACITY = 224;

uint8_t sequenceNumber = 0;
unsigned long lastStatusTxMs = 0;
unsigned long lastDebugMs = 0;
bool printedFirstFrame = false;

char registrationIdText[6] = "0";

WiFiUDP wifiTestUdp;
IPAddress wifiTestBroadcastIp;
bool wifiTestReady = false;
unsigned long lastWifiReconnectMs = 0;

EspSoftwareSerial::UART tronlongUart;

void initializeBoatIdentity()
{
  uint8_t mac[6] = {0};
  if (esp_read_mac(mac, ESP_MAC_WIFI_STA) != ESP_OK)
  {
    if (ENABLE_STATUS_TELEMETRY)
    {
      Serial.println("[BOAT ID] failed to read Wi-Fi STA MAC");
    }
    return;
  }

  const uint16_t macSuffixValue =
      (static_cast<uint16_t>(mac[4]) << 8) | mac[5];
  const uint16_t registrationId = macSuffixValue % 10000;
  snprintf(registrationIdText,
           sizeof(registrationIdText),
           "%u",
           static_cast<unsigned>(registrationId));

  if (ENABLE_STATUS_TELEMETRY)
  {
    Serial.printf("[BOAT ID] STA MAC=%02X:%02X:%02X:%02X:%02X:%02X registration=%s\n",
                  static_cast<unsigned>(mac[0]),
                  static_cast<unsigned>(mac[1]),
                  static_cast<unsigned>(mac[2]),
                  static_cast<unsigned>(mac[3]),
                  static_cast<unsigned>(mac[4]),
                  static_cast<unsigned>(mac[5]),
                  registrationIdText);
  }
}

void putU16(uint8_t *destination, uint16_t value)
{
  destination[0] = static_cast<uint8_t>(value >> 8);
  destination[1] = static_cast<uint8_t>(value & 0xFF);
}

uint16_t calculateProtocolCrc16(const uint8_t *data, size_t length)
{
  uint16_t crc = 0xFFFF;
  constexpr uint16_t polynomial = 0x8005;

  for (size_t i = 0; i < length; ++i)
  {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit)
    {
      crc = (crc & 0x8000U) ? static_cast<uint16_t>((crc << 1) ^ polynomial)
                            : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

int32_t scaleCoordinate(double degrees)
{
  const double scaled = degrees * 1000000.0;
  if (scaled > 2147483647.0)
  {
    return INT32_MAX;
  }
  if (scaled < -2147483648.0)
  {
    return INT32_MIN;
  }
  return static_cast<int32_t>(llround(scaled));
}

int16_t scaleSpeedToHundredthKnot(double speedKmh)
{
  double scaled = speedKmh * (100.0 / 1.852);
  if (scaled < 0.0)
  {
    scaled = 0.0;
  }
  if (scaled > 32767.0)
  {
    scaled = 32767.0;
  }
  return static_cast<int16_t>(lround(scaled));
}

uint16_t scaleHeadingToHundredthDegree(double headingDegrees)
{
  double normalized = fmod(headingDegrees, 360.0);
  if (normalized < 0.0)
  {
    normalized += 360.0;
  }
  return static_cast<uint16_t>(lround(normalized * 100.0)) % 36000U;
}

size_t buildStatusFrame(uint8_t *frame, size_t capacity)
{
  if (capacity < FRAME_HEADER_SIZE)
  {
    return 0;
  }

  const RtkDisplayData rtk = getLatestRtkDisplayData();
  const bool rtkFresh = rtk.valid && millis() - rtk.updatedAtMs <= DTU_RTK_STALE_MS;
  const ActuatorFeedback actuator = getActuatorFeedback();

  const int32_t longitude = rtkFresh ? scaleCoordinate(rtk.lon) : 0;
  const int32_t latitude = rtkFresh ? scaleCoordinate(rtk.lat) : 0;
  const int16_t speed = rtkFresh ? scaleSpeedToHundredthKnot(rtk.speedKmh) : 0;
  const uint16_t heading = rtkFresh ? scaleHeadingToHundredthDegree(rtk.headingDeg) : 0;
  const uint8_t rtkQuality = rtkFresh
                                 ? static_cast<uint8_t>(constrain(rtk.posQuality, 0, 255))
                                 : 0;

  char jsonBody[JSON_CAPACITY];
  const int jsonLength = snprintf(
      jsonBody,
      sizeof(jsonBody),
      "{\"L0A01\":\"%s\","
      "\"L1001\":%ld,\"L1002\":%ld,\"L1003\":%d,\"L1005\":%u,"
      "\"L1A01\":%u,\"L1A11\":%d,\"LA001\":%u,\"LA002\":%u,"
      "\"LA003\":%u,\"LA004\":%u}",
      registrationIdText,
      static_cast<long>(longitude),
      static_cast<long>(latitude),
      static_cast<int>(speed),
      static_cast<unsigned>(heading),
      static_cast<unsigned>(getControlModeCode()),
      static_cast<int>(getDriveGearValue()),
      static_cast<unsigned>(getSteeringStatusCode()),
      static_cast<unsigned>(actuator.positionTenthsMm),
      actuator.valid ? 1U : 0U,
      static_cast<unsigned>(rtkQuality));

  if (jsonLength < 0 || static_cast<size_t>(jsonLength) >= sizeof(jsonBody))
  {
    return 0;
  }

  const size_t bodyLength = static_cast<size_t>(jsonLength);
  const size_t frameLength = FRAME_HEADER_SIZE + bodyLength;
  if (frameLength > capacity || bodyLength > UINT16_MAX)
  {
    return 0;
  }

  frame[0] = 0xAA;
  frame[1] = 0xBB;
  frame[2] = 0x00; // CRC placeholder
  frame[3] = 0x00;
  putU16(frame + 4, COMMAND_PERIODIC_REPORT);
  frame[6] = sequenceNumber++;
  frame[7] = PROTOCOL_JSON_V1;
  frame[8] = 0x00; // no encryption
  frame[9] = 0x00; // no compression
  frame[10] = 0x00;
  frame[11] = 0x00;
  putU16(frame + 12, static_cast<uint16_t>(bodyLength));
  memcpy(frame + FRAME_HEADER_SIZE, jsonBody, bodyLength);

  const uint16_t crc = calculateProtocolCrc16(frame + 4, frameLength - 4);
  putU16(frame + 2, crc);
  return frameLength;
}

void printFrameHex(const uint8_t *frame, size_t length)
{
  Serial.printf("[DTU TX HEX] len=%u ", static_cast<unsigned>(length));
  for (size_t i = 0; i < length; ++i)
  {
    if (frame[i] < 0x10)
    {
      Serial.print('0');
    }
    Serial.print(frame[i], HEX);
    Serial.print(i + 1 == length ? '\n' : ' ');
  }
}

void printJsonBody(const uint8_t *frame, size_t length)
{
  Serial.print("[DTU TX JSON] ");
  for (size_t i = FRAME_HEADER_SIZE; i < length; ++i)
  {
    Serial.write(frame[i]);
  }
  Serial.println();
}

void setupWifiUdpTest()
{
  if (!ENABLE_WIFI_UDP_TEST)
  {
    return;
  }

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_UDP_TEST_SSID, WIFI_UDP_TEST_PASSWORD);
  lastWifiReconnectMs = millis();

  if (ENABLE_STATUS_TELEMETRY)
  {
    Serial.printf("[UDP TEST] connecting to Wi-Fi SSID=%s, UDP port=%u\n",
                  WIFI_UDP_TEST_SSID,
                  static_cast<unsigned>(WIFI_UDP_TEST_PORT));
  }
}

IPAddress calculateBroadcastIp()
{
  const IPAddress localIp = WiFi.localIP();
  const IPAddress subnet = WiFi.subnetMask();
  IPAddress broadcast;
  for (uint8_t index = 0; index < 4; ++index)
  {
    broadcast[index] = static_cast<uint8_t>(localIp[index] | (~subnet[index] & 0xFFU));
  }
  return broadcast;
}

void updateWifiUdpConnection()
{
  if (!ENABLE_WIFI_UDP_TEST)
  {
    return;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    if (!wifiTestReady)
    {
      if (!wifiTestUdp.begin(WIFI_UDP_TEST_PORT))
      {
        if (ENABLE_STATUS_TELEMETRY)
        {
          Serial.println("[UDP TEST] failed to bind local UDP port");
        }
        return;
      }

      wifiTestBroadcastIp = calculateBroadcastIp();
      wifiTestReady = true;
      if (ENABLE_STATUS_TELEMETRY)
      {
        Serial.printf("[UDP TEST] connected IP=%s broadcast=%s port=%u\n",
                      WiFi.localIP().toString().c_str(),
                      wifiTestBroadcastIp.toString().c_str(),
                      static_cast<unsigned>(WIFI_UDP_TEST_PORT));
      }
    }
    return;
  }

  if (wifiTestReady)
  {
    wifiTestUdp.stop();
    wifiTestReady = false;
    if (ENABLE_STATUS_TELEMETRY)
    {
      Serial.println("[UDP TEST] Wi-Fi disconnected");
    }
  }

  const unsigned long now = millis();
  if (now - lastWifiReconnectMs >= WIFI_UDP_RECONNECT_INTERVAL_MS)
  {
    lastWifiReconnectMs = now;
    WiFi.disconnect();
    WiFi.begin(WIFI_UDP_TEST_SSID, WIFI_UDP_TEST_PASSWORD);
    if (ENABLE_STATUS_TELEMETRY)
    {
      Serial.println("[UDP TEST] reconnecting Wi-Fi");
    }
  }
}

void handleWifiUdpTestReceive()
{
  updateWifiUdpConnection();
  if (!wifiTestReady)
  {
    return;
  }

  int packetLength = wifiTestUdp.parsePacket();
  while (packetLength > 0)
  {
    if (ENABLE_STATUS_TELEMETRY)
    {
      Serial.printf("[UDP TEST RX] %u bytes from %s:%u\n",
                    static_cast<unsigned>(packetLength),
                    wifiTestUdp.remoteIP().toString().c_str(),
                    static_cast<unsigned>(wifiTestUdp.remotePort()));
    }
    while (wifiTestUdp.available() > 0)
    {
      wifiTestUdp.read();
    }
    packetLength = wifiTestUdp.parsePacket();
  }
}

void sendFrameToWifiUdpTest(const uint8_t *frame, size_t length)
{
  if (!wifiTestReady)
  {
    return;
  }

  if (!wifiTestUdp.beginPacket(wifiTestBroadcastIp, WIFI_UDP_TEST_PORT))
  {
    if (ENABLE_STATUS_TELEMETRY)
    {
      Serial.println("[UDP TEST] beginPacket failed");
    }
    return;
  }

  const size_t written = wifiTestUdp.write(frame, length);
  const int result = wifiTestUdp.endPacket();
  if ((written != length || result != 1) && ENABLE_STATUS_TELEMETRY)
  {
    Serial.printf("[UDP TEST] send failed %u/%u result=%d\n",
                  static_cast<unsigned>(written),
                  static_cast<unsigned>(length),
                  result);
  }
}

void setupTronlongUartTest()
{
  if (!ENABLE_TRONLONG_UART_TEST)
  {
    return;
  }

  tronlongUart.begin(TRONLONG_UART_BAUD,
                     EspSoftwareSerial::SWSERIAL_8N1,
                     PIN_TRONLONG_RX,
                     PIN_TRONLONG_TX);
  if (ENABLE_STATUS_TELEMETRY)
  {
    Serial.printf("[TRONLONG] UART3 software TX=GPIO%d RX=GPIO%d baud=%lu 8N1\n",
                  PIN_TRONLONG_TX,
                  PIN_TRONLONG_RX,
                  TRONLONG_UART_BAUD);
  }
}

void sendFrameToTronlong(const uint8_t *frame, size_t length)
{
  if (!ENABLE_TRONLONG_UART_TEST)
  {
    return;
  }

  const size_t written = tronlongUart.write(frame, length);
  if (written != length && ENABLE_STATUS_TELEMETRY)
  {
    Serial.printf("[TRONLONG TX] short write %u/%u\n",
                  static_cast<unsigned>(written),
                  static_cast<unsigned>(length));
  }
}

void handleTronlongReceive()
{
  if (!ENABLE_TRONLONG_UART_TEST || tronlongUart.available() <= 0)
  {
    return;
  }

  if (ENABLE_TRONLONG_UART_DEBUG)
  {
    Serial.print("[TRONLONG RX] ");
  }

  while (tronlongUart.available() > 0)
  {
    const uint8_t value = static_cast<uint8_t>(tronlongUart.read());
    if (ENABLE_TRONLONG_UART_DEBUG)
    {
      if (value >= 0x20 && value <= 0x7E)
      {
        Serial.write(value);
      }
      else if (value == '\r' || value == '\n')
      {
        Serial.write(value);
      }
      else
      {
        Serial.printf("<%02X>", value);
      }
    }
  }

  if (ENABLE_TRONLONG_UART_DEBUG)
  {
    Serial.println();
  }
}

void handleDtuReceive()
{
  if (Serial1.available() <= 0)
  {
    return;
  }

  if (ENABLE_DTU_DEBUG)
  {
    Serial.print("[DTU RX HEX] ");
  }
  while (Serial1.available() > 0)
  {
    const uint8_t value = static_cast<uint8_t>(Serial1.read());
    if (ENABLE_DTU_DEBUG)
    {
      if (value < 0x10)
      {
        Serial.print('0');
      }
      Serial.print(value, HEX);
      Serial.print(' ');
    }
  }
  if (ENABLE_DTU_DEBUG)
  {
    Serial.println();
  }
}
} // namespace

void setupTelemetry()
{
  Serial1.begin(DTU_BAUD, SERIAL_8N1, PIN_DTU_RX, PIN_DTU_TX);
  initializeBoatIdentity();
  if (ENABLE_STATUS_TELEMETRY)
  {
    Serial.printf("[DTU] UART1 TX=GPIO%d RX=GPIO%d baud=%lu 8N1 protocol=JSON v1 over UDP transparent mode\n",
                  PIN_DTU_TX,
                  PIN_DTU_RX,
                  DTU_BAUD);
  }
  setupTronlongUartTest();
  setupWifiUdpTest();
}

void printBoth(const char *text)
{
  if (!ENABLE_STATUS_TELEMETRY)
  {
    return;
  }

  // Debug text must never enter the binary DTU/JSON stream.
  Serial.println(text);
}

void printfBoth(const char *format, ...)
{
  if (!ENABLE_STATUS_TELEMETRY)
  {
    return;
  }

  char buffer[320];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  Serial.print(buffer);
}

void sendDtuStatusNow()
{
  uint8_t frame[FRAME_CAPACITY];
  const size_t length = buildStatusFrame(frame, sizeof(frame));
  if (length == 0)
  {
    if (ENABLE_STATUS_TELEMETRY)
    {
      Serial.println("[DTU TX] JSON frame build failed");
    }
    return;
  }

  const size_t written = Serial1.write(frame, length);
  Serial1.flush();
  if (written != length && ENABLE_STATUS_TELEMETRY)
  {
    Serial.printf("[DTU TX] short write %u/%u\n",
                  static_cast<unsigned>(written),
                  static_cast<unsigned>(length));
  }

  sendFrameToWifiUdpTest(frame, length);
  sendFrameToTronlong(frame, length);

  if (ENABLE_DTU_DEBUG && !printedFirstFrame)
  {
    printedFirstFrame = true;
    printJsonBody(frame, length);
    printFrameHex(frame, length);
  }

  if (ENABLE_DTU_DEBUG && millis() - lastDebugMs >= DTU_DEBUG_INTERVAL_MS)
  {
    lastDebugMs = millis();
    Serial.printf("[DTU TX] seq=%u bytes=%u protocol=JSON\n",
                  static_cast<unsigned>(sequenceNumber - 1),
                  static_cast<unsigned>(length));
  }
}

void handleDtuTelemetry()
{
  handleWifiUdpTestReceive();
  handleDtuReceive();
  handleTronlongReceive();

  const unsigned long now = millis();
  if (now - lastStatusTxMs < DTU_STATUS_INTERVAL_MS)
  {
    return;
  }

  lastStatusTxMs = now;
  sendDtuStatusNow();
}
