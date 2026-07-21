#include "rtk_receiver.h"

#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "telemetry.h"

namespace
{
enum class RtkRxState
{
  WaitStart,
  ReadSentence
};

RtkRxState rtkRxState = RtkRxState::WaitStart;
char rtkSentence[RTK_SENTENCE_MAX_LEN];
size_t rtkSentenceIndex = 0;
unsigned long rtkByteCount = 0;
unsigned long lastRtkDebugByteCount = 0;
unsigned long lastRtkDebugMs = 0;
unsigned long lastRtkRawPreviewMs = 0;
uint8_t lastRtkByte = 0;
unsigned long rtkSentenceCount = 0;
unsigned long rtkOverflowCount = 0;
RtkDisplayData latestRtkData = {};

bool startsWith(const char *text, const char *prefix)
{
  return strncmp(text, prefix, strlen(prefix)) == 0;
}

bool shouldForwardRtkSentence(const char *sentence)
{
  return strstr(sentence, "GGA") != nullptr ||
         strstr(sentence, "RMC") != nullptr ||
         strstr(sentence, "VTG") != nullptr;
}

bool getCsvField(const char *sentence, int targetIndex, char *out, size_t outSize)
{
  if (outSize == 0)
  {
    return false;
  }

  size_t outIndex = 0;
  int fieldIndex = 0;

  for (size_t i = 0;; ++i)
  {
    const char c = sentence[i];
    const bool isEnd = c == '\0' || c == ',' || c == '*';

    if (fieldIndex == targetIndex && !isEnd)
    {
      if (outIndex < outSize - 1)
      {
        out[outIndex++] = c;
      }
    }

    if (isEnd)
    {
      if (fieldIndex == targetIndex)
      {
        out[outIndex] = '\0';
        return true;
      }

      if (c == '\0' || c == '*')
      {
        break;
      }

      ++fieldIndex;
    }
  }

  out[0] = '\0';
  return false;
}

double fieldToDouble(const char *sentence, int index)
{
  char field[24];
  if (!getCsvField(sentence, index, field, sizeof(field)) || field[0] == '\0')
  {
    return 0.0;
  }

  return atof(field);
}

int fieldToInt(const char *sentence, int index)
{
  char field[12];
  if (!getCsvField(sentence, index, field, sizeof(field)) || field[0] == '\0')
  {
    return 0;
  }

  return atoi(field);
}

const char *qualityName(int quality)
{
  switch (quality)
  {
  case 1:
    return "SINGLE";
  case 2:
    return "RTK_FLOAT";
  case 3:
    return "RTK_FIXED";
  case 0:
  default:
    return "INVALID";
  }
}

void handleKsxtSentence(const char *sentence)
{
  char utc[24];
  getCsvField(sentence, 1, utc, sizeof(utc));

  const double lon = fieldToDouble(sentence, 2);
  const double lat = fieldToDouble(sentence, 3);
  const double height = fieldToDouble(sentence, 4);
  const double heading = fieldToDouble(sentence, 5);
  const double track = fieldToDouble(sentence, 7);
  const double speedKmh = fieldToDouble(sentence, 8);
  const int posQuality = fieldToInt(sentence, 10);
  const int headingQuality = fieldToInt(sentence, 11);
  const int slaveSv = fieldToInt(sentence, 12);
  const int masterSv = fieldToInt(sentence, 13);

  latestRtkData.valid = true;
  latestRtkData.lon = lon;
  latestRtkData.lat = lat;
  latestRtkData.headingDeg = heading;
  latestRtkData.speedKmh = speedKmh;
  latestRtkData.posQuality = posQuality;
  latestRtkData.updatedAtMs = millis();

  if (ENABLE_STATUS_TELEMETRY)
  {
    printfBoth(
        "RTK_PARSED,utc=%s,lon=%.8f,lat=%.8f,height=%.4f,heading=%.2f,track=%.2f,speed=%.3f,pos=%s,head=%s,slave_sv=%d,master_sv=%d\n",
        utc,
        lon,
        lat,
        height,
        heading,
        track,
        speedKmh,
        qualityName(posQuality),
        qualityName(headingQuality),
        slaveSv,
        masterSv);
  }
}

void handleCompleteRtkSentence(const char *sentence)
{
  ++rtkSentenceCount;

  if (startsWith(sentence, "$KSXT"))
  {
    handleKsxtSentence(sentence);
    return;
  }

  if (shouldForwardRtkSentence(sentence))
  {
    if (ENABLE_STATUS_TELEMETRY)
    {
      printfBoth("[RTK %lu] %s\n", rtkSentenceCount, sentence);
    }
    return;
  }

  if (ENABLE_STATUS_TELEMETRY && millis() - lastRtkRawPreviewMs >= RTK_RAW_PREVIEW_INTERVAL_MS)
  {
    lastRtkRawPreviewMs = millis();
    Serial.printf("[RTK RAW %lu] %s\n", rtkSentenceCount, sentence);
  }
}

void processRtkByte(char c)
{
  switch (rtkRxState)
  {
  case RtkRxState::WaitStart:
    if (c == '$')
    {
      rtkSentenceIndex = 0;
      rtkSentence[rtkSentenceIndex++] = c;
      rtkRxState = RtkRxState::ReadSentence;
    }
    break;

  case RtkRxState::ReadSentence:
    if (c == '$')
    {
      rtkSentenceIndex = 0;
      rtkSentence[rtkSentenceIndex++] = c;
      break;
    }

    if (c == '\r')
    {
      break;
    }

    if (c == '\n')
    {
      rtkSentence[rtkSentenceIndex] = '\0';
      handleCompleteRtkSentence(rtkSentence);
      rtkSentenceIndex = 0;
      rtkRxState = RtkRxState::WaitStart;
      break;
    }

    if (rtkSentenceIndex < RTK_SENTENCE_MAX_LEN - 1)
    {
      rtkSentence[rtkSentenceIndex++] = c;
    }
    else
    {
      ++rtkOverflowCount;
      rtkSentenceIndex = 0;
      rtkRxState = RtkRxState::WaitStart;
    }
    break;
  }
}
} // namespace

void setupRtkReceiver()
{
  Serial2.begin(RTK_BAUD, SERIAL_8N1, PIN_RTK_RX, PIN_RTK_TX);
}

void handleRtkSerial()
{
  while (Serial2.available() > 0)
  {
    const int value = Serial2.read();
    if (value < 0)
    {
      continue;
    }

    lastRtkByte = static_cast<uint8_t>(value);
    ++rtkByteCount;
    processRtkByte(static_cast<char>(lastRtkByte));
  }
}

void printRtkDebug()
{
  if (!ENABLE_STATUS_TELEMETRY)
  {
    return;
  }

  if (millis() - lastRtkDebugMs < RTK_DEBUG_INTERVAL_MS)
  {
    return;
  }

  lastRtkDebugMs = millis();
  const unsigned long delta = rtkByteCount - lastRtkDebugByteCount;
  lastRtkDebugByteCount = rtkByteCount;

  Serial.printf("[RTK DEBUG] delta=%lu total=%lu sentences=%lu overflow=%lu last=0x%02X state=%s\n",
                delta,
                rtkByteCount,
                rtkSentenceCount,
                rtkOverflowCount,
                lastRtkByte,
                rtkRxState == RtkRxState::WaitStart ? "WAIT_START" : "READ_SENTENCE");
}

RtkDisplayData getLatestRtkDisplayData()
{
  return latestRtkData;
}
