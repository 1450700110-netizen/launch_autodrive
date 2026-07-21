#pragma once

struct RtkDisplayData
{
  bool valid;
  double lon;
  double lat;
  double headingDeg;
  double speedKmh;
  int posQuality;
  unsigned long updatedAtMs;
};

void setupRtkReceiver();
void handleRtkSerial();
void printRtkDebug();
RtkDisplayData getLatestRtkDisplayData();
