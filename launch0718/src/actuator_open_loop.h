#pragma once

#include <Arduino.h>

struct ActuatorFeedback
{
  uint32_t hallRaw;
  uint16_t positionTenthsMm;
  uint8_t positionPercent;
  unsigned long updatedMs;
  bool valid;
};

void setupActuatorOpenLoop();
void handleActuatorOpenLoop();
void actuatorOpenLoopExtend();
void actuatorOpenLoopRetract();
void actuatorOpenLoopBrake();
bool isActuatorBusGuardActive();
ActuatorFeedback getActuatorFeedback();
