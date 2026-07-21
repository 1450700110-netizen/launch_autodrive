#pragma once

#include <Arduino.h>

using SharedRs485FrameHandler = void (*)(const uint8_t *frame, size_t length);

void setupSharedRs485();
void handleSharedRs485();
size_t sharedRs485Write(const uint8_t *data, size_t length);
void setDgusRs485FrameHandler(SharedRs485FrameHandler handler);
void setActuatorRs485FrameHandler(SharedRs485FrameHandler handler);
