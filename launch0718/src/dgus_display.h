#pragma once

#include <Arduino.h>

void setupDgusDisplay();
void updateDgusDisplay();
void dgusWriteWord(uint16_t vp, uint16_t value);
void dgusWriteText(uint16_t vp, const char *text, uint8_t fieldBytes);
bool getScreenAutoRequest();
