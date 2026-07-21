#pragma once

#include <stdint.h>

void setupBoatControl();
void updateBoatControl();
const char *getSteeringStatusText();
const char *getGearStatusText();
const char *getDriveStatusText();
const char *getControlModeText();
bool isPhysicalManualModeActive();
uint8_t getSteeringStatusCode();
int8_t getDriveGearValue();
uint8_t getControlModeCode();
