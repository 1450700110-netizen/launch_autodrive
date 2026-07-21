#include "boat_control.h"

#include <Arduino.h>

#include "actuator_open_loop.h"
#include "config.h"
#include "dgus_display.h"
#include "telemetry.h"

namespace
{
enum class SteeringState
{
  Center,
  Left,
  Right,
  Error
};

enum class GearState
{
  Forward,
  Reverse
};

enum class DriveState
{
  Neutral,
  Forward,
  Reverse
};

enum class ControlMode
{
  Manual,
  Auto,
  Remote,
  Error
};

int readMajorityLevel(int pin)
{
  int highCount = 0;
  for (int i = 0; i < FILTER_SAMPLES; ++i)
  {
    if (digitalRead(pin) == HIGH)
    {
      ++highCount;
    }
    delayMicroseconds(FILTER_SAMPLE_INTERVAL_US);
  }

  return highCount >= FILTER_HIGH_COUNT ? HIGH : LOW;
}

struct DebouncedInput
{
  int pin;
  unsigned long debounceMs;
  int rawLevel;
  int stableLevel;
  unsigned long rawChangedAtMs;

  DebouncedInput(int inputPin, unsigned long stableTimeMs)
      : pin(inputPin),
        debounceMs(stableTimeMs),
        rawLevel(HIGH),
        stableLevel(HIGH),
        rawChangedAtMs(0)
  {
  }

  void begin(uint8_t mode)
  {
    pinMode(pin, mode);
    rawLevel = readMajorityLevel(pin);
    stableLevel = rawLevel;
    rawChangedAtMs = millis();
  }

  void update()
  {
    const int votedLevel = readMajorityLevel(pin);
    if (votedLevel != rawLevel)
    {
      rawLevel = votedLevel;
      rawChangedAtMs = millis();
    }

    if (stableLevel != rawLevel && millis() - rawChangedAtMs >= debounceMs)
    {
      stableLevel = rawLevel;
    }
  }
};

DebouncedInput steerLeftInput(PIN_STEER_LEFT, STEERING_DEBOUNCE_MS);
DebouncedInput steerRightInput(PIN_STEER_RIGHT, STEERING_DEBOUNCE_MS);
DebouncedInput gearInput(PIN_GEAR, GEAR_DEBOUNCE_MS);
DebouncedInput throttleInput(PIN_THROTTLE_DIGITAL, THROTTLE_DEBOUNCE_MS);
DebouncedInput modeSelectAInput(PIN_MODE_SELECT_A, MODE_SWITCH_DEBOUNCE_MS);
DebouncedInput modeSelectBInput(PIN_MODE_SELECT_B, MODE_SWITCH_DEBOUNCE_MS);

SteeringState lastState = SteeringState::Error;
SteeringState currentSteeringState = SteeringState::Error;
GearState currentGearState = GearState::Forward;
DriveState currentDriveState = DriveState::Neutral;
ControlMode currentControlMode = ControlMode::Manual;
ControlMode lastAppliedControlMode = ControlMode::Error;
bool modePrintInitialized = false;
unsigned long lastPrintMs = 0;

void stopActuator()
{
  actuatorOpenLoopBrake();
}

void extendActuator()
{
  actuatorOpenLoopExtend();
}

void retractActuator()
{
  actuatorOpenLoopRetract();
}

void writePropulsionPin(int pin, bool active)
{
  digitalWrite(pin, active == PROPULSION_ACTIVE_HIGH ? HIGH : LOW);
}

DriveState getDriveState(GearState gear, int throttleLevel)
{
  const bool throttleOn = throttleLevel == HIGH;
  if (!throttleOn)
  {
    return DriveState::Neutral;
  }

  return gear == GearState::Reverse ? DriveState::Reverse : DriveState::Forward;
}

void applyPropulsion(DriveState driveState)
{
  const bool throttleOn = driveState != DriveState::Neutral;
  const bool reverse = driveState == DriveState::Reverse;

  writePropulsionPin(PIN_THROTTLE_OUT, throttleOn);

  // In neutral, keep direction at the safe default: forward/LOW.
  digitalWrite(PIN_DIR_OUT, reverse == DIR_REVERSE_HIGH ? HIGH : LOW);
}

SteeringState readSteeringState()
{
  const bool leftActive = steerLeftInput.stableLevel == LOW;
  const bool rightActive = steerRightInput.stableLevel == LOW;

  if (leftActive && !rightActive)
  {
    return SteeringState::Left;
  }
  if (!leftActive && rightActive)
  {
    return SteeringState::Right;
  }
  if (!leftActive && !rightActive)
  {
    return SteeringState::Center;
  }

  return SteeringState::Error;
}

const char *stateName(SteeringState state)
{
  switch (state)
  {
  case SteeringState::Left:
    return "LEFT";
  case SteeringState::Right:
    return "RIGHT";
  case SteeringState::Center:
    return "CENTER";
  case SteeringState::Error:
  default:
    return "ERROR";
  }
}

const char *gearName(GearState state)
{
  return state == GearState::Forward ? "FORWARD" : "REVERSE";
}

const char *driveName(DriveState state)
{
  switch (state)
  {
  case DriveState::Forward:
    return "FORWARD";
  case DriveState::Reverse:
    return "REVERSE";
  case DriveState::Neutral:
  default:
    return "NEUTRAL";
  }
}

const char *modeName(ControlMode mode)
{
  switch (mode)
  {
  case ControlMode::Manual:
    return "MANUAL";
  case ControlMode::Auto:
    return "AUTO";
  case ControlMode::Remote:
    return "REMOTE";
  case ControlMode::Error:
  default:
    return "ERROR";
  }
}

ControlMode readControlMode()
{
  const int selectA = modeSelectAInput.stableLevel;
  const int selectB = modeSelectBInput.stableLevel;

  if (selectA == HIGH && selectB == LOW)
  {
    return ControlMode::Auto;
  }
  if (selectA == HIGH && selectB == HIGH)
  {
    return ControlMode::Manual;
  }
  if (selectA == LOW && selectB == HIGH)
  {
    return ControlMode::Remote;
  }

  return ControlMode::Error;
}

void writeModeLed(int pin, bool active)
{
  digitalWrite(pin, active == MODE_LED_ACTIVE_HIGH ? HIGH : LOW);
}

void applyModeIndicators(ControlMode mode)
{
  writeModeLed(PIN_MODE_LED_AUTO, mode == ControlMode::Auto);
  writeModeLed(PIN_MODE_LED_MANUAL, mode == ControlMode::Manual);
  writeModeLed(PIN_MODE_LED_REMOTE, mode == ControlMode::Remote);
}

void applySteeringState(SteeringState state)
{
  switch (state)
  {
  case SteeringState::Left:
    extendActuator();
    break;

  case SteeringState::Right:
    retractActuator();
    break;

  case SteeringState::Center:
  case SteeringState::Error:
  default:
    stopActuator();
    break;
  }
}

void printBoatStatus(SteeringState state, GearState gear, DriveState driveState)
{
  printfBoth(
      "mode_a raw/stable=%d/%d mode_b raw/stable=%d/%d mode=%s | left raw/stable=%d/%d right raw/stable=%d/%d state=%s | gear raw/stable=%d/%d gear=%s | throttle raw/stable=%d/%d drive=%s | throttle_out=%d dir_out=%d\n",
      modeSelectAInput.rawLevel,
      modeSelectAInput.stableLevel,
      modeSelectBInput.rawLevel,
      modeSelectBInput.stableLevel,
      modeName(currentControlMode),
      steerLeftInput.rawLevel,
      steerLeftInput.stableLevel,
      steerRightInput.rawLevel,
      steerRightInput.stableLevel,
      stateName(state),
      gearInput.rawLevel,
      gearInput.stableLevel,
      gearName(gear),
      throttleInput.rawLevel,
      throttleInput.stableLevel,
      driveName(driveState),
      digitalRead(PIN_THROTTLE_OUT),
      digitalRead(PIN_DIR_OUT));
}
} // namespace

void setupBoatControl()
{
  steerLeftInput.begin(INPUT_PULLUP);
  steerRightInput.begin(INPUT_PULLUP);

  // Gear/throttle are driven by external 0V/5V signals after divider.
  // Use INPUT, not INPUT_PULLUP.
  gearInput.begin(INPUT);
  throttleInput.begin(INPUT);
  modeSelectAInput.begin(INPUT_PULLUP);
  modeSelectBInput.begin(INPUT_PULLUP);

  pinMode(PIN_MODE_LED_AUTO, OUTPUT);
  pinMode(PIN_MODE_LED_MANUAL, OUTPUT);
  pinMode(PIN_MODE_LED_REMOTE, OUTPUT);
  applyModeIndicators(readControlMode());

  pinMode(PIN_THROTTLE_OUT, OUTPUT);
  pinMode(PIN_DIR_OUT, OUTPUT);
  writePropulsionPin(PIN_THROTTLE_OUT, false);
  digitalWrite(PIN_DIR_OUT, LOW);

  setupActuatorOpenLoop();

  if (ENABLE_STATUS_TELEMETRY)
  {
    printfBoth("LEFT input: GPIO%d, RIGHT input: GPIO%d\n", PIN_STEER_LEFT, PIN_STEER_RIGHT);
    printfBoth("GEAR input: GPIO%d, THROTTLE digital input: GPIO%d\n", PIN_GEAR, PIN_THROTTLE_DIGITAL);
    printfBoth("MODE selector: GPIO%d/GPIO%d, 10=AUTO 11=MANUAL 01=REMOTE 00=ERROR\n",
               PIN_MODE_SELECT_A,
               PIN_MODE_SELECT_B);
    printfBoth("MODE LEDs: AUTO=GPIO%d MANUAL=GPIO%d REMOTE=GPIO%d, HIGH=ON\n",
               PIN_MODE_LED_AUTO,
               PIN_MODE_LED_MANUAL,
               PIN_MODE_LED_REMOTE);
    printfBoth("THROTTLE output: GPIO%d, DIR output: GPIO%d\n", PIN_THROTTLE_OUT, PIN_DIR_OUT);
    printBoth("Steering logic: switch active = LOW, idle = HIGH");
    printBoth("Gear logic: LOW=FORWARD, HIGH=REVERSE");
    printBoth("Throttle logic: LOW=NEUTRAL/STOP, HIGH=RUN");
    printBoth("Propulsion logic: throttle LOW=neutral; throttle HIGH + gear decides forward/reverse");
    printBoth("Mode logic: physical GPIO11/GPIO12 selector has priority");
    printBoth("Input filter: 5-sample majority vote + debounce stable time");
    printfBoth("Debounce: steering=%lums, gear=%lums, throttle=%lums, mode=%lums\n",
               STEERING_DEBOUNCE_MS,
               GEAR_DEBOUNCE_MS,
               THROTTLE_DEBOUNCE_MS,
               MODE_SWITCH_DEBOUNCE_MS);
  }
}

void updateBoatControl()
{
  steerLeftInput.update();
  steerRightInput.update();
  gearInput.update();
  throttleInput.update();
  modeSelectAInput.update();
  modeSelectBInput.update();

  const SteeringState state = readSteeringState();
  const GearState gear = gearInput.stableLevel == LOW ? GearState::Forward : GearState::Reverse;
  const int throttleLevel = throttleInput.stableLevel;
  const ControlMode mode = readControlMode();
  const DriveState driveState = mode == ControlMode::Manual ? getDriveState(gear, throttleLevel) : DriveState::Neutral;

  currentSteeringState = state;
  currentGearState = gear;
  currentDriveState = driveState;

  if (!modePrintInitialized || mode != currentControlMode)
  {
    Serial.printf("[MODE] GPIO%d=%d GPIO%d=%d -> %s\n",
                  PIN_MODE_SELECT_A,
                  modeSelectAInput.stableLevel,
                  PIN_MODE_SELECT_B,
                  modeSelectBInput.stableLevel,
                  modeName(mode));
    modePrintInitialized = true;
  }

  currentControlMode = mode;

  applyModeIndicators(mode);
  applyPropulsion(driveState);

  const bool modeChanged = mode != lastAppliedControlMode;

  if (mode == ControlMode::Manual)
  {
    if (modeChanged || state != lastState)
    {
      applySteeringState(state);
    }
  }
  else if (modeChanged)
  {
    stopActuator();
  }

  lastState = state;
  lastAppliedControlMode = mode;

  if (millis() - lastPrintMs >= PRINT_INTERVAL_MS)
  {
    lastPrintMs = millis();
    if (ENABLE_STATUS_TELEMETRY)
    {
      printBoatStatus(state, gear, driveState);
    }
  }
}

const char *getSteeringStatusText()
{
  return stateName(currentSteeringState);
}

const char *getGearStatusText()
{
  return gearName(currentGearState);
}

const char *getDriveStatusText()
{
  return driveName(currentDriveState);
}

const char *getControlModeText()
{
  return modeName(currentControlMode);
}

bool isPhysicalManualModeActive()
{
  return modeSelectAInput.stableLevel == HIGH &&
         modeSelectBInput.stableLevel == HIGH;
}

uint8_t getSteeringStatusCode()
{
  switch (currentSteeringState)
  {
  case SteeringState::Center:
    return 0;
  case SteeringState::Left:
    return 1;
  case SteeringState::Right:
    return 2;
  case SteeringState::Error:
  default:
    return 3;
  }
}

int8_t getDriveGearValue()
{
  switch (currentDriveState)
  {
  case DriveState::Forward:
    return 1;
  case DriveState::Reverse:
    return -1;
  case DriveState::Neutral:
  default:
    return 0;
  }
}

uint8_t getControlModeCode()
{
  switch (currentControlMode)
  {
  case ControlMode::Manual:
    return 0;
  case ControlMode::Auto:
    return 1;
  case ControlMode::Remote:
    return 2;
  case ControlMode::Error:
  default:
    return 3;
  }
}
