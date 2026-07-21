#pragma once

#include <Arduino.h>

// Steering wheel wiring:
//   COM  -> GND
//   LEFT -> GPIO5
//   RIGHT-> GPIO6
constexpr int PIN_STEER_LEFT = 5;
constexpr int PIN_STEER_RIGHT = 6;

// Gear and throttle digital inputs after voltage divider.
constexpr int PIN_GEAR = 4;
constexpr int PIN_THROTTLE_DIGITAL = 3;

// Three-position physical mode selector. Both inputs use INPUT_PULLUP:
//   GPIO11/GPIO12 = 10 -> AUTO
//   GPIO11/GPIO12 = 11 -> MANUAL (center, both contacts open)
//   GPIO11/GPIO12 = 01 -> REMOTE
//   GPIO11/GPIO12 = 00 -> invalid (both contacts active)
constexpr int PIN_MODE_SELECT_A = 11;
constexpr int PIN_MODE_SELECT_B = 12;

// Mode indicator LEDs, active HIGH:
//   GPIO -> current-limiting resistor -> LED -> GND
constexpr int PIN_MODE_LED_AUTO = 40;
constexpr int PIN_MODE_LED_MANUAL = 41;
constexpr int PIN_MODE_LED_REMOTE = 42;

// Tronlong UART9 test link. GPIO1/2 are the board's UART3 connector,
// implemented with EspSoftwareSerial because UART0/1/2 are already in use.
constexpr int PIN_TRONLONG_RX = 2;
constexpr int PIN_TRONLONG_TX = 1;
constexpr unsigned long TRONLONG_UART_BAUD = 115200;
constexpr bool ENABLE_TRONLONG_UART_TEST = true;
constexpr bool ENABLE_TRONLONG_UART_DEBUG = false;

// Propulsion driver output test.
constexpr int PIN_THROTTLE_OUT = 9; // PWML on current schematic
constexpr int PIN_DIR_OUT = 10;     // PWMR on current schematic

// 4G DTU UART1 wiring through an automatic-direction TTL-to-RS485 module:
//   GPIO17/UART1_TX -> converter RX/DI
//   GPIO18/UART1_RX <- converter TX/RO
constexpr int PIN_DTU_RX = 18;
constexpr int PIN_DTU_TX = 17;
constexpr unsigned long DTU_BAUD = 115200;
constexpr unsigned long DTU_STATUS_INTERVAL_MS = 500;
constexpr unsigned long DTU_DEBUG_INTERVAL_MS = 2000;
constexpr unsigned long DTU_RTK_STALE_MS = 2000;
constexpr bool ENABLE_DTU_DEBUG = false;

// Temporary local UDP protocol test. The ESP32 joins the same router used by
// the wired PC, then broadcasts the same complete protocol frame over UDP.
constexpr bool ENABLE_WIFI_UDP_TEST = true;
constexpr char WIFI_UDP_TEST_SSID[] = "LSH_24G";
constexpr char WIFI_UDP_TEST_PASSWORD[] = "LSH_2025";
constexpr uint16_t WIFI_UDP_TEST_PORT = 5005;
constexpr unsigned long WIFI_UDP_RECONNECT_INTERVAL_MS = 5000;

// RTK UART2 wiring:
//   GPIO21/UART2_TX -> RTK RX1, optional for configuration
//   GPIO14/UART2_RX <- RTK TX1, required for NMEA data
constexpr int PIN_RTK_RX = 14;
constexpr int PIN_RTK_TX = 21;
constexpr unsigned long RTK_BAUD = 115200;
constexpr size_t RTK_SENTENCE_MAX_LEN = 160;

// DGUS screen RS485 on the board RS485 port.
// Uses software TX so RTK can keep Serial2 on its original wiring.
constexpr int PIN_DGUS_RS485_RX = 15;
constexpr int PIN_DGUS_RS485_TX = 16;
constexpr unsigned long DGUS_BAUD = 38400;
constexpr unsigned long DGUS_TEST_INTERVAL_MS = 2000;
constexpr uint8_t DGUS_TEXT_FIELD_BYTES = 12;
constexpr uint8_t DGUS_POSITION_TEXT_FIELD_BYTES = 20;
constexpr uint8_t DGUS_NAV_TEXT_FIELD_BYTES = 16;
constexpr bool ENABLE_DGUS_DISPLAY = true;
constexpr bool ENABLE_DGUS_PERIODIC_TX = true;
constexpr uint16_t DGUS_VP_ACTUATOR_POSITION = 0x2090; // 0.1 mm per count
constexpr uint16_t DGUS_VP_ACTUATOR_HALL = 0x2091;
constexpr uint16_t DGUS_VP_ACTUATOR_VALID = 0x2092;


constexpr bool ENABLE_ACTUATOR_OPEN_LOOP = true;
constexpr int PIN_ACTUATOR_RS485_RX = PIN_DGUS_RS485_RX;
constexpr int PIN_ACTUATOR_RS485_TX = PIN_DGUS_RS485_TX;
constexpr unsigned long ACTUATOR_RS485_BAUD = 38400;
constexpr unsigned long ACTUATOR_CONTROL_GUARD_MS = 200;
constexpr unsigned long ACTUATOR_HALL_READ_INTERVAL_MS = 200;
constexpr unsigned long ACTUATOR_HALL_RESPONSE_TIMEOUT_MS = 100;
constexpr unsigned long ACTUATOR_FEEDBACK_STALE_MS = 1000;
constexpr bool ENABLE_ACTUATOR_PERIODIC_HALL_READ = true;
constexpr uint32_t ACTUATOR_HALL_MIN = 0x00000000;
constexpr uint32_t ACTUATOR_HALL_MAX = 0x000002FB;
constexpr uint16_t ACTUATOR_STROKE_TENTHS_MM = 2500;
constexpr uint32_t ACTUATOR_TARGET_RIGHT_HALL = 0x00000000;
constexpr uint32_t ACTUATOR_TARGET_CENTER_HALL = 0x0000017D;
constexpr uint32_t ACTUATOR_TARGET_LEFT_HALL = 0x000002FB;

constexpr bool PROPULSION_ACTIVE_HIGH = true;
constexpr bool DIR_REVERSE_HIGH = true;
constexpr bool MODE_LED_ACTIVE_HIGH = true;

constexpr unsigned long PRINT_INTERVAL_MS = 200;
constexpr unsigned long RTK_DEBUG_INTERVAL_MS = 1000;
constexpr unsigned long RTK_RAW_PREVIEW_INTERVAL_MS = 1000;

constexpr bool ENABLE_STATUS_TELEMETRY = false;

constexpr int FILTER_SAMPLES = 5;
constexpr int FILTER_HIGH_COUNT = 3;
constexpr unsigned int FILTER_SAMPLE_INTERVAL_US = 200;

constexpr unsigned long STEERING_DEBOUNCE_MS = 50;
constexpr unsigned long GEAR_DEBOUNCE_MS = 80;
constexpr unsigned long THROTTLE_DEBOUNCE_MS = 80;
constexpr unsigned long MODE_SWITCH_DEBOUNCE_MS = 80;
