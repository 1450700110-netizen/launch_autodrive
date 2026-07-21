#include <Arduino.h>

#include "actuator_open_loop.h"
#include "boat_control.h"
#include "config.h"
#include "dgus_display.h"
#include "rtk_receiver.h"
#include "shared_rs485.h"
#include "telemetry.h"

void setup()
{
  Serial.begin(115200);
  setupSharedRs485();
  setupTelemetry();
  setupRtkReceiver();
  setupDgusDisplay();
  delay(500);

  if (ENABLE_STATUS_TELEMETRY)
  {
    printBoth("Steering wheel + gear lever + actuator + DTU + RTK + DGUS start");
    printfBoth("DTU UART1: TX=GPIO%d RX=GPIO%d baud=%lu 8N1\n", PIN_DTU_TX, PIN_DTU_RX, DTU_BAUD);
    printfBoth("RTK UART2: TX=GPIO%d RX=GPIO%d baud=%lu\n", PIN_RTK_TX, PIN_RTK_RX, RTK_BAUD);
    printfBoth("DGUS RS485 software: TX=GPIO%d RX=GPIO%d baud=%lu\n",
               PIN_DGUS_RS485_TX,
               PIN_DGUS_RS485_RX,
               DGUS_BAUD);
  }
  setupBoatControl();
}

void loop()
{
  handleSharedRs485();
  handleRtkSerial();
  updateBoatControl();
  handleActuatorOpenLoop();
  updateDgusDisplay();
  handleDtuTelemetry();
}
