#include "Arduino.h"
#include "common.h"
#include "gnss_handlers.h"
#include "setup.h"
#include "mongoose_startup.h"
#include "KeyaCANBUS.h"
#include "Autosteer.h"
#include "AutosteerPID.h"


void setup() {
  delay(3000); //Delay for tesing to allow opening serial terminal to see output
  Serial.begin(115200);
  while (!Serial) delay(50);
  Serial.print("\r\n\n\n*********************\r\nStarting setup...\r\n");
  Serial.print("Firmware version: ");
  Serial.print(inoVersion);

  LEDs.set(LED_ID::PWR_ETH, PWR_ETH_STATE::PWR_ON);

  setCpuFrequency(600 * 1000000);           // Set CPU speed, default is 600mhz, 150mhz still seems fast enough, setup.ino
  ethernet_init();
  mongoose_init();
  serialSetup();                            // setup.h
  parserSetup();                            // setup.h
  BNO.begin(SerialIMU);                     // BNO_RVC.cpp
  autosteerSetup();                         // Autosteer.h
  //CAN_Setup();                              //Start CAN3 for Keya

  Serial.println("\r\n\nEnd of setup, waiting for GPS...\r\n"); 
  delay(1);
  resetStartingTimersBuffers();             // setup.ino
}

void loop() {
  mongoose_poll();
}
