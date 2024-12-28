#include "Arduino.h"
#include "common.h"
#include "udpHandlers.h"
#include "gnss_handlers.h"
#include "setup.h"
#include "mongoose_start.h"
#include "KeyaCANBUS.h"
#include "Autosteer.h"
#include "AutosteerPID.h"
#include "serialRW.h"
#include "gps1Poll.h"
#include "gps2Poll.h"
#include "gpsProc.h"


void setup() {
  delay(3000); //Delay for tesing to allow opening serial terminal to see output
  Serial.begin(115200);
  while (!Serial) delay(50);
  Serial.print("\r\n\n\n*********************\r\nStarting setup...\r\n");
  Serial.print("Firmware version: ");
  Serial.print(inoVersion);

  LEDs.set(LED_ID::PWR_ETH, PWR_ETH_STATE::PWR_ON);

  setCpuFrequency(600 * 1000000);           // Set CPU speed, default is 600mhz, 150mhz still seems fast enough, setup.ino
  Eth_EEPROM();
  ethernet_init();
  mongoose_init();
  ipaddrSetup();
  udpSetup();
  serialSetup();                            // setup.h
  parserSetup();                            // setup.h
  BNO.begin(SerialIMU);                     // BNO_RVC.cpp
  autosteerSetup();                         // Autosteer.h
  CAN_Setup();                              //Start CAN3 for Keya

  Serial.println("\r\n\nEnd of setup, waiting for GPS...\r\n"); 
  delay(1);
  resetStartingTimersBuffers();             // setup.ino
  machinePTR = new MACHINE;
  const uint8_t pcaOutputPinNumbers[8] = { 1, 0, 12, 15, 9, 8, 6, 7 };    // all 8 PCA9555 section/machine output pin numbers on v5.0a
  const uint8_t pcaInputPinNumbers[]  = { 14, 13, 11, 10, 2, 3, 4, 5 };   // all 8 PCA9555 section/machine output "sensing" pin numbers on v5.0a
}

void loop() {
  gps1Poll();
  gps2Poll();
  gpsProc();
  KeyaBus_Receive();
  autoSteerUpdate();
  serialRW();
  mongoose_poll();
  LEDs.updateLoop();
  machinePTR->watchdogCheck();

  if (BNO.read()) {                         // there should be new data every 10ms (100hz)
    bnoStats.incHzCount();
    bnoStats.update(1);                     // 1 dummy value
  }

  // wait 40 msec (F9P) from prev GGA update, then update imu data for next PANDA sentence
  if (imuPandaSyncTrigger && imuPandaSyncTimer >= 40) {
    prepImuPandaData();
    imuPandaSyncTrigger = false;       // wait for next GGA update before resetting imuDelayTimer again
  }
}
