#include "Arduino.h"
#include "common.h"
#include "debug.h"
#include "udpHandlers.h"
#include "gnssHandlers.h"
#include "setup.h"
#include "mongooseStart.h"
#include "KeyaCANBUS.h"
#include "Autosteer.h"
#include "AutosteerPID.h"
#include "serialComm.h"

void setup()
{
  delay(3000); // Delay for tesing to allow opening serial terminal to see output
  Serial.begin(115200);
  Serial.print("\r\n\n\n*********************\r\nStarting setup...\r\n");
  Serial.print("Firmware version: ");
  Serial.print(inoVersion);

  LEDs.set(LED_ID::PWR_ETH, PWR_ETH_STATE::PWR_ON);

  setCpuFrequency(600 * 1000000); // Set CPU speed, default is 600mhz, 150mhz still seems fast enough, setup.ino
  ipSetup();                      // Load the IP address from EEPROM and setup the gateway and broadcast addresses
  load_gps();                     // Load the GPS settings from EEPROM
  load_config();                  // Sync the firmware EEPROM valuse to the GUI
  ethernet_init();                // Bring up the ethernet hardware
  mongoose_init();                // Bring the mongoose services
  udpSetup();                     // Bring up the UDP connections to/from AgIO
  serialSetup();                  // Configure the Serial comms
  parserSetup();                  // Load the NMEA parser callbacks
  BNO.begin(SerialIMU);           // Start the IMU
  autosteerSetup();               // Initialize autosteer
  CAN_Setup();                    // Start CAN3 for Keya

  machinePTR = new MACHINE;
  const uint8_t pcaOutputPinNumbers[8] = {1, 0, 12, 15, 9, 8, 6, 7}; // all 8 PCA9555 section/machine output pin numbers on v5.0a
  const uint8_t pcaInputPinNumbers[] = {14, 13, 11, 10, 2, 3, 4, 5}; // all 8 PCA9555 section/machine output "sensing" pin numbers on v5.0a
  if (outputs.begin())
  {
    Serial.print("\r\nSection outputs (PCA9555) detected (8 channels, low side switching)");
    machinePTR->init(&outputs, pcaOutputPinNumbers, pcaInputPinNumbers, 500); // mach.h
  }

  Serial.println("\r\n\nEnd of setup, waiting for GPS...\r\n");
  delay(1);
  resetStartingTimersBuffers(); // setup.ino
}

void loop()
{
  gpsPoll();
  gpsProc();
  serialESP32();
  KeyaBus_Receive();
  autoSteerUpdate();
  serialRTCM();
  mongoose_poll();

  LEDSusage.timeIn();
  LEDs.updateLoop();
  LEDSusage.timeOut();

  MACHusage.timeIn();
  machinePTR->watchdogCheck();
  MACHusage.timeOut();

  BNOusage.timeIn();
  if (BNO.read())
  { // there should be new data every 10ms (100hz)
    bnoRing.pushOverwrite(BNO.rvcData);
    IMU_DATA bnoRingTmp;
    IMU_DATA bnoTmp;
    bnoRing.peek(bnoRingData, 6); // 10=0ms ago, 9=10ms ago, 8=20ms ago, 7=30ms ago, 6=40ms ago, 5=50ms ago, 4=60ms ago, 3=70ms ago, 2=80ms ago, 1=90ms ago, 0=100ms ago
    itoa(BNO.rvcData.pitchX10, bnoTmp.pitch, 10); // the pitch x10
    itoa(BNO.rvcData.rollX10, bnoTmp.roll, 10);   // the roll x10
    itoa(bnoRingData.pitchX10, bnoRingTmp.pitch, 10); // the pitch x10
    itoa(bnoRingData.rollX10, bnoRingTmp.roll, 10);   // the roll x10

    Serial.print(millis());Serial.println(" Now");
    Serial.println(bnoTmp.roll);
    Serial.println(bnoTmp.pitch);
    Serial.print(millis());Serial.println(" Now-40ms");
    Serial.println(bnoRingTmp.roll);
    Serial.println(bnoRingTmp.pitch);
    Serial.println();

    bnoStats.incHzCount();
    bnoStats.update(1); // 1 dummy value
  }
  BNOusage.timeOut();

  // wait 40 msec (F9P) from prev GGA update, then update imu data for next PANDA sentence
  if (imuPandaSyncTrigger && imuPandaSyncTimer >= 40)
  {
    prepImuPandaData();
    imuPandaSyncTrigger = false; // wait for next GGA update before resetting imuDelayTimer again
  }

  // Check for debug input
  checkUSBSerial();

  // Print telemetry
  if (bufferStatsTimer > 5000)
    printTelem();

  // to count loop hz & get baseline cpu "idle" time
  LOOPusage.timeIn();
  testCounter++;
  LOOPusage.timeOut();
}
