// AIO_GUI is copyright 2025 by the AOG Group
// AiO_GUI is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
// AiO_GUI is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with Foobar. If not, see <https://www.gnu.org/licenses/>.
// Like most Arduino code, portions of this are based on other open source Arduino code with a compatiable license.

const char inoVersion[] = "AiO v5.0d Web GUI - " __DATE__ " " __TIME__;

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
  delay(3000);              // Delay for tesing to allow opening serial terminal to see output
  Serial.begin(115200);
  Serial.print("\r\n\n\n*********************\r\nStarting setup...\r\n");
  Serial.print("Firmware version: ");
  Serial.print(inoVersion);

  setCpuFrequency(600 * 1000000); // Set CPU speed, default is 600mhz, setup.ino

  // ** IP loading & Mongoose/Eth init needs to be first **
  ipSetup();                      // Load the IP address from EEPROM and setup the gateway & broadcast addresses
  load_gps();                     // Load the GPS settings from EEPROM
  load_config();                  // Sync the firmware EEPROM values to the GUI
  ethernet_init();                // Bring up the ethernet hardware
  mongoose_init();                // Bring up the mongoose services
  udpSetup();                     // Bring up the UDP connections to/from AgIO

  LEDs.init();
  LEDs.set(LED_ID::PWR_ETH, PWR_ETH_STATE::PWR_ON);

  outputsInit();                  // Initialize PCA9685 for LOCK, AUX & Sections/Machine outputs, enable AUX output but leave others Hi-Z
  serialSetup();                  // Configure the Serial comms
  parserSetup();                  // Load the NMEA parser callbacks
  BNO.begin(SerialIMU);           // Start the IMU
  autosteerSetup();               // Initialize autosteer
  CAN_Setup();                    // Start CAN3 for Keya

  /*machinePTR = new MACHINE;
  const uint8_t pcaOutputPinNumbers[8] = {1, 0, 12, 15, 9, 8, 6, 7}; // all 8 PCA9555 section/machine output pin numbers on v5.0a
  const uint8_t pcaInputPinNumbers[] = {14, 13, 11, 10, 2, 3, 4, 5}; // all 8 PCA9555 section/machine output "sensing" pin numbers on v5.0a
  if (outputs.begin())
  {
    Serial.print("\r\nSection outputs (PCA9555) detected (8 channels, low side switching)");
    machinePTR->init(&outputs, pcaOutputPinNumbers, pcaInputPinNumbers, 500); // mach.h
  }*/

  Serial.println("\r\n\nEnd of setup, waiting for GPS...\r\n");
  delay(1);
  resetStartingTimersBuffers();         // setup.ino
}

void loop()
{
  gpsPoll();                            // check for data on GPS1 & GPS2 UARTs
  serialESP32();                        // check for PGN replies on ESP32 UART
  KeyaBus_Receive();                    // check for Keya data on can bus 3
  autoSteerUpdate();                    // run autosteer loop
  serialRTCM();                         // check for RTCM data on Xbee/Radio UART

  GUIusage.timeIn();                    // *usage objects are used to track cpu usage on certain sections of code, see debug.h or misc.h
  mongoose_poll();                      // update all Mongoose processes, UDP/PGN/Web UI
  GUIusage.timeOut();

  LEDSusage.timeIn();
  LEDs.updateLoop();                    // update frontplate RGB LEDs
  LEDSusage.timeOut();

  MACHusage.timeIn();
  //machinePTR->watchdogCheck();
  MACHusage.timeOut();

  BNOusage.timeIn();
  if (BNO.read())                       // read IMU UART and check for completed RVC updated
  {                                     // there should be new data every 10ms (100hz)
    bnoRing.pushOverwrite(BNO.rvcData); // added IMU update to ring buffer
    bnoStats.incHzCount();              // *Stats objects are used to diag missing/skipped updates for GPS & IMU, see debug.h or misc.h
    bnoStats.update(1);                 // 1 dummy value, normally used to track UART hardware buffer usage
  }
  BNOusage.timeOut();

  checkUSBSerial();                     // Check for & process debug cmds

  if (bufferStatsTimer > 5000) printTelem(); // Print telemetry

  LOOPusage.timeIn();
  testCounter++;                        // to count loop hz & get baseline cpu "idle" time
  LOOPusage.timeOut();

  if (SerialRS232.available()) Serial.write(SerialRS232.read()); // just print to USB for testing

}
