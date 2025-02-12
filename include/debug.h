// AIO_GUI is copyright 2025 by the AOG Group
// AiO_GUI is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
// AiO_GUI is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with Foobar. If not, see <https://www.gnu.org/licenses/>.
// Like most Arduino code, portions of this are based on other open source Arduino code with a compatiable license.

#ifndef DEBUG_H
#define DEBUG_H

void printCpuPercent(uint32_t _time)
{
  Serial.printf("%4.1f", (float)_time / 10000.0);
  Serial.print("%");
}

void checkUSBSerial()
{
  if (Serial.available())
  {
    uint8_t usbRead = Serial.read();

    if (usbRead == 'r')
    {
      Serial.print("\r\n\n* Resetting hi/lo stats *");
      gps1Stats.resetAll();
      gps2Stats.resetAll();
      relJitterStats.resetAll();
      relTtrStats.resetAll();
      bnoStats.resetAll();
    }
    else if (usbRead == 'n') // output realtime GPS position update data
    {
      uint8_t usbRead2 = 0;
      if (Serial.available())
      {
        usbRead2 = Serial.peek(); // only peek in case it's not a 2, leave in Serial buffer for further processing other cmds
      }

      if (usbRead2 == '2')
      {
        Serial.read(); // clear the 2 out of the Serial buffer
        nmeaDebug2 = !nmeaDebug2;
        Serial.print("\r\nSetting NMEA2 debug: ");
        Serial.print(nmeaDebug2);
      }
      else
      {
        nmeaDebug = !nmeaDebug;
        ubxParser.debug = nmeaDebug;
        Serial.print("\r\nSetting NMEA debug: ");
        Serial.print(nmeaDebug);
      }
    }
    else if (usbRead == 'c') // output cpu usage stats
    {
      printCpuUsages = !printCpuUsages;
      Serial.print("\r\nSetting CPU usage debug: ");
      Serial.print(printCpuUsages);
    }
    else if (usbRead == 's') // output GPS, BNO update freq & buffer stats
    {
      printStats = !printStats;
      Serial.print("\r\nSetting Print Stats: ");
      Serial.print(printStats);
    }
    else if (usbRead == 'R')
    {
      SCB_AIRCR = 0x05FA0004;   // Teensy Reboot
    }
#ifdef AIOv50a
    else if (usbRead == 'm' && Serial.available() > 0) // set machine debug level
    {
      usbRead = Serial.read();
      if (usbRead >= '0' && usbRead <= '5')
      {
        //machinePTR->debugLevel = usbRead - '0'; // convert ASCII numerical char to byte
      }
      //Serial.print((String) "\r\nMachine debugLevel: " + machinePTR->debugLevel);
    }
#endif
    else if (usbRead == 'g' && Serial.available() > 0) // temporarily set GPS fix state according to standard GGA fix numbers (see LEDS.h, setGpsLED())
    {
      usbRead = Serial.read();
      if (usbRead >= '0' && usbRead <= '5')
      {
        LEDs.setGpsLED(usbRead - '0', true);
      }
    }
    else if (usbRead == 'l' && Serial.available() > 0) // set RGB brightness
    {
      usbRead = Serial.read();
      if (usbRead >= '0' && usbRead <= '5')
      {
        LEDs.setBrightness((usbRead - '0') * 50);
        Serial.print("\r\nSetting RGB brightness: ");
        Serial.print((usbRead - '0') * 50);
      }
    }
    else if (usbRead == '1')      // drv9243 testing, cycle LOCK through sleep, standby, active
    {
      uint32_t t1 = micros();
      static uint8_t state = 0; // sleep
      if (state == 0){
        Serial << "\r\nLOCK is in Sleep mode, sending wake signal";
        outputs.setPin(15, 0, 1); // sets PCA9685 pin HIGH 5V, init Wake, after 1ms should be in Standby
        state = 1;
      } else if (state == 1){
        Serial << "\r\nLOCK is in Standby mode, waiting for reset, sending reset pulse";
        outputs.setPin(15, 237, 1);  // Sleep reset pulse
        state = 2;
      } else if (state ==2) {
        Serial << "\r\nLOCK is in Active mode, issuing Sleep signal";
        outputs.setPin(15, 0, 0); // sets PCA9685 pin LOW 0V, Deep Sleep
        state = 0;
      }
      uint32_t t2 = micros();
      Serial << "\r\nLOCK " << t2 - t1 << "uS";
    }

    else if (usbRead == '2')      // drv9243 testing, cycle AUX through sleep, standby, active
    {
      uint32_t t1 = micros();
      static uint8_t state = 0; // sleep
      if (state == 0){
        Serial << "\r\nAUX is in Sleep mode, sending wake signal";
        outputs.setPin(14, 0, 1); // sets PCA9685 pin HIGH 5V, init Wake, after 1ms should be in Standby
        state = 1;
      } else if (state == 1){
        Serial << "\r\nAUX is in Standby mode, waiting for reset, sending reset pulse";
        outputs.setPin(14, 237, 1);  // Sleep reset pulse
        state = 2;
      } else if (state ==2) {
        Serial << "\r\nAUX is in Active mode, issuing Sleep signal";
        outputs.setPin(14, 0, 0); // sets PCA9685 pin LOW 0V, Deep Sleep
        state = 0;
      }
      uint32_t t2 = micros();
      Serial << "\r\nAUX " << t2 - t1 << "uS";
    }


    else if (usbRead == '3')      // drv8243 testing, turn on sec2
    {
      outputs.setPin(1, 0, 0);
    }


    else if (usbRead == '4')      // drv8243 testing, turn off sec2
    {
      outputs.setPin(1, 0, 1);
    }


    else if (usbRead == '5')      // drv8243 testing, Sleep all DRVs (no LEDs)
    {
      for (uint8_t drvNum = 0; drvNum < drvCnt; drvNum++){
        outputs.setPin(drvSleepPins[drvNum], 0, 0);
      }
    }

    else if (usbRead == '6')      // drv9243 testing, Standby all DRVs (red LEDs on LOCK & AUX)
    {
      for (uint8_t drvNum = 0; drvNum < drvCnt; drvNum++){
        outputs.setPin(drvSleepPins[drvNum], 0, 1); // sets PCA9685 pin HIGH 5V, initiate wake-up -> Standby state
      }
    }

    else if (usbRead == '7')      // drv9243 testing, Active all DRVs, AUX green LED, the others white LED if output is active
    {
      for (uint8_t drvNum = 0; drvNum < drvCnt; drvNum++){
        outputs.setPin(drvSleepPins[drvNum], 187, 1);  // Sleep reset pulse
      }
    }

    else if (usbRead == '8')      // drv9243, sleep, wake, activate all DRVs
    {
      for (uint8_t drvNum = 0; drvNum < drvCnt; drvNum++){
        outputs.setPin(drvSleepPins[drvNum], 0, 0); // sets PCA9685 pin LOW 0V, put DRVs to sleep
      }
      delayMicroseconds(150);  // wait max tSLEEP (120uS) for Sleep mode

      // this isn't necessary
      /*for (uint8_t drvNum = 0; drvNum < drvCnt; drvNum++){
        outputs.setPin(drvSleepPins[drvNum], 0, 1); // sets PCA9685 pin HIGH 5V, initiate wake-up -> Standby state
      }
      delayMicroseconds(1000);  // wait tREADY (1000uS) for Standby
      */

      for (uint8_t drvNum = 0; drvNum < drvCnt; drvNum++){
        outputs.setPin(drvSleepPins[drvNum], 187, 1); // LOW pulse, 187/4096 is 30uS at 1532hz, send nSLEEP reset pulse
      }
      //delayMicroseconds(1000);  // wait tREADY (1000uS) for Standby
      // doesn't seem necessary to wait 500uS to set all nSLEEP lines HIGH, just leave them pulsing the Reset pulse
      // This follow setPin isn't needed then either
      /*for (uint8_t drvNum = 0; drvNum < drvCnt; drvNum++){
        outputs.setPin(drvSleepPins[drvNum], 0, 1); // sets PCA9685 pin HIGH 5V, initiate wake-up -> Standby state
      }*/


      /*(for (uint8_t drvNum = 0; drvNum < drvCnt; drvNum++){
        outputs.setPin(drvSleepPins[drvNum], 0, 1); // sets PCA9685 pin HIGH 5V, hold nSLEEP HIGh to maintain Active state
      }*/
    }

    else if (usbRead == '9')      // drv9243 searching
    {
      Wire.beginTransmission(0x44);
      Serial.print("\r\n  - Section DRV8243 ");
      if (Wire.endTransmission() == 0)
        Serial.print("found");
      else
        Serial.print("*NOT found!*");

      Wire.beginTransmission(0x70);
      Serial.print("\r\n  - RGB DRV8243 ");
      if (Wire.endTransmission() == 0)
        Serial.print("found");
      else
        Serial.print("*NOT found!*");
    }

    else if (usbRead == '0')      // Sections drv9243 reset
    {
      //outputs.reset();
    }

    else if (usbRead == 13 || usbRead == 10) // ignore CR or LF
    {
      // do nothing
    }
    else
    {
      // USB serial data not recognized
      Serial << "\r\n\n*** Unrecognized USB serial input: \"" << usbRead << "\" ";
      while (Serial.available())
        Serial.read();
    }
  }

#if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
  static bool prevUSB1DTR;
  USB1DTR = SerialUSB1.dtr();
  if (USB1DTR != prevUSB1DTR)
  {
    Serial << "\r\n**SerialUSB1 " << (USB1DTR ? "bridged with GPS1" : "disconnected");
    if (USB1DTR)
    {
      if (SerialUSB1.baud() == GPS1BAUD)
        Serial << ", baud set at " << baudGPS << " (default)";
    }
    else
    {
      if (GPS1BAUD != baudGPS)
      {
        SerialGPS1.begin(baudGPS);
        GPS1BAUD = baudGPS;
        Serial << ", baud reverted back to default " << GPS1BAUD;
      }
    }
    prevUSB1DTR = USB1DTR;
  }

  if (USB1DTR)
  {
    if (SerialUSB1.baud() != GPS1BAUD)
    {
      SerialGPS1.begin(SerialUSB1.baud());
      GPS1BAUD = SerialUSB1.baud();
      Serial << "\r\n**GPS1 baud changed to " << GPS1BAUD;
      if (GPS1BAUD == baudGPS)
        Serial << " (default)";
    }
  }
#endif

#if defined(USB_TRIPLE_SERIAL)
  static bool prevUSB2DTR;
  USB2DTR = SerialUSB2.dtr();
  if (USB2DTR != prevUSB2DTR)
  {
    Serial << "\r\n**SerialUSB2 " << (USB2DTR ? "bridged with GPS2" : "disconnected");
    if (USB2DTR)
    {
      if (SerialUSB2.baud() == GPS2BAUD)
        Serial << ", baud set at " << baudGPS << " (default)";
    }
    else
    {
      if (GPS2BAUD != baudGPS)
      {
        SerialGPS2.begin(baudGPS);
        GPS2BAUD = baudGPS;
        Serial << ", baud reverted back to default " << GPS2BAUD;
      }
    }
    prevUSB2DTR = USB2DTR;
  }

  if (USB2DTR)
  {
    if (SerialUSB2.baud() != GPS2BAUD)
    {
      SerialGPS2.begin(SerialUSB2.baud());
      GPS2BAUD = SerialUSB2.baud();
      Serial << "\r\n**GPS2 baud changed to " << GPS2BAUD;
      if (GPS2BAUD == baudGPS)
        Serial << " (default)";
    }
  }
#endif
}

void printTelem()
{
  if (printStats || !gps1Stats.startupReset)
  {
    if (!gps1Stats.startupReset)
    {
      ggaMissed = 0;
      ubxParser.relMissed = 0;
    }
    gps1Stats.printStatsReport((char *)"GPS1");
    gps2Stats.printStatsReport((char *)"GPS2");
    relJitterStats.printStatsReport((char *)"RELj");
    relTtrStats.printStatsReport((char *)"RELr");
    bnoStats.printStatsReport((char *)"BNO");
    Serial.println();
  }

  if (printCpuUsages)
  {
    // just hammering testCounter++ in the main loop() uses some CPU time
    // baselineProcUsage gets that value, which is used to offset the other usage checks that are hammered in the same main loop() (at the same freq)
    uint32_t baselineProcUsage = LOOPusage.reportAve();
    uint32_t dacReport = DACusage.reportAve(); // subracted from AS cpu usage below

    Serial.print("\r\n\nLoop   cpu: ");
    printCpuPercent(baselineProcUsage);
    Serial.print(" ");
    Serial.print(testCounter / bufferStatsTimer);
    Serial.print("kHz");
    // 300,000+ hits/s (300+ khz loop() speed) on prev generation non-UI dev firmware
    // new Mongoose UI based firmware runs at 100+ khz 

    Serial.print("\r\nGUI    cpu: ");
    printCpuPercent(GUIusage.reportAve(baselineProcUsage));
    Serial.print("\r\nBNO_R  cpu: ");
    printCpuPercent(BNOusage.reportAve(baselineProcUsage));
    Serial.print("\r\nGPS1   cpu: ");
    printCpuPercent(GPS1usage.reportAve(baselineProcUsage));
    Serial.print("\r\nGPS2   cpu: ");
    printCpuPercent(GPS2usage.reportAve(baselineProcUsage));
    Serial.print("\r\nPGN    cpu: ");
    printCpuPercent(PGNusage.reportAve()); // uses a timed update, virtually no extra time penalty
    Serial.print("\r\nAS     cpu: ");
    printCpuPercent(ASusage.reportAve(dacReport)); // dac update loop is inside AS update loop (don't want to double count CPU time)
    Serial.print("\r\nNTRIP  cpu: ");
    printCpuPercent(NTRIPusage.reportAve()); // uses a timed update, virtually no extra time penalty
    //Serial.print("\r\nIMU_H  cpu: ");
    //printCpuPercent(IMU_Husage.reportAve());
    Serial.print("\r\nNMEA_P cpu: ");
    printCpuPercent(NMEA_Pusage.reportAve());
    Serial.print("\r\nUBX_P  cpu: ");
    printCpuPercent(UBX_Pusage.reportAve());
    Serial.print("\r\nUDP_S  cpu: ");
    printCpuPercent(UDP_Susage.reportAve());
    Serial.print("\r\nLEDS   cpu: ");
    printCpuPercent(LEDSusage.reportAve(baselineProcUsage));
    Serial.print("\r\nMach   cpu: ");
    printCpuPercent(MACHusage.reportAve(baselineProcUsage));
    //printCpuPercent(MACHusage.reportAve());
    Serial.print("\r\nESP32  cpu: ");
    printCpuPercent(ESP32usage.reportAve(baselineProcUsage));
    Serial.print("\r\nKEYA   cpu: ");
    printCpuPercent(KEYAusage.reportAve(baselineProcUsage));

#ifdef AIOv50a
    Serial.print("\r\nRS232  cpu: ");
    printCpuPercent(RS232usage.reportAve(baselineProcUsage));
#endif

#ifdef JD_DAC_H
    Serial.print("\r\nDAC    cpu: ");
    printCpuPercent(dacReport);
#endif

    Serial.println();
  }

  testCounter = 0;
  bufferStatsTimer = 0;
}

#endif