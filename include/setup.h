// AIO_GUI is copyright 2025 by the AOG Group
// AiO_GUI is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
// AiO_GUI is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with Foobar. If not, see <https://www.gnu.org/licenses/>.
// Like most Arduino code, portions of this are based on other open source Arduino code with a compatiable license.

#ifndef SETUP_H_
#define SETUP_H_
void setCpuFrequency(uint32_t _freq)
{
  set_arm_clock(_freq);
  Serial.printf("\r\n\nCPU speed set to: %i MHz\r\n", F_CPU_ACTUAL / 1000000);
  delay(10); // ?
}

void serialSetup()
{
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);  // disables the buzzer's FET driver

  // setup GPS serial ports here
  SerialGPS1.begin(baudGPS);
  GPS1BAUD = baudGPS;
  SerialGPS1.addMemoryForRead(GPS1rxbuffer, sizeof(GPS1rxbuffer));
  SerialGPS1.addMemoryForWrite(GPS1txbuffer, sizeof(GPS1txbuffer));

  SerialRTK.begin(baudRTK);
  SerialRTK.addMemoryForRead(RTKrxbuffer, sizeof(RTKrxbuffer));

  SerialGPS2.begin(baudGPS);
  GPS2BAUD = baudGPS;
  SerialGPS2.addMemoryForRead(GPS2rxbuffer, sizeof(GPS2rxbuffer));
  SerialGPS2.addMemoryForWrite(GPS2txbuffer, sizeof(GPS2txbuffer));

  SerialRS232.begin(baudRS232);
  // SerialRS232.addMemoryForRead(RS232rxbuffer, sizeof(RS232rxbuffer));    // not needed unless custom rs232 rx code is added
  SerialRS232.addMemoryForWrite(RS232txbuffer, sizeof(RS232txbuffer));

  SerialESP32.begin(baudESP32);
  SerialESP32.addMemoryForRead(ESP32rxbuffer, sizeof(ESP32rxbuffer));
  SerialESP32.addMemoryForWrite(ESP32txbuffer, sizeof(ESP32txbuffer));
}

void parserSetup()
{
  // the dash means wildcard
  nmeaParser.setErrorHandler(errorHandler);
  nmeaParser.addHandler("G-GGA", GGA_Handler);
  nmeaParser.addHandler("G-GNS", GNS_Handler);
  nmeaParser.addHandler("G-VTG", VTG_Handler);
  nmeaParser.addHandler("G-HPR", HPR_Handler);
  nmeaParser.addHandler("KSXT", KSXT_Handler);
}

void resetStartingTimersBuffers()
{
  // machine.watchdogTimer = 0;
  if (BNO.isActive)
    while (!BNO.read(true))
      ;
  SerialGPS1.clear();
  SerialGPS2.clear();
#ifdef AIOv50a
  SerialESP32.clear();
#endif
  // machine.watchdogTimer = 0;
  startup = true;
}

#endif // SETUP_H_