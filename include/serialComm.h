// AIO_GUI is copyright 2025 by the AOG Group
// AiO_GUI is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
// AiO_GUI is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with Foobar. If not, see <https://www.gnu.org/licenses/>.
// Like most Arduino code, portions of this are based on other open source Arduino code with a compatiable license.

#ifndef SERIALCOMM_H_
#define SERIALCOMM_H_

#include "common.h"
#include "udpHandlers.h"

// Poll GPS
void gpsPoll()
{
  // GPS1
  GPS1usage.timeIn();
  if (!USB1DTR) // carry on like normal
  {
    uint16_t gps1Available = SerialGPS1.available();
    if (gps1Available) // "if" is very crucial here, using "while" causes BNO overflow
    {
      if (gps1Available > sizeof(GPS1rxbuffer) - 10)
      { // this should not trigger except maybe at boot up
        SerialGPS1.clear();
        Serial.print((String) "\r\n" + millis() + " *SerialGPS1 buffer cleared!-Normal at startup*");
        return;
      }
      gps1Stats.update(gps1Available);

      uint8_t gps1Read = SerialGPS1.read();
      if (nmeaDebug)
        Serial.write(gps1Read);

      if (gpsConfig.gpsPass == true)
      {
        switch (gps1Read)
        {
        case '$':
          msgBuf[msgBufLen] = gps1Read;
          msgBufLen++;
          gotDollar = true;
          break;
        case '\r':
          msgBuf[msgBufLen] = gps1Read;
          msgBufLen++;
          gotCR = true;
          gotDollar = false;
          break;
        case '\n':
          msgBuf[msgBufLen] = gps1Read;
          msgBufLen++;
          gotLF = true;
          gotDollar = false;
          break;
        default:
          if (gotDollar)
          {
            msgBuf[msgBufLen] = gps1Read;
            msgBufLen++;
          }
          break;
        }
        if (gotCR && gotLF)
        {

          sendUDPchars(msgBuf);
          gotCR = false;
          gotLF = false;
          gotDollar = false;
          memset(msgBuf, 0, 254);
          msgBufLen = 0;
          ubxParser.relPosTimer = 0;
        }
      }

      nmeaParser << gps1Read;   // process after UDP passthrough check to send data to AgIO first

      GPS1usage.timeOut();
      RS232usage.timeIn();
      SerialRS232.write(gps1Read);
      RS232usage.timeOut();
    }
  }
// MultiUSB
#if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
  else
  { // in SerialUSB1<->SerialGPS1 bridge mode, for connecting via u-center
    if (SerialGPS1.available())
    {
      while (SerialGPS1.available())
      { // seems necessary to keep sentences/packets grouped as tight as possible
        SerialUSB1.write(SerialGPS1.read());
        // Serial.write(SerialGPS1.read());
      }
    }

    if (SerialUSB1.available())
    { // seems necessary to ensure UBX msgs from U-Center aren't interrupted by RTCM data (xbee or ntrip)
      while (SerialUSB1.available())
      {
        SerialGPS1.write(SerialUSB1.read());
      }
    }
  }
#endif

  GPS1usage.timeOut();

  // GPS2
  GPS2usage.timeIn();
  if (!USB2DTR) // carry on like normal
  {
    uint16_t gps2Available = SerialGPS2.available();
    if (gps2Available)
    {
      if (gps2Available > sizeof(GPS2rxbuffer) - 10)
      { // this should not trigger except maybe at boot up
        SerialGPS2.clear();
        Serial.print((String) "\r\n" + millis() + " *SerialGPS2 buffer cleared!-Normal at startup*\r\n");
        return;
      }
      gps2Stats.update(gps2Available);

      uint8_t gps2Read = SerialGPS2.read();
      if (nmeaDebug2)
        Serial << "(" << byte(gps2Read) << ")";
      GPS2usage.timeOut();
      UBX_Pusage.timeIn();
      ubxParser.parse(gps2Read);
      UBX_Pusage.timeOut();
    }
  }
#if defined(USB_TRIPLE_SERIAL)
  else
  { // in SerialUSB2<->SerialGPS2 bridge mode, for connecting via u-center
    if (SerialGPS2.available())
    {
      while (SerialGPS2.available())
      { // seems necessary to keep sentences/packets grouped as tight as possible
        SerialUSB2.write(SerialGPS2.read());
      }
    }
    if (SerialUSB2.available())
    { // seems necessary to ensure UBX msgs from U-Center aren't interrupted by RTCM data (xbee or ntrip)
      while (SerialUSB2.available())
      {
        SerialGPS2.write(SerialUSB2.read());
      }
    }
  }
#endif
  GPS2usage.timeOut();
}

// Forward PGN's from ESP32 to AgIO
void serialESP32()
{
  ESP32usage.timeIn();
  if (SerialESP32.available())
  {
    static uint8_t incomingBytes[50];
    static uint8_t incomingIndex;
    incomingBytes[incomingIndex] = SerialESP32.read();
    incomingIndex++;
    // Serial.print("\r\nindex: "); Serial.print(incomingIndex);
    // Serial.print(" ");
    // for (byte i = 0; i < incomingIndex; i++) {
    // Serial.print(incomingBytes[i]);
    // Serial.print(" ");
    //}
    if (incomingBytes[incomingIndex - 2] == 13 && incomingBytes[incomingIndex - 1] == 10)
    {
      if (incomingBytes[0] == 128 && incomingBytes[1] == 129)
      {

        // Modules--Wifi:9999-->ESP32--serial-->Teensy--ethernet:9999-->AgIO
        sendUDPbytes(incomingBytes, incomingIndex - 2);

        // pass data to USB for debug
        // Serial.print("\r\nE32-s->T41-e:9999->AgIO ");
        // for (byte i = 0; i < incomingIndex - 2; i++) {
        // Serial.print(incomingBytes[i]);
        // Serial.print(" ");
        //}
        // Serial.print((String)" (" + SerialESP32->available() + ")");
      }
      else
      {
        Serial.print("\r\n\nCR/LF detected but [0]/[1] bytes != 128/129\r\n");
      }
      incomingIndex = 0;
    }
  }
  ESP32usage.timeOut();
}

// Serial RTCM
void serialRTCM()
{
  if (SerialRTK.available())
  { // Check for RTK Radio RTCM data
    uint8_t rtcmByte = SerialRTK.read();
    if (!USB1DTR)
      SerialGPS1.write(rtcmByte); // send to GPS1
    // only send to GPS2 if using abnormal setup like OGX receiver on GPS2
    // if (!USB2DTR)
    // SerialGPS2.write(rtcmByte); // send to GPS2
    LEDs.queueBlueFlash(LED_ID::GPS);
  }
}

#endif