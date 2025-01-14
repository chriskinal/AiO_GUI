#ifndef MULTIUSB_H_
#define MULTIUSB_H_
#include "Arduino.h"

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

#ifdef RESET_H
teensyReset.update(); // reset.h
#endif

#endif