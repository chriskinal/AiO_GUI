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
                Serial.print((String) "\r\n" + millis() + " *** SerialGPS1 buffer cleared! ***");
                return;
            }
            gps1Stats.update(gps1Available);

            uint8_t gps1Read = SerialGPS1.read();
            if (nmeaDebug)
                Serial.write(gps1Read);

            if (udpPassthrough == false)
            {
                nmeaParser << gps1Read;
            }
            else
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
                    // } // Send USB GPS data if enabled in user settings
                    if (udpRunning)
                    {
                        sendUDPchars(msgBuf);
                    }
                    gotCR = false;
                    gotLF = false;
                    gotDollar = false;
                    memset(msgBuf, 0, 254);
                    msgBufLen = 0;
                    ubxParser.relPosTimer = 0;
                    imuPandaSyncTimer = 0;
                    LEDs.toggleTeensyLED();
                }
            }

            SerialRS232.write(gps1Read);
        }
    }
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
                Serial.print((String) "\r\n" + millis() + " *** SerialGPS2 buffer cleared! ***");
                return;
            }
            gps2Stats.update(gps2Available);

            uint8_t gps2Read = SerialGPS2.read();
            if (nmeaDebug2)
                Serial << "(" << byte(gps2Read) << ")";
            ubxParser.parse(gps2Read);
        }
    }
    GPS2usage.timeOut();
}

// Serial RTCM
void serialRTCM()
{
    RS232usage.timeIn();
    if (SerialRTK.available())
    { // Check for RTK Radio RTCM data
        uint8_t rtcmByte = SerialRTK.read();
        if (!USB1DTR)
            SerialGPS1.write(rtcmByte); // send to GPS1
        if (!USB2DTR)
            SerialGPS2.write(rtcmByte); // send to GPS2
        LEDs.queueBlueFlash(LED_ID::GPS);
    }

    if (SerialRS232.available())
    {                                     // Check for RS232 data
        Serial.write(SerialRS232.read()); // just print to USB for testing
    }
    RS232usage.timeOut();
}

#endif