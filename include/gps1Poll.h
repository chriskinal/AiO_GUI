#ifndef GPS1POLL_H_
#define GPS1POLL_H_
#include "common.h"
#include "udpHandlers.h"

void gps1Poll()
{
    if (!USB1DTR)                 // carry on like normal
    {
        uint16_t gps1Available = SerialGPS1.available();
        if (gps1Available)    // "if" is very crucial here, using "while" causes BNO overflow
        {
        if (gps1Available > sizeof(GPS1rxbuffer) - 10) {   // this should not trigger except maybe at boot up
            SerialGPS1.clear();
            Serial.print((String)"\r\n" + millis() + " *** SerialGPS1 buffer cleared! ***");
            return;
        }
        gps1Stats.update(gps1Available);

        uint8_t gps1Read = SerialGPS1.read();
        if (nmeaDebug) Serial.write(gps1Read);

        if ( udpPassthrough == false)
        {
            nmeaParser << gps1Read;
        } else {
            
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
            // Serial.print(msgBuf);
            // Serial.println(msgBufLen);
            // if (sendUSB)
            // {
            //   SerialAOG.write(msgBuf);
            // } // Send USB GPS data if enabled in user settings
            if ( udpRunning )
            {
                sendUDPchars(msgBuf);
            }
            gotCR = false;
            gotLF = false;
            gotDollar = false;
            memset(msgBuf, 0, 254);
            msgBufLen = 0;
            ubxParser.relPosTimer = 0;
            imuPandaSyncTimer =0;
            LEDs.toggleTeensyLED();
            }
        }
        
        SerialRS232.write(gps1Read);

        }

    }

}

#endif