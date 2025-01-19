#ifndef GPS2POLL_H_
#define GPS2POLL_H_
#include "common.h"

void gps2Poll()
{
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
}

#endif