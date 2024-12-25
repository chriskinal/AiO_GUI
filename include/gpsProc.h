#ifndef GPSPROC_H_
#define GPSPROC_H_
#include "common.h"

void gpsProc()
{
    // ******************* For SINGLE/RIGHT *******************
    if (imuPandaSyncTimer > 50 && startup) {   // to make sure old data isn't sent to AOG
        if (posReady) {
        posReady = 0;
        Serial.print("\r\n**Position data expired**\r\n");
        }
    
        if (extraCRLF && nmeaDebug) {
        Serial.print("\r\n");
        extraCRLF = false;
        }
    }

    if (imuPandaSyncTimer > 150 && startup) {
        imuPandaSyncTimer -= 100;
        ggaMissed++;
        if (nmeaDebug) Serial.println();
        Serial.print("\r\n"); Serial.print(millis()); Serial.print(" ");
        Serial.printf("                 *** GGA was missed or late! *** (%i)\r\n", ggaMissed);
        posReady = false;
        ubxParser.relPosNedReady = false;
    }

    // ******************* For DUAL LEFT *******************
    if (ubxParser.relPosNedReady && posReady) {   // if both GGA & relposNED are ready
        buildPandaOrPaogi(PAOGI_DUAL);              // build a PAOGI msg
        ubxParser.relPosNedReady = false;           // reset for next relposned trigger
        ubxParser.relPosNedRcvd = false;
        posReady = false;
    }

    if (ubxParser.relPosTimer > 50 && ubxParser.relPosNedReady && startup) {    // to make sure old data isn't sent to AOG
        ubxParser.relPosNedReady = 0;
        if (!ubxParser.firstHeadingDetected) {
        Serial.print("\r\n**Heading data expired**\r\n");
        ubxParser.firstHeadingDetected = 0;
        }
    }

    if (ubxParser.relPosTimer > 150 && ubxParser.useDual && startup) {
        ubxParser.relPosTimer -= 100;
        ubxParser.relMissed++;
        if (nmeaDebug) Serial.println();
        Serial.print("\r\n"); Serial.print(millis()); Serial.print(" ");
        Serial.printf("                   *** relposNED was missed or late! *** (%i)\r\n", ubxParser.relMissed);
        ubxParser.clearCount();
        posReady = false;
        ubxParser.relPosNedReady = false;
    }
}

#endif