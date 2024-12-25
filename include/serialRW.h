#ifndef SERIALRW_H_
#define SERIALRW_H_

void serialRW(){
    if (SerialRTK.available()) {              // Check for RTK Radio RTCM data
    uint8_t rtcmByte = SerialRTK.read();
    if (!USB1DTR) SerialGPS1.write(rtcmByte);    // send to GPS1
    if (!USB2DTR) SerialGPS2.write(rtcmByte);    // send to GPS2
    LEDs.queueBlueFlash(LED_ID::GPS);
    }

    if (SerialRS232.available()) {           // Check for RS232 data
        Serial.write(SerialRS232.read());      // just print to USB for testing
    }
}

#endif