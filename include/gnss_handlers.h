#ifndef GNSS_HANDLERS_H_
#define GNSS_HANDLERS_H_
// Conversion to Hexidecimal
const char *asciiHex = "0123456789ABCDEF";

// the new PANDA sentence buffer
// char nmea[100];
char *nmea = "\0";

struct GGA_DATA
{
  char fixTime[12];
  char latitude[15];
  char latNS[3];
  char longitude[15];
  char lonEW[3];
  char fixQuality[2];
  char numSats[4];
  char HDOP[5];
  char altitude[12];
  char ageDGPS[10];
};
GGA_DATA GGA;

struct VTG_DATA
{
  char heading[12];
  char speedKnots[10];
};
VTG_DATA VTG;

struct HPR_DATA
{
  char heading[8];
  char roll[8];
  int solQuality;
};
HPR_DATA HPR;

struct IMU_DATA
{
  char heading[6];
  char roll[6];
  char pitch[6];
  char yawRate[6];
};
IMU_DATA IMU;

// uint32_t nmeaPgnSendTime, nmeaPgnMaxPeriod, nmeaPgnAvePeriod, nmeaPgnMinPeriod = 99999;
// uint8_t nmeaCount;
elapsedMicros aogGpsToAutoSteerLoopTimer;
bool aogGpsToAutoSteerLoopTimerEnabled;

// If odd characters showed up
void errorHandler()
{
  if (startup)
    Serial.print("\r\n*** Unexpected characters in NMEA parser ***");
}

void prepImuPandaData() // run after GGA update + 40ms (timing for PANDA), for next GGA
{
  if (BNO.isActive)
  {
    itoa(BNO.rvcData.yawX10, IMU.heading, 10); // format IMU data for Panda Sentence - Heading

    if (BNO.isSwapXY)
    {
      itoa(BNO.rvcData.pitchX10, IMU.roll, 10); // the pitch x10
      itoa(BNO.rvcData.rollX10, IMU.pitch, 10); // the roll x10
    }
    else
    {
      itoa(BNO.rvcData.pitchX10, IMU.pitch, 10); // the pitch x10
      itoa(BNO.rvcData.rollX10, IMU.roll, 10);   // the roll x10
    }

    // Serial.print(BNO.angCounter);
    // Serial.print(", ");
    // Serial.print(BNO.rvcData.angVel);
    // Serial.print(", ");

    // YawRate
    double angVel;
    if (BNO.angCounter > 0)
    {
      angVel = ((double)BNO.rvcData.angVel) / (double)BNO.angCounter;
      angVel *= 10.0;
      BNO.angCounter = 0;
      BNO.rvcData.angVel = (int16_t)angVel;
    }
    else
    {
      BNO.rvcData.angVel = 0;
    }

    itoa(BNO.rvcData.angVel, IMU.yawRate, 10);
    BNO.rvcData.angVel = 0;

    // digitalWrite(GPS_RED_LED, 0);
    // digitalWrite(GPS_GRN_LED, 0);
  }
  else // No BNO in RVC mode or its disconnected, set IMU PANDA components to signal AOG that there's no IMU
  {
    itoa(65535, IMU.heading, 10);
    IMU.roll[0] = 0;
    IMU.pitch[0] = 0;
    IMU.yawRate[0] = 0;
  }
}

void CalculateChecksum(void)
{
  int16_t sum = 0;
  int16_t inx = 0;
  char tmp;

  // The checksum calc starts after '$' and ends before '*'
  for (inx = 1; inx < 200; inx++)
  {
    tmp = nmea[inx];

    // * Indicates end of data and start of checksum
    if (tmp == '*')
    {
      break;
    }

    sum ^= tmp; // Build checksum
  }

  byte chk = (sum >> 4);
  char hex[2] = {asciiHex[chk], 0};
  strcat(nmea, hex);

  chk = (sum % 16);
  char hex2[2] = {asciiHex[chk], 0};
  strcat(nmea, hex2);
}

void buildPandaOrPaogi(bool _panda) // only called by GGA_Handler (above)
{
  gpsActive = true;

  if (_panda)
    strcpy(nmea, "$PANDA,");
  else
    strcpy(nmea, "$PAOGI,");

  strcat(nmea, GGA.fixTime);
  strcat(nmea, ","); // field 1
  strcat(nmea, GGA.latitude);
  strcat(nmea, ",");
  strcat(nmea, GGA.latNS);
  strcat(nmea, ",");
  strcat(nmea, GGA.longitude);
  strcat(nmea, ",");
  strcat(nmea, GGA.lonEW);
  strcat(nmea, ","); // 5
  strcat(nmea, GGA.fixQuality);
  strcat(nmea, ",");
  strcat(nmea, GGA.numSats);
  strcat(nmea, ",");
  strcat(nmea, GGA.HDOP);
  strcat(nmea, ",");
  strcat(nmea, GGA.altitude);
  strcat(nmea, ","); // 9
  strcat(nmea, GGA.ageDGPS);
  strcat(nmea, ",");
  strcat(nmea, VTG.speedKnots);
  strcat(nmea, ",");

  if (_panda)
  { // use BNO values
    strcat(nmea, IMU.heading);
    strcat(nmea, ",");
    strcat(nmea, IMU.roll);
    strcat(nmea, ","); // 13
    strcat(nmea, IMU.pitch);
    strcat(nmea, ",");
    strcat(nmea, IMU.yawRate);
  }
  else
  { // use Dual values
    char temp[6];
    dtostrf(ubxParser.ubxData.baseRelH, 4, 2, temp);
    strcat(nmea, temp); // 12, heading
    strcat(nmea, ",");

    char temp2[6];
    dtostrf(ubxParser.ubxData.baseRelRoll, 4, 2, temp2);
    strcat(nmea, temp2); // 13, roll
    strcat(nmea, ",");

    strcat(nmea, ""); // blank pitch
    strcat(nmea, ",");
    strcat(nmea, ""); // blank yaw rate
  }

  strcat(nmea, "*");
  CalculateChecksum();
  strcat(nmea, "\r\n");

  if (nmeaDebug)
  {
    // Serial.print("\r\n");
    Serial.print(millis());
    Serial.print(" ");
    Serial.write(nmea);
    Serial.println();
    extraCRLF = false;
  }

  udpRunning = true;
  if (udpRunning) // If ethernet running send the GPS there
  {
    // send char stream
    //  Serial.print("Sentence: ");
    //  Serial.write(nmea);
    sendUDPchars(nmea);
  }
  else if (!nmeaDebug)
  {
    // Serial.write(nmea);  // if Eth is !connected, send USB GPS data
  }
}

void GGA_GNS_PostProcess() // called by either GGA or GNS handler
{
  posReady = true;       // we have new GGA or GNS sentence
  imuPandaSyncTimer = 0; // reset imu timer
  imuPandaSyncTrigger = true;
  extraCRLF = true;
  gps1Stats.incHzCount();
  LEDs.setGpsLED(atoi(GGA.fixQuality));
  aogGpsToAutoSteerLoopTimer = 0;
  // aogGpsToAutoSteerLoopTimerEnabled = 1;  // uncomment to print "AIO GPS->AOG->Steer Data back to AIO" delay

  if (!ubxParser.useDual)
  {                                  // if not using Dual
    buildPandaOrPaogi(PANDA_SINGLE); // build the PANDA sentence right away
    posReady = false;
  } // otherwise wait in main loop() until relposned(f9p) or hpr(um982) arrives
}

void GNS_Handler() // Rec'd GNS
{
  NMEA_Pusage.timeIn();

  nmeaParser.getArg(0, GGA.fixTime);  // fix time
  nmeaParser.getArg(1, GGA.latitude); // latitude
  nmeaParser.getArg(2, GGA.latNS);
  nmeaParser.getArg(3, GGA.longitude); // longitude
  nmeaParser.getArg(4, GGA.lonEW);

  char temp[4];
  nmeaParser.getArg(5, temp);
  switch (temp[0])
  { // check GPS fix qual
  case 'A':
    itoa(1, GGA.fixQuality, 10); // 1: autonomous, no correction
    break;
  case 'D':
    itoa(2, GGA.fixQuality, 10); // 2: differential (WAAS)
    break;
  case 'F':
    itoa(5, GGA.fixQuality, 10); // 5: FLOAT
    break;
  case 'R':
    itoa(4, GGA.fixQuality, 10); // 4: RTK FIX
    break;
  case 'E':
    itoa(6, GGA.fixQuality, 10); // 6: Dead reckoning
    break;
  case 'S':
    itoa(4, GGA.fixQuality, 10); // ?: Simulator
    break;
  case 'N':
  default:
    itoa(0, GGA.fixQuality, 10); // 0: fix not valid
    break;
  }

  nmeaParser.getArg(6, GGA.numSats);  // satellite #
  nmeaParser.getArg(7, GGA.HDOP);     // HDOP
  nmeaParser.getArg(8, GGA.altitude); // altitude
  nmeaParser.getArg(10, GGA.ageDGPS); // time of last DGPS update

  if (nmeaDebug)
  {
    // Serial.print("\r\n");
    Serial.print(millis());
    Serial.printf(" GNS update (%i)", ggaMissed);
    Serial.print(imuPandaSyncTimer);
    Serial.print(" ");
    Serial.println(atoi(&GGA.fixTime[strlen(GGA.fixTime) - 2]));
  }
  GGA_GNS_PostProcess();
  LEDs.toggleTeensyLED();
  // gpsLostTimer = 0;  // Used for GGA timeout (LED's ETC)
  NMEA_Pusage.timeOut();
}

void GGA_Handler() // Rec'd GGA
{
  NMEA_Pusage.timeIn();
  nmeaParser.getArg(0, GGA.fixTime);  // fix time
  nmeaParser.getArg(1, GGA.latitude); // latitude
  nmeaParser.getArg(2, GGA.latNS);
  nmeaParser.getArg(3, GGA.longitude); // longitude
  nmeaParser.getArg(4, GGA.lonEW);
  nmeaParser.getArg(5, GGA.fixQuality); // fix quality
  nmeaParser.getArg(6, GGA.numSats);    // satellite #
  nmeaParser.getArg(7, GGA.HDOP);       // HDOP
  nmeaParser.getArg(8, GGA.altitude);   // altitude
  nmeaParser.getArg(12, GGA.ageDGPS);   // time of last DGPS update

  if (nmeaDebug)
  {
    // Serial.print("\r\n");
    Serial.print(millis());
    Serial.printf(" GGA update (%i)", ggaMissed);
    Serial.print(imuPandaSyncTimer);
    Serial.print(" ");
    Serial.println(atoi(&GGA.fixTime[strlen(GGA.fixTime) - 2]));
  }
  GGA_GNS_PostProcess();
  LEDs.toggleTeensyLED();
  // gpsLostTimer = 0;  // Used for GGA timeout (LED's ETC)
  NMEA_Pusage.timeOut();
}

void VTG_Handler()
{
  nmeaParser.getArg(0, VTG.heading);    // vtg heading
  nmeaParser.getArg(4, VTG.speedKnots); // vtg Speed knots
}

void PVT_Handler()
{
  Serial << "\r\n"
         << millis() << " PVT received\r\n";
}

void HPR_Handler()
{
  NMEA_Pusage.timeIn();

  nmeaParser.getArg(1, HPR.heading);    // UM982 heading
  nmeaParser.getArg(2, HPR.roll);       // UM982 roll (pitch)
  nmeaParser.getArg(4, HPR.solQuality); // UM982 heading solution quality
  char fixTime[12];
  nmeaParser.getArg(0, fixTime); // fix time

  // Keep ubx stuff in main loop happy
  ubxParser.relPosNedReady = true;
  if (!ubxParser.useDual)
    ubxParser.firstHeadingDetected = 1;
  ubxParser.useDual = true;
  ubxParser.relPosTimer = 0;

  if (fuseImu.fuseData.useFUSEImu)
  { // Three separate if/else cluases for clarity. Can be one.
    // Send data to FUSEImu
    fuseImu.fuseData.rollDual = atof(HPR.roll);
    fuseImu.fuseData.heading = atof(HPR.heading);
    fuseImu.fuseData.correctionHeading = BNO.rvcData.yawX10;
    fuseImu.fuseData.rollImu = BNO.rvcData.pitchX10;
    fuseImu.imuDualDelta();
  }

  if (fuseImu.fuseData.useFUSEImu)
  { // Three separate if/else cluases for clarity. Can be one.
    ubxParser.ubxData.baseRelH = fuseImu.fuseData.imuCorrected;
  }
  else
  {
    ubxParser.ubxData.baseRelH = atof(HPR.heading);
  }

  if (fuseImu.fuseData.useFUSEImu)
  { // Three separate if/else cluases for clarity. Can be one.
    // if ( HPR.solQuality == 4 ) {
    ubxParser.ubxData.baseRelRoll = fuseImu.fuseData.rollDeltaSmooth;
    // } else {
    //   ubxParser.ubxData.baseRelRoll *= 0.9;     // "level off" dual roll
    // }
  }
  else
  {
    if (HPR.solQuality == 4)
    {
      ubxParser.ubxData.baseRelRoll = atof(HPR.roll);
    }
    else
    {
      ubxParser.ubxData.baseRelRoll *= 0.9; // "level off" dual roll
    }
  }

  if (nmeaDebug)
  {
    // Serial.print("\r\n");
    Serial.print(millis());
    Serial.printf(" HPR update "); //(%i)", headingMissed);
    // Serial.print(headingTimer); Serial.print(" ");
    Serial.println(atoi(&fixTime[strlen(fixTime) - 2]));
  }

  // headingTimer = 0;
  // hprReady = 1;

  NMEA_Pusage.timeOut();
}

#endif // GNSSHANDLERS_H_
