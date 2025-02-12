// AIO_GUI is copyright 2025 by the AOG Group
// AiO_GUI is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
// AiO_GUI is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with Foobar. If not, see <https://www.gnu.org/licenses/>.
// Like most Arduino code, portions of this are based on other open source Arduino code with a compatiable license.

#ifndef UDPHANDLERS_H_
#define UDPHANDLERS_H_
#include "Arduino.h"
#include "mongoose_glue.h"
//#include "machine.h"
#include "Autosteer.h"
#include "common.h"

// Send byte arrays to AgIO
void sendUDPbytes(uint8_t *message, int msgLen)
{
  UDP_Susage.timeIn();
  if (g_mgr.ifp->state != MG_TCPIP_STATE_READY)
    return; // Check if IP stack is up.
  // Send data
  if (mg_send(sendAgio, message, msgLen) <= 0)
  {
    Serial.println("UDP Send to AgIO failed.\r\n");
  }
  else
  {
    mg_iobuf_del(&sendAgio->send, 0, sendAgio->send.len);
  }
  UDP_Susage.timeOut();
}

// Send char arrays to AgIO
void sendUDPchars(char *stuff)
{
  UDP_Susage.timeIn();
  if (g_mgr.ifp->state != MG_TCPIP_STATE_READY)
    return; // Check if IP stack is up.
  mg_printf(sendAgio, stuff);
  UDP_Susage.timeOut();
}

void printPgnAnnoucement(struct mg_connection *udpPacket, char* _pgnName)
{
  // One line PGN data display
  Serial.print("\r\n0x"); Serial.print(udpPacket->recv.buf[3], HEX);
  Serial.print("("); Serial.print(udpPacket->recv.buf[3]); Serial.print(")-");
  Serial.print(_pgnName);
  for (uint8_t i = strlen(_pgnName); i < 20; i++) Serial.print(" ");  // to align PGN name length for PGN data dump
  Serial.printf(" %2i Data>", udpPacket->recv.len);
  //Serial.print(millis());

  //Serial.print(" Data: ");
  for (byte i = 4; i < udpPacket->recv.len - 1; i++) {      // -1 skips printing CRC
    Serial.printf("%3i", udpPacket->recv.buf[i]); Serial.print(" ");    // aligns all data bytes using empty/leading whitespace
    //Serial.print(packet.data()[i]); Serial.print(" ");          // no data byte alignment
  }
  
  // Two line PGN data display
  /*Serial.print("\r\n0x"); Serial.print(udpPacket->recv.buf[3], HEX);
  Serial.print(" ("); Serial.print(udpPacket->recv.buf[3]); Serial.print(") - ");
  Serial.print(_pgnName); Serial.print(", "); Serial.print(udpPacket->recv.len); Serial.print(" bytes ");
  Serial.print(millis());

  Serial.print("\r\nData: ");
  for (byte i = 0; i < udpPacket->recv.len; i++) {
    Serial.print(udpPacket->recv.buf[i]); Serial.print(" ");
  }
  Serial.print(" Length: ");
  Serial.print(udpPacket->recv.len);

  Serial.println();*/
}

// Process data received on port 8888
void pgnHandler(struct mg_connection *udpPacket, int ev, void *ev_data, void *fn_data)
{
  if (g_mgr.ifp->state != MG_TCPIP_STATE_READY)
    return; // Check if IP stack is up.
  if (ev == MG_EV_ERROR)
  {
    Serial.printf("Error: %s", (char *)ev_data);
  }

  if (ev == MG_EV_READ && mg_ntohs(udpPacket->rem.port) == 9999 && udpPacket->recv.len >= 5)
  {
    //printPgnAnnoucement(udpPacket, (char*)"All PGNs");

    // Verify first 3 PGN header bytes
    if (udpPacket->recv.buf[0] != 128 || udpPacket->recv.buf[1] != 129 || udpPacket->recv.buf[2] != 127)
      return;

    #ifdef MACHINE_H
      // 0xE5 (229) - 64 Section Data
      // 0xEB (235) - Section Dimensions
      // 0xEC (236) - Machine Pin Config
      // 0xEE (238) - Machine Config
      // 0xEF (239) - Machine Data
      if (machinePTR->parsePGN(udpPacket->recv.buf, udpPacket->recv.len, udpPacket->rem.ip, netConfig.currentIP))     // look for Machine PGNs, return TRUE if machine specific PGN was found
      {
        return;   // abort further PGN processing if machine specific PGN as received/parsed
      }
    #endif

    PGNusage.timeIn();

    // 0x64 (100) - Corrected Position
    if (udpPacket->recv.buf[3] == 100 && udpPacket->recv.len == 30)
    {
      //printPgnAnnoucement(udpPacket, (char*)"Corrected Position");
      return; // no further PGN processing needed, skip the rest of the checks
    }

    // 0xC8 (200) - Hello from AgIO
    if (udpPacket->recv.buf[3] == 200 && udpPacket->recv.len == 9)
    {
      //printPgnAnnoucement(udpPacket, (char*)"Hello from AgIO");
      LEDs.set(LED_ID::PWR_ETH, PWR_ETH_STATE::AGIO_CONNECTED, true);
      uint8_t helloFromAutoSteer[] = {128, 129, 126, 126, 5, 0, 0, 0, 0, 0, 71};
      if (autoSteerEnabled)
      {
        int16_t sa = (int16_t)(steerAngleActual * 100);

        helloFromAutoSteer[5] = (uint8_t)sa;
        helloFromAutoSteer[6] = sa >> 8;

        uint16_t helloSteerPosition = steeringPosition; // - 6800; steeringPosition is already centered & offset in Autosteer.ino
        helloFromAutoSteer[7] = (uint8_t)helloSteerPosition;
        helloFromAutoSteer[8] = helloSteerPosition >> 8;
        helloFromAutoSteer[9] = switchByte;

        sendUDPbytes(helloFromAutoSteer, sizeof(helloFromAutoSteer));
      }

      // reply as IMU if equipped
      if (BNO.isActive)
      {
        uint8_t helloFromIMU[] = {128, 129, 121, 121, 5, 0, 0, 0, 0, 0, 71};
        sendUDPbytes(helloFromIMU, sizeof(helloFromIMU));
      }

/*#ifdef MACHINE_H
      if (machinePTR->isInit)
      {
        uint8_t helloFromMachine[] = {0x80, 0x81, 123, 123, 5, 0, 0, 0, 0, 0, 71};
        helloFromMachine[5] = B10101010; // should be changed to read actual machine output states
        helloFromMachine[6] = B01010101;
        sendUDPbytes(helloFromMachine, sizeof(helloFromMachine));
      }
#endif*/
      return; // no further PGN processing needed, skip the rest of the checks
    }

    // Subnet Change
    if (udpPacket->recv.buf[3] == 201 && udpPacket->recv.len == 11)
    {
      printPgnAnnoucement(udpPacket, (char*)"Subnet Change");

      if (udpPacket->recv.buf[4] == 5 && udpPacket->recv.buf[5] == 201 && udpPacket->recv.buf[6] == 201) // save in EEPROM and restart
      {
        // Serial << "\r\n- IP changed from " << currentIP;
        netConfig.currentIP[0] = udpPacket->recv.buf[7];
        netConfig.currentIP[1] = udpPacket->recv.buf[8];
        netConfig.currentIP[2] = udpPacket->recv.buf[9];

        // Serial << " to " << currentIP;
        Serial << "\r\n- Saving to EEPROM and restarting Teensy";

        save_current_net(); // save in EEPROM and restart
        delay(10);
        SCB_AIRCR = 0x05FA0004; // Teensy Reset
      }
      return; // no further PGN processing needed, skip the rest of the checks
    }

    // 0xCA (202) - Scan Request
    if (udpPacket->recv.buf[3] == 202 && udpPacket->recv.len == 9)
    {
      if (udpPacket->recv.buf[4] == 3 && udpPacket->recv.buf[5] == 202 && udpPacket->recv.buf[6] == 202)
      {
        printPgnAnnoucement(udpPacket, (char*)"Scan Request");

        uint8_t scanReplySteer[] = {128, 129, 126, 203, 7,
                                    netConfig.currentIP[0], netConfig.currentIP[1], netConfig.currentIP[2], netConfig.currentIP[3],
                                    udpPacket->rem.ip[0], udpPacket->rem.ip[1], udpPacket->rem.ip[2], 23};
        int16_t CK_A = 0;
        for (uint8_t i = 2; i < sizeof(scanReplySteer) - 1; i++)
        {
          CK_A = (CK_A + scanReplySteer[i]);
        }
        scanReplySteer[sizeof(scanReplySteer) - 1] = CK_A;
        sendUDPbytes(scanReplySteer, sizeof(scanReplySteer));

        if (BNO.isActive)
        {
          uint8_t scanReplyIMU[] = {128, 129, 121, 203, 7,
                                    netConfig.currentIP[0], netConfig.currentIP[1], netConfig.currentIP[2], netConfig.currentIP[3],
                                    udpPacket->rem.ip[0], udpPacket->rem.ip[1], udpPacket->rem.ip[2], 23};
          CK_A = 0;
          for (uint8_t i = 2; i < sizeof(scanReplyIMU) - 1; i++)
          {
            CK_A = (CK_A + scanReplyIMU[i]);
          }
          scanReplyIMU[sizeof(scanReplyIMU) - 1] = CK_A;
          sendUDPbytes(scanReplyIMU, sizeof(scanReplyIMU));
        }

        if (gpsActive)
        {
          uint8_t scanReplyGPS[] = {128, 129, 120, 203, 7,
                                    netConfig.currentIP[0], netConfig.currentIP[1], netConfig.currentIP[2], netConfig.currentIP[3],
                                    udpPacket->rem.ip[0], udpPacket->rem.ip[1], udpPacket->rem.ip[2], 23};
          CK_A = 0;
          for (uint8_t i = 2; i < sizeof(scanReplyGPS) - 1; i++)
          {
            CK_A = (CK_A + scanReplyGPS[i]);
          }
          scanReplyGPS[sizeof(scanReplyGPS) - 1] = CK_A;
          sendUDPbytes(scanReplyGPS, sizeof(scanReplyGPS));
        }

        Serial.printf("\r\n ---------\r\n%s\r\nCPU Temp:  %.1f°C\r\nCPU Speed: %iMhz\r\nGPS Baud:  %i", inoVersion, tempmonGetTemp(), F_CPU_ACTUAL / 1000000, baudGPS);
        Serial.printf("\r\nAgIO IP:   %i.%i.%i.%i", udpPacket->rem.ip[0], udpPacket->rem.ip[1], udpPacket->rem.ip[2], udpPacket->rem.ip[3]);
        Serial.printf("\r\nModule IP: %i.%i.%i.%i", netConfig.currentIP[0], netConfig.currentIP[1], netConfig.currentIP[2], netConfig.currentIP[3]);

        if (BNO.isActive)
          Serial.print("\r\nBNO08x available via Serial/RVC Mode");
        else
          Serial.print("\r\n* No IMU available *");

        Serial.println("\r\r\n ---------");
      }
      return; // no further PGN processing needed, skip the rest of the checks
    }

    // 0xFB (251) - SteerConfig
    if (udpPacket->recv.buf[3] == 251 && udpPacket->recv.len == 14)
    {
      printPgnAnnoucement(udpPacket, (char*)"Steer Config");

      uint8_t sett = udpPacket->recv.buf[5]; // setting0
      if (bitRead(sett, 0)) steerConfig.InvertWAS = 1; else steerConfig.InvertWAS = 0;
      if (bitRead(sett, 1)) steerConfig.IsRelayActiveHigh = 1; else steerConfig.IsRelayActiveHigh = 0;
      if (bitRead(sett, 2)) steerConfig.MotorDriveDirection = 1; else steerConfig.MotorDriveDirection = 0;
      if (bitRead(sett, 3)) steerConfig.SingleInputWAS = 1; else steerConfig.SingleInputWAS = 0;
      if (bitRead(sett, 4)) steerConfig.CytronDriver = 1; else steerConfig.CytronDriver = 0;
      if (bitRead(sett, 5)) steerConfig.SteerSwitch = 1; else steerConfig.SteerSwitch = 0;
      if (bitRead(sett, 6)) steerConfig.SteerButton = 1; else steerConfig.SteerButton = 0;
      if (bitRead(sett, 7)) steerConfig.ShaftEncoder = 1; else steerConfig.ShaftEncoder = 0;

      steerConfig.PulseCountMax = udpPacket->recv.buf[6];
      steerConfig.MinSpeed = udpPacket->recv.buf[7];

      sett = udpPacket->recv.buf[8]; // setting1 - Danfoss valve etc
      if (bitRead(sett, 0)) steerConfig.IsDanfoss = 1; else steerConfig.IsDanfoss = 0;
      if (bitRead(sett, 1)) steerConfig.PressureSensor = 1; else steerConfig.PressureSensor = 0;
      if (bitRead(sett, 2)) steerConfig.CurrentSensor = 1; else steerConfig.CurrentSensor = 0;
      if (bitRead(sett, 3)) steerConfig.IsUseY_Axis = 1; else steerConfig.IsUseY_Axis = 0;

      Serial.print("\r\nInvertWAS "); Serial.print(steerConfig.InvertWAS);
      Serial.print("\r\nIsRelayActiveHigh "); Serial.print(steerConfig.IsRelayActiveHigh);
      Serial.print("\r\nMotorDriveDirection "); Serial.print(steerConfig.MotorDriveDirection);
      Serial.print("\r\nSingleInputWAS "); Serial.print(steerConfig.SingleInputWAS);
      Serial.print("\r\nCytronDriver "); Serial.print(steerConfig.CytronDriver);
      Serial.print("\r\nSteerSwitch "); Serial.print(steerConfig.SteerSwitch);
      Serial.print("\r\nSteerButton "); Serial.print(steerConfig.SteerButton);
      Serial.print("\r\nShaftEncoder "); Serial.print(steerConfig.ShaftEncoder);
      Serial.print("\r\nIsDanfoss "); Serial.print(steerConfig.IsDanfoss);
      Serial.print("\r\nPressureSensor "); Serial.print(steerConfig.PressureSensor);
      Serial.print("\r\nCurrentSensor "); Serial.print(steerConfig.CurrentSensor);
      Serial.print("\r\nIsUseY_Axis "); Serial.print(steerConfig.IsUseY_Axis);
      Serial.print("\r\nPulseCountMax "); Serial.print(steerConfig.PulseCountMax);
      Serial.print("\r\nMinSpeed "); Serial.print(steerConfig.MinSpeed);
      Serial.println();

      EEPROM.put(200, steerConfig);
      steerConfigInit();
      return; // no further PGN processing needed, skip the rest of the checks
    } // 0xFB (251) - SteerConfig

    // 0xFC (252) - Steer Settings
    if (udpPacket->recv.buf[3] == 252 && udpPacket->recv.len == 14)
    {
      printPgnAnnoucement(udpPacket, (char*)"Steer Settings");

      // PID values
      steerSettings.Kp = ((float)udpPacket->recv.buf[5]);   // read Kp from AgOpenGPS
      steerSettings.highPWM = udpPacket->recv.buf[6];       // read high pwm
      steerSettings.lowPWM = (float)udpPacket->recv.buf[7]; // read lowPWM from AgOpenGPS
      steerSettings.minPWM = udpPacket->recv.buf[8];        // read the minimum amount of PWM for instant on

      float temp = (float)steerSettings.minPWM * 1.2;
      steerSettings.lowPWM = (byte)temp;

      steerSettings.steerSensorCounts = udpPacket->recv.buf[9]; // sent as setting displayed in AOG

      int16_t newWasOffset = (udpPacket->recv.buf[10]); // read was zero offset Lo
      newWasOffset |= (udpPacket->recv.buf[11] << 8);   // read was zero offset Hi

#ifdef JD_DAC_H
      jdDac.setMaxPWM(steerSettings.highPWM);
      if (newWasOffset != steerSettings.wasOffset)
      {
        jdDac.centerDac();
      }
#endif

      steerSettings.wasOffset = newWasOffset;
      steerSettings.AckermanFix = (float)udpPacket->recv.buf[12] * 0.01;

      Serial.print("\r\n Kp "); Serial.print(steerSettings.Kp);
      Serial.print("\r\n highPWM "); Serial.print(steerSettings.highPWM);
      Serial.print("\r\n lowPWM "); Serial.print(steerSettings.lowPWM);
      Serial.print("\r\n minPWM "); Serial.print(steerSettings.minPWM);
      Serial.print("\r\n steerSensorCounts "); Serial.print(steerSettings.steerSensorCounts);
      Serial.print("\r\n wasOffset "); Serial.print(steerSettings.wasOffset);
      Serial.print("\r\n AckermanFix "); Serial.print(steerSettings.AckermanFix);

      EEPROM.put(100, steerSettings);
      steerSettingsInit();
      return; // no further PGN processing needed, skip the rest of the checks
    } // 0xFC (252) - Steer Settings

    // 0xFE (254) - Steer Data (sent at GPS freq, ie 10hz (100ms))
    if (udpPacket->recv.buf[3] == 254 && udpPacket->recv.len == 14)
    {
      //printPgnAnnoucement(udpPacket, (char*)"Steer Data");

      gpsSpeed = ((float)(udpPacket->recv.buf[5] | udpPacket->recv.buf[6] << 8)) * 0.1; // speed data comes in as km/hr x10
      speedPulse.updateSpeed(gpsSpeed);

      prevGuidanceStatus = guidanceStatus;
      guidanceStatus = udpPacket->recv.buf[7];
      guidanceStatusChanged = (guidanceStatus != prevGuidanceStatus);

      // Bit 8,9    set point steer angle * 100 is sent
      steerAngleSetPoint = ((float)(udpPacket->recv.buf[8] | ((int8_t)udpPacket->recv.buf[9]) << 8)) * 0.01; // high low bytes

      // udpData[10] is XTE (cross track error)
      // udpData[11 & 12] is section 1-16

      if ((bitRead(guidanceStatus, 0) == 0) || (steerState == 0))
      {                                       // || (gpsSpeed < 0.1)) {
        watchdogTimer = WATCHDOG_FORCE_VALUE; // turn off steering motor
        // Serial.print(" OFF");
      }
      else
      {                    // valid conditions to turn on autosteer
        watchdogTimer = 0; // reset watchdog
        // Serial.print(" ON");
      }

      // Bit 10 XTE
      xte = udpPacket->recv.buf[10];
      // Serial.print("\r\nXTE:"); Serial.print(xte-127);

      // Bit 11
      // relay = udpData[11];

      // Bit 12
      // relayHi = udpData[12];

      //----------------------------------------------------------------------------
      // Reply to send to AgIO
      // fromAutoSteerData FD 253 - ActualSteerAngle*100 -56, SwitchByte-7, pwmDisplay-8
      uint8_t PGN_253[] = {0x80, 0x81, 126, 0xFD, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC};

      int16_t sa = (int16_t)(steerAngleActual * 100);
      PGN_253[5] = (uint8_t)sa;
      PGN_253[6] = sa >> 8;

      // heading
      PGN_253[7] = (uint8_t)9999;
      PGN_253[8] = 9999 >> 8;

      // roll
      PGN_253[9] = (uint8_t)8888;
      PGN_253[10] = 8888 >> 8;

      PGN_253[11] = switchByte;
      PGN_253[12] = (uint8_t)abs(pwmDisplay);

      // checksum
      int16_t CK_A = 0;
      for (uint8_t i = 2; i < sizeof(PGN_253) - 1; i++)
        CK_A = (CK_A + PGN_253[i]);

      PGN_253[sizeof(PGN_253) - 1] = CK_A;

      // off to AOG
      sendUDPbytes(PGN_253, sizeof(PGN_253));

      if (aog2Count++ > 1)
      { // send 1/2 of Steer Data rate (GPS hz / 2)
        // fromAutoSteerData FD 250 - sensor values etc
        uint8_t PGN_250[] = {0x80, 0x81, 126, 0xFA, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC};

        if (steerConfig.PressureSensor || steerConfig.CurrentSensor)
        {
          PGN_250[5] = (byte)sensorReading;
        }
        else
        {
          PGN_250[5] = (byte)pulseCount;
        }

        CK_A = 0;
        for (uint8_t i = 2; i < sizeof(PGN_250) - 1; i++)
        {
          CK_A = (CK_A + PGN_250[i]);
        }
        PGN_250[sizeof(PGN_250) - 1] = CK_A;

        sendUDPbytes(PGN_250, sizeof(PGN_250));
        aog2Count = 0;
      }
      return; // no further PGN processing needed, skip the rest of the checks
    } // 0xFE (254) - Steer Data

    mg_iobuf_del(&udpPacket->recv, 0, udpPacket->recv.len);
    PGNusage.timeOut();
  }
  else
  {
    mg_iobuf_del(&udpPacket->recv, 0, udpPacket->recv.len);
  }
  PGNusage.timeOut();   // ensure we stop counting cpu time
}

// Process data received on port 2233
void rtcmHandler(struct mg_connection *rtcm, int ev, void *ev_data, void *fn_data)
{
  NTRIPusage.timeIn();
  if (g_mgr.ifp->state != MG_TCPIP_STATE_READY)
    return; // Check if IP stack is up.
  if (ev == MG_EV_READ && mg_ntohs(rtcm->rem.port) == 9999 && rtcm->recv.len >= 5)
  {
    for (int i = 0; i <= rtcm->recv.len; i++)
    {
      if (!USB1DTR)
        SerialGPS1.write(rtcm->recv.buf[i]);
      // only send to GPS2 if using abnormal setup like OGX receiver on GPS2
      /*if (!USB2DTR)
        SerialGPS2.write(rtcm->recv.buf[i]);*/
      LEDs.queueBlueFlash(LED_ID::GPS);
    }
    mg_iobuf_del(&rtcm->recv, 0, rtcm->recv.len);
  }
  else
  {
    mg_iobuf_del(&rtcm->recv, 0, rtcm->recv.len);
  }
  NTRIPusage.timeOut();
}

// Setup UDP comms channels
void udpSetup()
{
  g_mgr.ifp->enable_dhcp_client = 0;
  g_mgr.ifp->ip = ipv4ary(netConfig.currentIP);
  g_mgr.ifp->gw = ipv4ary(netConfig.gatewayIP);
  g_mgr.ifp->mask = MG_IPV4(255, 255, 255, 0);

  char pgnListenURL[50];
  char rtcmListen[50];
  mg_snprintf(pgnListenURL, sizeof(pgnListenURL), "udp://%d.%d.%d.126:8888", netConfig.currentIP[0], netConfig.currentIP[1], netConfig.currentIP[2]);
  // Serial.println(steerListen);
  mg_snprintf(rtcmListen, sizeof(rtcmListen), "udp://%d.%d.%d.126:2233", netConfig.currentIP[0], netConfig.currentIP[1], netConfig.currentIP[2]);
  // Serial.println(rtcmListen);
  //bool listenPGNs = false;
  //bool listenRtcm = false;
  //bool agioConnect = false;

  if (mg_listen(&g_mgr, pgnListenURL, pgnHandler, NULL) != NULL)
  {
    //listenPGNs = true;
    MG_DEBUG(("Listening for AgIO on UDP 8888"));
  }
  else
  {
    MG_DEBUG(("AgIO on UDP 8888 did not open"));
  }

  if (mg_listen(&g_mgr, rtcmListen, rtcmHandler, NULL) != NULL)
  {
    //listenRtcm = true;
    MG_DEBUG(("Listening for RTCM on UDP 2233"));
  }
  else
  {
    MG_DEBUG(("RTCM on UDP 2233 did not open"));
  }

  // Create UDP connection to broadcast address
  char agioURL[25];
  strcpy(agioURL, "udp://");
  itoa(netConfig.currentIP[0], agioURL + strlen(agioURL), 10);
  strcat(agioURL, ".");
  itoa(netConfig.currentIP[1], agioURL + strlen(agioURL), 10);
  strcat(agioURL, ".");
  itoa(netConfig.currentIP[2], agioURL + strlen(agioURL), 10);
  strcat(agioURL, ".255:9999");

  sendAgio = mg_connect(&g_mgr, agioURL, NULL, NULL);
  if (sendAgio == !NULL)
  {
    //agioConnect = true;
    MG_DEBUG(("Connected to AgIO"));
  }
  else
  {
    MG_DEBUG(("Trying to connect to AgIO"));
    return;
  }
}

// callback function for Machine class to send data back to AgIO/AOG
void machinePgnReplies(uint8_t *pgnData, uint8_t len, IPAddress destIP)
{
  sendUDPbytes(pgnData, len);
  /*Serial.printf("\r\nSending PGN reply: %i >", len);
  for (uint8_t i = 0; i < len; i++){
    Serial.print(pgnData[i]);
    Serial.print(":");
  }*/
}

#endif // UDPHANDLERS_H_
