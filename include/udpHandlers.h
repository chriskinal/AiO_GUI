#include "Arduino.h"
#include "mongoose.h"

// Process data to be sent to AgIO
void sendUDP(char *message, int msgLen)
{
  // Create connection URL
  struct mg_connection *sendAgio;
  String agioURL = String("udp://") + String(currentIP[0]) + String(".") + String(currentIP[1]) + String(".") + String(currentIP[2]) + String(".255:9999");
  char agioSend[agioURL.length() + 1] ={};
  strcpy(agioSend, agioURL.c_str());

  // Create UDP connection to broadcast address
  sendAgio = mg_connect(&g_mgr, agioSend, NULL, NULL);
  if (sendAgio == NULL) {
    Serial.println("Failed to connect to AgIO");
    return;
  }
  // Send data
  if (mg_send(sendAgio, message, msgLen) <= 0) {
    Serial.println("Failed to send data\r\n");
  }

  // Close the UDP connection
  mg_close_conn(sendAgio);
}

// Process data received on port 8888
void steerHandler(struct mg_connection *steer, int ev, void *ev_data, void *fn_data)
{
  if ( ev == MG_EV_READ && mg_ntohs(steer->rem.port) == 9999 && steer->recv.len >= 5)
  {
    // Serial.println("\r\nSteer UDP received");
    // Serial.println(steer->recv.len);
    // for ( int i = 0; i <= steer->recv.len; i++ )
    // {
    // Serial.print(steer->recv.buf[i]);
    // Serial.print(" ");
    // }
    // Serial.println();

    //Verify first 3 PGN header bytes
    if ( steer->recv.buf[0] != 128 || steer->recv.buf[1] != 129 || steer->recv.buf[2] != 127 ) return;

    // 0x64 (100) - Corrected Position
    if ( steer->recv.buf[3] == 100 && steer->recv.len == 22 ) {}

    // 0xC8 (200) - Hello from AgIO
    if ( steer->recv.buf[3] == 200 && steer->recv.len == 9 )
    {
      LEDs.set(LED_ID::PWR_ETH, PWR_ETH_STATE::AGIO_CONNECTED, true);
      uint8_t helloFromAutoSteer[] = { 128, 129, 126, 126, 5, 0, 0, 0, 0, 0, 71 };
      if (autoSteerEnabled)
      {
      int16_t sa = (int16_t)(steerAngleActual * 100);

      helloFromAutoSteer[5] = (uint8_t)sa;
      helloFromAutoSteer[6] = sa >> 8;

      uint16_t helloSteerPosition = steeringPosition; // - 6800; steeringPosition is already centered & offset in Autosteer.ino
      helloFromAutoSteer[7] = (uint8_t)helloSteerPosition;
      helloFromAutoSteer[8] = helloSteerPosition >> 8;
      helloFromAutoSteer[9] = switchByte;

      sendUDP(helloFromAutoSteer, sizeof(helloFromAutoSteer));
      }
    }
    
    // Subnet Change
    if (steer->recv.buf[3] == 201 && steer->recv.len == 11)
    {
      Serial.println("Subnet Change");

      if (steer->recv.buf[4] == 5 && steer->recv.buf[5] == 201 && steer->recv.buf[6] == 201)  //save in EEPROM and restart
      {
        //Serial << "\r\n- IP changed from " << currentIP;
        currentIP[0] = steer->recv.buf[7];
        currentIP[1] = steer->recv.buf[8];
        currentIP[2] = steer->recv.buf[9];

        //Serial << " to " << currentIP;
        Serial << "\r\n- Saving to EEPROM and restarting Teensy";

        SaveCurModuleIP();  //save in EEPROM and restart
        delay(10);
        SCB_AIRCR = 0x05FA0004;  //Teensy Reset
      }
      return;                    // no other processing needed
    }

    // reply as IMU if equipped
    if (BNO.isActive) {
      uint8_t helloFromIMU[] = { 128, 129, 121, 121, 5, 0, 0, 0, 0, 0, 71 };
      sendUDP(helloFromIMU, sizeof(helloFromIMU));
    }

    // 0xCA (202) - Scan Request
    if (steer->recv.buf[3] == 202 && steer->recv.len == 9)
    {
      if (steer->recv.buf[4] == 3 && steer->recv.buf[5] == 202 && steer->recv.buf[6] == 202) {

        uint8_t scanReplySteer[] = { 128, 129, 126, 203, 7,
                                currentIP[0], currentIP[1], currentIP[2], currentIP[3],
                                steer->rem.ip[0], steer->rem.ip[1], steer->rem.ip[2], 23 };
        int16_t CK_A = 0;
        for (uint8_t i = 2; i < sizeof(scanReplySteer) - 1; i++) {
          CK_A = (CK_A + scanReplySteer[i]);
        }
        scanReplySteer[sizeof(scanReplySteer) - 1] = CK_A;
        sendUDP(scanReplySteer, sizeof(scanReplySteer));

        if (BNO.isActive) {
          uint8_t scanReplyIMU[] = { 128, 129, 121, 203, 7,
                                  currentIP[0], currentIP[1], currentIP[2], currentIP[3],
                                  steer->rem.ip[0], steer->rem.ip[1], steer->rem.ip[2], 23 };
          CK_A = 0;
          for (uint8_t i = 2; i < sizeof(scanReplyIMU) - 1; i++) {
            CK_A = (CK_A + scanReplyIMU[i]);
          }
          scanReplyIMU[sizeof(scanReplyIMU) - 1] = CK_A;
          sendUDP(scanReplyIMU, sizeof(scanReplyIMU));
        }

        if (gpsActive) {
          uint8_t scanReplyGPS[] = { 128, 129, 120, 203, 7,
                                  currentIP[0], currentIP[1], currentIP[2], currentIP[3],
                                  steer->rem.ip[0], steer->rem.ip[1], steer->rem.ip[2], 23 };
          CK_A = 0;
          for (uint8_t i = 2; i < sizeof(scanReplyGPS) - 1; i++) {
            CK_A = (CK_A + scanReplyGPS[i]);
          }
          scanReplyGPS[sizeof(scanReplyGPS) - 1] = CK_A;
          sendUDP(scanReplyGPS, sizeof(scanReplyGPS));
        }

        #ifdef MACHINE_H
        if (machinePTR->isInit) {
          uint8_t scanReplyMachine[] = { 128, 129, 123, 203, 7,
                                  currentIP[0], currentIP[1], currentIP[2], currentIP[3],
                                  steer->rem.ip[0], steer->rem.ip[1], steer->rem.ip[2], 23 };
          CK_A = 0;
          for (uint8_t i = 2; i < sizeof(scanReplyMachine) - 1; i++) {
            CK_A = (CK_A + scanReplyMachine[i]);
          }
          scanReplyMachine[sizeof(scanReplyMachine) - 1] = CK_A;
          sendUDP(scanReplyMachine, sizeof(scanReplyMachine));
        }
        #endif

        Serial.printf("\r\n ---------\r\n%s\r\nCPU Temp:%.1f CPU Speed:%iMhz GPS Baud:%i", inoVersion, tempmonGetTemp(), F_CPU_ACTUAL / 1000000, baudGPS);
        Serial.printf("\r\nAgIO IP:   ", steer->rem.ip[0], steer->rem.ip[1], steer->rem.ip[2], steer->rem.ip[3] );
        Serial.printf("\r\nModule IP: ", currentIP[0], currentIP[1], currentIP[2], currentIP[3] );
        // Serial.print("\r\nAgIO IP:   "); Serial.print(rem_ip);
        // Serial.print("\r\nModule IP: "); Serial.print(UDP.myIP);

        if (BNO.isActive) Serial.print("\r\nBNO08x available via Serial/RVC Mode");
        else Serial.print("\r\n* No IMU available *");
        
        Serial.println("\r\r\n ---------");
      }
      return;
    }

    if (steer->recv.buf[3] == 0xFB && steer->recv.len == 14)  // 0xFB (251) - SteerConfig
    {
      uint8_t sett = steer->recv.buf[5]; //setting0
      if (bitRead(sett, 0)) steerConfig.InvertWAS = 1; else steerConfig.InvertWAS = 0;
      if (bitRead(sett, 1)) steerConfig.IsRelayActiveHigh = 1; else steerConfig.IsRelayActiveHigh = 0;
      if (bitRead(sett, 2)) steerConfig.MotorDriveDirection = 1; else steerConfig.MotorDriveDirection = 0;
      if (bitRead(sett, 3)) steerConfig.SingleInputWAS = 1; else steerConfig.SingleInputWAS = 0;
      if (bitRead(sett, 4)) steerConfig.CytronDriver = 1; else steerConfig.CytronDriver = 0;
      if (bitRead(sett, 5)) steerConfig.SteerSwitch = 1; else steerConfig.SteerSwitch = 0;
      if (bitRead(sett, 6)) steerConfig.SteerButton = 1; else steerConfig.SteerButton = 0;
      if (bitRead(sett, 7)) steerConfig.ShaftEncoder = 1; else steerConfig.ShaftEncoder = 0;

      steerConfig.PulseCountMax = steer->recv.buf[6];
      steerConfig.MinSpeed = steer->recv.buf[7];

      sett = steer->recv.buf[8]; //setting1 - Danfoss valve etc
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

      EEPROM.put(40, steerConfig);            
      return;             // no other processing needed
    }  // 0xFB (251) - SteerConfig

    mg_iobuf_del(&steer->recv, 0, steer->recv.len);
  }
    else
    {
    mg_iobuf_del(&steer->recv, 0, steer->recv.len);  
    }
}

// Process data received on port 2233
void rtcmHandler(struct mg_connection *rtcm, int ev, void *ev_data, void *fn_data)
{
    if ( ev == MG_EV_READ && mg_ntohs(rtcm->rem.port) == 9999 && rtcm->recv.len >= 5 )
    {
        for ( int i = 0; i <= rtcm->recv.len; i++ )
        {
        if (!USB1DTR) SerialGPS1.write(rtcm->recv.buf[i]);
        if (!USB2DTR) SerialGPS2.write(rtcm->recv.buf[i]);
        LEDs.queueBlueFlash(LED_ID::GPS);
        }
        mg_iobuf_del(&rtcm->recv, 0, rtcm->recv.len);
    }
    else
    {
        mg_iobuf_del(&rtcm->recv, 0, rtcm->recv.len);  
    }
}

