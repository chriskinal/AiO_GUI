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
    
    // reply as IMU if equipped
    if (BNO.isActive) {
      uint8_t helloFromIMU[] = { 128, 129, 121, 121, 5, 0, 0, 0, 0, 0, 71 };
      sendUDP(helloFromIMU, sizeof(helloFromIMU));
    }

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

