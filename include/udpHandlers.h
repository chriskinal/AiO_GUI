#include "mongoose.h"

// Process data received on port 8888
void steerHandler(struct mg_connection *steer, int ev, void *ev_data, void *fn_data)
{
  if ( ev == MG_EV_READ && mg_ntohs(steer->rem.port) == 9999 && steer->recv.len >= 5)
  {
    Serial.println("\r\nSteer UDP received");
    Serial.println(steer->recv.len);
    for ( int i = 0; i <= steer->recv.len; i++ )
    {
    Serial.print(steer->recv.buf[i]);
    Serial.print(" ");
    }
    Serial.println();
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
        }
        mg_iobuf_del(&rtcm->recv, 0, rtcm->recv.len);
    }
    else
    {
        mg_iobuf_del(&rtcm->recv, 0, rtcm->recv.len);  
    }
}

// Process data to be sent to AgIO

void sendUDP(char *message)
{
  // Create connection URL
  struct mg_connection *sendAgio;
  String agioURL = String("udp://") + String(currentIP[0]) + String(".") + String(currentIP[1]) + String(".") + String(currentIP[2]) + String(".255:9999");
  char agioSend[agioURL.length() + 1] ={};
  strcpy(agioSend, agioURL.c_str());

  // Create UDP connection to broadcast address
  sendAgio = mg_connect(&g_mgr, agioSend, NULL, NULL);
  if (sendAgio == NULL) {
    Serial.println("Failed to send message to AgIO");
    return;
  }

// Send data
  if (mg_send(sendAgio, message, strlen(message)) <= 0) {
    Serial.println("Failed to send data\r\n");
  }

  // Close the UDP connection
  mg_close_conn(sendAgio);
}

