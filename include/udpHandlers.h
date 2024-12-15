#include "mongoose.h"

// Process data received on port 8888
void steerHandler(struct mg_connection *steer, int ev, void *ev_data, void *fn_data)
{
  if (ev == MG_EV_READ && mg_ntohs(steer->rem.port) == 9999 && steer->recv.len >= 5)
  {
    Serial.println("\r\nSteer UDP received");
    Serial.println(steer->recv.len);
    for (int i = 0; i <= steer->recv.len; i++)
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
  if (ev == MG_EV_READ && mg_ntohs(rtcm->rem.port) == 9999 && rtcm->recv.len >= 5)
  {

    Serial.println(rtcm->recv.len);

    Serial.println("\r\nRTCM UDP received");
    Serial.println(rtcm->recv.len);
    for (int i = 0; i <= rtcm->recv.len; i++)
    {
      Serial.print(rtcm->recv.buf[i]);
      Serial.print(" ");
    }
    Serial.println();
    mg_iobuf_del(&rtcm->recv, 0, rtcm->recv.len);
  }
  else
  {
    mg_iobuf_del(&rtcm->recv, 0, rtcm->recv.len);
  }
}
