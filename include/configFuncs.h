#ifndef CONFIGFUNCS_H
#define CONFIGFUNCS_H

#include "Arduino.h"

extern "C" void save_config()
{
    Serial.println("Saving config ...");
    glue_get_settings(&aio_settings);
    MG_DEBUG(("aio_settings: %s,%d,%d,%d,%d,%d,%d", aio_settings.fversion, aio_settings.bd_ip1, aio_settings.bd_ip2, aio_settings.bd_ip3, aio_settings.bd_ip4 ,aio_settings.gps_type, aio_settings.gps_pass));
    currentIP[0] = aio_settings.bd_ip1;
    currentIP[1] = aio_settings.bd_ip2;
    currentIP[2] = aio_settings.bd_ip3;

    MG_DEBUG(("currentIP: %d,%d,%d,126", currentIP[0], currentIP[1], currentIP[2]));


    // strcpy(aio_settings.fversion, inoVersion);
    // glue_set_settings(&aio_settings);
}

// Write default IP to module
void save_default_ip(void)
{
  // IP stored in 300
  EEPROM.put(300, defaultIP[0]);
  EEPROM.put(301, defaultIP[1]);
  EEPROM.put(302, defaultIP[2]);
}

// Write current IP to module
void save_current_ip(void)
{
  // ID stored in 60
  EEPROM.put(300, currentIP[0]);
  EEPROM.put(301, currentIP[1]);
  EEPROM.put(302, currentIP[2]);
}

void load_current_ip(void)
{
  // ID stored in 60
  EEPROM.get(300, currentIP[0]);
  EEPROM.get(301, currentIP[1]);
  EEPROM.get(302, currentIP[2]);
}

void ipSetup()
{

  uint16_t eth_ee_read;
  EEPROM.get(1, eth_ee_read);

  if (eth_ee_read != EE_ver)
  { // if EE is out of sync, write defaults to EE
    EEPROM.put(1, EE_ver);
    save_default_ip();
    Serial.print("\r\n\nWriting IP address defaults to EEPROM\r\n");
  }
  else
  {
    EEPROM.get(300, currentIP[0]);
    EEPROM.get(301, currentIP[1]);
    EEPROM.get(302, currentIP[2]);
    Serial.print("\r\n\nLoaded IP address from EEPROM\r\n");
  }

  gatewayIP[0] = currentIP[0];
  gatewayIP[1] = currentIP[1];
  gatewayIP[2] = currentIP[2];
  gatewayIP[3] = 1;

  broadcastIP[0] = currentIP[0];
  broadcastIP[1] = currentIP[1];
  broadcastIP[2] = currentIP[2];
  broadcastIP[3] = 255; // same subnet as module's IP but use broadcast

}

static uint32_t ipv4str(const char *str)
{
  struct mg_addr a = {};
  mg_aton(mg_str(str), &a);
  return *(uint32_t *)&a.ip;
}

static uint32_t ipv4ary(const uint8_t input[])
{
  char buf[16];
  mg_snprintf(buf, sizeof(buf), "%d.%d.%d.%d", input[0], input[1], input[2], input[3]);
  struct mg_addr a = {};
  mg_aton(mg_str(buf), &a);
  return *(uint32_t *)&a.ip;
}

#endif