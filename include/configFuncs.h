#ifndef CONFIGFUNCS_H
#define CONFIGFUNCS_H

#include "Arduino.h"
#include "mongoose_glue.h"

// Write default IP to EEPROM
void save_default_ip()
{
    // IP stored in 300
    EEPROM.put(300, defaultIP[0]);
    EEPROM.put(301, defaultIP[1]);
    EEPROM.put(302, defaultIP[2]);
}

// Write current IP to EEPROM
void save_current_ip()
{
    // IP stored in 300
    EEPROM.put(300, currentIP[0]);
    EEPROM.put(301, currentIP[1]);
    EEPROM.put(302, currentIP[2]);
}

// Load current IP from EEPROM
void load_current_ip()
{
    // IP loaded from 300
    EEPROM.get(300, currentIP[0]);
    EEPROM.get(301, currentIP[1]);
    EEPROM.get(302, currentIP[2]);
}

// Save GPS settings to EEPROM
void save_gps()
{
    // GPS saved to 400
    EEPROM.put(400, gpsType);
    EEPROM.put(401, gpsPass);
}

// load GPS settings from EEPROM
void load_gps()
{
    // GPS read from 400
    EEPROM.get(400, gpsType);
    EEPROM.get(401, gpsPass);
}

// Save the config values from the GUI to firmware variables and EEPROM
extern "C" void save_config()
{
    Serial.println("Saving config ...");
    glue_get_settings(&aio_settings);
    MG_DEBUG(("aio_settings: %s,%d,%d,%d,%d,%d,%d", aio_settings.fversion, aio_settings.bd_ip1, aio_settings.bd_ip2, aio_settings.bd_ip3, aio_settings.bd_ip4, aio_settings.gps_type, aio_settings.gps_pass));

    currentIP[0] = aio_settings.bd_ip1;
    currentIP[1] = aio_settings.bd_ip2;
    currentIP[2] = aio_settings.bd_ip3;
    save_current_ip();

    gpsType = aio_settings.gps_type;
    gpsPass = aio_settings.gps_pass;
    save_gps();
}

// Load the config values from the firmware to the GUI
extern "C" void load_config()
{
    Serial.println("Loading config ...");

    load_current_ip();
    aio_settings.bd_ip1 = currentIP[0];
    aio_settings.bd_ip2 = currentIP[1];
    aio_settings.bd_ip3 = currentIP[2];

    load_gps();
    aio_settings.gps_type = gpsType;
    aio_settings.gps_pass = gpsPass;

    strcpy(aio_settings.fversion, inoVersion);

    MG_DEBUG(("aio_settings: %s,%d,%d,%d,%d,%d,%d", aio_settings.fversion, aio_settings.bd_ip1, aio_settings.bd_ip2, aio_settings.bd_ip3, aio_settings.bd_ip4, aio_settings.gps_type, aio_settings.gps_pass));
    glue_set_settings(&aio_settings);
    glue_update_state();
}

// Load the IP address from EEPROM
void ipSetup()
{
    uint16_t eth_ee_read;
    EEPROM.get(1, eth_ee_read);

    if (eth_ee_read != EE_ver)
    { // if EE is out of sync, write defaults to EE
        EEPROM.put(1, EE_ver);
        save_default_ip();
        load_current_ip();
        Serial.print("\r\n\nWriting IP address defaults to EEPROM\r\n");
    }
    else
    {
        load_current_ip();
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