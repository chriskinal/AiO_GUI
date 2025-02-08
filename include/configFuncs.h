// AIO_GUI is copyright 2025 by the AOG Group
// AiO_GUI is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
// AiO_GUI is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with Foobar. If not, see <https://www.gnu.org/licenses/>.
// Like most Arduino code, portions of this are based on other open source Arduino code with a compatiable license.

#ifndef CONFIGFUNCS_H
#define CONFIGFUNCS_H

#include "Arduino.h"
#include "mongoose_glue.h"

// Write default IP to EEPROM
void save_default_net()
{
    // IP stored in 300
    EEPROM.put(300, defaultNet);
}

// Write current IP to EEPROM
void save_current_net()
{
    // IP stored in 300
    netConfig.gatewayIP[0] = netConfig.currentIP[0];
    netConfig.gatewayIP[1] = netConfig.currentIP[1];
    netConfig.gatewayIP[2] = netConfig.currentIP[2];
    netConfig.gatewayIP[3] = 1;

    netConfig.broadcastIP[0] = netConfig.currentIP[0];
    netConfig.broadcastIP[1] = netConfig.currentIP[1];
    netConfig.broadcastIP[2] = netConfig.currentIP[2];
    netConfig.broadcastIP[3] = 255; // same subnet as module's IP but use broadcast

    EEPROM.put(300, netConfig);
}

// Load current IP from EEPROM
void load_current_net()
{
    // IP loaded from 300
    EEPROM.get(300, netConfig);
}

// Save GPS settings to EEPROM
void save_gps()
{
    // GPS saved to 400
    EEPROM.put(400, gpsConfig);
}

// load GPS settings from EEPROM
void load_gps()
{
    // GPS read from 400
    EEPROM.get(400, gpsConfig);
}

// Save the config values from the GUI to firmware variables and EEPROM
extern "C" void save_config()
{
    Serial.println("Saving config ...");
    glue_get_settings(&aio_settings);
    MG_DEBUG(("aio_settings: %s,%d,%d,%d,%d,%d,%d", aio_settings.fversion, aio_settings.bd_ip1, aio_settings.bd_ip2, aio_settings.bd_ip3, aio_settings.bd_ip4, aio_settings.gps_sync, aio_settings.gps_pass));

    netConfig.currentIP[0] = aio_settings.bd_ip1;
    netConfig.currentIP[1] = aio_settings.bd_ip2;
    netConfig.currentIP[2] = aio_settings.bd_ip3;
    netConfig.currentIP[3] = aio_settings.bd_ip4;
    save_current_net();

    gpsConfig.gpsSync = aio_settings.gps_sync;
    gpsConfig.gpsPass = aio_settings.gps_pass;
    save_gps();
}

// Load the config values from the firmware to the GUI
extern "C" void load_config()
{
    Serial.println("Loading config ...");

    load_current_net();
    aio_settings.bd_ip1 = netConfig.currentIP[0];
    aio_settings.bd_ip2 = netConfig.currentIP[1];
    aio_settings.bd_ip3 = netConfig.currentIP[2];
    aio_settings.bd_ip4 = netConfig.currentIP[3];
    load_gps();
    aio_settings.gps_sync = gpsConfig.gpsSync;
    aio_settings.gps_pass = gpsConfig.gpsPass;

    strcpy(aio_settings.fversion, inoVersion);

    MG_DEBUG(("aio_settings: %s,%d,%d,%d,%d,%d,%d", aio_settings.fversion, aio_settings.bd_ip1, aio_settings.bd_ip2, aio_settings.bd_ip3, aio_settings.bd_ip4, aio_settings.gps_sync, aio_settings.gps_pass));
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
        save_default_net();
        load_current_net();
        Serial.print("\r\n\nWriting IP address defaults to EEPROM\r\n");
    }
    else
    {
        load_current_net();
        Serial.print("\r\n\nLoaded IP address from EEPROM\r\n");
    }
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