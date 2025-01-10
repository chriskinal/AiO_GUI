#ifndef CONFIG_UTILS_H_
#define CONFIG_UTILS_H_

#include "mongoose_glue.h"
#include "LittleFS.h"
#include "ArduinoJson-v7.3.0.h"

// Configuration system variables
LittleFS_Program aioFS;
#define NF 10
static File files[NF + 1];
static int fsInit = 0;
const char *filename = "/config.txt";
settings s_config;
// End

// Initialize LittleFS disk
void fileInit()
{
  aioFS.begin(1 * 1024 * 1024);
  Serial.print("\r\nLittleFS: initialized");
  Serial.printf("\r\nTotal Size: %llu bytes, Used: %llu\r\n", aioFS.totalSize(), aioFS.usedSize());
  for (int i = 0; i <= NF; i++)
  {
    files[i] = 0;
  }
  fsInit = 1;
}

// Saves the configuration to a file
extern "C" void saveConfig(const char *filename, const settings &s_config)
{
  Serial.println(F("Saving configuration..."));
  // glue_update_state();
  glue_get_settings(&s_config);
  // Delete existing file, otherwise the configuration is appended to the file
  aioFS.remove(filename);

  File file = aioFS.open(filename, FILE_WRITE);
  if (!file)
  {
    Serial.println(F("Failed to create file"));
    return;
  }

  JsonDocument doc;

  doc["fversion"] = s_config.fversion;
  doc["bd_ip1"] = s_config.bd_ip1;
  doc["bd_ip2"] = s_config.bd_ip2;
  doc["bd_ip3"] = s_config.bd_ip3;
  doc["gps_type"] = s_config.gps_type;
  doc["gps_pass"] = s_config.gps_pass;
  doc["ss_Kp"] = steerSettings.Kp;
  doc["ss_lowPWM"] = steerSettings.lowPWM;
  doc["ss_wasOffset"] = steerSettings.wasOffset;
  doc["ss_minPWM"] = steerSettings.minPWM;
  doc["ss_highPWM"] = steerSettings.highPWM;
  doc["ss_steerSensorCounts"] = steerSettings.steerSensorCounts;
  doc["ss_AckermanFix"] = steerSettings.AckermanFix;
  doc["sc_InvertWAS"] = steerConfig.InvertWAS;
  doc["sc_IsRelayActiveHigh"] = steerConfig.IsRelayActiveHigh;
  doc["sc_MotorDriveDirection"] = steerConfig.MotorDriveDirection;
  doc["sc_SingleInputWAS"] = steerConfig.SingleInputWAS;
  doc["sc_CytronDriver"] = steerConfig.CytronDriver;
  doc["sc_SteerSwitch"] = steerConfig.SteerSwitch;
  doc["sc_SteerButton"] = steerConfig.SteerButton;
  doc["sc_ShaftEncoder"] = steerConfig.ShaftEncoder;
  doc["sc_PressureSensor"] = steerConfig.PressureSensor;
  doc["sc_CurrentSensor"] = steerConfig.CurrentSensor;
  doc["sc_PulseCountMax"] = steerConfig.PulseCountMax;
  doc["sc_IsDanfoss"] = steerConfig.IsDanfoss;
  doc["sc_IsUseY_Axis"] = steerConfig.IsUseY_Axis;
  doc["sc_MinSpeed"] = steerConfig.MinSpeed;


  if (serializeJson(doc, file) == 0)
  {
    Serial.println(F("Failed to write to file"));
  }

  file.close();
  Serial.printf("s_config: %s,%d,%d,%d,%d,%o\r\n", s_config.fversion, s_config.bd_ip1, s_config.bd_ip2, s_config.bd_ip3, s_config.gps_type, s_config.gps_pass);
  Serial.println(F("Saved configuration..."));
}

// Load configuration - Should load default config if run for the first time
void loadConfig(const char *filename, settings &s_config)
{
  Serial.println(F("Loading configuration..."));
  // Serial.printf("%s,%d,%d,%d,%d,%o\r\n", s_config.fversion,s_config.bd_ip1,s_config.bd_ip2, s_config.bd_ip3, s_config.gps_type,s_config.gps_pass);
  File file = aioFS.open(filename);
  JsonDocument doc;

  DeserializationError error = deserializeJson(doc, file);
  if ( error ) {Serial.println(F("Failed to read config file, using default configuration"));}
  // Populate the configuation array from the JSON doc | from the default values
  strlcpy(s_config.fversion,                     // <- destination
          doc["fversion"] | "AiO v5.0a Web GUI", // <- source Note: value after the | is the default if "fversion" is empty in the JSON document.
          sizeof(s_config.fversion));            // <- destination's capacity
  s_config.bd_ip1 = doc["bd_ip1"] | 192;
  s_config.bd_ip2 = doc["bd_ip2"] | 168;
  s_config.bd_ip3 = doc["bd_ip3"] | 5;
  s_config.gps_type = doc["gps_type"] | 2;
  s_config.gps_pass = doc["gps_pass"] | false;
  steerSettings.Kp = doc["ss_Kp"] | 40;
  steerSettings.lowPWM = doc["ss_lowPWM"] | 10;
  steerSettings.wasOffset = doc["ss_wasOffset"] | 0;
  steerSettings.minPWM = doc["ss_minPWM"] | 9;
  steerSettings.highPWM = doc["ss_highPWM"] | 150;
  steerSettings.steerSensorCounts = doc["ss_steerSensorCounts"] | 120;
  steerSettings.AckermanFix = doc["ss_AckermanFix"] | 1;
  steerConfig.InvertWAS = doc["sc_InvertWAS"] | 0;
  steerConfig.IsRelayActiveHigh= doc["sc_IsRelayActiveHigh"] | 0;
  steerConfig.MotorDriveDirection = doc["sc_MotorDriveDirection"] | 0;
  steerConfig.SingleInputWAS = doc["sc_SingleInputWAS"] | 1;
  steerConfig.CytronDriver = doc["sc_CytronDriver"] | 0;
  steerConfig.SteerSwitch = doc["sc_SteerSwitch"] | 0;
  steerConfig.SteerButton =doc["sc_SteerButton"] | 0;
  steerConfig.ShaftEncoder = doc["sc_ShaftEncoder"] | 0;
  steerConfig.PressureSensor = doc["sc_PressureSensor"] | 0;
  steerConfig.CurrentSensor = doc["sc_CurrentSensor"] | 0;
  steerConfig.PulseCountMax = doc["sc_PulseCountMax"] |3;
  steerConfig.IsDanfoss = doc["sc_IsDanfoss"] | 0;
  steerConfig.IsUseY_Axis = doc["sc_IsUseY_Axis"] | 0;
  steerConfig.MinSpeed = doc["sc_MinSpeed"] | 0;
  // End
  glue_set_settings(&s_config);
  if ( error ) { saveConfig(filename, s_config); } // save the default config if there was a load error.
  file.close();
}

// Prints the JSON content of a file to the Serial port
void printFile(const char *filename)
{
  // Dump config file
  Serial.println(F("Print config file..."));
  File file = aioFS.open(filename);
  if (!file)
  {
    Serial.println(F("Failed to read file"));
    return;
  }

  while (file.available())
  {
    Serial.print((char)file.read());
  }
  Serial.println();

  file.close();
}

#endif